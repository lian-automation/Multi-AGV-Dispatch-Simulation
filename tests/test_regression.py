# -*- coding: utf-8 -*-
"""
test_regression.py —— 最小回归测试（审查报告06 · 路线图第6条）
==============================================================

覆盖：
    1. P1-1 回归：任务取货点 == AGV 脚下格（"原地接单"退化分配）时，
       不再产生伪移动 / ghost cell，"一格一车"不变式全程成立；
    2. commit_arrival 守卫：old==new 不删除脚下格占用；
    3. 不变式校验器：注入"两车同格 / 占格无主"违规必须被捕获（fail-fast）；
    4. P2-3 回归：同一物理死锁环持续存在时 deadlock_detected 去重，
       仲裁触发按冷却期照常累加，环消失才计"消除"；
    5. 仲裁自旋修复 回归：让路者 goal 恰为被堵格（互等方脚下格）时，
       仲裁不得凭 A* 终点豁免的"假成功"路径无限自旋——必须有限时间内
       破环（升级侧避改道）或如实计入 unresolved 上报；
    6. 可观测化改造 回归：不变式违规的可观测化处置——压测/批处理（strict）
       违规 fail-fast 中止、报告如实写入违规详情、进程非零退出码；
       realtime 看板（observable）违规不杀引擎线程：violation 事件入流、
       引擎安全停机（不再派单、车辆制动停车）、快照可见告警状态。

运行方式（无第三方测试依赖）：
    python -m unittest discover -s tests -v      # 在项目根目录执行
    或 python tests/test_regression.py
"""

import os
import sys
import threading
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from dispatch.dispatcher import SimulationEngine, Task
from simulator.agv import AGV


def make_engine(fleet=2):
    """构造一个小规模引擎（压测模式：全速、故障等效复位）。"""
    return SimulationEngine(fleet_size=fleet, lam=0.3, realtime=False,
                            seed=20240601, stress=True)


class InPlacePickupRegression(unittest.TestCase):
    """P1-1 回归：原地接单不得打破"一格一车"。"""

    def test_degenerate_path_normalized(self):
        eng = make_engine(1)
        agv = eng.agvs[0]
        task = Task("outbound", agv.pos, eng.map.station_pos(0), 0.0)
        eng.dispatcher.tasks.append(task)
        eng.dispatcher.assign_pending(eng.sim_time)
        # 原地接单：无伪移动路径，直接进入取货态
        self.assertTrue(task.assigned_at is not None)
        self.assertEqual(agv.state, AGV.LOADING)
        self.assertEqual(agv.path, [])
        # 脚下格占用记录完整（ghost cell 破口的直接回归点）
        self.assertEqual(eng.traffic.cell_owner.get(agv.pos), agv.id)

    def test_invariant_holds_through_full_task(self):
        eng = make_engine(1)
        agv = eng.agvs[0]
        task = Task("outbound", agv.pos, eng.map.station_pos(0), 0.0)
        eng.dispatcher.tasks.append(task)
        eng.dispatcher.assign_pending(eng.sim_time)
        # 推进 100 拍（20 仿真秒）：取货 -> 送货途中，逐拍校验占格归属
        for _ in range(100):
            eng.tick(config.DT)
            self.assertEqual(eng.traffic.cell_owner.get(agv.pos), agv.id,
                             f"t={eng.sim_time}: AGV{agv.id} 脚下格 {agv.pos} 无主/错主")
        # 引擎 tick 内置不变式断言全程未抛错，即"一格一车"成立

    def test_assign_guard_fails_fast_on_unowned_cell(self):
        eng = make_engine(1)
        agv = eng.agvs[0]
        del eng.traffic.cell_owner[agv.pos]        # 人为制造 ghost 状态
        task = Task("outbound", eng.map.station_pos(0), agv.pos, 0.0)
        eng.dispatcher.tasks.append(task)
        with self.assertRaises(AssertionError):
            eng.dispatcher.assign_pending(eng.sim_time)


class CommitArrivalGuard(unittest.TestCase):
    """P1-1 防御层：old==new 不得删除脚下格占用。"""

    def test_same_cell_commit_keeps_owner(self):
        eng = make_engine(1)
        agv = eng.agvs[0]
        cell = agv.pos
        eng.traffic.commit_arrival(agv, cell, cell)
        self.assertEqual(eng.traffic.cell_owner.get(cell), agv.id)

    def test_normal_move_transfers_owner(self):
        eng = make_engine(1)
        agv = eng.agvs[0]
        cell = agv.pos
        target = eng.map.neighbors(*cell)[0]
        eng.traffic.commit_arrival(agv, cell, target)
        self.assertEqual(eng.traffic.cell_owner.get(target), agv.id)
        self.assertIsNone(eng.traffic.cell_owner.get(cell))


class InvariantChecker(unittest.TestCase):
    """P2-5：违规必须被捕获，计数必须累加。"""

    def test_detects_two_agvs_on_same_cell(self):
        eng = make_engine(2)
        eng.agvs[1].pos = eng.agvs[0].pos           # 注入"两车同格"
        before = eng.traffic.invariant_violations
        with self.assertRaises(AssertionError):
            eng.traffic.check_invariants(eng.agvs)
        self.assertEqual(eng.traffic.invariant_violations, before + 1)

    def test_detects_unowned_cell(self):
        eng = make_engine(1)
        del eng.traffic.cell_owner[eng.agvs[0].pos]  # 注入"占格无主"
        with self.assertRaises(AssertionError):
            eng.traffic.check_invariants(eng.agvs)

    def test_clean_engine_passes(self):
        eng = make_engine(3)
        for _ in range(30):
            eng.tick(config.DT)                     # 内置断言未抛错即通过
        self.assertEqual(eng.traffic.invariant_violations, 0)


class DeadlockCountDedup(unittest.TestCase):
    """P2-3：同一物理死锁环的计数去重。"""

    def _make_deadlock(self):
        """构造 2 车对头互等：AGV1 的下一格是 AGV2 的脚下格，反之亦然。

        让路者的 goal_cell 置为其脚下格（无路可去），使让路重规划必失败，
        从而覆盖"让路无解原地等待"分支（环持续存在）。
        """
        eng = make_engine(2)
        t = eng.traffic
        a1, a2 = eng.agvs
        c1, c2 = a1.pos, a2.pos
        # 手工摆位：把两车挪到相邻格（占用表同步登记）
        for c in (c1, c2):
            t.cell_owner.pop(c, None)
        c2 = eng.map.neighbors(*c1)[0]
        a1.pos, a2.pos = c1, c2
        a1.goal_cell, a2.goal_cell = c1, c2         # 目标=脚下格 => 重规划必无解
        t.cell_owner[c1], t.cell_owner[c2] = a1.id, a2.id
        a1.path, a2.path = [c2], [c1]               # 互等：1→2 的脚下格，2→1 的脚下格
        return eng

    def test_persistent_cycle_counted_once(self):
        eng = self._make_deadlock()
        t = eng.traffic
        clock = [0.0]
        t.clock = lambda: clock[0]
        for _ in range(6):                          # 6 个远隔冷却期的检测周期
            clock[0] += config.DEADLOCK_COOLDOWN_SECONDS * 10
            t.detect_and_resolve(eng.agvs)
        self.assertEqual(t.deadlock_detected, 1)    # 同一物理死锁只计 1 次
        self.assertEqual(t.arbitration_total, 6)    # 仲裁每次冷却到期照常触发
        self.assertEqual(t.unresolved_waits, 1)     # 让路无解每环至多 1 次
        self.assertEqual(t.deadlock_resolved, 0)    # 环仍在，不计"消除"

    def test_cycle_disappearance_counts_resolved(self):
        eng = self._make_deadlock()
        t = eng.traffic
        clock = [0.0]
        t.clock = lambda: clock[0]
        clock[0] += 10.0
        t.detect_and_resolve(eng.agvs)
        self.assertEqual(t.deadlock_detected, 1)
        eng.agvs[1].path = []                       # 阻挡者腾位：环消失
        clock[0] += 10.0
        t.detect_and_resolve(eng.agvs)              # 清扫活跃环集合
        self.assertEqual(t.deadlock_resolved, 1)


class DodgeGoalBlockedSpin(unittest.TestCase):
    """仲裁自旋修复 回归：让路者 goal 恰为被堵格时的仲裁自旋。

    场景：对头互等且互为目标——AGV1 的 goal=AGV2 脚下格，反之亦然。
    旧缺陷下，让路重规划因 A* 终点豁免恒返回同一条 1 步路径并被判"成功"，
    装载后下一拍仍被拒，环永不消、仲裁按 3s 冷却期无限自旋
    （seed=5 实测：仲裁 2176 次 / 物理死锁仅 4 起 / unresolved=0，吞吐塌陷至
    0.16 任务/min）。修复后：仲裁装载的路径必须"首格当前可通行"（有效性
    判据），目标被堵时升级为"先侧避再回原目标"的改道；无侧避格才计入
    unresolved。不允许无限自旋。
    """

    def _make_head_on(self):
        """构造对头互等且互为目标的 2 车死锁（goal 恰为被堵格）。"""
        eng = make_engine(2)
        t = eng.traffic
        a1, a2 = eng.agvs
        c1 = a1.pos
        c2 = eng.map.neighbors(*c1)[0]
        for c in (a1.pos, a2.pos):
            t.cell_owner.pop(c, None)
        a1.pos, a2.pos = c1, c2
        # 挂真实任务对象（TO_DELIVER）：破环后车辆到点能安全走完卸货结算，
        # 引擎级 tick 测试不会因缺任务对象而报错
        a1.task = Task("transfer", c2, eng.map.slots[0], 0.0)
        a2.task = Task("transfer", c1, eng.map.slots[-1], 0.0)
        a1.state = a2.state = AGV.TO_DELIVER
        a1.goal_cell, a2.goal_cell = c2, c1     # 互为目标格（恰为对方脚下格）
        t.cell_owner[c1], t.cell_owner[c2] = a1.id, a2.id
        a1.path, a2.path = [c2], [c1]           # 互等：下一步都是对方的脚下格
        return eng

    def test_arbitration_loads_effective_path_or_counts_unresolved(self):
        """单次仲裁：装载的路径首格必须当前可通行，或如实计 unresolved。"""
        eng = self._make_head_on()
        t = eng.traffic
        clock = [0.0]
        t.clock = lambda: clock[0]
        before = {a.id: list(a.path) for a in eng.agvs}
        clock[0] += 10.0
        t.detect_and_resolve(eng.agvs)
        changed = [a for a in eng.agvs if a.path != before[a.id]]
        if t.unresolved_waits >= 1:
            # 判定不可解环时不得再装载任何"假成功"路径
            self.assertEqual(changed, [])
            return
        self.assertEqual(len(changed), 1, "仲裁应恰好为让路者装载一条改道路径")
        a = changed[0]
        self.assertGreaterEqual(len(a.path), 1)
        first = a.path[0]
        owner = t.cell_owner.get(first)
        booker = t.reservations.get(first)
        self.assertTrue(owner is None or owner == a.id,
                        f"装载路径首格 {first} 仍被 AGV{owner} 占用"
                        "——假成功重规划（自旋根源）未根治")
        self.assertTrue(booker is None or booker == a.id,
                        f"装载路径首格 {first} 仍被 AGV{booker} 预约"
                        "——假成功重规划（自旋根源）未根治")

    def test_head_on_trap_breaks_in_finite_time_or_counted(self):
        """引擎级：30 仿真秒内必须破环（有位移）或计入 unresolved，
        不允许无限自旋（旧缺陷：同窗口零位移、零上报）。"""
        eng = self._make_head_on()
        t = eng.traffic
        start = {a.id: a.pos for a in eng.agvs}
        for _ in range(150):                    # 150 拍 = 30 仿真秒
            eng.tick(config.DT)
            if t.deadlock_resolved >= 1:
                break
        self.assertTrue(t.deadlock_resolved >= 1 or t.unresolved_waits >= 1,
                        "30s 内既未破环也未计入 unresolved——仲裁仍在自旋")
        # 有界性：同一环受 3s 冷却期门控，30s 内仲裁次数必须有限
        self.assertLessEqual(t.arbitration_total, 25)
        if t.unresolved_waits == 0:
            self.assertTrue(any(a.pos != start[a.id] for a in eng.agvs),
                            "判定破环成功却全程零位移，不合常理")
        # 安全底线：整个过程中"一格一车"不变式始终成立
        self.assertEqual(eng.traffic.invariant_violations, 0)


class InvariantViolationObservability(unittest.TestCase):
    """可观测化改造 回归：不变式违规的可观测化处置。

    旧行为的两处问题：① realtime 模式违规抛 AssertionError 直接杀死引擎
    线程——看板静默冻结（无日志无提示）；② 压测报告 inv>0 分支因违规即
    中止而不可达（死代码）。修复后两模式口径：
        strict（压测/批处理，auto 默认）：违规即抛（fail-fast 强度不变），
        场景中止、报告如实写入违规详情、进程非零退出码；
        observable（realtime 看板，auto 默认）：违规不杀线程——violation
        事件入流、详情入 metrics/快照、引擎安全停机（不再派单、车辆制动
        停车）、看板显示 INVARIANT VIOLATION 横幅。
    """

    def _strict_engine(self, fleet=2):
        # stress=True 且非 realtime => auto 解析为 strict（压测口径）
        eng = SimulationEngine(fleet_size=fleet, lam=1e-9, realtime=False,
                               seed=20240601, stress=True)
        self.assertEqual(eng.invariant_mode, "strict")
        return eng

    def _observable_engine(self, fleet=2):
        # realtime=True 且非压测 => auto 解析为 observable（看板口径）
        eng = SimulationEngine(fleet_size=fleet, lam=1e-9, realtime=True,
                               seed=20240601, stress=False)
        self.assertEqual(eng.invariant_mode, "observable")
        return eng

    @staticmethod
    def _inject_collision(eng):
        """注入"两车同格"违规：把 AGV2 传送到 AGV1 脚下格。"""
        eng.agvs[1].pos = eng.agvs[0].pos

    # ---------------- strict：压测/批处理口径 ----------------
    def test_strict_mode_fail_fast_on_violation(self):
        """压测引擎 tick：违规即抛 AssertionError（fail-fast 保持），详情登记。"""
        eng = self._strict_engine()
        self._inject_collision(eng)
        before = eng.traffic.invariant_violations
        with self.assertRaises(AssertionError):
            eng.tick(config.DT)
        self.assertEqual(eng.traffic.invariant_violations, before + 1)
        d = eng.traffic.violation_details[0]
        self.assertEqual(d["kind"], "两车同格")
        self.assertEqual(sorted(d["agvs"]), [1, 2])
        self.assertEqual(d["cell"], list(eng.agvs[0].pos))
        self.assertTrue(d["message"].startswith("不变式违规[两车同格]"))

    def test_stress_scenario_reports_violation_and_nonzero_exit(self):
        """压测链路：run_scenario 捕获违规 → 报告行带详情（inv>0 分支接通）
        → build_report 含违规事实 → finalize 返回非零退出码且报告落盘。"""
        import run_stress
        eng = self._strict_engine()
        self._inject_collision(eng)
        row = run_stress.run_scenario(2, 5, 0.6, 20240601, engine=eng)
        self.assertTrue(row["aborted_on_invariant"])
        self.assertIn("两车同格", row["invariant_detail"])
        self.assertGreaterEqual(row["invariant_violations"], 1)
        report = run_stress.build_report([row], 0.6, 5, 20240601)
        self.assertIn("实测违规", report)
        self.assertIn("两车同格", report)
        self.assertIn("非零退出码", report)
        with tempfile.TemporaryDirectory() as td:   # 不触碰 docs/压测报告.md
            path = os.path.join(td, "report.md")
            code = run_stress.finalize([row], 0.6, 5, 20240601, report_path=path)
            self.assertEqual(code, 1)
            with open(path, encoding="utf-8") as fp:
                self.assertIn("两车同格", fp.read())
            # 正常场景（无违规中止标记）退出码必须仍为 0
            clean = dict(row)
            clean.pop("aborted_on_invariant", None)
            clean.pop("invariant_detail", None)
            self.assertEqual(
                run_stress.finalize([clean], 0.6, 5, 20240601,
                                    report_path=os.path.join(td, "ok.md")),
                0)

    # ---------------- observable：realtime 看板口径 ----------------
    def test_observable_mode_safe_stop_visible_and_audible(self):
        """看板引擎 tick：违规不抛不杀线程——violation 事件入流、安全停机、
        快照可见、不再派发新任务、车辆制动停车、无事件刷屏。"""
        eng = self._observable_engine()
        self._inject_collision(eng)
        eng.tick(config.DT)                     # 不得抛异常（线程不被杀死）
        self.assertTrue(eng.halted)
        self.assertEqual(eng.halt_reason, "INVARIANT_VIOLATION")
        kinds = [e["kind"] for e in eng.events.tail(100)]
        self.assertIn("violation", kinds)       # 事件流有 violation 事件（无静默）
        self.assertGreaterEqual(eng.traffic.invariant_violations, 1)
        self.assertTrue(eng.metrics()["invariant_violation_details"])
        # 快照可见告警状态（看板横幅数据源）
        snap = eng.snapshot()
        self.assertTrue(snap["halted"])
        self.assertEqual(snap["halt_reason"], "INVARIANT_VIOLATION")
        self.assertTrue(snap["invariant_details"])
        # 安全停机：继续推进不抛错，车辆位置/路径冻结、不再派发新任务
        task = eng.dispatcher.create_task("outbound", eng.sim_time)
        frozen = {a.id: a.pos for a in eng.agvs}
        for _ in range(25):                     # 5 仿真秒
            eng.tick(config.DT)
        self.assertTrue(all(a.pos == frozen[a.id] and a.path == []
                            and a.next_cell is None for a in eng.agvs))
        self.assertIsNone(task.assigned_at)     # 停机后不派新单
        # 无事件刷屏：停机态不再重复计数/记录
        count = eng.traffic.invariant_violations
        for _ in range(10):
            eng.tick(config.DT)
        self.assertEqual(eng.traffic.invariant_violations, count)

    def test_realtime_run_survives_unexpected_exception(self):
        """realtime 主循环兜底（N3 同类"无提示停机"的根治）：未预期异常
        不杀引擎线程——登记 fault 事件并进入安全停机，看板保持可见。"""
        eng = self._observable_engine()
        calls = {"n": 0}
        orig_tick = eng.tick

        def flaky(dt):
            calls["n"] += 1
            if calls["n"] == 1:
                raise RuntimeError("注入的引擎内部异常")
            orig_tick(dt)

        eng.tick = flaky
        t = threading.Thread(target=eng.run, daemon=True, name="SimEngineThread")
        t.start()
        t.join(5.0)
        self.assertTrue(t.is_alive(), "引擎线程被未预期异常杀死（静默冻结回归）")
        self.assertTrue(eng.halted)
        self.assertEqual(eng.halt_reason, "ENGINE_ERROR")
        self.assertTrue(any(e["kind"] == "fault" for e in eng.events.tail(50)))
        eng.stop()
        t.join(5.0)

    # ---------------- 模式解析：auto / 显式 / 非法 ----------------
    def test_mode_resolution_auto_explicit_invalid(self):
        """auto 按压测/realtime 判定（默认语义保持）；显式配置可覆盖；
        非法配置构造引擎时即报错（fail-fast）。"""
        self.assertEqual(make_engine(2).invariant_mode, "strict")   # 压测=>strict
        self.assertEqual(self._observable_engine().invariant_mode,
                         "observable")                              # 看板=>observable
        old = config.INVARIANT_VIOLATION_MODE
        try:
            config.INVARIANT_VIOLATION_MODE = "observable"          # 显式覆盖压测引擎
            eng = SimulationEngine(fleet_size=2, lam=1e-9, realtime=False,
                                   seed=20240601, stress=True)
            self.assertEqual(eng.invariant_mode, "observable")
            self._inject_collision(eng)
            eng.tick(config.DT)                     # 显式 observable 不抛错
            self.assertTrue(eng.halted)
            config.INVARIANT_VIOLATION_MODE = "bogus"
            with self.assertRaises(ValueError):
                SimulationEngine(fleet_size=1, lam=1e-9, realtime=False,
                                 stress=True)
        finally:
            config.INVARIANT_VIOLATION_MODE = old


if __name__ == "__main__":
    unittest.main(verbosity=2)
