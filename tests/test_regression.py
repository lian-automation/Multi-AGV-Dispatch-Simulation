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
    5. 复审06 N1 回归：让路者 goal 恰为被堵格（互等方脚下格）时，
       仲裁不得凭 A* 终点豁免的"假成功"路径无限自旋——必须有限时间内
       破环（升级侧避改道）或如实计入 unresolved 上报。

运行方式（无第三方测试依赖）：
    python -m unittest discover -s tests -v      # 在项目根目录执行
    或 python tests/test_regression.py
"""

import os
import sys
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
    """复审06 N1 回归：让路者 goal 恰为被堵格时的仲裁自旋。

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
