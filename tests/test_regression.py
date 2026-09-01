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
       仲裁触发按冷却期照常累加，环消失才计"消除"。

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
