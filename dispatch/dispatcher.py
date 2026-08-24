# -*- coding: utf-8 -*-
"""
dispatcher.py —— 任务分配 + A* 寻路 + 仿真引擎主循环
====================================================

对应规格 3，本文件包含四个部分：
    1. Task            —— 搬运任务对象（入库/出库/移库）
    2. EventLog        —— 事件日志环形缓冲（看板"事件日志"栏的数据源）
    3. AssignmentStrategy / NearestFirstStrategy
                       —— 分配策略接口 + 默认实现"最近空闲车优先"
                          （预留拍卖法扩展位，见 AuctionStrategy 注释）
    4. Dispatcher      —— 泊松流任务生成器 + 每拍分配循环
    5. SimulationEngine—— 仿真引擎：串联 地图/车辆/调度/交通管制/Modbus，
                          提供 tick() 离散步进与 run() 主循环、统计快照

线程模型：
    - 引擎在独立线程按 DT 节拍推进（realtime=True 时与真实时间同步）；
    - Flask / Modbus 线程只通过 snapshot() 与队列交互，
      关键数据访问用 self.lock 保护，避免读到大半截的状态。
"""

import threading
import time
from collections import deque

import numpy as np

import config
from map.grid_map import load_map
from simulator.agv import AGV
from traffic.controller import TrafficController


# ======================================================================
# 一、搬运任务
# ======================================================================
class Task:
    """一次搬运任务：从 src 格取货送到 dst 格。id 全局自增。"""

    _next_id = 1                      # 类级自增编号
    TYPE_NAMES = {"inbound": "入库", "outbound": "出库", "transfer": "移库"}

    def __init__(self, ttype, src, dst, born_at, station_index=None):
        self.id = Task._next_id
        Task._next_id += 1
        self.type = ttype             # inbound / outbound / transfer
        self.src = tuple(src)         # 取货点（格子）
        self.dst = tuple(dst)         # 卸货点（格子）
        self.born_at = born_at        # 任务到达时刻（仿真秒）——响应时间起点
        self.station_index = station_index  # 关联站台序号 0~3（Modbus 完成码用）；移库为 None
        self.assigned_at = None       # 被分配时刻
        self.finished_at = None       # 完成时刻
        self.agv_id = None            # 承接车辆

    @property
    def response_time(self):
        """任务响应时间 = 到达 -> 被分配（规格定义的响应时间）。"""
        if self.assigned_at is None:
            return None
        return self.assigned_at - self.born_at

    @property
    def cycle_time(self):
        """任务周期 = 到达 -> 完成（附加指标）。"""
        if self.finished_at is None:
            return None
        return self.finished_at - self.born_at

    def to_dict(self):
        return {
            "id": self.id,
            "type": self.type,
            "type_name": self.TYPE_NAMES[self.type],
            "src": list(self.src),
            "dst": list(self.dst),
            "agv_id": self.agv_id,
            "state": ("完成" if self.finished_at is not None
                      else "运输中" if self.assigned_at is not None else "待分配"),
            "wait": (round(self.response_time, 2)
                     if self.response_time is not None else None),
        }


# ======================================================================
# 二、事件日志
# ======================================================================
class EventLog:
    """
    事件日志（环形缓冲）：记录 分配/让路/死锁/回充/故障/完成 等关键事件。
    kind 取值：task=任务到达, assign=派单, done=完成, reroute=让路重规划,
              deadlock=死锁, charge=回充, fault=故障, info=系统
    """

    def __init__(self, capacity=None):
        capacity = capacity or config.EVENT_LOG_CAPACITY
        self._buf = deque(maxlen=capacity)
        self.clock = lambda: 0.0     # 由引擎注入 sim_time 读取函数

    def add(self, kind, msg):
        self._buf.append({"t": round(self.clock(), 1), "kind": kind, "msg": msg})

    def tail(self, n=100):
        return list(self._buf)[-n:]


# ======================================================================
# 三、分配策略（接口 + 默认实现 + 拍卖法扩展位）
# ======================================================================
class AssignmentStrategy:
    """
    分配策略接口：给定一个待分配任务和空闲车队，选出承接车或返回 None。

    扩展新策略只需继承本类并实现 select()，再把实例赋给
    dispatcher.strategy 即可全局生效——这就是"预留策略接口"。
    """

    name = "base"

    def select(self, task, idle_agvs, engine):
        raise NotImplementedError("策略必须实现 select()")


class NearestFirstStrategy(AssignmentStrategy):
    """
    默认策略：最近空闲车优先。

    "最近"的度量用 A* 实际路程长度而非曼哈顿直线距离——
    因为有货架墙和单向巷道，直线近不代表绕路少；
    A* 不可达的车直接视为无穷远。距离相同取编号小的车（结果稳定可复现）。

    规划避障口径：只把其他车"实际占用的格子"视为硬障碍；
    预约格不参与规划约束——预约的最终裁决在进格那一刻的
    request_cell 路权审查（见 traffic/controller.py），这样既保证
    一格一车的安全性，又不会因别人的预约把自己规划成无路可走。
    """

    name = "nearest_first"

    def select(self, task, idle_agvs, engine):
        best_agv, best_cost = None, float("inf")
        # 任务老化放行：等待超过 ASSIGN_AGE_RELAX 的任务不再受距离上限约束
        aged = (engine.sim_time - task.born_at) >= config.ASSIGN_AGE_RELAX
        for agv in idle_agvs:
            start = agv.next_cell if agv.next_cell is not None else agv.pos
            path = engine.map.find_path(
                start, task.src, blocked=engine.planning_blocked(agv))
            cost = len(path) - 1 if path is not None else float("inf")
            # 拥堵治理：太远的单先不派（防长途穿场制造交叉冲突），老化后放行
            if cost > config.ASSIGN_MAX_DIST and not aged:
                continue
            if cost < best_cost or (cost == best_cost and best_agv is not None
                                    and agv.id < best_agv.id):
                best_agv, best_cost = agv, cost
        return best_agv


class AuctionStrategy(AssignmentStrategy):
    """
    【扩展位】拍卖法（合同网协议）：每台空闲车对任务出价
    （报价 = 预估行驶代价 - 电量折扣 - 负载均衡折扣），价低者中标。

    本项目一期未启用（最近优先已满足演示与压测需求），保留接口以示架构可扩展性；
    若要启用：补全 select() 并在 main.py 中切换 dispatcher.strategy。
    """

    name = "auction"

    def select(self, task, idle_agvs, engine):
        raise NotImplementedError("拍卖法为二期扩展位：请实现出价函数后启用")


# ======================================================================
# 四、调度器：泊松流生成 + 每拍分配
# ======================================================================
class Dispatcher:
    """任务生成器 + 分配循环。持有全部任务的权威列表。"""

    def __init__(self, engine):
        self.engine = engine
        self.tasks = []               # 所有任务（含已完成，压测后用于统计）
        self.strategy = NearestFirstStrategy()   # 可整体替换的策略槽
        self._next_arrival = None     # 下一次任务到达时刻（泊松过程）

    # ------------------------------------------------------------------
    # 任务生成（泊松流：到达间隔服从指数分布）
    # ------------------------------------------------------------------
    def arm_poisson(self, now):
        """设定下一次到达时刻（指数分布，均值 1/λ）。"""
        lam = self.engine.lam
        self._next_arrival = now + self.engine.rng.exponential(1.0 / lam)

    def maybe_spawn(self, now):
        """引擎每拍调用：到达时刻到了就生成任务（可能一拍内连到多个）。"""
        # 任务到达总量封顶（压测模式）：到量后停止注入，专心测量消化能力
        if self.engine.max_tasks is not None \
                and len(self.tasks) >= self.engine.max_tasks:
            return
        while self._next_arrival is not None and now >= self._next_arrival:
            self.spawn_random_task(now)
            if self.engine.max_tasks is not None \
                    and len(self.tasks) >= self.engine.max_tasks:
                return
            self.arm_poisson(now)

    def spawn_random_task(self, now):
        """按 config.TASK_TYPE_WEIGHTS 的权重随机生成一类任务。"""
        types = list(config.TASK_TYPE_WEIGHTS.keys())
        weights = np.array([config.TASK_TYPE_WEIGHTS[t] for t in types], dtype=float)
        weights /= weights.sum()
        ttype = str(types[self.engine.rng.choice(len(types), p=weights)])
        return self.create_task(ttype, now)

    def create_task(self, ttype, now, station_index=None):
        """
        按类型构造任务（station_index 非 None 时表示由 Modbus 站台呼叫触发）。
            入库 inbound ：随机站台 -> 随机货位
            出库 outbound：随机货位 -> 站台（呼叫触发时站台固定）
            移库 transfer：货位A -> 货位B
        """
        m = self.engine.map
        rng = self.engine.rng
        station_index_out = None
        # 统一用纯 Python int 元组表示格子（避免 numpy 类型流入任务/事件/JSON）
        def pick_slot():
            return m.slots[int(rng.integers(0, len(m.slots)))]
        if ttype == "inbound":
            si = int(rng.integers(0, len(m.stations))) if station_index is None else station_index
            src, dst, station_index_out = m.station_pos(si), pick_slot(), si
        elif ttype == "outbound":
            si = int(rng.integers(0, len(m.stations))) if station_index is None else station_index
            src, dst, station_index_out = pick_slot(), m.station_pos(si), si
        elif ttype == "transfer":
            i = int(rng.integers(0, len(m.slots)))
            j = int(rng.integers(0, len(m.slots)))
            while j == i:                       # 保证两个货位不同
                j = int(rng.integers(0, len(m.slots)))
            src, dst = m.slots[i], m.slots[j]
        else:
            raise ValueError(f"未知任务类型 {ttype}")

        task = Task(ttype,
                    (int(src[0]), int(src[1])),
                    (int(dst[0]), int(dst[1])),
                    now, station_index_out)
        self.tasks.append(task)
        self.engine.events.add(
            "task", f"任务#{task.id}({Task.TYPE_NAMES[ttype]}) 到达："
                    f"{src} -> {dst}" + (f"［站台{si+1}呼叫］" if station_index is not None else ""))
        return task

    # ------------------------------------------------------------------
    # 任务分配（每拍执行；FIFO 顺序遍历待分配任务）
    # ------------------------------------------------------------------
    def assign_pending(self, now):
        """把排队中的任务按策略分给空闲车；分配成功立即 A* 规划去程路径。"""
        for task in self.tasks:
            idle = [a for a in self.engine.agvs if a.is_busy]
            if not idle:
                break                                      # 无车可用，下拍再试
            if task.assigned_at is not None:
                continue                                   # 已分配/已完成
            agv = self.strategy.select(task, idle, self.engine)
            if agv is None:
                continue                                   # 该任务暂时无车可达，跳过等下拍
            # A* 规划去取货点的路径：只避其他车实际占用的格子（见 planning_blocked）
            start = agv.next_cell if agv.next_cell is not None else agv.pos
            path = self.engine.map.find_path(start, task.src,
                                             blocked=self.engine.planning_blocked(agv))
            if path is None:
                continue                                   # 规划失败（极端拥堵），留给下拍
            agv.assign_task(task, path)
            agv.goal_cell = task.src
            task.assigned_at = now
            task.agv_id = agv.id
            self.engine.note_response_sample(task.response_time)  # 供 30s 滑动平均
            self.engine.events.add(
                "assign", f"任务#{task.id} 分配给 AGV{agv.id}"
                          f"（响应 {task.response_time:.2f}s，去程 {len(path)-1} 步）")

    def pending_count(self):
        return sum(1 for t in self.tasks if t.assigned_at is None)


# ======================================================================
# 五、仿真引擎
# ======================================================================
class SimulationEngine:
    """
    仿真引擎主循环：地图 + 车队 + 调度器 + 交通管制器 的粘合剂。
    同时充当 AGV.step() 所需的 ctx（提供回调接口）。
    """

    def __init__(self, fleet_size=None, lam=None, realtime=None,
                 seed=None, map_obj=None, stress=False, max_tasks=None):
        # --- 可覆盖配置（压测脚本会传入不同参数复用同一套引擎）---
        self.fleet_size = fleet_size or config.FLEET_SIZE
        self.lam = lam or config.TASK_LAMBDA
        self.realtime = config.REALTIME if realtime is None else realtime
        self.stress = stress            # 压测模式：故障等效自动复位、全速推进
        self.max_tasks = max_tasks      # 任务到达总量封顶（None=不限，压测用）
        seed = config.RANDOM_SEED if seed is None else seed
        self.rng = np.random.default_rng(seed)

        # --- 核心组件 ---
        self.map = map_obj or load_map()
        self.events = EventLog()
        self.events.clock = lambda: self.sim_time
        self.traffic = TrafficController(self.map, self.events)
        self.traffic.clock = lambda: self.sim_time   # 注入仿真时钟（死锁冷却期用）
        self.dispatcher = Dispatcher(self)
        self.modbus = None              # ModbusBridge，由 main.py 注入（可为 None）

        # --- 时间与状态 ---
        self.sim_time = 0.0
        self.paused = False
        self.stopped = False
        self.lock = threading.RLock()   # 保护 tick/snapshot 的互斥锁

        # --- 建立车队：初始停在充电站附近空地，避免开局堵门 ---
        self.agvs = []
        spawn_cells = self._pick_spawn_cells(self.fleet_size)
        for i in range(self.fleet_size):
            self.agvs.append(AGV(i + 1, spawn_cells[i], self.rng))
            self.traffic.cell_owner[spawn_cells[i]] = i + 1   # 登记初始占用

        # --- 统计容器 ---
        self.low_battery_events = 0     # 低电回充触发次数（成功率分母）
        self.queue_peak = 0             # 待分配队列峰值长度（拥堵程度观察）
        self.throughput_series = []     # [(t, 累计完成数)] 每 5s 采样，画吞吐曲线
        self.response_series = []       # [(t, 近30s平均响应)] 每 30s 采样
        self._resp_bucket = []          # 当前 30s 窗口内的响应时间样本
        self._sample_timer_tp = 0.0
        self._sample_timer_resp = 0.0

        self.dispatcher.arm_poisson(0.0)
        self.events.add("info", f"引擎启动：{self.fleet_size} 台 AGV，λ={self.lam}/s，"
                                f"地图 {self.map.cols}x{self.map.rows}")

    # ------------------------------------------------------------------
    # 初始化辅助
    # ------------------------------------------------------------------
    def _pick_spawn_cells(self, n):
        """挑 n 个互不相同、互相不挨着的通行格做初始停车位（优先充电站周边）。"""
        candidates = []
        for _, cx, cy, _ in self.map.charges:
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    c = (cx + dx, cy + dy)
                    if self.map.passable(*c):
                        candidates.append(c)
        candidates += list(self.map.slots)
        picked = []
        for c in candidates:
            if all(abs(c[0]-p[0]) + abs(c[1]-p[1]) >= 2 for p in picked):
                picked.append(c)
            if len(picked) == n:
                break
        while len(picked) < n:          # 兜底：随便再找空格
            for y in range(self.map.rows):
                for x in range(self.map.cols):
                    c = (x, y)
                    if self.map.passable(*c) and c not in picked:
                        picked.append(c)
        return picked[:n]

    # ------------------------------------------------------------------
    # 规划辅助
    # ------------------------------------------------------------------
    def planning_blocked(self, agv):
        """
        规划用避障集合：只含其他车"实际占用"的格子。
        预约格不作为规划障碍（进格时由路权审查最终裁决），保证规划几乎总有解。
        """
        return {c for c, a in self.traffic.cell_owner.items() if a != agv.id}

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def run(self):
        """阻塞式主循环：realtime 模式按墙钟对齐节拍，压测模式全速跑。"""
        start_wall = time.monotonic()
        ticks = 0
        while not self.stopped and self.sim_time < config.MAX_SIM_SECONDS:
            with self.lock:
                self.tick(config.DT)
            if self.realtime:
                target = start_wall + (ticks + 1) * config.DT
                sleep_for = target - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
            ticks += 1

    def stop(self):
        self.stopped = True

    def tick(self, dt):
        """推进一步仿真（顺序见注释——先产生需求，再决策，再执行，再管制）。"""
        if self.paused:
            return
        self.sim_time += dt

        # 1) 任务到达（泊松流） + Modbus 站台呼叫转任务
        self.dispatcher.maybe_spawn(self.sim_time)
        if self.modbus is not None:
            self.modbus.poll_calls(self)          # 从站寄存器 -> 出库任务

        # 2) 任务分配（最近空闲车优先 + A* 去程规划）
        self.dispatcher.assign_pending(self.sim_time)

        # 3) 车辆个体步进（移动/取放货/充电/电量）
        for agv in self.agvs:
            agv.step(dt, self)

        # 4) 交通管制：死锁检测 + 让路仲裁
        self.traffic.tick(dt, self.agvs)

        # 5) 行驶受阻超时的单車重规划（路径被占自动重规划，规格 3 的要求）
        self._reroute_stuck_agvs()

        # 6) 压测模式下故障车等效自动复位
        if self.stress:
            for agv in self.agvs:
                reset_at = getattr(agv, "fault_reset_at", None)
                if agv.state == AGV.FAULT and reset_at is not None \
                        and self.sim_time >= reset_at:
                    agv.reset_fault()
                    agv.fault_reset_at = None
                    self.events.add("fault", f"AGV{agv.id} 已复位（压测等效人工处理），恢复作业")

        # 7) 指标采样（吞吐曲线 / 平均响应曲线）
        self._sample_metrics(dt)

    # ------------------------------------------------------------------
    # 受阻重规划（非死锁的一般性拥堵绕行）
    # ------------------------------------------------------------------
    def _reroute_stuck_agvs(self):
        for agv in self.agvs:
            if agv.state not in AGV.MOVING_STATES or agv.waiting_seconds \
                    < config.REROUTE_WAIT_LIMIT:
                continue
            # ---- 特例：阻挡者是"不参与等图"的静止车（空闲停靠/故障）----
            # 空闲车没有待执行路径 => 等待图里没有它的出边 => 永远凑不出环，
            # 死锁仲裁对它无能为力；必须由调度引擎主动"请求让位"。
            target = agv.path[0] if agv.path else None
            if target is not None:
                blockers = {self.traffic.cell_owner.get(target),
                            self.traffic.reservations.get(target)}
                for bid in blockers:
                    if bid is None or bid == agv.id:
                        continue
                    other = self.agvs[bid - 1]
                    if other.state == AGV.IDLE and not other.path:
                        self._request_dodge(other)
            if agv.consecutive_reroutes >= config.MAX_CONSECUTIVE_REROUTE:
                continue        # 连续多次都无更优路 => 原地等待，交给死锁仲裁
            # 两档避障：先严格（避占用+预约），失败再宽松（只避占用）
            new_path = self.map.find_path(
                agv.pos, agv.goal_cell, blocked=self.traffic.blocked_for(agv))
            if new_path is None:
                new_path = self.map.find_path(
                    agv.pos, agv.goal_cell, blocked=self.planning_blocked(agv))
            if new_path is not None and len(new_path) - 1 > 0:
                self.traffic.release_reservations(agv)
                agv.next_cell = None
                agv.set_path(new_path)
                agv.consecutive_reroutes += 1
                self.traffic.reroute_total += 1   # 全网唯一计数口径（修复双计）
                self.events.add("reroute",
                                f"AGV{agv.id} 前方被占，自动重规划"
                                f"（改走 {len(new_path)-1} 步）")
            agv.waiting_seconds = 0.0    # 无论成败都清零计时，进入下一轮观察窗

    def _request_dodge(self, parked):
        """
        请求一辆空闲停靠车"让位"：给它规划一条 ≤6 步的就近躲避路径。
        车辆以空闲状态走完该路径（IDLE 也允许走 path），把路腾出来。
        """
        candidates = []
        for radius in range(1, 7):                 # 由近及远找第一个可站之格
            for nx, ny in self.map.neighbors(*parked.pos):
                c = (nx, ny)
                if c in self.traffic.cell_owner or c in self.traffic.reservations:
                    continue
                p = self.map.find_path(parked.pos, c,
                                       blocked=self.planning_blocked(parked))
                if p is not None:
                    candidates.append((len(p), c, p))
            if candidates:
                break
        if not candidates:
            return                                 # 无处可让：只能等死锁仲裁
        _, _, path = min(candidates, key=lambda x: x[0])
        parked.set_path(path)
        self.events.add("reroute",
                        f"AGV{parked.id} 收到让位指令，挪车 {len(path)-1} 步"
                        f"为后车让行")

    # ------------------------------------------------------------------
    # AGV 回调接口（AGV.step 的 ctx 约定，见 agv.py）
    # ------------------------------------------------------------------
    def on_pickup_arrived(self, agv):
        agv.state = AGV.LOADING
        agv.dwell_left = config.LOAD_DWELL_SECONDS
        self.events.add("assign", f"AGV{agv.id} 到达取货点{agv.pos}，取货中…")

    def on_loading_finished(self, agv):
        task = agv.task
        start = agv.next_cell if agv.next_cell is not None else agv.pos
        # 送货规划：只避其他车实际占用的格子（预约冲突由进格路权审查兜底）
        path = self.map.find_path(start, task.dst,
                                  blocked=self.planning_blocked(agv))
        if path is None:
            # 物理上被彻底封死（极罕见）：0.2s 后重试，保持取货完成状态等待
            agv.dwell_left = 0.2
            return
        agv.state = AGV.TO_DELIVER
        agv.goal_cell = task.dst
        agv.set_path(path)
        self.events.add("assign", f"AGV{agv.id} 取货完成，送货 -> {task.dst}"
                                  f"（{len(path)-1} 步）")

    def on_dropoff_arrived(self, agv):
        agv.state = AGV.UNLOADING
        agv.dwell_left = config.UNLOAD_DWELL_SECONDS

    def on_unloading_finished(self, agv):
        """任务结算：完成统计 + 故障判定 + 回充决策。"""
        task = agv.task
        task.finished_at = self.sim_time
        agv.tasks_done += 1
        agv.task = None
        self.events.add("done", f"任务#{task.id} 完成！AGV{agv.id} 卸货于{agv.pos}"
                                f"（全程 {task.cycle_time:.1f}s）")

        # Modbus 完成码：送达站台则点亮对应 HR10~13
        if task.station_index is not None and self.modbus is not None:
            self.modbus.set_completion(task.station_index)

        # 故障判定：每完成一次搬运 1% 概率故障（规格值，可在 config 调整）
        if self.rng.random() < config.FAULT_PROB_PER_TASK:
            agv.state = AGV.FAULT
            agv.path = []
            agv.fault_count += 1
            if self.stress:
                agv.fault_reset_at = self.sim_time + config.FAULT_AUTO_RESET_SECONDS
            self.events.add("fault", f"⚠ AGV{agv.id} 发生故障，等待人工复位！"
                                     f"（看板点击该车->故障复位）")
            return

        agv.state = AGV.IDLE
        # 低电检查：送货完成后若低于阈值立即回充
        if agv.need_charge():
            self.try_send_to_charge(agv)

    def on_recharge_done(self, agv):
        """充满：回到空闲，并向旁边空格挪一步把充电桩让出来。"""
        agv.state = AGV.IDLE
        agv.charge_target = None
        self.events.add("charge", f"AGV{agv.id} 电量充满，返回待命（第 "
                                  f"{agv.recharge_count} 次满充）")
        vacate = self._free_neighbor_cell(agv.pos)
        if vacate is not None:
            path = self.map.find_path(agv.pos, vacate)
            if path is not None:
                agv.set_path(path)      # IDLE 状态也允许走完这段挪车路径

    def try_send_to_charge(self, agv):
        """低电回充：选最近的"空闲充电桩"，规划路线并出发。"""
        if agv.state != AGV.IDLE or not agv.need_charge():
            return
        if agv.next_cell is not None:
            return   # 正在挪车让位等动作中：等下一步走完再评估（send_to 前置条件）
        occupied = {c for c, a in self.traffic.cell_owner.items() if a != agv.id}
        free_chargers = [(x, y) for _, x, y, _ in self.map.charges
                         if (x, y) not in occupied]
        if not free_chargers:
            return                       # 桩都被占，留在原地下一拍再试
        best, best_len = None, None
        for c in free_chargers:
            p = self.map.find_path(agv.pos, c,
                                   blocked=self.planning_blocked(agv))
            if p is not None and (best_len is None or len(p) < best_len):
                best, best_len = c, len(p)
        if best is None:
            return
        path = self.map.find_path(agv.pos, best,
                                  blocked=self.planning_blocked(agv))
        agv.send_to(AGV.TO_CHARGE, path, charge_target=best)
        agv.goal_cell = best
        self.low_battery_events += 1
        self.events.add("charge", f"🔋 AGV{agv.id} 电量 {agv.battery:.0f}% < "
                                  f"{config.LOW_BATTERY_THRESHOLD}%，自动回充 -> {best}")

    def _free_neighbor_cell(self, cell):
        """找一个既可通行又无人占/预约的相邻格（让出充电位用）。"""
        for nx, ny in self.map.neighbors(*cell):
            c = (nx, ny)
            if c not in self.traffic.cell_owner and c not in self.traffic.reservations:
                return c
        return None

    # ------------------------------------------------------------------
    # 指标采样
    # ------------------------------------------------------------------
    def _sample_metrics(self, dt):
        completed = sum(1 for t in self.dispatcher.tasks
                        if t.finished_at is not None)
        self._sample_timer_tp += dt
        if self._sample_timer_tp >= 5.0:
            self._sample_timer_tp = 0.0
            # 队列峰值：每 5s 采样一次待分配队列长度（拥堵观察指标）
            self.queue_peak = max(self.queue_peak,
                                  self.dispatcher.pending_count())
            self.throughput_series.append((round(self.sim_time, 1), completed))
        # 响应时间滑动平均（30 秒窗口）：样本在分配成功时已写入 _resp_bucket
        self._sample_timer_resp += dt
        if self._sample_timer_resp >= 30.0:
            self._sample_timer_resp = 0.0
            avg = float(np.mean(self._resp_bucket)) if self._resp_bucket else 0.0
            self.response_series.append((round(self.sim_time, 1), round(avg, 3)))
            self._resp_bucket = []

    # 分配成功时记录响应时间样本（供滑动平均）
    def note_response_sample(self, rt):
        self._resp_bucket.append(rt)

    # ------------------------------------------------------------------
    # 对外快照（Flask /api/state 与压测报告共用）
    # ------------------------------------------------------------------
    def metrics(self):
        """终局统计指标（压测报告数据源）。所有数值均为仿真验证值。"""
        tasks = self.dispatcher.tasks
        finished = [t for t in tasks if t.finished_at is not None]
        assigned = [t for t in tasks if t.assigned_at is not None]
        resp = [t.response_time for t in assigned if t.response_time is not None]
        cells_empty = sum(a.cells_empty for a in self.agvs)
        cells_loaded = sum(a.cells_loaded for a in self.agvs)
        total_cells = cells_empty + cells_loaded
        ts = self.traffic.stats()
        recharge_ok = sum(a.recharge_count for a in self.agvs)
        return {
            "sim_seconds": round(self.sim_time, 1),
            "tasks_total": len(tasks),
            "tasks_assigned": len(assigned),
            "tasks_completed": len(finished),
            "queue_peak": self.queue_peak,
            "throughput_per_min": round(len(finished) / (self.sim_time / 60.0), 2)
                                  if self.sim_time > 0 else 0.0,
            "avg_response_s": round(float(np.mean(resp)), 3) if resp else None,
            "max_response_s": round(float(np.max(resp)), 3) if resp else None,
            "avg_cycle_s": (round(float(np.mean([t.cycle_time for t in finished])), 2)
                            if finished else None),
            "empty_rate": round(cells_empty / total_cells, 4) if total_cells else None,
            "cells_empty": cells_empty,
            "cells_loaded": cells_loaded,
            "deadlock_detected": ts["deadlock_detected"],
            "deadlock_resolved": ts["deadlock_resolved"],
            "unresolved_waits": ts["unresolved_waits"],
            "replan_count": ts["reroute_total"],  # 全网唯一口径：拥堵绕行+死锁让路
            "low_battery_events": self.low_battery_events,
            "recharge_success": recharge_ok,
            "recharge_success_rate": round(recharge_ok / self.low_battery_events, 4)
                                     if self.low_battery_events else None,
            "faults": sum(a.fault_count for a in self.agvs),
        }

    def snapshot(self):
        """看板全量快照（线程安全：与 tick 互斥）。"""
        with self.lock:
            recent_tasks = sorted(self.dispatcher.tasks[-12:],
                                  key=lambda t: -t.id)
            return {
                "sim_time": round(self.sim_time, 1),
                "paused": self.paused,
                "map": {
                    "cols": self.map.cols,
                    "rows": self.map.rows,
                    "grid": self.map.grid_types,
                    "stations": [{"id": s[0], "x": s[1], "y": s[2]} for s in self.map.stations],
                    "charges": [{"id": c[0], "x": c[1], "y": c[2]} for c in self.map.charges],
                    "slots": [list(c) for c in self.map.slots],
                },
                "agvs": [a.to_dict() for a in self.agvs],
                "tasks": [t.to_dict() for t in reversed(recent_tasks)],
                "pending": self.dispatcher.pending_count(),
                "metrics": self.metrics(),
                "series": {
                    "throughput": self.throughput_series[-120:],
                    "response": self.response_series[-60:],
                },
                "events": self.events.tail(80),
            }
