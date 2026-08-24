# -*- coding: utf-8 -*-
"""
agv.py —— AGV 个体模型
========================

职责（对应规格 2）：
    1. 维护单台 AGV 的位置 / 朝向 / 速度 / 电量 / 状态机 / 当前任务与路径；
    2. 实现五段式状态机：空闲 → 接单(去取货) → 取货 → 送货 → (回充)，另含充电中、故障态；
    3. 低电(<20%)自动去最近充电站，充满回到空闲；
    4. 随机小概率故障(1%，按"每完成一次搬运任务"判定)进入故障态，等待人工复位
       （看板上点"故障复位"按钮；压测模式按 config.FAULT_AUTO_RESET_SECONDS 等效自动复位）。

移动模型（离散仿真）：
    AGV 一次只朝"下一个格子"移动，move_progress ∈ [0,1] 表示在该格内的行进进度；
    进入下一格之前必须先向交通管制器(traffic)预约成功（一格一车，防对撞）。
    对外暴露的显示坐标 = 当前格 + 进度×方向，供 Canvas 做平滑插值动画。

注意：本类不决定"去哪"（那是调度层 dispatch 的职责），只负责"怎么走、走没到、
      电量够不够、坏没坏"——即车辆本体层。
"""

import config


class AGV:
    """一台 AGV。所有状态字段均为仿真权威数据，看板只读快照。"""

    # 状态常量（字符串便于 JSON 直传前端）
    IDLE = "idle"                # 空闲
    TO_PICKUP = "to_pickup"      # 接单：驶向取货点
    LOADING = "loading"          # 取货中（停留计时）
    TO_DELIVER = "to_deliver"    # 送货：驶向卸货点
    UNLOADING = "unloading"      # 放货中（停留计时）
    TO_CHARGE = "to_charge"      # 回充：驶向充电站
    CHARGING = "charging"        # 充电中
    FAULT = "fault"              # 故障，待人工复位

    # 状态中文名（看板详情/日志用）
    STATE_NAMES = {
        IDLE: "空闲", TO_PICKUP: "接单-去取货", LOADING: "取货中",
        TO_DELIVER: "送货中", UNLOADING: "放货中",
        TO_CHARGE: "低电回充", CHARGING: "充电中", FAULT: "故障待复位",
    }

    # "正在移动"的状态集合：这些状态下才走 step_move 逻辑
    MOVING_STATES = {TO_PICKUP, TO_DELIVER, TO_CHARGE}
    # "停留计时"的状态集合
    DWELL_STATES = {LOADING, UNLOADING, CHARGING}

    def __init__(self, agv_id, cell, rng):
        """
        :param agv_id: 车辆编号，从 1 开始
        :param cell:   初始格 (x, y)
        :param rng:    共享随机数发生器（numpy RandomState，保证可复现）
        """
        self.id = agv_id
        self.pos = tuple(cell)          # 当前所在格 (x, y)（整数格）
        self.goal_cell = tuple(cell)    # 当前行程的终点格（死锁让路重规划时需要）
        self.facing = (1, 0)            # 朝向（最后一步的移动向量）
        self.battery = config.BATTERY_INIT
        self.state = self.IDLE
        self.task = None                # 当前任务对象（dispatch.task.Task）
        self.path = []                  # 待走格子序列（不含当前格）
        self.next_cell = None           # 已预约、正在驶入的下一格
        self.move_progress = 0.0        # 驶入 next_cell 的进度 0~1
        self.dwell_left = 0.0           # 停留状态剩余秒数（取货/放货/充电）
        self.waiting_seconds = 0.0      # 本次被路权阻塞已等待的秒数（死锁判定输入）
        self.consecutive_reroutes = 0   # 连续重规划次数（活锁保护计数）
        self.charge_target = None       # 回充目标充电站格
        self.rng = rng

        # ---- 统计字段（压测指标来源）----
        self.cells_empty = 0            # 空载走过格数
        self.cells_loaded = 0           # 重载走过格数
        self.tasks_done = 0             # 完成任务数
        self.fault_count = 0            # 累计故障次数
        # 注：重规划次数统一由 traffic.reroute_total 全网计数（每事件恰好一次），
        # 车辆侧不再重复维护，避免同一事件被统计两次
        self.recharge_count = 0         # 完成回充次数

    # ------------------------------------------------------------------
    # 基础属性
    # ------------------------------------------------------------------
    @property
    def is_loaded(self):
        """是否重载（已取货未放货）。"""
        return self.state in (self.TO_DELIVER, self.UNLOADING)

    @property
    def speed(self):
        """当前速度：重载略慢（config 可调）。"""
        return config.AGV_SPEED_LOADED if self.is_loaded else config.AGV_SPEED

    @property
    def display_pos(self):
        """显示坐标（浮点）：当前格 + 进度×方向，供 Canvas 平滑动画。"""
        if self.next_cell is None:
            return (float(self.pos[0]), float(self.pos[1]))
        dx = self.next_cell[0] - self.pos[0]
        dy = self.next_cell[1] - self.pos[1]
        return (self.pos[0] + dx * self.move_progress,
                self.pos[1] + dy * self.move_progress)

    @property
    def is_busy(self):
        """调度器视角：是否可接新单（空闲且非故障）。"""
        return self.state == self.IDLE

    # ------------------------------------------------------------------
    # 状态机主步进（由仿真引擎每拍调用）
    # ------------------------------------------------------------------
    def step(self, dt, ctx):
        """
        推进一个仿真节拍。

        :param dt:  本拍时长（秒）
        :param ctx: 仿真引擎上下文（鸭子类型），需要提供：
            - ctx.map            栅格地图
            - ctx.traffic        交通管制器（路权预约/释放）
            - ctx.events         事件记录器
            - ctx.plan_path(agv, target, avoid_blocked=True)  委托调度层规划路径
            - ctx.on_pickup_arrived(agv)   取货点到达回调（进入 LOADING 并安排后续）
            - ctx.on_dropoff_arrived(agv)  卸货点到达回调（结算任务/触发故障判定）
            - ctx.on_recharge_done(agv)    充满回调
        """
        if self.state == self.FAULT:
            return  # 故障车完全停摆，等待人工复位

        if self.state in self.DWELL_STATES:
            self._step_dwell(dt, ctx)
            return

        if self.state in self.MOVING_STATES:
            self._step_move(dt, ctx)
            return

        if self.state == self.IDLE:
            if self.path:
                # 空闲但仍有路径（例如充满电后让出充电位的挪车）：继续走完
                self._step_move(dt, ctx)
                return
            # 空闲车低电也回充（例如故障复位后电量已低于阈值）
            self._maybe_go_charge(ctx)
            return

    # ------------------------------------------------------------------
    # 移动逻辑：预约 -> 前进 -> 进格结算
    # ------------------------------------------------------------------
    def _step_move(self, dt, ctx):
        # --- 情形 A：路径走完了 => 到达目标点 ---
        if not self.path and self.next_cell is None:
            self._on_target_reached(ctx)
            return

        # --- 情形 B：还没预约下一格 => 向交通管制器申请路权 ---
        if self.next_cell is None:
            target = self.path[0]
            granted = ctx.traffic.request_cell(self, target, soft=False)
            if granted:
                self.next_cell = target
                self.move_progress = 0.0
                self.waiting_seconds = 0.0
                self.consecutive_reroutes = 0   # 拿到路权 => 重规划计数复位
                # 纵深软预约：再尝试锁定前方 RESERVATION_DEPTH-1 格，
                # 提前向全网暴露意图（失败无所谓，进格前还有硬审查兜底）
                ctx.traffic.prune_reservations(
                    self, keep=set(self.path[:config.RESERVATION_DEPTH]))
                for ahead in self.path[1:config.RESERVATION_DEPTH]:
                    if not ctx.traffic.request_cell(self, ahead, soft=True):
                        break
            else:
                # 被堵：累计等待时间（交通管制器据此做死锁环检测与让路仲裁）
                self.waiting_seconds += dt
                return

        # --- 情形 C：正在驶向已预约的格子 ---
        self.move_progress += self.speed * dt
        if self.move_progress >= 1.0:
            self._commit_cell(ctx)

    def _commit_cell(self, ctx):
        """真正进入 next_cell：向交通管制器登记占用，结算电量与统计。"""
        old = self.pos
        self.pos = self.next_cell
        self.next_cell = None
        self.move_progress = 0.0
        self.facing = (self.pos[0] - old[0], self.pos[1] - old[1])
        # 通知交通管制器：旧格释放、新格占用（内部维护 cell -> agv 归属表）
        ctx.traffic.commit_arrival(self, old, self.pos)
        # 电量：每走一格耗电（可配），保底 0 防负数
        self.battery = max(0.0, self.battery - config.BATTERY_DRAIN_PER_CELL)
        # 里程统计：区分空载/重载
        if self.is_loaded:
            self.cells_loaded += 1
        else:
            self.cells_empty += 1
        # 弹出已走完的路径节点
        if self.path and self.path[0] == self.pos:
            self.path.pop(0)

    # ------------------------------------------------------------------
    # 到达目标点后的状态迁移
    # ------------------------------------------------------------------
    def _on_target_reached(self, ctx):
        if self.state == self.TO_PICKUP:
            ctx.on_pickup_arrived(self)      # -> LOADING（计时）
        elif self.state == self.TO_DELIVER:
            ctx.on_dropoff_arrived(self)     # -> UNLOADING（计时+任务结算安排）
        elif self.state == self.TO_CHARGE:
            self.state = self.CHARGING       # 插枪充电
            self.dwell_left = 999999.0       # 充电时长由电量决定，不用计时器控制
            ctx.events.add("charge", f"AGV{self.id} 到达充电站开始充电"
                                     f"（电量 {self.battery:.0f}%）")

    def _step_dwell(self, dt, ctx):
        """停留类状态计时：取货/放货结束后由引擎回调切换状态；充电按电量充满。"""
        if self.state == self.CHARGING:
            self.battery = min(100.0, self.battery + config.CHARGE_RATE_PER_SECOND * dt)
            if self.battery >= 100.0:
                self.recharge_count += 1
                ctx.on_recharge_done(self)   # 引擎负责释放充电位、状态回 IDLE
            return
        self.dwell_left -= dt
        if self.dwell_left <= 0:
            if self.state == self.LOADING:
                ctx.on_loading_finished(self)    # -> TO_DELIVER（引擎规划去程路径）
            elif self.state == self.UNLOADING:
                ctx.on_unloading_finished(self)  # 结算任务、判定故障、决定是否回充

    # ------------------------------------------------------------------
    # 电量管理
    # ------------------------------------------------------------------
    def need_charge(self):
        """是否需要回充（低电阈值可配）。"""
        return self.battery < config.LOW_BATTERY_THRESHOLD

    def _maybe_go_charge(self, ctx):
        """空闲状态下低电则申请回充（引擎辅助规划路径并锁定充电位）。"""
        ctx.try_send_to_charge(self)

    # ------------------------------------------------------------------
    # 任务接口（由调度器调用）
    # ------------------------------------------------------------------
    def assign_task(self, task, path_to_pickup):
        """接单：登记任务、切换状态、装载去取货点的路径。"""
        self.task = task
        self.state = self.TO_PICKUP
        self.path = list(path_to_pickup)
        self.next_cell = None
        self.move_progress = 0.0
        self.waiting_seconds = 0.0
        self.consecutive_reroutes = 0

    def _store_path(self, path, caller):
        """
        set_path / send_to 共用的起点契约校验与存储。

        契约：path[0] 必须是"当前格"或"已预约正在驶入的下一格"，
        保证已有路权预约依然有效；存储时去掉起点格，只留待走序列。
        违反契约立即抛错，让调用方问题尽早暴露。
        """
        if not path:
            self.path = []
            return
        first = self.next_cell if self.next_cell is not None else self.pos
        if path[0] == first or path[0] == self.pos:
            self.path = list(path[1:])
        else:
            raise ValueError(
                f"AGV{self.id}.{caller} 起点契约错误：path[0]={path[0]}，"
                f"当前格={self.pos}，预约格={self.next_cell}"
            )

    def set_path(self, path):
        """
        替换剩余路径（调度层规划/重规划的统一入口）。

        若需要连已预约的格子一起放弃（让路场景），必须先由交通管制器
        调用 traffic.release_reservations(agv) 回收预约，再置空 next_cell。
        """
        self._store_path(path, "set_path")

    def send_to(self, state, path, charge_target=None):
        """
        通用"派车去某处"：回充场景使用。

        与 set_path 的起点契约一致。额外约束：调用时不得带着未完成的
        进格预约（next_cell 必须为空）——本方法会清空 next_cell，
        若此时仍有预约会泄漏到交通管制器的预约表里。
        引擎侧已在派单前保证该前置条件（见 try_send_to_charge）。
        """
        if self.next_cell is not None:
            raise ValueError(
                f"AGV{self.id}.send_to 前置条件不满足：仍持有预约格 {self.next_cell}，"
                f"请先由引擎回收预约"
            )
        self.state = state
        self.charge_target = charge_target
        self.next_cell = None
        self.move_progress = 0.0
        self.waiting_seconds = 0.0
        self._store_path(path, "send_to")

    def reset_fault(self):
        """人工复位故障（看板按钮调用）。复位后回空闲，电量若低会自动申请回充。"""
        if self.state == self.FAULT:
            self.state = self.IDLE
            self.path = []
            self.task = None
            self.next_cell = None
            return True
        return False

    # ------------------------------------------------------------------
    # 看板快照
    # ------------------------------------------------------------------
    def to_dict(self):
        """输出给前端 /api/state 的 JSON 快照（只读、精简）。"""
        dx, dy = self.display_pos
        return {
            "id": self.id,
            "x": round(dx, 3),            # 显示坐标（浮点，平滑动画）
            "y": round(dy, 3),
            "cell": list(self.pos),       # 逻辑格（整数）
            "facing": list(self.facing),
            "state": self.state,
            "state_name": self.STATE_NAMES[self.state],
            "battery": round(self.battery, 1),
            "task_id": self.task.id if self.task is not None else None,
            "waiting": round(self.waiting_seconds, 1),
            "path": [list(c) for c in self.path[:60]],  # 截断防止大 JSON
        }
