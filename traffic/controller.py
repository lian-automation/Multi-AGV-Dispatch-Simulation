# -*- coding: utf-8 -*-
"""
controller.py —— 交通管制器（本项目灵魂模块）
==============================================

对应规格 4，解决两个问题：
    A. 防对撞 —— 目标格预约制路权管理
       - 一格一车：cell_owner[cell] 记录"当前谁占着这个格子"；
       - 进格前预约：AGV 想进入下一格，必须先调用 request_cell() 拿到路权；
         同一时刻一个格子最多有 1 个占用者 + 1 个预约者，且二者不能是不同车，
         从机制上杜绝了两辆车同时驶入同一格（对撞）的可能。
    B. 解死锁 —— 等待环检测 + 让路重规划
       - 谁在等谁：每当车辆的路权申请被拒，就形成一条等待边
         "申请者 -> 阻挡者"，全图构成一张有向"等待图"；
       - 找圈：周期性对等待图做 DFS 找有向环。存在环 <=> 存在一组车互相等待，
         谁也不肯动 => 这就是死锁（对应死锁四必要条件中的"循环等待"）;
       - 解除：选中环内"剩余路径最长的低优先级车"作为让路者，
         回收其全部预约后按"避开其他车占/预约格"重新 A*；
         若重规划无解，则原地等待并计数上报（unresolved_waits）。

并发模型说明：整个引擎运行在单线程仿真循环里，因此本控制器无需加锁；
Modbus/Flask 线程只通过引擎提供的线程安全入口间接读快照。
"""

from collections import defaultdict

import config
from simulator.agv import AGV   # 仅取状态常量；simulator 不反向依赖 traffic，无循环导入


class TrafficController:
    """路权管理 + 死锁检测与解除。"""

    def __init__(self, gmap, events):
        """
        :param gmap:   GridMap 地图对象（重规划时用）
        :param events: 事件日志（duck type：有 .add(kind, msg) 方法）
        """
        self.map = gmap
        self.events = events

        # ---------- 路权核心表 ----------
        self.cell_owner = {}      # cell(x,y) -> agv_id ：当前占用（一格一车的"一"）
        self.reservations = {}    # cell(x,y) -> agv_id ：进格前预约（每格至多一个预约）

        # ---------- 死锁相关 ----------
        self.wait_graph = defaultdict(set)   # agv_id -> set(阻挡它的 agv_id)
        self.deadlock_detected = 0           # 死锁发生次数（每检出一条环计 1 次）
        self.deadlock_resolved = 0           # 通过让路重规划成功解除的次数
        self.unresolved_waits = 0            # 重规划仍无解、只能原地等待上报的次数
        self.reroute_total = 0               # 让路重规划执行总次数

        # 检测节奏与仲裁状态
        self._check_timer = 0.0
        self.clock = lambda: 0.0             # 由引擎注入 sim_time 读取函数
        self._cycle_cooldown = {}            # frozenset(环成员) -> 上次仲裁时刻
        self._victim_cursor = 0              # 让路车轮换游标（防活锁：不总盯同一辆）

    # ==================================================================
    # 一、路权管理（防对撞）
    # ==================================================================
    def request_cell(self, agv, cell, soft=True):
        """
        AGV 申请进入 cell 的路权。

        规则（一格一车 + 进格前预约）：
            允许 <=> cell 既没有被别的车"占用"，也没有被别的车"预约"。

        :param soft: True=纵深软预约（失败不记等待边——车辆并未真正受阻，
                     只是少预约了几格纵深）；False=移动关键申请（失败即受阻，
                     记入等待图参与死锁环检测）
        :return: True=授权成功（登记预约）；False=被拒
        """
        if cell == agv.pos:
            return True  # 自己脚下不需要预约

        owner = self.cell_owner.get(cell)
        booker = self.reservations.get(cell)
        if (owner is None or owner == agv.id) and (booker is None or booker == agv.id):
            self.reservations[cell] = agv.id
            if not soft:
                self.wait_graph.pop(agv.id, None)  # 关键申请成功 => 清空等待记录
            return True

        if not soft:
            # ---- 关键申请被拒：登记"谁在等谁"的边（死锁检测的数据来源）----
            blockers = set()
            if owner is not None and owner != agv.id:
                blockers.add(owner)
            if booker is not None and booker != agv.id:
                blockers.add(booker)
            self.wait_graph[agv.id] |= blockers
        return False

    def prune_reservations(self, agv, keep):
        """清理某车的过期待预约：只保留 keep 集合内的格子（纵深预约滚动更新用）。"""
        for c, aid in list(self.reservations.items()):
            if aid == agv.id and c not in keep and c != agv.next_cell:
                del self.reservations[c]

    def commit_arrival(self, agv, old_cell, new_cell):
        """车辆真正跨入 new_cell：占用转移（新格占用、旧格释放、预约核销）。"""
        self.cell_owner[new_cell] = agv.id
        self.reservations.pop(new_cell, None)          # 预约升级为占用，核销
        if self.cell_owner.get(old_cell) == agv.id:
            del self.cell_owner[old_cell]              # 尾巴离开旧格，释放
        self.wait_graph.pop(agv.id, None)

    def release_reservations(self, agv):
        """回收某车的全部进格预约（不改变它脚下的占用）。用于让路/复位前的清场。"""
        for c, aid in list(self.reservations.items()):
            if aid == agv.id:
                del self.reservations[c]
        self.wait_graph.pop(agv.id, None)

    def blocked_for(self, agv):
        """返回对指定车而言"别人占着/预约着"的格子集合（A* 动态避障输入）。"""
        cells = set(self.cell_owner.keys()) | set(self.reservations.keys())
        mine = {c for c, a in self.cell_owner.items() if a == agv.id}
        mine |= {c for c, a in self.reservations.items() if a == agv.id}
        return cells - mine

    def occupied_summary(self):
        """看板调试用：返回 (占用格数, 预约格数)。"""
        return len(self.cell_owner), len(self.reservations)

    # ==================================================================
    # 二、死锁检测与解除（每个检测周期由引擎调用一次）
    # ==================================================================
    def tick(self, dt, agvs):
        """周期性执行死锁检测。dt 为本拍秒数。"""
        self._check_timer += dt
        if self._check_timer < config.DEADLOCK_CHECK_INTERVAL:
            return
        self._check_timer = 0.0
        self.detect_and_resolve(agvs)

    def _build_wait_graph(self, agvs):
        """
        依据当前路权表重建等待图（比增量维护更可靠：无陈旧边）。
        边定义：a -> b 表示"a 的下一步被 b 挡住"。
        """
        graph = defaultdict(set)
        by_id = {a.id: a for a in agvs}
        for a in agvs:
            if a.state == AGV.FAULT or a.next_cell is not None:
                continue  # 故障车不参与；已在移动中的车不存在"下一步被挡"
            if not a.path:
                continue
            target = a.path[0]
            owner = self.cell_owner.get(target)
            booker = self.reservations.get(target)
            for blocker in (owner, booker):
                if blocker is not None and blocker != a.id:
                    graph[a.id].add(blocker)
        return graph, by_id


    def detect_and_resolve(self, agvs):
        """
        在等待图中找有向环；发现死锁则仲裁让路。

        仲裁策略（对应规格"低优先级车让路重规划，无解则原地等待计数上报"）：
            1. 冷却期：同一个环在 DEADLOCK_COOLDOWN_SECONDS 秒内不重复仲裁
               （防止每拍空转刷计数）；
            2. 让路者优先级：剩余路径最长的车先让（离目标最远、绕行代价最小）；
               若反复失败则按轮换游标换一辆，避免永远盯死同一辆造成活锁；
            3. 渐进松弛重规划：
               第1次 避开其他车的占用格+预约格（严格，不打扰任何人）；
               第2次 只避占用格（允许借道别人"已预约但尚未进入"的格子——
                     真正进格时仍要过 request_cell 路权审查，安全不受影响）；
               都失败才判"无解"，原地等待并计数上报。
        """
        graph, by_id = self._build_wait_graph(agvs)
        cycle = self._find_cycle(graph)
        if cycle is None:
            return  # 无环：系统健康

        # ---- 冷却期检查：同一批成员的环在冷却期内只仲裁一次 ----
        key = frozenset(cycle)
        now = self.clock()
        if now - self._cycle_cooldown.get(key, -99.0) \
                < config.DEADLOCK_COOLDOWN_SECONDS:
            return
        self._cycle_cooldown[key] = now

        self.deadlock_detected += 1
        names = "->".join(f"AGV{i}" for i in sorted(cycle))
        self.events.add("deadlock", f"检测到死锁环：{names}，启动让路仲裁")

        # ---- 选让路者：按"剩余路径最长"优先排序 + 轮换游标防活锁 ----
        ordered = sorted(cycle, key=lambda i: len(by_id[i].path), reverse=True)
        self._victim_cursor += 1
        victim = ordered[(self._victim_cursor - 1) % len(ordered)]
        victim_agv = by_id[victim]
        goal = getattr(victim_agv, "goal_cell", victim_agv.pos)

        # 清场：回收让路者的全部预约，从当前位置重新规划
        self.release_reservations(victim_agv)
        victim_agv.next_cell = None
        victim_agv.move_progress = 0.0

        # ---- 渐进松弛的两档避障集合 ----
        occupied_all = set(self.cell_owner.keys()) | set(self.reservations.keys())
        strict_blocked = occupied_all - {c for c, a in self.cell_owner.items()
                                         if a == victim} \
            - {c for c, a in self.reservations.items() if a == victim}
        loose_blocked = {c for c, a in self.cell_owner.items() if a != victim}

        new_path = None
        for blocked in (strict_blocked, loose_blocked):
            new_path = self.map.find_path(victim_agv.pos, goal,
                                          blocked=blocked)
            if new_path is not None:
                break

        if new_path is not None and len(new_path) > 1:
            victim_agv.set_path(new_path)
            self.reroute_total += 1
            self.deadlock_resolved += 1
            self.events.add("reroute",
                            f"死锁解除：AGV{victim} 让路重规划"
                            f"（新路径 {len(new_path)-1} 步），环已打破")
        else:
            # 两档都无解（如狭窄巷道两侧被堵死）：原地等待并计数上报，
            # 等阻挡者自行腾位后，等待图自然消环、车辆自动恢复行驶
            self.unresolved_waits += 1
            self.events.add("deadlock",
                            f"AGV{victim} 让路重规划无解，原地等待（累计 "
                            f"{self.unresolved_waits} 次，待道路腾空后自动恢复）")

    @staticmethod
    def _find_cycle(graph):
        """
        有向图找环（DFS 三色标记法）。
        :return: 环上节点列表（按环序）；无环返回 None。
        白=未访问 0，灰=在递归栈中 1，黑=已完成 2。
        """
        color = defaultdict(int)
        parent = {}

        def dfs(u):
            color[u] = 1
            for v in graph.get(u, ()):
                if color[v] == 0:
                    parent[v] = u
                    found = dfs(v)
                    if found:
                        return found
                elif color[v] == 1:          # 指向灰点 => 发现回边 => 有环
                    cycle = [v]
                    node = u
                    while node != v:
                        cycle.append(node)
                        node = parent[node]
                    cycle.reverse()
                    return cycle
            color[u] = 2
            return None

        for node in list(graph.keys()):
            if color[node] == 0:
                found = dfs(node)
                if found:
                    return found
        return None

    # ==================================================================
    # 三、统计快照（压测报告 / 看板数据源）
    # ==================================================================
    def stats(self):
        """输出死锁相关统计指标。"""
        return {
            "deadlock_detected": self.deadlock_detected,
            "deadlock_resolved": self.deadlock_resolved,
            "reroute_total": self.reroute_total,
            "unresolved_waits": self.unresolved_waits,
        }
