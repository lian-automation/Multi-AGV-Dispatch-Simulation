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
       - 解除：优先选"目标格未被堵"的环内车作为让路者，回收其全部预约后
         按"避开其他车占/预约格"重新 A*，且新路径首格必须当前可通行
         （有效性判据，防终点豁免造成的"假成功"重规划空转——复审06 N1）；
         目标恰被堵时升级为"先侧避再回原目标"的改道；侧避也无解才判
         "不可解环"，原地等待并按环首次判定计数上报（unresolved_waits）。

    C. 不变式校验 —— 让"一格一车/零对撞"成为被测量的结论
       check_invariants() 每拍校验：任意两车 pos 互异、每车脚下格必有
       归属且归属为本人、占用/预约表无幽灵记录。任一违反即计入
       invariant_violations 并抛 AssertionError（fail-fast），
       "对撞为 0"由此从设计推断升级为运行时实测结论。

并发模型说明：整个引擎运行在单线程仿真循环里，因此本控制器无需加锁；
Modbus/Flask 线程只通过引擎提供的线程安全入口（持 engine.lock）间接读写。
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
        # 计数口径（P2-3 去重）：deadlock_detected 是"物理死锁数"——以活跃环
        # 集合按环成员去重，同一批车互等的同一个环只计 1 次"发生"；
        # deadlock_resolved 是"环消失次数"（让路重规划或阻挡者腾位后自行消解）；
        # arbitration_total 是"仲裁触发次数"（冷却门控后的每次仲裁，含重复仲裁）。
        self.deadlock_detected = 0           # 物理死锁数（按环去重，首次检出计 1）
        self.deadlock_resolved = 0           # 环消失次数（让路成功或自行消解）
        self.arbitration_total = 0           # 仲裁触发次数（冷却门控后实际执行）
        self.unresolved_waits = 0            # 发生过"让路无解原地等待"的物理死锁数
        self.reroute_total = 0               # 让路重规划执行总次数
        self.dodge_detours = 0               # 仲裁升级：让路者目标被堵而"先侧避再回
                                             # 原目标"的改道次数（复审06 N1 根治观测量）

        # 检测节奏与仲裁状态
        self._check_timer = 0.0
        self.clock = lambda: 0.0             # 由引擎注入 sim_time 读取函数
        self._cycle_cooldown = {}            # frozenset(环成员) -> 上次仲裁时刻
        self._active_cycles = {}             # frozenset(环成员) -> 环的活跃档案
        self._cycle_stale_after = config.DEADLOCK_CHECK_INTERVAL * 2 + 0.1
        self._victim_cursor = 0              # 让路车轮换游标（防活锁：不总盯同一辆）

        # ---------- 运行时不变式（P2-5：零对撞的测量口径）----------
        self.invariant_violations = 0        # 占格冲突/不变式违规累计（预期恒 0）

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
        """
        车辆真正跨入 new_cell：占用转移（新格占用、旧格释放、预约核销）。

        守卫（P1-1 修复，防御层）：old_cell == new_cell（原地"移动"）时
        不得删除脚下格的占用记录——否则车辆仍站在格上、占用表却为空，
        形成 ghost cell 打破"一格一车"不变式。
        """
        self.cell_owner[new_cell] = agv.id
        self.reservations.pop(new_cell, None)          # 预约升级为占用，核销
        if old_cell != new_cell and self.cell_owner.get(old_cell) == agv.id:
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

        计数口径（P2-3 去重，双口径输出）：
            - deadlock_detected：物理死锁数。以 frozenset(环成员) 为键维护
              "活跃死锁集合"，环首次进入集合才计 1 次"发生"；
              同一个环持续互等（冷却期反复到期）不再重复计数。
            - arbitration_total：仲裁触发次数。冷却期门控后每次实际仲裁计 1
              （反映系统为同一/不同死锁付出的仲裁工作量）。
            - deadlock_resolved：环从活跃集合消失（让路重规划生效或阻挡者
              腾位后自行消解）才计 1 次"消除"。
            - unresolved_waits：发生过"让路无解原地等待"的物理死锁数
              （每个环至多计 1 次，不再随冷却期反复累加）。

        仲裁策略（对应规格"低优先级车让路重规划，无解则原地等待计数上报"）：
            1. 冷却期：同一个环在 DEADLOCK_COOLDOWN_SECONDS 秒内不重复仲裁
               （防止每拍空转刷计数）；
            2. 让路者优先级：剩余路径最长的车先让（离目标最远、绕行代价最小）；
               且优先选"目标格未被堵"的候选——目标恰为被堵格的车重规划会因
               A* 终点豁免而"假成功"，指派它让路无法破环（复审06 N1 根治）；
               若反复失败则按轮换游标换一辆，避免永远盯死同一辆造成活锁；
            3. 渐进松弛重规划：
               第1次 避开其他车的占用格+预约格（严格，不打扰任何人）；
               第2次 只避占用格（允许借道别人"已预约但尚未进入"的格子——
                     真正进格时仍要过 request_cell 路权审查，安全不受影响）；
            4. 有效性判据与升级（复审06 N1 根治）：重规划"成功"必须以
               "装载后下一步即能获得路权"为准——新路径首格仍被其他车占/预约
               （终点豁免的典型假成功）不得直接装载，升级为"先侧避空格、
               再回原目标"的改道；侧避也无解才判"不可解环"，原地等待并
               按 ring 首次判定计入 unresolved_waits 上报。
        """
        graph, by_id = self._build_wait_graph(agvs)
        now = self.clock()
        cycle = self._find_cycle(graph)

        # ---- 活跃环簿记：环首次检出才计"物理死锁发生"，持续互等不重复计 ----
        if cycle is not None:
            key = frozenset(cycle)
            entry = self._active_cycles.get(key)
            if entry is None:
                self._active_cycles[key] = {"last": now, "unresolved": False}
                self.deadlock_detected += 1
                names = "->".join(f"AGV{i}" for i in sorted(cycle))
                self.events.add("deadlock", f"检测到死锁环：{names}，启动让路仲裁")
            else:
                entry["last"] = now

        # ---- 活跃环清扫：连续多个检测周期未再检出的环视为已消失
        #      （让路重规划生效或阻挡者腾位后自行消解）才计"消除"。
        #      宽限期取 2 个检测周期，兼防环成员瞬时抖动造成的误判。----
        for key in list(self._active_cycles):
            if now - self._active_cycles[key]["last"] > self._cycle_stale_after:
                del self._active_cycles[key]
                self._cycle_cooldown.pop(key, None)   # 顺带清理冷却表（防慢泄漏）
                self.deadlock_resolved += 1

        if cycle is None:
            return  # 无环：系统健康

        # ---- 冷却期检查：同一批成员的环在冷却期内只仲裁一次 ----
        key = frozenset(cycle)
        if now - self._cycle_cooldown.get(key, -99.0) \
                < config.DEADLOCK_COOLDOWN_SECONDS:
            return
        self._cycle_cooldown[key] = now
        self.arbitration_total += 1

        # ---- 选让路者：按"剩余路径最长"优先排序 + 轮换游标防活锁 ----
        # 复审06 N1 根治：优先在"目标格当前未被其他车占/预约"的候选中轮换
        # （目标被堵者的重规划会因终点豁免而假成功，指派它无法破环）；
        # 全员目标被堵（对头互等）才退回全集轮换，由下面的升级改道破环。
        ordered = sorted(cycle, key=lambda i: len(by_id[i].path), reverse=True)
        pool = [i for i in ordered
                if not self._goal_blocked_by_others(by_id[i])] or ordered
        self._victim_cursor += 1
        victim = pool[(self._victim_cursor - 1) % len(pool)]
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

        if new_path is not None and len(new_path) > 1 \
                and self.first_step_grantable(victim_agv, new_path):
            victim_agv.set_path(new_path)
            self.reroute_total += 1
            # 注：deadlock_resolved 不在此处计数——环真正从活跃集合消失时
            # （下一检测周期扫到环已不存在）才计"消除"，避免
            # "重规划成功但环仍在"被误记为已解除（P2-3 口径修复）。
            self.events.add("reroute",
                            f"死锁仲裁：AGV{victim} 让路重规划"
                            f"（新路径 {len(new_path)-1} 步），等待环待消解")
        elif new_path is not None and len(new_path) > 1:
            # ---- 假成功重规划的升级处理（复审06 N1 根治）----
            # 新路径首格当前仍被其他车占/预约——典型：goal 恰为互等方的脚下格，
            # A* 终点豁免使重规划恒返回同一条直达路径；直接装载则下一拍仍原地
            # 被拒、等待关系不变，仲裁按冷却期无限自旋而环永不消。
            # 升级：为让路者规划"先侧避空格、再回原目标"的改道（原目标仍是
            # 终点，到点取/放货语义不变）；让路者一挪，互等格腾出，环即消散。
            detour = self._plan_dodge_detour(victim_agv, goal, strict_blocked)
            if detour is not None:
                victim_agv.set_path(detour)
                self.reroute_total += 1
                self.dodge_detours += 1
                self.events.add("reroute",
                                f"死锁仲裁：AGV{victim} 目标格{goal}被互等方占用，"
                                f"升级为先侧避再回原目标（共 {len(detour)-1} 步）")
            else:
                self._mark_unresolved(
                    cycle, key, victim, "目标被堵且半径内无侧避格")
        else:
            # 两档都无解（如狭窄巷道两侧被堵死，或 goal 即脚下的退化无目标
            # 情形）：原地等待并上报。每个物理死锁环至多计 1 次（P2-3 去重）。
            self._mark_unresolved(cycle, key, victim, "让路重规划两档均无解")

    # ------------------------------------------------------------------
    # 仲裁辅助（复审06 N1 根治引入）
    # ------------------------------------------------------------------
    def _goal_blocked_by_others(self, agv):
        """候选让路者的当前目标格是否恰好被其他车占用/预约（被堵格/环内格）。"""
        goal = getattr(agv, "goal_cell", agv.pos)
        owner = self.cell_owner.get(goal)
        booker = self.reservations.get(goal)
        return (owner is not None and owner != agv.id) \
            or (booker is not None and booker != agv.id)

    def first_step_grantable(self, agv, path):
        """
        重规划有效性判据（复审06 N1）：
        path 为含起点的完整路径（find_path 口径），检查装载后首个待走格
        （path[1]）当前能否获得路权（未被其他车占用/预约）。
        False => 装载后下一拍仍原地被拒、等待关系不变，属"假成功"重规划，
        不得计入有效让路（自旋根源）。
        """
        if len(path) <= 1:
            return False
        first = path[1]
        owner = self.cell_owner.get(first)
        booker = self.reservations.get(first)
        return (owner is None or owner == agv.id) \
            and (booker is None or booker == agv.id)

    def _plan_dodge_detour(self, agv, goal, blocked):
        """
        为"目标格恰被互等方占用"的让路者规划侧避改道：
        先退到半径内一个当前完全空闲的侧避格（BFS 由近及远，镜像
        dispatcher._request_dodge 的分层搜索），再从侧避格规划回原目标，
        两段拼接、原目标仍为终点——车辆走完全程才触发到点结算，
        取/放货任务语义不被破坏。
        :return: 完整路径（含起点与原目标终点）；半径内无侧避格返回 None
        """
        occupied = set(self.cell_owner.keys()) | set(self.reservations.keys())
        seen = {agv.pos}
        frontier = [agv.pos]
        for _ in range(config.DODGE_MAX_RADIUS):
            nxt = []
            for cx, cy in frontier:
                for nb in self.map.neighbors(cx, cy):
                    if nb not in seen:
                        seen.add(nb)
                        nxt.append(nb)
            # 本层内取 A* 路程最短的空闲侧避格，逐个尝试接"回原目标"段
            candidates = []
            for c in sorted(nxt):                      # 排序保证结果确定性
                if c in occupied:
                    continue
                p1 = self.map.find_path(agv.pos, c, blocked=blocked)
                if p1 is not None:
                    candidates.append((len(p1), c, p1))
            for _, c, p1 in sorted(candidates, key=lambda x: (x[0], x[1])):
                p2 = self.map.find_path(c, goal, blocked=blocked)
                if p2 is not None:
                    return p1 + p2[1:]
            frontier = nxt
        return None

    def _mark_unresolved(self, cycle, key, victim, reason):
        """
        不可解环上报：按环首次判定计 1 次 unresolved_waits（P2-3 去重口径），
        并发出显著告警事件（复审06 N1/N2：自旋/无解不得静默，
        杜绝"全部环已消解"的失真结论）。
        """
        entry = self._active_cycles.get(key)
        if entry is not None and not entry["unresolved"]:
            entry["unresolved"] = True
            self.unresolved_waits += 1
            names = "->".join(f"AGV{i}" for i in sorted(cycle))
            self.events.add("deadlock",
                            f"⚠ 不可解环[{names}]：AGV{victim} {reason}，"
                            f"原地等待外部格局变化（已计入 unresolved 上报）")

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
    # 三、运行时不变式校验（"一格一车/零对撞"的被测量保证）
    # ==================================================================
    def check_invariants(self, agvs):
        """
        每拍校验"一格一车"核心不变式（O(F)，F=车辆数）：
            1. 任意两台 AGV 的 pos 互异（物理上不允许两车同格）；
            2. 每台 AGV 脚下格必有 cell_owner 归属且归属为本人
               （不存在"人站在无主格"的 ghost cell——P1-1 的破口）；
            3. 占用/预约表无幽灵记录：cell_owner/reservations 的值必须是
               在场车辆 id，且同一格的"占用者"与"预约者"不得是不同的车。
        任一违反：先计入 invariant_violations（占格冲突测量口径），
        再抛 AssertionError（fail-fast，配置 INVARIANT_CHECK=True 时启用）。
        """
        by_pos = {}
        for a in agvs:
            other = by_pos.get(a.pos)
            if other is not None:
                self.invariant_violations += 1
                raise AssertionError(
                    f"不变式违规[两车同格]：AGV{a.id} 与 AGV{other} 同在 {a.pos}")
            by_pos[a.pos] = a.id
            owner = self.cell_owner.get(a.pos)
            if owner != a.id:
                self.invariant_violations += 1
                raise AssertionError(
                    f"不变式违规[占格无主/错主]：AGV{a.id} 站于 {a.pos}，"
                    f"cell_owner 记录为 {owner}")
        valid_ids = {a.id for a in agvs}
        for c, oid in self.cell_owner.items():
            if oid not in valid_ids:
                self.invariant_violations += 1
                raise AssertionError(
                    f"不变式违规[幽灵占用记录]：{c} 的占用者 AGV{oid} 不在场")
        for c, bid in self.reservations.items():
            owner = self.cell_owner.get(c)
            if bid not in valid_ids or (owner is not None and owner != bid):
                self.invariant_violations += 1
                raise AssertionError(
                    f"不变式违规[幽灵/冲突预约记录]：{c} 预约者 AGV{bid}，"
                    f"占用者 {owner}")

    # ==================================================================
    # 四、统计快照（压测报告 / 看板数据源）
    # ==================================================================
    def stats(self):
        """输出死锁与不变式相关统计指标。"""
        return {
            "deadlock_detected": self.deadlock_detected,   # 物理死锁数（去重）
            "deadlock_resolved": self.deadlock_resolved,   # 环消失数（消除）
            "arbitration_total": self.arbitration_total,   # 仲裁触发次数
            "reroute_total": self.reroute_total,
            "dodge_detours": self.dodge_detours,   # 仲裁升级侧避改道次数（N1）
            "unresolved_waits": self.unresolved_waits,
            "invariant_violations": self.invariant_violations,
        }
