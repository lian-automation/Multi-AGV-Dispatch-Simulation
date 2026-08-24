# -*- coding: utf-8 -*-
"""
grid_map.py —— 栅格地图加载与 A* 最短路寻路
=============================================

职责：
    1. 加载并校验 map/map_30x20.json（30×20 栅格）；
    2. 提供通行判断、邻格扩展（支持单向巷道约束）；
    3. 提供 A* 最短路函数（曼哈顿启发），输出"格子序列"路径；
       —— 支持传入 blocked 集合，供调度层绕开已被占用/预约的格子重规划；
    4. 预计算"货位点"（紧贴货架的巷道格），作为入库/出库/移库任务的取放货坐标。

坐标系约定：
    (x, y) = (列, 行)，原点在左上角；y 向下增大（与 Canvas 像素方向一致）。

A* 要点（答辩高频考点，详见 docs/系统设计说明书.md）：
    f(n) = g(n) + h(n)，g 为已走步数，h 为曼哈顿距离 |dx|+|dy|。
    网格四邻接且每步代价为 1 时，曼哈顿距离是可采纳启发（不会高估），
    因此 A* 保证找到最短路，同时比 Dijkstra 少展开大量无关节点。
"""

import heapq
import json

import config

# 单向巷道的方向向量表：dir 字符 -> (dx, dy)
_DIR_VEC = {
    "E": (1, 0),   # 向东（右）
    "W": (-1, 0),  # 向西（左）
    "N": (0, -1),  # 向北（上）
    "S": (0, 1),   # 向南（下）
}


class GridMap:
    """栅格地图对象：负责地图数据结构、通行判断与路径规划。"""

    def __init__(self, data):
        """
        :param data: 从 JSON 读出的字典（结构见 map/map_30x20.json）
        """
        # ---------- 1. 基础字段与尺寸校验 ----------
        self.name = data["name"]
        self.cols = int(data["cols"])
        self.rows = int(data["rows"])
        if self.cols != config.MAP_COLS or self.rows != config.MAP_ROWS:
            raise ValueError(
                f"地图尺寸 ({self.cols}x{self.rows}) 与 config.MAP_COLS/ROWS "
                f"({config.MAP_COLS}x{config.MAP_ROWS}) 不一致，请检查配置！"
            )

        # ---------- 2. 解析栅格字符 ----------
        # grid_types[y][x]：'#'=货架(不可通行)，'.'=巷道(可通行)
        raw_grid = data["grid"]
        if len(raw_grid) != self.rows:
            raise ValueError(f"栅格行数 {len(raw_grid)} 与声明 rows={self.rows} 不符！")
        self.grid_types = []
        for y, line in enumerate(raw_grid):
            if len(line) != self.cols:
                raise ValueError(f"第 {y} 行长度 {len(line)} 与声明 cols={self.cols} 不符！")
            row = []
            for x, ch in enumerate(line):
                if ch == "#":
                    row.append("#")
                elif ch == ".":
                    row.append(".")
                else:
                    raise ValueError(f"非法栅格字符 '{ch}' @({x},{y})，只允许 '#' 和 '.'")
            self.grid_types.append(row)

        # ---------- 3. 特殊格子：站台 / 充电站 ----------
        # 站台：列表保序，索引 i 对应 Modbus HRi 呼叫寄存器
        self.stations = []           # [(id, x, y, desc), ...]
        for s in data.get("stations", []):
            self._check_road(s["x"], s["y"], s["id"])
            self.stations.append((s["id"], int(s["x"]), int(s["y"]), s.get("desc", "")))
        # 充电站：AGV 低电时的回充目标
        self.charges = []
        for c in data.get("charges", []):
            self._check_road(c["x"], c["y"], c["id"])
            self.charges.append((c["id"], int(c["x"]), int(c["y"]), c.get("desc", "")))
        if len(self.stations) < 4:
            raise ValueError("规格要求至少 4 个出入库站台！")
        if len(self.charges) < 2:
            raise ValueError("规格要求至少 2 个充电站！")

        # ---------- 4. 单向巷道约束 ----------
        # oneway[(x,y)] = 'E'/'W'/'N'/'S'：进入该格只允许沿该方向通过
        self.oneway = {}
        for ow in data.get("oneway_cells", []):
            f, t, d = ow["from"], ow["to"], ow["dir"]
            if d not in _DIR_VEC:
                raise ValueError(f"未知单向方向 '{d}'")
            for x in range(int(f["x"]), int(t["x"]) + 1):
                for y in range(int(f["y"]), int(t["y"]) + 1):
                    self.oneway[(x, y)] = d

        # ---------- 5. 预计算货位点（任务取放坐标池） ----------
        # 货位点 = 至少与一个货架格正交相邻的巷道格（模拟货架前的拣选位）
        self.slots = []
        for y in range(self.rows):
            for x in range(self.cols):
                if self.grid_types[y][x] != ".":
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.cols and 0 <= ny < self.rows \
                            and self.grid_types[ny][nx] == "#":
                        self.slots.append((x, y))
                        break
        if not self.slots:
            raise ValueError("未找到任何货位点：请检查货架区是否与巷道相邻！")

        # 单向开关：True=启用 JSON 里的单向约束（演示用）；False=全部双向（对照实验用）
        self.one_way_enable = True

    # ------------------------------------------------------------------
    # 基础查询
    # ------------------------------------------------------------------
    def _check_road(self, x, y, tag):
        """校验特殊格子必须落在可通行的巷道上，否则直接报错（配置错误尽早暴露）。"""
        if not (0 <= x < self.cols and 0 <= y < self.rows):
            raise ValueError(f"{tag} 坐标 ({x},{y}) 越界！")
        if self.grid_types[y][x] != ".":
            raise ValueError(f"{tag} 坐标 ({x},{y}) 不是巷道格，无法放置设备！")

    def in_bounds(self, x, y):
        """坐标是否在地图范围内。"""
        return 0 <= x < self.cols and 0 <= y < self.rows

    def passable(self, x, y):
        """格子是否可通行（在界内且不是货架墙）。"""
        return self.in_bounds(x, y) and self.grid_types[y][x] == "."

    def can_enter(self, cx, cy, nx, ny):
        """
        从 (cx,cy) 移动到 (nx,ny) 是否满足单向巷道约束。

        单向语义（重要）：方向约束只作用于"沿巷道轴向"的通行——
            东西向单行格(E/W)：允许 ①沿轴正向通过 ②南北方向垂直穿越（路口）；
            南北向单行格(N/S)：允许 ①沿轴正向通过 ②东西方向垂直穿越。
        若无此豁免，十字交口处垂直穿越会被误禁，
        车辆一旦驶入货架间的纵向通道格就会被困死（陷阱格）。
        """
        if not self.passable(nx, ny):
            return False
        d = self.oneway.get((nx, ny))
        if d is None or not self.one_way_enable:
            return True
        move = (nx - cx, ny - cy)
        axis_allowed = _DIR_VEC[d]                       # 沿轴正向：唯一许可的轴向
        crossing = {                                     # 垂直穿越向量集合
            "E": {(0, -1), (0, 1)},
            "W": {(0, -1), (0, 1)},
            "N": {(1, 0), (-1, 0)},
            "S": {(1, 0), (-1, 0)},
        }[d]
        return move == axis_allowed or move in crossing

    def station_pos(self, index):
        """按序号（0~3）取站台坐标——与 Modbus HR0~HR3 一一对应。"""
        _, x, y, _ = self.stations[index % len(self.stations)]
        return (x, y)

    # ------------------------------------------------------------------
    # 邻格扩展（A* 的核心依赖）
    # ------------------------------------------------------------------
    def neighbors(self, x, y):
        """四邻接扩展：依次尝试 东/南/西/北，过滤越界、墙体与单向违例。"""
        result = []
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            nx, ny = x + dx, y + dy
            if self.can_enter(x, y, nx, ny):
                result.append((nx, ny))
        return result

    # ------------------------------------------------------------------
    # A* 最短路（曼哈顿启发）
    # ------------------------------------------------------------------
    def find_path(self, start, goal, blocked=None):
        """
        A* 最短路搜索。

        :param start:  起点格 (x, y)
        :param goal:   终点格 (x, y)
        :param blocked: 额外视为不可通行的格子集合（已被其他车占用/预约的格子），
                        用于交通管制下的动态重规划；start/goal 本身不受此限制
        :return: 格子序列 [(x0,y0), (x1,y1), ...]，含起终点；不可达时返回 None
        """
        return astar(self, start, goal, blocked)


def astar(gmap, start, goal, blocked=None):
    """
    模块级 A* 函数（GridMap.find_path 的底层实现）。

    算法流程：
        1. 开放列表用最小堆，按 f = g + h 排序（h=曼哈顿距离）；
        2. 每次弹出 f 最小的节点，若是终点则回溯路径返回；
        3. 否则扩展其可通行邻格，松弛 g 值并记录来向。
    复杂度：O(E log V)；网格图上 E≈4V，实际远快于 Dijkstra 的全图扩张。
    """
    if start == goal:
        return [start]
    blocked = blocked or frozenset()

    # 启发函数 h：曼哈顿距离。网格每步代价 1 且四邻接 => h 可采纳（不高估）=> 结果最优
    def h(cell):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    open_heap = [(h(start), 0, start)]   # (f值, 序列号防比较冲突, 节点)
    g_score = {start: 0}                 # 起点 到 各格的最小步数
    came_from = {}                       # 回溯表：节点 -> 前驱节点
    closed = set()                       # 已确定最优 g 的节点集合
    seq = 0                              # 堆内自增序列号，保证同 f 时先进先出（结果确定性）

    while open_heap:
        f, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue                      # 过期堆条目（已有更优路径）跳过
        closed.add(cur)

        if cur == goal:                   # 终点出堆 => 最短路径确定，开始回溯
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path

        cx, cy = cur
        for nxt in gmap.neighbors(cx, cy):
            # 目标格若被其他车占用则绕行；但终点本身豁免（否则永远到不了有人下货的站台）
            if nxt in blocked and nxt != goal:
                continue
            tentative_g = g_score[cur] + 1
            if tentative_g < g_score.get(nxt, float("inf")):
                g_score[nxt] = tentative_g
                came_from[nxt] = cur
                seq += 1
                heapq.heappush(open_heap, (tentative_g + h(nxt), seq, nxt))

    return None  # 开放列表耗尽仍未到达终点 => 不可达


def load_map(path=None):
    """从 JSON 文件加载地图并返回 GridMap 对象（默认路径取 config.MAP_JSON_PATH）。"""
    path = path or config.MAP_JSON_PATH
    with open(path, "r", encoding="utf-8") as fp:
        data = json.load(fp)
    return GridMap(data)


# ----------------------------------------------------------------------
# 模块自检：python -m map.grid_map 直接运行可验证地图与 A* 是否正常
# ----------------------------------------------------------------------
if __name__ == "__main__":
    m = load_map()
    print(f"[自检] 地图《{m.name}》 {m.cols}x{m.rows} 加载成功")
    print(f"[自检] 站台数={len(m.stations)} 充电站数={len(m.charges)} "
          f"货位点数={len(m.slots)} 单向格数={len(m.oneway)}")
    p1 = m.find_path(m.stations[0][1:3], m.slots[0])
    print(f"[自检] A* 站台ST1{m.stations[0][1:3]} -> 货位{m.slots[0]}："
          f"{len(p1)-1 if p1 else '不可达'} 步")
    # 单向巷道对照测试：禁用单向约束后路径可能更短
    m.one_way_enable = False
    p2 = m.find_path(m.stations[0][1:3], m.slots[0])
    print(f"[自检] 关闭单向约束后同一起讫：{len(p2)-1 if p2 else '不可达'} 步")
