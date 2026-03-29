"""
path_planning.py — AGV A* 路径规划模块
智慧港口 AGV 调度系统 · 核心算法层

功能：
  - A* 最短路径规划
  - 动态障碍物规避（实时重规划）
  - 资源占用锁（防止路径冲突）
  - 路径平滑（减少不必要转向）
"""

import heapq
from dataclasses import dataclass, field
from typing import Optional


# ─────────────────────────── 数据结构 ────────────────────────────

@dataclass(order=True)
class PriorityItem:
    priority: float
    item: tuple = field(compare=False)


@dataclass
class PathResult:
    success: bool
    path: list[tuple[int, int]]
    cost: float
    replanned: bool = False

    def __repr__(self):
        status = "OK" if self.success else "FAIL"
        return f"PathResult({status}, len={len(self.path)}, cost={self.cost:.1f})"


# ─────────────────────────── 地图模型 ────────────────────────────

class PortMap:
    """
    港口地图：网格化表示
    0 = 可通行  1 = 障碍/集装箱堆场  2 = 充电桩  3 = 装卸区
    """

    PASSABLE = 0
    OBSTACLE = 1
    CHARGER  = 2
    TERMINAL = 3

    def __init__(self, width: int, height: int):
        self.width  = width
        self.height = height
        self.grid   = [[self.PASSABLE] * width for _ in range(height)]

        # 被 AGV 预占用的节点 {(x,y): agv_id}
        self.reserved: dict[tuple, str] = {}

    def set_obstacle(self, x: int, y: int):
        self.grid[y][x] = self.OBSTACLE

    def set_region(self, x1, y1, x2, y2, cell_type: int):
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                self.grid[y][x] = cell_type

    def is_passable(self, x: int, y: int, ignore_agv: Optional[str] = None) -> bool:
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        if self.grid[y][x] == self.OBSTACLE:
            return False
        occupant = self.reserved.get((x, y))
        if occupant and occupant != ignore_agv:
            return False
        return True

    def reserve(self, path: list[tuple], agv_id: str):
        """预占路径节点（排除起点，起点由 AGV 自身持有）"""
        for pos in path[1:]:
            self.reserved[pos] = agv_id

    def release(self, agv_id: str):
        """释放该 AGV 的所有预占节点"""
        to_del = [k for k, v in self.reserved.items() if v == agv_id]
        for k in to_del:
            del self.reserved[k]

    def neighbors(self, x: int, y: int, ignore_agv: Optional[str] = None):
        """返回 4 邻域可通行节点（可扩展为 8 邻域）"""
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x + dx, y + dy
            if self.is_passable(nx, ny, ignore_agv):
                yield (nx, ny)


# ────────────────────────── A* 核心 ──────────────────────────────

def heuristic(a: tuple, b: tuple) -> float:
    """曼哈顿距离启发函数（网格地图最优）"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(
    port_map: PortMap,
    start: tuple[int, int],
    goal:  tuple[int, int],
    agv_id: Optional[str] = None,
    weight: float = 1.2,          # 加权 A*：weight > 1 牺牲最优换速度
) -> PathResult:
    """
    加权 A* 路径规划
    weight=1.0  → 标准 A*（最优）
    weight>1.0  → 快速次优解（推荐实时场景）
    """
    if start == goal:
        return PathResult(True, [start], 0.0)

    open_heap: list[PriorityItem] = []
    heapq.heappush(open_heap, PriorityItem(0.0, start))

    came_from: dict[tuple, Optional[tuple]] = {start: None}
    g_score:   dict[tuple, float]           = {start: 0.0}

    while open_heap:
        current = heapq.heappop(open_heap).item

        if current == goal:
            return PathResult(
                success=True,
                path=_reconstruct(came_from, current),
                cost=g_score[goal],
            )

        for neighbor in port_map.neighbors(*current, ignore_agv=agv_id):
            tentative_g = g_score[current] + 1.0   # 均匀代价

            # 转向惩罚：减少路径锯齿
            if came_from[current] is not None:
                prev = came_from[current]
                d_prev = (current[0]-prev[0], current[1]-prev[1])
                d_next = (neighbor[0]-current[0], neighbor[1]-current[1])
                if d_prev != d_next:
                    tentative_g += 0.3

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tentative_g
                f = tentative_g + weight * heuristic(neighbor, goal)
                heapq.heappush(open_heap, PriorityItem(f, neighbor))

    return PathResult(success=False, path=[], cost=float('inf'))


def astar_with_replanning(
    port_map: PortMap,
    start: tuple,
    goal:  tuple,
    agv_id: str,
    max_retries: int = 3,
) -> PathResult:
    """
    带动态重规划的 A*：
    首次规划失败时，忽略部分预占节点重试（降级策略）
    """
    result = astar(port_map, start, goal, agv_id)
    if result.success:
        return result

    # 降级：临时释放自身预占，重新规划
    for attempt in range(max_retries):
        port_map.release(agv_id)
        result = astar(port_map, start, goal, agv_id)
        if result.success:
            result.replanned = True
            return result

    return PathResult(success=False, path=[], cost=float('inf'), replanned=True)


# ─────────────────────────── 工具函数 ────────────────────────────

def _reconstruct(came_from: dict, current: tuple) -> list[tuple]:
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    return path[::-1]


def smooth_path(path: list[tuple]) -> list[tuple]:
    """
    路径平滑：去除共线冗余节点
    例: [(0,0),(1,0),(2,0)] → [(0,0),(2,0)]
    """
    if len(path) <= 2:
        return path
    smoothed = [path[0]]
    for i in range(1, len(path) - 1):
        prev = path[i-1]
        curr = path[i]
        nxt  = path[i+1]
        # 不共线则保留
        if (curr[0]-prev[0], curr[1]-prev[1]) != (nxt[0]-curr[0], nxt[1]-curr[1]):
            smoothed.append(curr)
    smoothed.append(path[-1])
    return smoothed


# ──────────────────────────── 快速测试 ───────────────────────────

if __name__ == "__main__":
    port = PortMap(20, 10)
    port.set_region(5, 2, 5, 7, PortMap.OBSTACLE)   # 垂直墙

    result = astar(port, (0, 5), (19, 5))
    print(result)
    print("路径:", smooth_path(result.path))
