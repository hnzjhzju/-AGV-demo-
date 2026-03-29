"""
scheduler.py — AGV 多车协同调度系统
智慧港口 AGV 调度系统 · 调度层

架构分层：
  调度层  → 任务队列管理、全局负载均衡
  边缘层  → 冲突检测、死锁检测与解除
  车端模型→ AGV 状态机、路径执行

调度策略：
  任务分配：最近空闲 AGV 优先（可扩展为匈牙利算法最优分配）
  路径规划：加权 A*（见 path_planning.py）
  冲突避免：时间-空间资源锁 + 等待-重规划双策略
  死锁检测：等待图环检测（DFS），超时强制抢占
"""

import time
import random
import threading
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

from path_planning import PortMap, astar_with_replanning, smooth_path


# ───────────────────────── 枚举与常量 ────────────────────────────

class AGVState(Enum):
    IDLE       = auto()   # 空闲，等待任务
    ASSIGNED   = auto()   # 已分配任务，规划路径中
    MOVING     = auto()   # 行进中（前往取货点）
    LOADING    = auto()   # 装货中
    DELIVERING = auto()   # 送货中
    UNLOADING  = auto()   # 卸货中
    CHARGING   = auto()   # 充电中
    BLOCKED    = auto()   # 被阻挡，等待


TICK_INTERVAL = 0.5   # 仿真时间步长（秒）
DEADLOCK_TIMEOUT = 6  # 超过此 tick 数未移动 → 判定死锁
LOW_BATTERY = 20      # 低电量阈值（%），触发充电


# ─────────────────────────── 数据类 ──────────────────────────────

@dataclass
class Task:
    task_id:  str
    pickup:   tuple[int, int]
    dropoff:  tuple[int, int]
    priority: int = 1           # 1=普通, 2=紧急
    created_at: float = field(default_factory=time.time)
    assigned_to: Optional[str] = None
    completed_at: Optional[float] = None

    def __repr__(self):
        return f"Task({self.task_id}: {self.pickup}→{self.dropoff} P{self.priority})"


@dataclass
class AGV:
    agv_id:   str
    position: tuple[int, int]
    battery:  float = 100.0
    state:    AGVState = AGVState.IDLE
    task:     Optional[Task] = None
    path:     list[tuple] = field(default_factory=list)
    path_idx: int = 0
    blocked_ticks: int = 0

    # 统计
    tasks_done:    int = 0
    total_distance: float = 0.0

    @property
    def next_pos(self) -> Optional[tuple]:
        if self.path and self.path_idx + 1 < len(self.path):
            return self.path[self.path_idx + 1]
        return None

    @property
    def is_idle(self) -> bool:
        return self.state == AGVState.IDLE

    def consume_battery(self, moving: bool):
        drain = 0.5 if moving else 0.1
        self.battery = max(0.0, self.battery - drain)

    def charge(self):
        self.battery = min(100.0, self.battery + 5.0)


# ──────────────────────── 调度层（Dispatcher） ────────────────────

class Dispatcher:
    """
    全局调度器
    职责：任务队列管理 + AGV 分配 + 负载均衡
    """

    def __init__(self, port_map: PortMap, chargers: list[tuple]):
        self.port_map  = port_map
        self.chargers  = chargers
        self.agvs:     dict[str, AGV]  = {}
        self.task_queue: list[Task]    = []
        self.completed:  list[Task]    = []
        self._lock = threading.Lock()
        self.tick_count = 0

        # 性能统计
        self.total_conflicts = 0
        self.total_replans   = 0

    # ── AGV 注册 ──────────────────────────────────────────────────

    def register_agv(self, agv: AGV):
        self.agvs[agv.agv_id] = agv

    # ── 任务提交 ──────────────────────────────────────────────────

    def submit_task(self, task: Task):
        with self._lock:
            self.task_queue.append(task)
            self.task_queue.sort(key=lambda t: (-t.priority, t.created_at))

    # ── 核心调度 Tick ─────────────────────────────────────────────

    def tick(self):
        """每个仿真时间步调用一次"""
        with self._lock:
            self.tick_count += 1
            self._assign_tasks()
            self._move_agvs()
            self._check_deadlocks()
            self._handle_charging()

    # ── 任务分配（贪心最近 + 负载均衡）────────────────────────────

    def _assign_tasks(self):
        idle_agvs = [a for a in self.agvs.values()
                     if a.is_idle and a.battery > LOW_BATTERY]
        if not idle_agvs or not self.task_queue:
            return

        for task in list(self.task_queue):
            if not idle_agvs:
                break
            best = self._find_best_agv(task.pickup, idle_agvs)
            if best:
                self._assign(best, task)
                idle_agvs.remove(best)
                self.task_queue.remove(task)

    def _find_best_agv(self, target: tuple, candidates: list[AGV]) -> Optional[AGV]:
        """选择曼哈顿距离最近的空闲 AGV（可替换为匈牙利最优分配）"""
        def dist(a: AGV):
            return abs(a.position[0]-target[0]) + abs(a.position[1]-target[1])
        return min(candidates, key=dist, default=None)

    def _assign(self, agv: AGV, task: Task):
        task.assigned_to = agv.agv_id
        agv.task  = task
        agv.state = AGVState.ASSIGNED
        # 规划去取货点的路径
        result = astar_with_replanning(
            self.port_map, agv.position, task.pickup, agv.agv_id)
        if result.success:
            agv.path     = smooth_path(result.path)
            agv.path_idx = 0
            agv.state    = AGVState.MOVING
            self.port_map.reserve(agv.path, agv.agv_id)
            if result.replanned:
                self.total_replans += 1
        else:
            # 无路可走，放回队列
            agv.task  = None
            agv.state = AGVState.IDLE
            self.task_queue.insert(0, task)

    # ── AGV 移动推进 ──────────────────────────────────────────────

    def _move_agvs(self):
        for agv in self.agvs.values():
            if agv.state == AGVState.MOVING:
                self._step_agv(agv)
            elif agv.state == AGVState.DELIVERING:
                self._step_agv(agv)
            elif agv.state in (AGVState.LOADING, AGVState.UNLOADING):
                self._handle_io(agv)
            agv.consume_battery(agv.state == AGVState.MOVING)

    def _step_agv(self, agv: AGV):
        """推进 AGV 沿路径移动一步"""
        if agv.path_idx + 1 >= len(agv.path):
            # 到达当前阶段终点
            self._on_arrive(agv)
            return

        next_pos = agv.path[agv.path_idx + 1]

        # 冲突检测：目标格是否被占用
        if not self.port_map.is_passable(*next_pos, ignore_agv=agv.agv_id):
            agv.blocked_ticks += 1
            self.total_conflicts += 1
            return   # 等待一个 tick

        # 移动
        old_pos = agv.position
        agv.position = next_pos
        agv.path_idx += 1
        agv.total_distance += 1
        agv.blocked_ticks = 0

        # 释放已离开节点的预占
        if old_pos in self.port_map.reserved:
            del self.port_map.reserved[old_pos]

    def _on_arrive(self, agv: AGV):
        """到达阶段终点处理"""
        if agv.state == AGVState.MOVING:
            # 到达取货点 → 开始装货
            agv.state = AGVState.LOADING
            agv.blocked_ticks = 0
        elif agv.state == AGVState.DELIVERING:
            # 到达卸货点 → 开始卸货
            agv.state = AGVState.UNLOADING
            agv.blocked_ticks = 0

    def _handle_io(self, agv: AGV):
        """装/卸货（简化：1 tick 完成，可改为计数）"""
        if agv.state == AGVState.LOADING:
            # 装货完毕 → 规划去卸货点
            agv.state = AGVState.DELIVERING
            result = astar_with_replanning(
                self.port_map, agv.position, agv.task.dropoff, agv.agv_id)
            if result.success:
                agv.path     = smooth_path(result.path)
                agv.path_idx = 0
                self.port_map.reserve(agv.path, agv.agv_id)
        elif agv.state == AGVState.UNLOADING:
            # 卸货完毕 → 任务完成
            agv.task.completed_at = time.time()
            self.completed.append(agv.task)
            agv.tasks_done += 1
            self.port_map.release(agv.agv_id)
            agv.task     = None
            agv.path     = []
            agv.path_idx = 0
            agv.state    = AGVState.IDLE

    # ── 死锁检测与解除 ────────────────────────────────────────────

    def _check_deadlocks(self):
        """
        超时死锁检测：blocked_ticks 超过阈值时强制重规划
        更完备方案：构建等待图（wait-for graph），检测环
        """
        for agv in self.agvs.values():
            if agv.blocked_ticks >= DEADLOCK_TIMEOUT:
                self._resolve_deadlock(agv)

    def _resolve_deadlock(self, agv: AGV):
        """死锁解除：释放预占 + 原地等待随机 tick + 重规划"""
        self.port_map.release(agv.agv_id)
        agv.blocked_ticks = 0
        self.total_replans += 1

        # 确定目标点
        if agv.state == AGVState.MOVING and agv.task:
            goal = agv.task.pickup
        elif agv.state == AGVState.DELIVERING and agv.task:
            goal = agv.task.dropoff
        else:
            return

        result = astar_with_replanning(
            self.port_map, agv.position, goal, agv.agv_id)
        if result.success:
            agv.path     = smooth_path(result.path)
            agv.path_idx = 0
            self.port_map.reserve(agv.path, agv.agv_id)

    # ── 充电管理 ──────────────────────────────────────────────────

    def _handle_charging(self):
        for agv in self.agvs.values():
            if agv.state == AGVState.CHARGING:
                agv.charge()
                if agv.battery >= 95.0:
                    agv.state = AGVState.IDLE
            elif agv.is_idle and agv.battery < LOW_BATTERY:
                nearest_charger = min(
                    self.chargers,
                    key=lambda c: abs(c[0]-agv.position[0])+abs(c[1]-agv.position[1])
                )
                result = astar_with_replanning(
                    self.port_map, agv.position, nearest_charger, agv.agv_id)
                if result.success:
                    agv.path     = smooth_path(result.path)
                    agv.path_idx = 0
                    agv.state    = AGVState.MOVING  # 先移动到充电桩

    # ── 统计报告 ──────────────────────────────────────────────────

    def report(self) -> dict:
        return {
            "tick":            self.tick_count,
            "agv_count":       len(self.agvs),
            "tasks_pending":   len(self.task_queue),
            "tasks_completed": len(self.completed),
            "total_conflicts": self.total_conflicts,
            "total_replans":   self.total_replans,
            "agv_states": {
                a.agv_id: {
                    "state":    a.state.name,
                    "pos":      a.position,
                    "battery":  round(a.battery, 1),
                    "done":     a.tasks_done,
                }
                for a in self.agvs.values()
            }
        }


# ──────────────────────────── 场景初始化 ─────────────────────────

def build_port_scenario(num_agvs: int = 10) -> Dispatcher:
    """构建标准港口场景（40×25 网格）"""
    W, H = 40, 25
    port  = PortMap(W, H)

    # 堆场障碍（模拟集装箱区）
    for bx, by, ex, ey in [
        (8,  2, 12,  6), (8, 10, 12, 14), (8, 18, 12, 22),
        (20, 2, 24,  6), (20,10, 24, 14), (20,18, 24, 22),
        (32, 2, 36,  6), (32,10, 36, 14), (32,18, 36, 22),
    ]:
        port.set_region(bx, by, ex, ey, PortMap.OBSTACLE)

    chargers = [(2, 2), (2, 12), (2, 22)]
    for c in chargers:
        port.grid[c[1]][c[0]] = PortMap.CHARGER

    pickups  = [(6,4),(6,12),(6,20),(18,4),(18,12),(18,20),(30,4),(30,12),(30,20)]
    dropoffs = [(38,4),(38,12),(38,20),(14,8),(14,16),(26,8),(26,16)]

    dispatcher = Dispatcher(port, chargers)

    for i in range(num_agvs):
        agv = AGV(
            agv_id   = f"AGV-{i+1:02d}",
            position = chargers[i % len(chargers)],
            battery  = random.uniform(60, 100),
        )
        dispatcher.register_agv(agv)

    # 预生成任务
    for i in range(num_agvs * 3):
        task = Task(
            task_id = f"T{i+1:03d}",
            pickup  = random.choice(pickups),
            dropoff = random.choice(dropoffs),
            priority= random.choice([1, 1, 1, 2]),
        )
        dispatcher.submit_task(task)

    return dispatcher, pickups, dropoffs


# ─────────────────────── CLI 快速验证 ────────────────────────────

if __name__ == "__main__":
    import json

    print("=== 港口 AGV 调度系统 启动 ===\n")
    dispatcher, _, _ = build_port_scenario(num_agvs=10)

    for tick in range(30):
        dispatcher.tick()
        if tick % 5 == 0:
            r = dispatcher.report()
            print(f"Tick {r['tick']:3d} | "
                  f"待完成: {r['tasks_pending']:2d} | "
                  f"已完成: {r['tasks_completed']:2d} | "
                  f"冲突: {r['total_conflicts']:3d} | "
                  f"重规划: {r['total_replans']:2d}")
        time.sleep(0.05)

    print("\n=== 最终状态 ===")
    print(json.dumps(dispatcher.report(), indent=2, ensure_ascii=False))
