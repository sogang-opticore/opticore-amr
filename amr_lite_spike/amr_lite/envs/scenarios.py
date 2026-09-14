from __future__ import annotations

import hashlib
import json
import math
import zlib
from dataclasses import dataclass

import numpy as np

from .dynamics import RobotState
from .geometry import Rectangle
from .map import WarehouseMap


@dataclass
class DynamicObstacle:
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 0.28
    motion_type: str = "cross"
    start_delay: float = 0.0
    stop_time: float | None = None

    def update(self, time: float, dt: float) -> None:
        if time < self.start_delay or (self.stop_time is not None and time >= self.stop_time):
            return
        self.x += self.vx * dt
        self.y += self.vy * dt


@dataclass
class Scenario:
    scenario_id: str
    warehouse_map: WarehouseMap
    start: RobotState
    goal: tuple[float, float]
    dynamic_obstacles: list[DynamicObstacle]
    solvable: bool = True
    instance_id: str = ""


def _instance_id(scenario_id: str, seed: int, start: RobotState, goal: tuple[float, float],
                 dynamics: list[DynamicObstacle], randomized: bool) -> str:
    payload = {
        "scenario_id": scenario_id,
        "seed": seed if randomized else 0,
        "randomized": randomized,
        "start": [start.x, start.y, start.theta],
        "goal": list(goal),
        "dynamic_obstacles": [
            [item.x, item.y, item.vx, item.vy, item.radius, item.start_delay, item.stop_time]
            for item in dynamics
        ],
    }
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()[:16]
    seed_label = seed if randomized else 0
    return f"{scenario_id}-{seed_label}-{digest}"


def _valid_point(warehouse_map: WarehouseMap, x: float, y: float, radius: float = 0.40) -> bool:
    return not warehouse_map.collides(x, y, radius)


def _jitter_pose(rng: np.random.Generator, state: RobotState, warehouse_map: WarehouseMap,
                 xy_amount: float, theta_amount: float) -> RobotState:
    for _ in range(32):
        candidate = RobotState(
            state.x + float(rng.uniform(-xy_amount, xy_amount)),
            state.y + float(rng.uniform(-xy_amount, xy_amount)),
            state.theta + float(rng.uniform(-theta_amount, theta_amount)),
        )
        if _valid_point(warehouse_map, candidate.x, candidate.y):
            return candidate
    return RobotState(state.x, state.y, state.theta)


def _jitter_goal(rng: np.random.Generator, goal: tuple[float, float], warehouse_map: WarehouseMap,
                 amount: float) -> tuple[float, float]:
    for _ in range(32):
        candidate = (goal[0] + float(rng.uniform(-amount, amount)),
                     goal[1] + float(rng.uniform(-amount, amount)))
        if _valid_point(warehouse_map, *candidate):
            return candidate
    return goal


def _randomize_dynamic_obstacles(rng: np.random.Generator, dynamics: list[DynamicObstacle],
                                 config: dict) -> list[DynamicObstacle]:
    position = float(config.get("dynamic_position", 0.0))
    speed_fraction = float(config.get("dynamic_speed_fraction", 0.0))
    delay = float(config.get("dynamic_start_delay", 0.0))
    radius = float(config.get("dynamic_radius", 0.0))
    randomized = []
    for item in dynamics:
        speed_scale = max(0.1, 1.0 + float(rng.uniform(-speed_fraction, speed_fraction)))
        stop_time = item.stop_time
        if stop_time is not None:
            stop_time = max(0.0, stop_time + float(rng.uniform(-delay, delay)))
        randomized.append(DynamicObstacle(
            x=item.x + float(rng.uniform(-position, position)),
            y=item.y + float(rng.uniform(-position, position)),
            vx=item.vx * speed_scale,
            vy=item.vy * speed_scale,
            radius=max(0.05, item.radius + float(rng.uniform(-radius, radius))),
            motion_type=item.motion_type,
            start_delay=max(0.0, item.start_delay + float(rng.uniform(0.0, delay))),
            stop_time=stop_time,
        ))
    return randomized


def make_scenario(scenario_id: str, seed: int = 0, randomization: dict | None = None) -> Scenario:
    specs: dict[str, tuple[list[Rectangle], RobotState, tuple[float, float], bool]] = {
        "S0": ([], RobotState(1.0, 4.0, 0.0), (11.0, 4.0), True),
        "S1": ([Rectangle(5.0, 0.0, 5.5, 5.7)], RobotState(1.0, 2.0, 0.0), (9.5, 6.5), True),
        "S2": ([Rectangle(3.0, 0.0, 9.0, 3.25), Rectangle(3.0, 4.75, 9.0, 8.0)], RobotState(1.0, 4.0, 0.0), (11.0, 4.0), True),
        "S3": ([Rectangle(5.3, 3.2, 6.2, 4.8)], RobotState(1.0, 4.0, 0.0), (11.0, 4.0), True),
        "S4": ([Rectangle(5.0, 4.55, 7.0, 6.2)], RobotState(1.0, 4.0, 0.0), (11.0, 4.0), True),
        "S5": ([Rectangle(4.0, 5.0, 8.0, 5.6)], RobotState(1.0, 6.5, -0.4), (11.0, 4.0), True),
        "S6": ([Rectangle(4.8, 0.0, 5.5, 5.9), Rectangle(7.5, 2.1, 8.2, 8.0)], RobotState(1.0, 4.0, 0.0), (11.0, 4.0), True),
        "S7": ([Rectangle(4.0, 1.0, 4.6, 7.0), Rectangle(0.0, 1.0, 4.0, 1.6), Rectangle(0.0, 6.4, 4.0, 7.0)], RobotState(2.0, 4.0, 0.0), (10.0, 4.0), False),
    }
    static_id = scenario_id if scenario_id in specs else "S0"
    obstacles, start, goal, solvable = specs[static_id]
    warehouse_map = WarehouseMap(obstacles=list(obstacles))
    dynamics: list[DynamicObstacle] = []
    if scenario_id == "D0":
        dynamics = [DynamicObstacle(6.0, 2.0, 0.0, 0.7, motion_type="cross")]
    elif scenario_id == "D1":
        dynamics = [DynamicObstacle(9.0, 4.0, -0.45, 0.0, motion_type="approach")]
    elif scenario_id == "D2":
        dynamics = [DynamicObstacle(4.0, 4.0, 0.25, 0.0, motion_type="same_direction")]
    elif scenario_id == "D3":
        dynamics = [DynamicObstacle(6.0, 4.0, 0.0, 0.0, motion_type="stopped")]
    elif scenario_id == "D4":
        dynamics = [DynamicObstacle(6.0, 2.0, 0.0, 0.7, motion_type="stop", stop_time=3.0)]
    randomization = randomization or {}
    randomized = bool(randomization.get("enabled", False))
    if randomized:
        scenario_key = zlib.crc32(scenario_id.encode("utf-8"))
        rng = np.random.default_rng(np.random.SeedSequence([int(seed), scenario_key]))
        start = _jitter_pose(rng, start, warehouse_map,
                             float(randomization.get("start_xy", 0.0)),
                             float(randomization.get("start_theta", 0.0)))
        goal = _jitter_goal(rng, goal, warehouse_map, float(randomization.get("goal_xy", 0.0)))
        dynamics = _randomize_dynamic_obstacles(rng, dynamics, randomization)
        start.theta = math.atan2(math.sin(start.theta), math.cos(start.theta))
    instance_id = _instance_id(scenario_id, int(seed), start, goal, dynamics, randomized)
    return Scenario(scenario_id, warehouse_map, start, goal, dynamics, solvable, instance_id)
