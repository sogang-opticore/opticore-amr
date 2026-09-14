from __future__ import annotations

import math

import numpy as np

from .dynamics import RobotState
from .geometry import ray_circle_distance, ray_rectangle_distance
from .map import WarehouseMap
from .scenarios import DynamicObstacle


def lidar_scan(state: RobotState, warehouse_map: WarehouseMap,
               dynamic_obstacles: list[DynamicObstacle], ray_count: int = 72,
               fov: float = math.radians(270.0), max_range: float = 8.0) -> np.ndarray:
    angles = np.linspace(-fov / 2.0, fov / 2.0, ray_count, dtype=np.float64)
    ranges = np.full(ray_count, max_range, dtype=np.float32)
    for index, relative in enumerate(angles):
        angle = state.theta + float(relative)
        dx, dy = math.cos(angle), math.sin(angle)
        distance = max_range
        for rect in warehouse_map.all_rectangles():
            distance = min(distance, ray_rectangle_distance(state.x, state.y, dx, dy, rect, max_range))
        for obstacle in dynamic_obstacles:
            distance = min(distance, ray_circle_distance(state.x, state.y, dx, dy,
                                                          obstacle.x, obstacle.y, obstacle.radius, max_range))
        ranges[index] = max(0.0, min(max_range, distance))
    return ranges
