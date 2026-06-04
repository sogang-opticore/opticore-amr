#!/usr/bin/env python3
"""
lidar_clustering — fused_tracker P-6용 LiDAR 클러스터링 + static 필터 (ROS 비의존 순수 함수).

노드 독립성(융합 옵션 A) 위해 amr_navigation/dwa_node.py의
cluster_obstacle_points / occupancy_grid_has_static_obstacle_near 로직을
amr_perception에 자체 복제한다. 알고리즘은 동일.

ROS 메시지 타입에 직접 의존하지 않는다(단위 테스트 용이):
  - scan_to_points: ranges/angle 스칼라만 받음
  - has_static_obstacle_near: grid.info / grid.data 만 duck-typing
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass
class LidarCluster:
    """scan 점들로 만든 컴팩트 클러스터 (입력 좌표계 기준 — 보통 lidar_link)."""
    center: Tuple[float, float]
    radius: float
    count: int


def scan_to_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> List[Tuple[float, float]]:
    """LaserScan ranges → 센서 프레임 직교좌표 점 리스트.

    유효하지 않은 측정(inf/nan/범위 밖)은 버린다. scan 각도 순서는 유지되므로
    다음 단계의 'scan 순서 인접' 클러스터링이 성립한다.
    """
    points: List[Tuple[float, float]] = []
    lo = max(range_min, 1e-3)
    for i, r in enumerate(ranges):
        rf = float(r)
        if not math.isfinite(rf) or rf < lo or rf > range_max:
            continue
        a = angle_min + i * angle_increment
        points.append((rf * math.cos(a), rf * math.sin(a)))
    return points


def cluster_points(
    points: Sequence[Tuple[float, float]],
    join_distance: float,
    min_points: int,
    max_radius: float,
) -> List[LidarCluster]:
    """scan 순서 점들을 jump-distance 규칙으로 클러스터링.

    인접 점 간 거리가 join_distance를 넘으면 클러스터를 끊는다.
    min_points 미만이거나 radius가 max_radius 초과인 클러스터는 버린다
    (긴 벽/선형 구조물 제거). DWA cluster_obstacle_points와 동일.
    """
    if not points:
        return []
    join_distance = max(0.05, join_distance)
    min_points = max(1, int(min_points))
    max_radius = max(0.05, max_radius)

    clusters: List[LidarCluster] = []
    current: List[Tuple[float, float]] = []

    def flush() -> None:
        if len(current) < min_points:
            return
        cx = sum(p[0] for p in current) / len(current)
        cy = sum(p[1] for p in current) / len(current)
        radius = max(math.hypot(px - cx, py - cy) for px, py in current)
        if radius <= max_radius:
            clusters.append(LidarCluster((cx, cy), radius, len(current)))

    prev: Optional[Tuple[float, float]] = None
    for p in points:
        if prev is not None and math.hypot(p[0] - prev[0], p[1] - prev[1]) > join_distance:
            flush()
            current = []
        current.append(p)
        prev = p
    flush()
    return clusters


def _world_to_cell(info, wx: float, wy: float) -> Optional[Tuple[int, int]]:
    res = float(info.resolution)
    if res <= 0.0:
        return None
    col = int((wx - info.origin.position.x) / res)
    row = int((wy - info.origin.position.y) / res)
    if 0 <= col < int(info.width) and 0 <= row < int(info.height):
        return col, row
    return None


def has_static_obstacle_near(
    grid,  # nav_msgs/OccupancyGrid (duck-typed: grid.info, grid.data)
    world_xy: Tuple[float, float],
    radius: float,
    occupied_threshold: int,
    unknown_as_static: bool,
) -> bool:
    """map 좌표 점 주변 radius 안에 정적 점유 셀이 있으면 True.

    occupied_threshold 이상이면 점유, unknown(-1)은 unknown_as_static이면 정적 취급.
    DWA occupancy_grid_has_static_obstacle_near와 동일.
    """
    info = grid.info
    cell = _world_to_cell(info, world_xy[0], world_xy[1])
    if cell is None:
        return False

    res = float(info.resolution)
    radius = max(0.0, float(radius))
    radius_cells = max(0, int(math.ceil(radius / res)))
    col, row = cell
    width = int(info.width)
    height = int(info.height)
    threshold = max(0, min(100, int(occupied_threshold)))
    data = grid.data

    for rr in range(max(0, row - radius_cells), min(height, row + radius_cells + 1)):
        dy = (rr - row) * res
        for cc in range(max(0, col - radius_cells), min(width, col + radius_cells + 1)):
            dx = (cc - col) * res
            if math.hypot(dx, dy) > radius + 0.5 * res:
                continue
            idx = rr * width + cc
            if idx >= len(data):
                continue
            value = int(data[idx])
            if value >= threshold or (unknown_as_static and value < 0):
                return True
    return False
