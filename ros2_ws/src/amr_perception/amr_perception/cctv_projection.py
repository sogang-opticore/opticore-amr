"""CCTV no-go 투영 코어 — 순수 함수(ROS 비의존, 단위테스트 가능).

좌표계 (EXPLORE 실측 검증, PLAN §1.4):
  - gz 카메라 광축 = 센서 link **+X**, image-right = −Y, image-down = −Z.
  - 픽셀(u,v) → link 방향 ray = (1, −(u−cx)/fx, −(v−cy)/fy).
  - world 회전 R = Rz(yaw)·Ry(pitch)·Rx(roll)  (SDF extrinsic XYZ RPY).
  - z=0 바닥평면 교점 → world(X,Y).   (1n 광축(320,240)→(14.5,16.41), foot(359,341)→(13.93,18.03) 실증)
  - world→map: map = world + (map_offset_x, map_offset_y)   (실측 offset = (−2.96,−15.12)).
  - map→cell: col=floor((mx−origin_x)/res), row=floor((my−origin_y)/res)  (DWA occupancy_grid_world_to_cell 미러).
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def rot_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF extrinsic RPY → 3x3 회전행렬 R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def project_pixel_to_ground(
    u: float,
    v: float,
    cam_pose: Tuple[float, float, float, float, float, float],
    intrinsics: Tuple[float, float, float, float],
    z_plane: float = 0.0,
) -> Optional[Tuple[float, float]]:
    """픽셀(u,v) → world z=z_plane 평면 교점 (Xw,Yw). 광선이 평면을 향하지 않으면 None.

    cam_pose = (x, y, z, roll, pitch, yaw)  [world 프레임]
    intrinsics = (fx, fy, cx, cy)
    """
    fx, fy, cx, cy = intrinsics
    if fx == 0.0 or fy == 0.0:
        return None
    ray_link = np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
    direction = rot_rpy(cam_pose[3], cam_pose[4], cam_pose[5]) @ ray_link
    cz = cam_pose[2]
    dz = direction[2]
    # 카메라가 평면 위(cz>z_plane)이면 평면에 닿으려면 dz<0 이어야 함.
    denom = dz
    if abs(denom) < 1e-9:
        return None
    t = (z_plane - cz) / denom
    if t <= 0.0:
        return None  # 평면이 카메라 뒤/반대방향
    return (cam_pose[0] + t * direction[0], cam_pose[1] + t * direction[1])


def world_to_map(
    xw: float, yw: float, map_offset_x: float, map_offset_y: float
) -> Tuple[float, float]:
    """gz world → map 프레임.  map = world + offset  (실측 offset=(−2.96,−15.12))."""
    return (xw + map_offset_x, yw + map_offset_y)


def map_to_cell(
    mx: float, my: float, origin_x: float, origin_y: float, resolution: float
) -> Optional[Tuple[int, int]]:
    """map 프레임 좌표 → OccupancyGrid (col,row). DWA occupancy_grid_world_to_cell 와 동일(yaw=0 가정)."""
    if resolution <= 0.0:
        return None
    col = int(math.floor((mx - origin_x) / resolution))
    row = int(math.floor((my - origin_y) / resolution))
    return (col, row)


def pixel_to_cell(
    u: float,
    v: float,
    cam_pose: Tuple[float, float, float, float, float, float],
    intrinsics: Tuple[float, float, float, float],
    map_offset_x: float,
    map_offset_y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    z_plane: float = 0.0,
) -> Optional[Tuple[int, int, float, float]]:
    """전체 파이프라인: pixel → world → map → cell. 반환 (col,row,map_x,map_y) 또는 None(투영 실패/맵 밖)."""
    world = project_pixel_to_ground(u, v, cam_pose, intrinsics, z_plane)
    if world is None:
        return None
    mx, my = world_to_map(world[0], world[1], map_offset_x, map_offset_y)
    cell = map_to_cell(mx, my, origin_x, origin_y, resolution)
    if cell is None:
        return None
    col, row = cell
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    return (col, row, mx, my)
