#!/usr/bin/env python3
"""
DWA Local Planner Node — 2026-05-29 상태 머신 리팩토링 (YS)

Opticore AMR — 직접 구현 DWA (Nav2 dwb_local_planner 의존 금지).

═══════════════════════════════════════════════════════════════════════
설계 원칙 (2026-05-25 재설계 핵심, 유지)
═══════════════════════════════════════════════════════════════════════

원칙 1) **DWA 의 모든 내부 계산은 odom_filtered frame 안에서**.
원칙 2) **매 cycle TF lookup 없음** — EKF /odometry/filtered 의 pose+twist 직접 사용.
원칙 3) **global_path 는 받을 때 한 번만** map → odom_filtered 로 변환·캐싱.
원칙 4) AMCL TF freeze / 보정 점프 의 영향이 cycle 단위로 안 들어옴.

═══════════════════════════════════════════════════════════════════════
2026-05-29 리팩토링 (YS)
═══════════════════════════════════════════════════════════════════════

기존 _control_loop 의 암묵적 상태 (if/elif 중첩) → NavState enum 명시화.

NavState 전이 규칙:
    NORMAL  → ALIGN       : |α| > align_thresh
    NORMAL  → EMERGENCY   : collision_imminent
    NORMAL  → SPIN        : is_velocity_blocked (stuck 1.5s)
    ALIGN   → NORMAL      : |α| < align_exit
    SPIN    → FORWARD_ONLY: spin 완료 + 전방 확보 → 짧게 전진 후 재계획
    SPIN    → EMERGENCY   : 회전해도 전방 미확보 (정지 + A* 재계획 대기)
    FORWARD_ONLY → NORMAL : 짧은 전진 완료 → A* 재계획 대기
    EMERGENCY → NORMAL    : 외부 트리거 (A* 새 path)
    * → REACHED           : dist_to_goal <= tolerance

2026-05-31 (SW · dwa-ys): Recovery 를 제자리 회전(SPIN)만으로 단순화.
    후진(BACKUP) 분기·파라미터 제거 — stuck/EMERGENCY 복구는 회전 → (전방 열리면)
    짧은 전진 → A* 재계획, 회전해도 안 열리면 EMERGENCY 정지.
    (사용자 지시: 후진 거동이 번거로워 회전만으로 복귀. CLAUDE.md §7.2.0.)

각 상태 실행 함수:
    _execute_normal()   Pure Pursuit + adaptive velocity
    _execute_align()    In-place PD 회전
    _execute_spin()     Recovery spin (제자리 회전)
    _execute_emergency() 완전 정지 + A* 재계획 대기

외부 인터페이스 (토픽, 파라미터) 는 기존과 완전 동일.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException


# ═══════════════════════════════════════════════════════════════════════
# 데이터 구조 — ROS 비의존 순수 자료형
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    """로봇 상태 (frame 무관 — 호출자가 frame 일관성 책임)."""
    x: float
    y: float
    theta: float
    v: float
    w: float


@dataclass
class DynamicWindow:
    """Dynamic Window — 도달 가능한 (v, w) 영역."""
    v_min: float
    v_max: float
    w_min: float
    w_max: float


@dataclass
class VelocityCommand:
    """평가 결과로 선택된 (v, w) 명령."""
    v: float
    w: float
    score: float = 0.0


@dataclass
class PathProjection:
    """로봇 위치를 path 선분 위에 투영한 결과."""
    segment_idx: int
    point: Tuple[float, float]
    offset: float
    signed_offset: float
    yaw: float


@dataclass
class PathSample:
    """Path projection point에서 arc-length로 떨어진 path 위 샘플."""
    point: Tuple[float, float]
    yaw: float
    distance: float


@dataclass
class RejoinTarget:
    """Path 이탈 시 부드럽게 재합류하기 위한 미래 목표점."""
    point: Tuple[float, float]
    yaw: float
    distance: float
    alpha: float
    arrival_error: float
    curvature: float
    clearance: float
    desired_distance: float
    score: float


@dataclass
class DynamicPathBlockage:
    """LiDAR obstacle cluster that blocks the near-term global path corridor."""
    blocked: bool
    distance: float
    count: int
    side_bias: float
    min_margin: float


@dataclass
class DynamicObstacleCluster:
    """A compact local-frame cluster made from dynamic LiDAR points."""
    center: Tuple[float, float]
    radius: float
    count: int


@dataclass
class DynamicObstacleTrack:
    """Odom-frame dynamic obstacle track used for short-horizon prediction."""
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    count: int
    age: int
    last_seen: float
    missed: int = 0


@dataclass
class DynamicMotionEstimate:
    """Relative motion estimate for the dynamic obstacle that matters most."""
    state: str
    track_id: int = -1
    distance: float = float("inf")
    speed: float = 0.0
    closing_speed: float = 0.0
    t_cpa: float = float("inf")
    d_cpa: float = float("inf")
    age: int = 0
    confidence: float = 0.0
    side: int = 0
    radius: float = 0.0


@dataclass
class DynamicObstacleMapBlock:
    """Map-frame temporary no-go area made from dynamic obstacle tracks."""
    block_id: int
    x: float
    y: float
    radius: float
    vx: float
    vy: float
    first_seen: float
    last_seen: float
    expire_at: float
    clear_since: Optional[float] = None
    trail: List[Tuple[float, float, float]] = field(default_factory=list)


@dataclass
class DynamicAvoidTarget:
    """Temporary local bypass target used while the static global path is blocked."""
    point: Tuple[float, float]
    yaw: float
    distance: float
    alpha: float
    arrival_error: float
    curvature: float
    clearance: float
    rejoin_clearance: float
    side: int
    offset: float
    blocked_distance: float
    score: float
    mode: str = "side_lane"


class NavState(Enum):
    """명시적 내비게이션 상태 — fleet BT 연결 준비."""
    NORMAL       = "NORMAL"        # Pure Pursuit 정상 추종
    REJOIN       = "REJOIN"        # path 이탈 후 미래 path 지점으로 부드럽게 재합류
    ALIGN        = "ALIGN"         # in-place 회전 (heading 오차 큼)
    DYNAMIC_BLOCKED = "DYNAMIC_BLOCKED"  # 동적 장애물이 path corridor를 막음
    APPROACHING_DYNAMIC = "APPROACHING_DYNAMIC"  # closing dynamic obstacle
    CROSSING_DYNAMIC = "CROSSING_DYNAMIC"  # moving obstacle is likely crossing
    RECEDING_DYNAMIC = "RECEDING_DYNAMIC"  # moving obstacle is moving away
    STOPPED_DYNAMIC = "STOPPED_DYNAMIC"  # tracked obstacle is stopped on path
    AVOIDING_DYNAMIC = "AVOIDING_DYNAMIC"  # side-offset local bypass 중
    SPIN         = "SPIN"          # spin recovery (stuck → 제자리 회전 탈출)
    FORWARD_ONLY = "FORWARD_ONLY"  # spin 완료 후 현재 heading으로 짧게 전진
    EMERGENCY    = "EMERGENCY"     # 충돌 임박 / 회전해도 전방 막힘
    REACHED   = "REACHED"     # goal 도달


# ═══════════════════════════════════════════════════════════════════════
# 순수 알고리즘 함수 — 단위 테스트 가능 (기존과 동일)
# ═══════════════════════════════════════════════════════════════════════

def compute_dynamic_window(
    state: RobotState,
    v_max: float,
    v_min: float,
    w_max: float,
    a_max: float,
    alpha_max: float,
    dt: float,
) -> DynamicWindow:
    v_lo_accel = state.v - a_max * dt
    v_hi_accel = state.v + a_max * dt
    w_lo_accel = state.w - alpha_max * dt
    w_hi_accel = state.w + alpha_max * dt
    return DynamicWindow(
        v_min=max(v_min, v_lo_accel),
        v_max=min(v_max, v_hi_accel),
        w_min=max(-w_max, w_lo_accel),
        w_max=min(w_max, w_hi_accel),
    )


def sample_velocities(
    window: DynamicWindow,
    n_v: int,
    n_w: int,
) -> List[Tuple[float, float]]:
    samples: List[Tuple[float, float]] = []
    if n_v < 1 or n_w < 1:
        return samples
    if n_v == 1:
        v_step_values = [(window.v_min + window.v_max) / 2.0]
    else:
        v_step = (window.v_max - window.v_min) / (n_v - 1)
        v_step_values = [window.v_min + i * v_step for i in range(n_v)]
    if n_w == 1:
        w_step_values = [(window.w_min + window.w_max) / 2.0]
    else:
        w_step = (window.w_max - window.w_min) / (n_w - 1)
        w_step_values = [window.w_min + j * w_step for j in range(n_w)]
    for v in v_step_values:
        for w in w_step_values:
            samples.append((v, w))
    return samples


def forward_simulate(
    state: RobotState,
    v: float,
    w: float,
    dt: float,
    sim_time: float,
) -> List[Tuple[float, float, float]]:
    if dt <= 0 or sim_time <= 0:
        return []
    steps = int(sim_time / dt)
    if steps <= 0:
        return []
    traj: List[Tuple[float, float, float]] = []
    x, y, theta = state.x, state.y, state.theta
    for _ in range(steps):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        theta = math.atan2(math.sin(theta), math.cos(theta))
        traj.append((x, y, theta))
    return traj


def heading_score(traj_end_x: float, traj_end_y: float, traj_end_theta: float,
                  goal_x: float, goal_y: float) -> float:
    """[기존] test_dwa.py 호환을 위해 유지."""
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    if dx == 0.0 and dy == 0.0:
        return 1.0
    angle_to_goal = math.atan2(dy, dx)
    diff = angle_to_goal - traj_end_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


def heading_score_dist(traj_end_x: float, traj_end_y: float,
                       goal_x: float, goal_y: float,
                       max_dist: float = 1.0) -> float:
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    dist = math.sqrt(dx * dx + dy * dy)
    return max(0.0, 1.0 - dist / max_dist)


def path_tangent_score(traj_end_theta: float, path_tangent_local: float) -> float:
    diff = path_tangent_local - traj_end_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


def compute_path_tangent_local(path_xy: List[Tuple[float, float]],
                                start_idx: int,
                                look_ahead: int,
                                robot_x: float, robot_y: float,
                                robot_theta: float) -> float:
    if not path_xy or start_idx >= len(path_xy):
        return 0.0
    target_idx = min(start_idx + look_ahead, len(path_xy) - 1)
    if target_idx <= start_idx:
        return 0.0
    px1, py1 = path_xy[start_idx]
    px2, py2 = path_xy[target_idx]
    path_yaw_world = math.atan2(py2 - py1, px2 - px1)
    diff = path_yaw_world - robot_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return diff


def clearance_score(trajectory_xy: List[Tuple[float, float]],
                    obstacle_points: List[Tuple[float, float]],
                    max_clearance: float = 1.0) -> float:
    if not trajectory_xy:
        return 0.0
    if not obstacle_points:
        return 1.0
    min_dist_sq = float("inf")
    for tx, ty in trajectory_xy:
        for ox, oy in obstacle_points:
            d_sq = (tx - ox) * (tx - ox) + (ty - oy) * (ty - oy)
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
    if min_dist_sq == float("inf"):
        return 1.0
    min_dist = math.sqrt(min_dist_sq)
    if max_clearance <= 0.0:
        return 0.0
    return min(min_dist / max_clearance, 1.0)


def velocity_score(v: float, v_max: float) -> float:
    if v_max <= 0.0:
        return 0.0
    return max(0.0, v) / v_max


def min_clearance_distance(trajectory_xy: List[Tuple[float, float]],
                           obstacle_points: List[Tuple[float, float]]) -> float:
    if not trajectory_xy or not obstacle_points:
        return float("inf")
    best = float("inf")
    for tx, ty in trajectory_xy:
        for ox, oy in obstacle_points:
            d_sq = (tx - ox) * (tx - ox) + (ty - oy) * (ty - oy)
            if d_sq < best:
                best = d_sq
    return math.sqrt(best) if best != float("inf") else float("inf")


def find_nearest_idx(
    path_xy: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    start_idx: int = 0,
) -> int:
    if not path_xy:
        return 0
    s = max(0, min(start_idx, len(path_xy) - 1))
    rx, ry = robot_xy
    nearest_idx = s
    nearest_d_sq = float("inf")
    for i in range(s, len(path_xy)):
        px, py = path_xy[i]
        d_sq = (px - rx) * (px - rx) + (py - ry) * (py - ry)
        if d_sq < nearest_d_sq:
            nearest_d_sq = d_sq
            nearest_idx = i
    return nearest_idx


def project_to_path(
    path_xy: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    start_idx: int = 0,
) -> Optional[PathProjection]:
    """가장 가까운 path 선분 위 투영점을 구한다.

    기존 nearest point 방식은 코너에서 로봇이 선분 사이를 지날 때 lookahead가
    튀기 쉽다. 선분 투영을 쓰면 경로의 실제 중심선 기준 횡오차를 안정적으로
    계산할 수 있다.
    """
    if not path_xy:
        return None
    rx, ry = robot_xy
    if len(path_xy) == 1:
        px, py = path_xy[0]
        return PathProjection(
            segment_idx=0,
            point=(px, py),
            offset=math.hypot(rx - px, ry - py),
            signed_offset=0.0,
            yaw=0.0,
        )

    start = max(0, min(start_idx, len(path_xy) - 2) - 5)
    best: Optional[PathProjection] = None
    best_d_sq = float("inf")

    for i in range(start, len(path_xy) - 1):
        x1, y1 = path_xy[i]
        x2, y2 = path_xy[i + 1]
        sx = x2 - x1
        sy = y2 - y1
        seg_len_sq = sx * sx + sy * sy
        if seg_len_sq <= 1e-12:
            continue

        t = ((rx - x1) * sx + (ry - y1) * sy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        px = x1 + t * sx
        py = y1 + t * sy
        dx = rx - px
        dy = ry - py
        d_sq = dx * dx + dy * dy
        if d_sq >= best_d_sq:
            continue

        seg_len = math.sqrt(seg_len_sq)
        tx = sx / seg_len
        ty = sy / seg_len
        signed_offset = tx * dy - ty * dx
        best = PathProjection(
            segment_idx=i,
            point=(px, py),
            offset=math.sqrt(d_sq),
            signed_offset=signed_offset,
            yaw=math.atan2(sy, sx),
        )
        best_d_sq = d_sq

    if best is not None:
        return best

    nearest_idx = find_nearest_idx(path_xy, robot_xy, start_idx)
    px, py = path_xy[nearest_idx]
    return PathProjection(
        segment_idx=max(0, min(nearest_idx, len(path_xy) - 2)),
        point=(px, py),
        offset=math.hypot(rx - px, ry - py),
        signed_offset=0.0,
        yaw=0.0,
    )


def sample_path_from_projection(
    path_xy: List[Tuple[float, float]],
    projection: PathProjection,
    distance: float,
) -> Optional[PathSample]:
    """투영점에서 path arc-length 기준 샘플 point/yaw 를 반환한다."""
    if not path_xy:
        return None
    if len(path_xy) == 1:
        return PathSample(point=path_xy[0], yaw=0.0, distance=0.0)

    cumulative = 0.0
    prev_x, prev_y = projection.point
    start_seg = max(0, min(projection.segment_idx, len(path_xy) - 2))
    last_yaw = projection.yaw

    for j in range(start_seg + 1, len(path_xy)):
        x, y = path_xy[j]
        seg_len = math.hypot(x - prev_x, y - prev_y)
        if seg_len <= 1e-9:
            prev_x, prev_y = x, y
            continue
        seg_yaw = math.atan2(y - prev_y, x - prev_x)
        last_yaw = seg_yaw
        if cumulative + seg_len >= distance:
            remain = distance - cumulative
            ratio = max(0.0, min(1.0, remain / seg_len))
            return PathSample(
                point=(
                    prev_x + ratio * (x - prev_x),
                    prev_y + ratio * (y - prev_y),
                ),
                yaw=seg_yaw,
                distance=max(0.0, distance),
            )
        cumulative += seg_len
        prev_x, prev_y = x, y

    return PathSample(point=path_xy[-1], yaw=last_yaw, distance=cumulative)


def pick_lookahead_from_projection(
    path_xy: List[Tuple[float, float]],
    projection: PathProjection,
    lookahead_dist: float,
) -> Optional[Tuple[float, float]]:
    """투영점에서 path arc-length 기준 lookahead 지점을 고른다."""
    sample = sample_path_from_projection(path_xy, projection, lookahead_dist)
    return sample.point if sample is not None else None


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def should_use_rejoin(
    path_offset: float,
    heading_error: float,
    was_rejoining: bool,
    entry_offset: float,
    exit_offset: float,
    exit_heading: float,
    predicted_offset: Optional[float] = None,
    predicted_exit_offset: Optional[float] = None,
) -> bool:
    entry_metric = path_offset
    if predicted_offset is not None:
        entry_metric = max(entry_metric, predicted_offset)
    if not was_rejoining:
        return entry_metric > entry_offset

    predicted_retain_offset = (
        entry_offset if predicted_exit_offset is None else predicted_exit_offset
    )
    if path_offset > entry_offset:
        return True
    if (predicted_offset is not None
            and predicted_offset > max(0.0, predicted_retain_offset)):
        return True
    return path_offset > exit_offset or abs(heading_error) > exit_heading


def should_force_rejoin_for_short_lookahead(
    local_lookahead_distance: float,
    effective_lookahead: float,
    dist_to_goal: float,
    goal_tolerance: float,
    min_distance: float,
    ratio: float,
    goal_margin: float,
) -> bool:
    """Detect a folded path lookahead that would make PP crawl/ALIGN in place."""
    if not math.isfinite(local_lookahead_distance):
        return False
    if effective_lookahead <= 0.0 or not math.isfinite(effective_lookahead):
        return False
    if dist_to_goal <= goal_tolerance + max(0.0, goal_margin):
        return False

    threshold = max(0.0, min_distance, effective_lookahead * max(0.0, ratio))
    return local_lookahead_distance < threshold


def predict_signed_path_offset(
    signed_offset: float,
    heading_error: float,
    speed: float,
    horizon: float,
) -> float:
    """Estimate short-horizon lateral error in the current path tangent frame."""
    forward_speed = max(0.0, speed)
    return signed_offset - forward_speed * math.sin(heading_error) * max(0.0, horizon)


def clamp_forward_velocity(v_cmd: float, allow_backward: bool) -> float:
    if allow_backward:
        return v_cmd
    return max(0.0, v_cmd)


def rate_limit_linear_velocity(
    current_v: float,
    target_v: float,
    accel_step: float,
    brake_step: float,
    allow_backward: bool,
) -> float:
    target_v = clamp_forward_velocity(target_v, allow_backward)
    accel_step = max(0.0, accel_step)
    brake_step = max(accel_step, brake_step)
    reducing = abs(target_v) < abs(current_v)
    crossing_zero = current_v * target_v < 0.0
    step = brake_step if reducing or crossing_zero else accel_step
    v_cmd = max(current_v - step, min(current_v + step, target_v))
    return clamp_forward_velocity(v_cmd, allow_backward)


def should_rearm_reached_with_path(
    reached: bool,
    robot_xy: Optional[Tuple[float, float]],
    path_goal_xy: Optional[Tuple[float, float]],
    min_goal_separation: float,
) -> bool:
    if not reached or robot_xy is None or path_goal_xy is None:
        return False
    dx = path_goal_xy[0] - robot_xy[0]
    dy = path_goal_xy[1] - robot_xy[1]
    return math.hypot(dx, dy) > max(0.0, min_goal_separation)


def speed_limit_from_clearance(
    clearance: float,
    stop_distance: float,
    acceleration: float,
) -> float:
    if math.isinf(clearance):
        return float("inf")
    free_distance = max(0.0, clearance - max(0.0, stop_distance))
    acceleration = max(0.0, acceleration)
    if acceleration <= 0.0 or free_distance <= 0.0:
        return 0.0
    return math.sqrt(2.0 * acceleration * free_distance)


def turn_demand_intensity(
    alpha: float,
    path_heading_error: float,
    w_target: float,
    w_max: float,
    brake_angle: float,
) -> float:
    angle_scale = max(abs(alpha), abs(path_heading_error)) / max(brake_angle, 1e-3)
    angular_scale = abs(w_target) / max(abs(w_max), 1e-3)
    return max(0.0, min(1.0, max(angle_scale, angular_scale)))


def turn_clearance_speed_limit(
    v_target: float,
    forward_clearance: float,
    stop_distance: float,
    acceleration: float,
    turn_intensity: float,
) -> float:
    if v_target <= 0.0 or turn_intensity <= 0.0 or math.isinf(forward_clearance):
        return max(0.0, v_target)
    clearance_limit = speed_limit_from_clearance(
        forward_clearance, stop_distance, acceleration)
    if clearance_limit >= v_target:
        return v_target
    t = max(0.0, min(1.0, turn_intensity))
    return max(0.0, (1.0 - t) * v_target + t * clearance_limit)


def near_wall_escape_adjustment(
    motion_clear: float,
    forward_clearance: float,
    curvature: float,
    escape_bias: float,
    stop_distance: float,
    slowdown_distance: float,
    escape_clearance: float,
    escape_speed: float,
    escape_turn: float,
    escape_max_curvature: float,
    acceleration: float,
) -> Tuple[float, float, bool]:
    """Small open-front nudge away from a side wall, bounded by stop distance."""
    if escape_speed <= 0.0 and escape_turn <= 0.0:
        return 0.0, 0.0, False
    if not math.isfinite(motion_clear):
        return 0.0, 0.0, False
    if not (math.isfinite(forward_clearance) or math.isinf(forward_clearance)):
        return 0.0, 0.0, False
    if motion_clear <= stop_distance or motion_clear >= escape_clearance:
        return 0.0, 0.0, False
    if forward_clearance <= slowdown_distance:
        return 0.0, 0.0, False
    if abs(curvature) > max(0.0, escape_max_curvature):
        return 0.0, 0.0, False
    if abs(escape_bias) <= 1e-6:
        return 0.0, 0.0, False

    denom = max(escape_clearance - stop_distance, 1e-3)
    strength = max(0.0, min(1.0, (escape_clearance - motion_clear) / denom))
    free_distance = max(0.0, motion_clear - stop_distance)
    brake_limited_speed = math.sqrt(2.0 * max(0.0, acceleration) * free_distance)
    speed_floor = min(max(0.0, escape_speed), brake_limited_speed)
    turn_bias = math.copysign(max(0.0, escape_turn) * strength, escape_bias)
    return speed_floor, turn_bias, True


def goal_approach_speed_limit(
    dist_to_goal: float,
    goal_tolerance: float,
    approach_distance: float,
    approach_speed: float,
) -> float:
    if approach_distance <= goal_tolerance or dist_to_goal >= approach_distance:
        return float("inf")
    ratio = max(0.0, dist_to_goal - goal_tolerance) / \
        max(approach_distance - goal_tolerance, 1e-3)
    return max(0.0, approach_speed) * ratio


def should_mark_goal_reached(
    dist_to_goal: float,
    goal_tolerance: float,
    goal_reached_epsilon: float,
    robot_speed: float,
    stopped_speed: float,
) -> bool:
    tolerance = max(0.0, goal_tolerance)
    if dist_to_goal <= tolerance + 1e-9:
        return True
    stop_band = tolerance + max(0.0, goal_reached_epsilon)
    return dist_to_goal <= stop_band and abs(robot_speed) <= max(0.0, stopped_speed)


def safe_forward_only_distance(
    forward_clearance: float,
    stop_distance: float,
    margin: float,
    desired_distance: float,
) -> float:
    if math.isinf(forward_clearance):
        return max(0.0, desired_distance)
    free_distance = forward_clearance - max(0.0, stop_distance) - max(0.0, margin)
    return max(0.0, min(max(0.0, desired_distance), free_distance))


def rate_limit_angular_velocity(
    current_w: float,
    target_w: float,
    accel_step: float,
    brake_step: float,
) -> float:
    accel_step = max(0.0, accel_step)
    brake_step = max(accel_step, brake_step)
    reducing = abs(target_w) < abs(current_w)
    crossing_zero = current_w * target_w < 0.0
    step = brake_step if reducing or crossing_zero else accel_step
    return max(current_w - step, min(current_w + step, target_w))


def should_finish_forward_only(
    now: float,
    until: float,
    dist_moved: float,
    target_dist: float,
    collision_near: bool,
    path_offset: Optional[float],
    min_dist_before_path_exit: float,
    path_rejoin_offset: float,
) -> Tuple[bool, str]:
    if collision_near:
        return True, "forward obstacle"
    if dist_moved >= target_dist:
        return True, f"target distance reached ({dist_moved:.2f}m)"
    if now >= until:
        return True, "timeout"
    if (path_offset is not None
            and dist_moved >= min_dist_before_path_exit
            and path_offset <= path_rejoin_offset):
        return True, f"path rejoined (offset={path_offset:.2f}m)"
    return False, ""


def should_release_align(
    alpha: float,
    is_rejoining: bool,
    release_angle: float,
    rejoin_release_angle: float,
    rotate_clearance: float,
    release_clearance: float,
) -> bool:
    threshold = rejoin_release_angle if is_rejoining else release_angle
    return abs(alpha) < threshold and rotate_clearance > release_clearance


def point_segment_distance(
    point: Tuple[float, float],
    seg_start: Tuple[float, float],
    seg_end: Tuple[float, float],
) -> float:
    px, py = point
    ax, ay = seg_start
    bx, by = seg_end
    sx = bx - ax
    sy = by - ay
    seg_len_sq = sx * sx + sy * sy
    if seg_len_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * sx + (py - ay) * sy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx = ax + t * sx
    cy = ay + t * sy
    return math.hypot(px - cx, py - cy)


def segment_clearance_margin(
    seg_start: Tuple[float, float],
    seg_end: Tuple[float, float],
    obstacle_points: List[Tuple[float, float]],
    robot_radius: float,
) -> float:
    if not obstacle_points:
        return float("inf")
    best = float("inf")
    for obstacle in obstacle_points:
        best = min(best, point_segment_distance(obstacle, seg_start, seg_end))
    return max(0.0, best - max(0.0, robot_radius))


def pure_pursuit_arc_clearance_margin(
    local_target: Tuple[float, float],
    obstacle_points: List[Tuple[float, float]],
    robot_radius: float,
    max_arc_length: float = 1.6,
    step: float = 0.05,
) -> float:
    """Clearance margin along the initial pure-pursuit arc to a local target."""
    if not obstacle_points:
        return float("inf")
    lx, ly = local_target
    L = math.hypot(lx, ly)
    if L < 1e-6:
        return float("inf")

    kappa = 2.0 * ly / (L * L)
    arc_length = min(max_arc_length, max(0.35, L))
    count = max(2, int(arc_length / max(0.02, step)))
    points: List[Tuple[float, float]] = []
    for i in range(1, count + 1):
        s = arc_length * i / count
        if abs(kappa) < 1e-6:
            x = s
            y = 0.0
        else:
            x = math.sin(kappa * s) / kappa
            y = (1.0 - math.cos(kappa * s)) / kappa
        points.append((x, y))

    center_dist = min_clearance_distance(points, obstacle_points)
    if center_dist == float("inf"):
        return float("inf")
    return max(0.0, center_dist - max(0.0, robot_radius))


def cluster_obstacle_points(
    obstacle_points: List[Tuple[float, float]],
    join_distance: float,
    min_points: int,
    max_radius: float,
) -> List[DynamicObstacleCluster]:
    """Cluster scan-ordered obstacle points with a small jump-distance rule."""
    if not obstacle_points:
        return []
    join_distance = max(0.05, join_distance)
    min_points = max(1, min_points)
    max_radius = max(0.05, max_radius)
    clusters: List[DynamicObstacleCluster] = []
    current: List[Tuple[float, float]] = []

    def flush() -> None:
        if len(current) < min_points:
            return
        cx = sum(p[0] for p in current) / len(current)
        cy = sum(p[1] for p in current) / len(current)
        radius = max(math.hypot(px - cx, py - cy) for px, py in current)
        if radius <= max_radius:
            clusters.append(DynamicObstacleCluster((cx, cy), radius, len(current)))

    prev: Optional[Tuple[float, float]] = None
    for point in obstacle_points:
        if prev is not None and math.hypot(
                point[0] - prev[0], point[1] - prev[1]) > join_distance:
            flush()
            current = []
        current.append(point)
        prev = point
    flush()
    return clusters


def classify_dynamic_motion(
    track: Optional[DynamicObstacleTrack],
    robot: RobotState,
    robot_radius: float,
    stopped_speed: float,
    moving_speed: float,
    approaching_speed: float,
    receding_speed: float,
    cpa_horizon: float,
    cpa_margin: float,
    min_age: int,
) -> DynamicMotionEstimate:
    """Classify one tracked obstacle by relative velocity and closest approach."""
    if track is None:
        return DynamicMotionEstimate("UNKNOWN")

    dx = track.x - robot.x
    dy = track.y - robot.y
    distance = math.hypot(dx, dy)
    speed = math.hypot(track.vx, track.vy)
    confidence = min(1.0, max(0.0, track.age / max(1, min_age + 2)))
    side = 1 if world_to_local((track.x, track.y), robot)[1] >= 0.0 else -1
    if track.age < max(1, min_age):
        return DynamicMotionEstimate(
            "UNKNOWN", track.track_id, distance, speed, 0.0,
            float("inf"), float("inf"), track.age, confidence, side, track.radius)

    robot_vx = robot.v * math.cos(robot.theta)
    robot_vy = robot.v * math.sin(robot.theta)
    rel_vx = track.vx - robot_vx
    rel_vy = track.vy - robot_vy
    if distance > 1e-6:
        closing_speed = -((dx * rel_vx + dy * rel_vy) / distance)
    else:
        closing_speed = 0.0

    rel_v2 = rel_vx * rel_vx + rel_vy * rel_vy
    if rel_v2 > 1e-6:
        t_cpa = -(dx * rel_vx + dy * rel_vy) / rel_v2
        t_cpa = max(0.0, min(max(0.0, cpa_horizon), t_cpa))
        cpa_x = dx + rel_vx * t_cpa
        cpa_y = dy + rel_vy * t_cpa
        d_cpa = (
            math.hypot(cpa_x, cpa_y)
            - max(0.0, robot_radius)
            - max(0.0, track.radius)
        )
    else:
        t_cpa = float("inf")
        d_cpa = distance - max(0.0, robot_radius) - max(0.0, track.radius)

    if speed <= stopped_speed:
        state = "STOPPED"
    elif (closing_speed >= approaching_speed and
          t_cpa <= cpa_horizon and
          d_cpa <= cpa_margin):
        state = "APPROACHING"
    elif closing_speed <= -receding_speed:
        state = "RECEDING"
    elif speed >= moving_speed:
        state = "CROSSING"
    else:
        state = "UNKNOWN"

    return DynamicMotionEstimate(
        state, track.track_id, distance, speed, closing_speed,
        t_cpa, d_cpa, track.age, confidence, side, track.radius)


def point_segment_projection(
    point: Tuple[float, float],
    seg_start: Tuple[float, float],
    seg_end: Tuple[float, float],
) -> Tuple[float, float, Tuple[float, float]]:
    px, py = point
    ax, ay = seg_start
    bx, by = seg_end
    sx = bx - ax
    sy = by - ay
    seg_len_sq = sx * sx + sy * sy
    if seg_len_sq <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0, seg_start
    t = ((px - ax) * sx + (py - ay) * sy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx = ax + t * sx
    cy = ay + t * sy
    return math.hypot(px - cx, py - cy), t, (cx, cy)


def detect_path_corridor_blockage(
    path_xy: List[Tuple[float, float]],
    robot: RobotState,
    projection: PathProjection,
    obstacles_local: List[Tuple[float, float]],
    corridor_width: float,
    check_distance: float,
    min_points: int,
    step: float,
) -> DynamicPathBlockage:
    """Detect whether LiDAR points occupy the near-term corridor around global path."""
    if not path_xy or not obstacles_local or check_distance <= 0.0:
        return DynamicPathBlockage(False, float("inf"), 0, 0.0, 0.0)

    corridor_width = max(0.05, corridor_width)
    min_points = max(1, min_points)
    step = max(0.05, step)

    samples: List[Tuple[float, Tuple[float, float]]] = []
    count = int(check_distance / step) + 1
    for i in range(count + 1):
        distance = min(check_distance, i * step)
        sample = sample_path_from_projection(path_xy, projection, distance)
        if sample is None:
            continue
        local_point = world_to_local(sample.point, robot)
        if not samples or math.hypot(
                local_point[0] - samples[-1][1][0],
                local_point[1] - samples[-1][1][1]) > 1e-3:
            samples.append((distance, local_point))

    if len(samples) < 2:
        return DynamicPathBlockage(False, float("inf"), 0, 0.0, 0.0)

    hit_count = 0
    min_along = float("inf")
    min_margin = 0.0
    side_bias = 0.0
    seen: set[int] = set()

    for obs_idx, (ox, oy) in enumerate(obstacles_local):
        if ox < -0.20:
            continue
        if math.hypot(ox, oy) > check_distance + corridor_width + 0.5:
            continue

        best_dist = float("inf")
        best_along = float("inf")
        best_side = 0.0
        for (d0, p0), (d1, p1) in zip(samples, samples[1:]):
            dist, t, _ = point_segment_projection((ox, oy), p0, p1)
            if dist >= best_dist:
                continue
            sx = p1[0] - p0[0]
            sy = p1[1] - p0[1]
            side = sx * (oy - p0[1]) - sy * (ox - p0[0])
            best_dist = dist
            best_along = d0 + t * max(0.0, d1 - d0)
            best_side = 1.0 if side > 0.0 else -1.0 if side < 0.0 else 0.0

        if best_dist <= corridor_width and obs_idx not in seen:
            seen.add(obs_idx)
            hit_count += 1
            min_along = min(min_along, best_along)
            min_margin = max(min_margin, corridor_width - best_dist)
            side_bias += best_side / max(0.3, math.hypot(ox, oy))

    if hit_count == 0:
        return DynamicPathBlockage(False, float("inf"), 0, 0.0, 0.0)

    side_bias /= max(1, hit_count)
    return DynamicPathBlockage(
        blocked=hit_count >= min_points,
        distance=min_along,
        count=hit_count,
        side_bias=side_bias,
        min_margin=min_margin,
    )


def choose_dynamic_close_bypass_target(
    path_xy: List[Tuple[float, float]],
    robot: RobotState,
    projection: PathProjection,
    blockage: DynamicPathBlockage,
    obstacles_local: List[Tuple[float, float]],
    robot_radius: float,
    offsets: List[float],
    min_clearance: float,
    min_lookahead: float,
    max_lookahead: float,
    previous_side: int = 0,
    side_switch_penalty: float = 0.0,
) -> Optional[DynamicAvoidTarget]:
    """Pick a short local sidestep when path-based bypass candidates are too tight."""
    if not offsets:
        return None

    min_clearance = max(0.0, min_clearance)
    min_lookahead = max(0.10, min_lookahead)
    max_lookahead = max(min_lookahead, max_lookahead)
    desired_forward = min(
        max_lookahead,
        max(0.45, blockage.distance + 0.55),
    )
    raw_forwards = [
        0.45,
        0.65,
        min_lookahead,
        1.20,
        desired_forward,
    ]
    forward_candidates = sorted({
        max(0.25, min(max_lookahead, float(distance)))
        for distance in raw_forwards
    })

    side_candidates = [previous_side, -previous_side] if previous_side else [1, -1]
    if previous_side == 0 and blockage.side_bias > 0.0:
        side_candidates = [-1, 1]
    elif previous_side == 0 and blockage.side_bias < 0.0:
        side_candidates = [1, -1]

    ordered_sides: List[int] = []
    for side in side_candidates:
        if side != 0 and side not in ordered_sides:
            ordered_sides.append(side)

    best: Optional[DynamicAvoidTarget] = None
    fallback: Optional[DynamicAvoidTarget] = None
    for side in ordered_sides:
        for forward in forward_candidates:
            sample_distance = min(
                max_lookahead,
                max(min_lookahead, forward + 0.50),
            )
            sample = sample_path_from_projection(
                path_xy, projection, sample_distance)
            target_yaw = sample.yaw if sample is not None else robot.theta
            for offset in offsets:
                lx = forward
                ly = side * offset
                L = math.hypot(lx, ly)
                if L < 0.10:
                    continue

                direct_clear = segment_clearance_margin(
                    (0.0, 0.0), (lx, ly), obstacles_local, robot_radius)
                arc_clear = pure_pursuit_arc_clearance_margin(
                    (lx, ly), obstacles_local, robot_radius)
                clearance = min(direct_clear, arc_clear)
                alpha = math.atan2(ly, lx)
                curvature = abs(2.0 * ly / (L * L)) if L >= 1e-3 else float("inf")
                approach_yaw = normalize_angle(robot.theta + alpha)
                arrival_error = normalize_angle(target_yaw - approach_yaw)
                distance_error = abs(forward - desired_forward)
                clearance_penalty = 0.0
                if min_clearance > 0.0 and clearance < min_clearance:
                    ratio = (min_clearance - max(0.0, clearance)) / min_clearance
                    clearance_penalty = 5.5 * ratio * ratio
                switch_penalty = (
                    side_switch_penalty
                    if previous_side and side != previous_side else 0.0
                )
                side_bias_penalty = 0.45 * max(0.0, side * blockage.side_bias)
                score = (
                    0.75 * abs(alpha)
                    + 0.45 * abs(arrival_error)
                    + 0.18 * curvature
                    + 0.16 * distance_error
                    + 0.05 * offset
                    + clearance_penalty
                    + switch_penalty
                    + side_bias_penalty
                )
                target = DynamicAvoidTarget(
                    point=local_to_world((lx, ly), robot),
                    yaw=target_yaw,
                    distance=L,
                    alpha=alpha,
                    arrival_error=arrival_error,
                    curvature=curvature,
                    clearance=clearance,
                    rejoin_clearance=float("inf"),
                    side=side,
                    offset=offset,
                    blocked_distance=blockage.distance,
                    score=score,
                    mode="close_sidestep",
                )

                if fallback is None or target.score < fallback.score:
                    fallback = target
                if clearance < min_clearance:
                    continue
                if best is None or target.score < best.score:
                    best = target

    if best is not None:
        return best

    # close-sidestep fallback도 stop margin 아래까지 허용하면
    # 다음 tick에서 STOPPED_NEAR_WALL로 굳기 쉽다. 기본 min_clearance=0.55 기준
    # 0.37m 부근을 최저선으로 둔다.
    soft_floor = max(
        0.20,
        min_clearance * 0.67,
    )
    if fallback is not None and fallback.clearance >= soft_floor:
        return fallback
    return None


def choose_dynamic_avoid_target(
    path_xy: List[Tuple[float, float]],
    robot: RobotState,
    projection: PathProjection,
    blockage: DynamicPathBlockage,
    obstacles_local: List[Tuple[float, float]],
    robot_radius: float,
    lateral_offsets: List[float],
    min_clearance: float,
    min_lookahead: float,
    max_lookahead: float,
    rejoin_distance: float,
    step: float,
    previous_side: int = 0,
    side_switch_penalty: float = 0.0,
) -> Optional[DynamicAvoidTarget]:
    """Choose a side-offset bypass target and a later rejoin point on the path."""
    if not path_xy or not blockage.blocked:
        return None

    offsets = sorted({max(0.10, float(offset)) for offset in lateral_offsets})
    if not offsets:
        return None

    min_clearance = max(0.0, min_clearance)
    min_lookahead = max(0.10, min_lookahead)
    max_lookahead = max(min_lookahead, max_lookahead)
    step = max(0.05, step)
    desired_distance = min(
        max_lookahead,
        max(min_lookahead, blockage.distance + max(0.0, rejoin_distance)),
    )

    best: Optional[DynamicAvoidTarget] = None
    fallback: Optional[DynamicAvoidTarget] = None
    side_candidates = [previous_side, -previous_side] if previous_side else [1, -1]
    if previous_side == 0 and blockage.side_bias > 0.0:
        side_candidates = [-1, 1]
    elif previous_side == 0 and blockage.side_bias < 0.0:
        side_candidates = [1, -1]

    min_candidate_distance = min(
        max_lookahead,
        max(0.20, blockage.distance + 0.25),
    )
    count = int((max_lookahead - min_lookahead) / step) + 1
    for i in range(count + 1):
        distance = min(max_lookahead, min_lookahead + i * step)
        if distance + 1e-6 < min_candidate_distance:
            continue
        sample = sample_path_from_projection(path_xy, projection, distance)
        if sample is None:
            continue

        normal_x = -math.sin(sample.yaw)
        normal_y = math.cos(sample.yaw)
        rejoin_lx, rejoin_ly = world_to_local(sample.point, robot)
        for side in side_candidates:
            if side == 0:
                continue
            for offset in offsets:
                target_point = (
                    sample.point[0] + side * offset * normal_x,
                    sample.point[1] + side * offset * normal_y,
                )
                lx, ly = world_to_local(target_point, robot)
                if lx <= 0.05:
                    continue
                L = math.hypot(lx, ly)
                if L < 0.10:
                    continue

                direct_clear = segment_clearance_margin(
                    (0.0, 0.0), (lx, ly), obstacles_local, robot_radius)
                arc_clear = pure_pursuit_arc_clearance_margin(
                    (lx, ly), obstacles_local, robot_radius)
                rejoin_clear = segment_clearance_margin(
                    (lx, ly), (rejoin_lx, rejoin_ly), obstacles_local,
                    robot_radius)
                # 동적 회피는 옆 차선으로 먼저 빠지고, 장애물이 clear된 뒤
                # 정상 path tracking으로 재합류하는 rolling two-step이다.
                # 여기서 재합류선을 hard gate로 두면 path 위 장애물 때문에
                # 출발 가능한 side target까지 모두 폐기될 수 있다.
                clearance = min(direct_clear, arc_clear)
                alpha = math.atan2(ly, lx)
                approach_yaw = math.atan2(
                    target_point[1] - robot.y,
                    target_point[0] - robot.x,
                )
                arrival_error = normalize_angle(sample.yaw - approach_yaw)
                curvature = abs(2.0 * ly / (L * L)) if L >= 1e-3 else float("inf")
                distance_error = abs(distance - desired_distance)
                clearance_penalty = 0.0
                if min_clearance > 0.0 and clearance < min_clearance:
                    ratio = (min_clearance - max(0.0, clearance)) / min_clearance
                    clearance_penalty = 5.0 * ratio * ratio
                rejoin_clearance_penalty = 0.0
                if min_clearance > 0.0 and rejoin_clear < min_clearance:
                    ratio = (min_clearance - max(0.0, rejoin_clear)) / min_clearance
                    rejoin_clearance_penalty = 0.85 * ratio * ratio
                switch_penalty = (
                    side_switch_penalty
                    if previous_side and side != previous_side else 0.0
                )
                side_bias_penalty = 0.35 * max(0.0, side * blockage.side_bias)
                score = (
                    0.85 * abs(alpha)
                    + 0.85 * abs(arrival_error)
                    + 0.25 * curvature
                    + 0.20 * distance_error
                    + 0.08 * offset
                    + clearance_penalty
                    + rejoin_clearance_penalty
                    + switch_penalty
                    + side_bias_penalty
                )
                target = DynamicAvoidTarget(
                    point=target_point,
                    yaw=sample.yaw,
                    distance=distance,
                    alpha=alpha,
                    arrival_error=arrival_error,
                    curvature=curvature,
                    clearance=clearance,
                    rejoin_clearance=rejoin_clear,
                    side=side,
                    offset=offset,
                    blocked_distance=blockage.distance,
                    score=score,
                    mode="side_lane",
                )

                if fallback is None or target.score < fallback.score:
                    fallback = target
                if clearance < min_clearance:
                    continue
                if best is None or target.score < best.score:
                    best = target

    if best is not None:
        return best
    close_target = choose_dynamic_close_bypass_target(
        path_xy=path_xy,
        robot=robot,
        projection=projection,
        blockage=blockage,
        obstacles_local=obstacles_local,
        robot_radius=robot_radius,
        offsets=offsets,
        min_clearance=min_clearance,
        min_lookahead=min_lookahead,
        max_lookahead=max_lookahead,
        previous_side=previous_side,
        side_switch_penalty=side_switch_penalty,
    )
    if close_target is not None:
        return close_target
    # Path 기반 side-lane fallback은 long chord가 장애물 옆을 스치기 쉬워
    # close-sidestep보다 보수적으로 받아들인다.
    soft_floor = max(
        0.20,
        min_clearance * 0.67,
    )
    if fallback is not None and fallback.clearance >= soft_floor:
        return fallback
    return None


def choose_rejoin_target(
    path_xy: List[Tuple[float, float]],
    robot: RobotState,
    projection: PathProjection,
    min_lookahead: float,
    max_lookahead: float,
    step: float,
    heading_weight: float,
    distance_weight: float,
    curvature_weight: float,
    effective_offset: Optional[float] = None,
    obstacles_local: Optional[List[Tuple[float, float]]] = None,
    robot_radius: float = 0.0,
    clearance_min: float = 0.0,
    clearance_weight: float = 0.0,
) -> Optional[RejoinTarget]:
    """가장 가까운 점이 아니라, 작은 조향으로 합류 가능한 미래 path 점을 고른다."""
    if not path_xy:
        return None

    step = max(0.05, step)
    min_lookahead = max(0.0, min_lookahead)
    max_lookahead = max(min_lookahead, max_lookahead)
    count = int((max_lookahead - min_lookahead) / step) + 1
    offset_for_distance = projection.offset
    if effective_offset is not None:
        offset_for_distance = max(offset_for_distance, effective_offset)
    desired_distance = min(
        max_lookahead,
        max(
            min_lookahead,
            0.8 + 2.2 * offset_for_distance + 0.5 * max(0.0, robot.v),
        ),
    )

    best: Optional[RejoinTarget] = None
    fallback: Optional[RejoinTarget] = None

    for i in range(count + 1):
        distance = min(max_lookahead, min_lookahead + i * step)
        sample = sample_path_from_projection(path_xy, projection, distance)
        if sample is None:
            continue

        lx, ly = world_to_local(sample.point, robot)
        alpha = math.atan2(ly, lx)
        dx = sample.point[0] - robot.x
        dy = sample.point[1] - robot.y
        approach_yaw = math.atan2(dy, dx)
        arrival_error = normalize_angle(sample.yaw - approach_yaw)
        L = math.hypot(lx, ly)
        curvature = abs(2.0 * ly / (L * L)) if L >= 1e-3 else float("inf")
        distance_error = abs(sample.distance - desired_distance)
        clearance = segment_clearance_margin(
            (0.0, 0.0),
            (lx, ly),
            obstacles_local or [],
            robot_radius,
        )
        clearance_penalty = 0.0
        if (clearance_weight > 0.0
                and clearance_min > 0.0
                and clearance < clearance_min):
            ratio = (clearance_min - max(0.0, clearance)) / clearance_min
            clearance_penalty = clearance_weight * ratio * ratio
        score = (
            abs(alpha)
            + heading_weight * abs(arrival_error)
            + curvature_weight * curvature
            + distance_weight * distance_error
            + clearance_penalty
        )
        target = RejoinTarget(
            point=sample.point,
            yaw=sample.yaw,
            distance=sample.distance,
            alpha=alpha,
            arrival_error=arrival_error,
            curvature=curvature,
            clearance=clearance,
            desired_distance=desired_distance,
            score=score,
        )

        if fallback is None or target.score < fallback.score:
            fallback = target
        if lx <= 0.05:
            continue
        if best is None or target.score < best.score:
            best = target

    return best if best is not None else fallback


def pick_lookahead_point(
    path_xy: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    lookahead_dist: float,
    start_idx: int = 0,
) -> Optional[Tuple[float, float]]:
    if not path_xy:
        return None
    nearest_idx = find_nearest_idx(path_xy, robot_xy, start_idx)
    cumulative = 0.0
    prev_x, prev_y = path_xy[nearest_idx]
    for j in range(nearest_idx + 1, len(path_xy)):
        x, y = path_xy[j]
        cumulative += math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
        if cumulative >= lookahead_dist:
            return (x, y)
        prev_x, prev_y = x, y
    return path_xy[-1]


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_local(world_xy: Tuple[float, float],
                   robot: RobotState) -> Tuple[float, float]:
    dx = world_xy[0] - robot.x
    dy = world_xy[1] - robot.y
    cos_t = math.cos(-robot.theta)
    sin_t = math.sin(-robot.theta)
    return (cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy)


def local_to_world(local_xy: Tuple[float, float],
                   robot: RobotState) -> Tuple[float, float]:
    lx, ly = local_xy
    cos_t = math.cos(robot.theta)
    sin_t = math.sin(robot.theta)
    return (
        robot.x + cos_t * lx - sin_t * ly,
        robot.y + sin_t * lx + cos_t * ly,
    )


def occupancy_grid_world_to_cell(
    grid: OccupancyGrid,
    world_xy: Tuple[float, float],
) -> Optional[Tuple[int, int]]:
    info = grid.info
    resolution = float(info.resolution)
    if resolution <= 0.0:
        return None

    origin = info.origin
    dx = world_xy[0] - origin.position.x
    dy = world_xy[1] - origin.position.y
    q = origin.orientation
    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
    cos_t = math.cos(-yaw)
    sin_t = math.sin(-yaw)
    mx = cos_t * dx - sin_t * dy
    my = sin_t * dx + cos_t * dy
    col = int(math.floor(mx / resolution))
    row = int(math.floor(my / resolution))
    if col < 0 or row < 0 or col >= int(info.width) or row >= int(info.height):
        return None
    return col, row


def occupancy_grid_has_static_obstacle_near(
    grid: OccupancyGrid,
    world_xy: Tuple[float, float],
    radius: float,
    occupied_threshold: int,
    unknown_as_static: bool = False,
) -> bool:
    cell = occupancy_grid_world_to_cell(grid, world_xy)
    if cell is None:
        return False

    info = grid.info
    resolution = float(info.resolution)
    radius = max(0.0, float(radius))
    radius_cells = max(0, int(math.ceil(radius / resolution)))
    col, row = cell
    width = int(info.width)
    height = int(info.height)
    threshold = max(0, min(100, int(occupied_threshold)))

    for rr in range(max(0, row - radius_cells),
                    min(height, row + radius_cells + 1)):
        dy = (rr - row) * resolution
        for cc in range(max(0, col - radius_cells),
                        min(width, col + radius_cells + 1)):
            dx = (cc - col) * resolution
            if math.hypot(dx, dy) > radius + 0.5 * resolution:
                continue
            idx = rr * width + cc
            if idx >= len(grid.data):
                continue
            value = int(grid.data[idx])
            if value >= threshold or (unknown_as_static and value < 0):
                return True
    return False


# ═══════════════════════════════════════════════════════════════════════
# DWA Planner Node
# ═══════════════════════════════════════════════════════════════════════

class DwaPlannerNode(Node):
    """DWA Local Planner — NavState 기반 명시적 상태 머신 (2026-05-29 리팩토링).

    구독:
        /odometry/filtered   EKF 출력 (1순위)
        /odom                DiffDrive plugin 출력 (fallback)
        /global_path         A* 의 전역 경로 (frame: map)
        /lidar               장애물 점

    발행:
        /cmd_vel             제어 명령 (Twist)
        /dwa/trajectories    후보 trajectory (MarkerArray, base_link frame)
        /dwa/best_trajectory 선택된 trajectory (Marker, base_link frame)
        /dwa/status          상태 String (1Hz)
    """

    LOCAL_FRAME  = "odom_filtered"
    GLOBAL_FRAME = "map"
    ROBOT_FRAME  = "base_footprint"

    # ───────────────────────────────────────────────────────────────
    # __init__
    # ───────────────────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__("dwa_planner")

        # ── 파라미터 선언 (기존과 완전 동일) ──────────────────────────
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("v_min", -1.0)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("a_max", 1.0)
        self.declare_parameter("v_brake_a_max", 5.0)
        self.declare_parameter("alpha_max", 1.5)
        self.declare_parameter("a_lat_max", 1.0)
        self.declare_parameter("sample_v_n", 11)
        self.declare_parameter("sample_w_n", 21)
        self.declare_parameter("predict_horizon", 1.0)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("weight_heading", 0.6)
        self.declare_parameter("weight_clearance", 1.2)
        self.declare_parameter("weight_velocity", 0.2)
        self.declare_parameter("weight_path_tangent", 0.8)
        self.declare_parameter("path_tangent_lookahead", 10)
        self.declare_parameter("lookahead_dist", 0.65)
        self.declare_parameter("lookahead_time", 0.55)
        self.declare_parameter("path_heading_gain", 0.6)
        self.declare_parameter("path_cross_track_gain", 0.8)
        self.declare_parameter("path_error_slowdown_offset", 0.25)
        self.declare_parameter("path_error_min_speed_scale", 0.42)
        self.declare_parameter("path_error_predict_time", 0.55)
        self.declare_parameter("rejoin_entry_offset", 0.30)
        self.declare_parameter("rejoin_exit_offset", 0.22)
        self.declare_parameter("rejoin_exit_heading", 0.52)
        self.declare_parameter("rejoin_min_lookahead", 0.80)
        self.declare_parameter("rejoin_max_lookahead", 3.50)
        self.declare_parameter("rejoin_step", 0.20)
        self.declare_parameter("rejoin_heading_weight", 1.2)
        self.declare_parameter("rejoin_distance_weight", 0.12)
        self.declare_parameter("rejoin_curvature_weight", 0.18)
        self.declare_parameter("rejoin_clearance_min", 0.80)
        self.declare_parameter("rejoin_clearance_weight", 2.8)
        self.declare_parameter("rejoin_cross_track_gain_scale", 0.42)
        self.declare_parameter("dynamic_avoid_enabled", True)
        self.declare_parameter("dynamic_path_check_distance", 3.2)
        self.declare_parameter("dynamic_path_corridor_width", 0.50)
        self.declare_parameter("dynamic_path_corridor_step", 0.25)
        self.declare_parameter("dynamic_path_min_block_points", 2)
        self.declare_parameter("dynamic_block_enter_ticks", 2)
        self.declare_parameter("dynamic_block_exit_ticks", 5)
        self.declare_parameter("dynamic_avoid_min_clearance", 0.55)
        self.declare_parameter("dynamic_avoid_lateral_offsets",
                               [0.55, 0.75, 0.95, 1.15])
        self.declare_parameter("dynamic_avoid_min_lookahead", 0.90)
        self.declare_parameter("dynamic_avoid_max_lookahead", 3.40)
        self.declare_parameter("dynamic_avoid_rejoin_distance", 1.55)
        self.declare_parameter("dynamic_avoid_step", 0.25)
        self.declare_parameter("dynamic_avoid_side_switch_penalty", 2.0)
        self.declare_parameter("dynamic_avoid_side_hold_sec", 1.5)
        self.declare_parameter("dynamic_avoid_cross_track_gain_scale", 0.15)
        self.declare_parameter("dynamic_static_filter_enabled", True)
        self.declare_parameter("dynamic_static_filter_radius", 0.30)
        self.declare_parameter("dynamic_static_filter_occupied_threshold", 65)
        self.declare_parameter("dynamic_static_filter_unknown_as_static", False)
        self.declare_parameter("dynamic_static_filter_tf_timeout", 0.01)
        self.declare_parameter("dynamic_track_cluster_distance", 0.35)
        self.declare_parameter("dynamic_track_min_points", 3)
        self.declare_parameter("dynamic_track_max_radius", 0.85)
        self.declare_parameter("dynamic_track_association_distance", 0.90)
        self.declare_parameter("dynamic_track_timeout", 1.0)
        self.declare_parameter("dynamic_motion_min_age", 2)
        self.declare_parameter("dynamic_motion_stopped_speed", 0.08)
        self.declare_parameter("dynamic_motion_moving_speed", 0.15)
        self.declare_parameter("dynamic_motion_approach_speed", 0.18)
        self.declare_parameter("dynamic_motion_recede_speed", 0.12)
        self.declare_parameter("dynamic_motion_cpa_horizon", 2.5)
        self.declare_parameter("dynamic_motion_cpa_margin", 0.35)
        self.declare_parameter("dynamic_approach_reverse_enabled", True)
        self.declare_parameter("dynamic_approach_reverse_clearance", 0.80)
        self.declare_parameter("dynamic_approach_reverse_speed", 0.16)
        self.declare_parameter("dynamic_approach_turn_speed", 0.45)
        self.declare_parameter("dynamic_layer_enabled", True)
        self.declare_parameter("dynamic_layer_prefer_global_replan", True)
        self.declare_parameter("dynamic_layer_topic", "/dynamic_obstacle_layer")
        self.declare_parameter("dynamic_layer_publish_period", 0.50)
        self.declare_parameter("dynamic_layer_ttl_sec", 300.0)
        self.declare_parameter("dynamic_layer_min_hold_sec", 5.0)
        self.declare_parameter("dynamic_layer_clear_confirm_sec", 2.0)
        self.declare_parameter("dynamic_layer_radius_margin", 0.95)
        self.declare_parameter("dynamic_layer_min_radius", 0.85)
        self.declare_parameter("dynamic_layer_max_radius", 2.25)
        self.declare_parameter("dynamic_layer_observation_range", 6.0)
        self.declare_parameter("dynamic_layer_clear_range", 7.0)
        self.declare_parameter("dynamic_layer_prediction_horizon", 4.0)
        self.declare_parameter("dynamic_layer_prediction_max_distance", 3.00)
        self.declare_parameter("dynamic_layer_prediction_speed_max", 1.50)
        self.declare_parameter("dynamic_layer_trail_ttl_sec", 300.0)
        self.declare_parameter("dynamic_layer_trail_min_distance", 0.25)
        self.declare_parameter("dynamic_layer_trail_max_points", 80)
        self.declare_parameter("dynamic_layer_escape_distance", 1.20)
        self.declare_parameter("dynamic_layer_escape_t_cpa", 1.00)
        self.declare_parameter("dynamic_layer_occupied_value", 100)
        self.declare_parameter("rejoin_predicted_exit_offset", 0.42)
        self.declare_parameter("rejoin_align_angle_thresh", 1.75)
        self.declare_parameter("short_lookahead_rejoin_min_distance", 0.35)
        self.declare_parameter("short_lookahead_rejoin_ratio", 0.55)
        self.declare_parameter("short_lookahead_goal_margin", 1.0)
        self.declare_parameter("max_clearance", 1.0)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("goal_reached_epsilon", 0.03)
        self.declare_parameter("goal_reached_stopped_speed", 0.03)
        self.declare_parameter("goal_approach_distance", 1.20)
        self.declare_parameter("goal_approach_speed", 0.80)
        self.declare_parameter("goal_align_stop_distance", 1.50)
        self.declare_parameter("clearance_slowdown_distance", 0.80)
        self.declare_parameter("clearance_stop_distance", 0.30)
        self.declare_parameter("turn_clearance_brake_angle", 0.45)
        self.declare_parameter("near_wall_creep_speed", 0.12)
        self.declare_parameter("near_wall_creep_min_clearance", 0.60)
        self.declare_parameter("rejoin_creep_min_clearance", 0.70)
        self.declare_parameter("near_wall_escape_clearance", 0.45)
        self.declare_parameter("near_wall_escape_speed", 0.28)
        self.declare_parameter("near_wall_escape_turn", 0.22)
        self.declare_parameter("near_wall_escape_max_curvature", 0.80)
        self.declare_parameter("align_angle_thresh", 1.10)
        self.declare_parameter("align_angle_exit", 0.262)
        self.declare_parameter("align_kp", 1.5)
        self.declare_parameter("align_kd", 0.5)
        self.declare_parameter("align_v_blend_max", 0.55)
        self.declare_parameter("align_drive_angle", 1.57)
        self.declare_parameter("align_release_angle", 0.70)
        self.declare_parameter("rejoin_align_release_angle", 0.95)
        self.declare_parameter("align_release_clearance", 0.60)
        self.declare_parameter("align_cooldown", 1.0)
        # 2026-05-31 추가 (SW · dwa-ys):
        #   P4 ALIGN 진입 시간 hysteresis — |alpha|>thresh 가 N틱 연속일 때만 진입.
        #   (P3 v_target LPF 는 2026-05-31 제거 — 상향 재가속 lag 로 주행이 둔하다는
        #    사용자 피드백. CLAUDE.md §7.2.0 #4 "후퇴 시 revert".)
        self.declare_parameter("align_trigger_ticks", 3)      # TODO: 시뮬 측정 후 확정(미확정 초안)
        self.declare_parameter("w_min_rotate", 0.3)
        self.declare_parameter("w_brake_alpha_max", 6.0)
        self.declare_parameter("allow_backward", False)
        self.declare_parameter("max_path_offset", 1.0)
        self.declare_parameter("recovery_path_accept_offset", 1.8)
        self.declare_parameter("recovery_path_accept_duration", 5.0)
        self.declare_parameter("path_lost_offset", 1.8)
        self.declare_parameter("stuck_recovery_sec", 1.5)
        self.declare_parameter("recovery_cooldown", 3.0)
        self.declare_parameter("spin_duration", 2.0)       # spin recovery 지속 시간
        self.declare_parameter("forward_only_dist", 0.35)
        self.declare_parameter("forward_only_timeout", 1.2)
        self.declare_parameter("forward_only_speed_scale", 0.35)
        self.declare_parameter("forward_only_settle_w", 0.20)
        self.declare_parameter("forward_only_min_dist", 0.10)
        self.declare_parameter("forward_only_rejoin_offset", 0.25)
        self.declare_parameter("spin_forward_clearance_margin", 0.15)
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("hard_collision_distance", 0.05)
        self.declare_parameter("safety_distance", 0.30)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("scan_range_max", 25.0)
        self.declare_parameter("lidar_offset_x", 0.25)
        self.declare_parameter("lidar_offset_y", 0.0)
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_fallback_topic", "/odom")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("goal_dedup_dist", 0.10)
        self.declare_parameter("goal_dedup_yaw", 0.10)
        self.declare_parameter("reached_new_path_rearm_dist", 0.75)
        self.declare_parameter("scan_topic", "/lidar")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("state_log_period", 1.0)
        self.declare_parameter("candidate_log_period", 1.0)
        self.declare_parameter("path_tf_timeout", 1.0)

        self._load_params()

        # ── ROS 입출력 상태 ───────────────────────────────────────────
        self._state: Optional[RobotState] = None
        self._path_local: Optional[Path] = None
        self._latest_scan: Optional[LaserScan] = None
        self._static_map: Optional[OccupancyGrid] = None
        self._last_odom_time: Optional[float] = None
        self._last_odom_was_fallback = False
        self._path_warn_logged = False
        self._goal_version = 0
        self._path_goal_version = 0
        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._last_goal_yaw: Optional[float] = None
        self._path_goal_xy_global: Optional[Tuple[float, float]] = None

        # ── NavState 머신 ─────────────────────────────────────────────
        self._nav_state: NavState = NavState.NORMAL

        # NORMAL/ALIGN 공통
        self._path_progress_idx = 0
        self._last_kappa: float = 0.0
        self._dynamic_block_ticks = 0
        self._dynamic_clear_ticks = 0
        self._dynamic_avoid_side = 0
        self._dynamic_avoid_until = 0.0
        self._dynamic_tracks: dict[int, DynamicObstacleTrack] = {}
        self._next_dynamic_track_id = 1
        self._dynamic_layer_blocks: dict[int, DynamicObstacleMapBlock] = {}
        self._next_dynamic_layer_block_id = 1
        self._last_dynamic_layer_publish_time = -float("inf")
        self._last_dynamic_layer_active = False

        # ALIGN 전용
        self._in_align_mode = False      # 하위 호환 (path 콜백에서 리셋)
        self._align_cooldown_until = 0.0
        self._align_trigger_count = 0    # P4: ALIGN 진입 연속 tick 카운터 (2026-05-31)

        # SPIN 전용
        self._spin_until = 0.0
        self._spin_direction = 1.0       # +1 왼쪽, -1 오른쪽

        # FORWARD_ONLY 전용 (spin 완료 후 짧은 전진)
        self._forward_only_until = 0.0   # 전진 종료 시각
        self._forward_only_dist  = 0.0   # 목표 전진 거리 [m]
        self._forward_only_start_x = 0.0 # 전진 시작 위치 x
        self._forward_only_start_y = 0.0 # 전진 시작 위치 y

        # RECOVERY 공통 (SPIN/FORWARD_ONLY 완료 후 cooldown)
        self._recovery_cooldown_until = 0.0
        self._relaxed_path_accept_until = 0.0
        self._stuck_counter = 0

        # REACHED
        self._reached = False

        # 진단
        self._state_log_counter = 0
        self._candidate_log_counter = 0

        # ── TF ────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        path_qos = QoSProfile(depth=10,
                              reliability=QoSReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, self.p_odom_topic, self._on_odom, 10)
        self.create_subscription(
            Odometry, self.p_odom_fallback_topic, self._on_odom_fallback, 10)
        self.create_subscription(
            PoseStamped, self.p_goal_pose_topic, self._on_goal_pose, 10)
        self.create_subscription(
            Path, self.p_global_path_topic, self._on_global_path, path_qos)
        self.create_subscription(
            LaserScan, self.p_scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(
            OccupancyGrid, self.p_map_topic, self._on_map, map_qos)

        # ── 발행 ─────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, self.p_cmd_vel_topic, 10)
        self._traj_pub = self.create_publisher(MarkerArray, "/dwa/trajectories", 10)
        self._best_pub = self.create_publisher(Marker, "/dwa/best_trajectory", 10)
        self._status_pub = self.create_publisher(String, "/dwa/status", 10)
        self._dynamic_layer_pub = self.create_publisher(
            OccupancyGrid, self.p_dynamic_layer_topic, map_qos)

        # ── 타이머 ───────────────────────────────────────────────────
        period = 1.0 / max(self.p_control_rate, 1.0)
        self._control_timer = self.create_timer(period, self._control_loop)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"DWA Planner 시작 (NavState 머신 2026-05-29) — "
            f"v_max={self.p_v_max} w_max={self.p_w_max} "
            f"control_rate={self.p_control_rate}Hz"
        )

    # ───────────────────────────────────────────────────────────────
    # 파라미터 로드
    # ───────────────────────────────────────────────────────────────
    def _load_params(self) -> None:
        gp = self.get_parameter
        self.p_v_max                    = gp("v_max").value
        self.p_v_min                    = gp("v_min").value
        self.p_w_max                    = gp("w_max").value
        self.p_a_max                    = gp("a_max").value
        self.p_v_brake_a_max            = gp("v_brake_a_max").value
        self.p_alpha_max                = gp("alpha_max").value
        self.p_a_lat_max                = gp("a_lat_max").value
        self.p_sample_v_n               = gp("sample_v_n").value
        self.p_sample_w_n               = gp("sample_w_n").value
        self.p_predict_horizon          = gp("predict_horizon").value
        self.p_dt                       = gp("dt").value
        self.p_control_rate             = gp("control_rate").value
        self.p_w_heading                = gp("weight_heading").value
        self.p_w_clearance              = gp("weight_clearance").value
        self.p_w_velocity               = gp("weight_velocity").value
        self.p_w_path_tangent           = gp("weight_path_tangent").value
        self.p_path_tangent_lookahead   = gp("path_tangent_lookahead").value
        self.p_lookahead_dist           = gp("lookahead_dist").value
        self.p_lookahead_time           = gp("lookahead_time").value
        self.p_path_heading_gain        = gp("path_heading_gain").value
        self.p_path_cross_track_gain    = gp("path_cross_track_gain").value
        self.p_path_error_slowdown_offset = gp("path_error_slowdown_offset").value
        self.p_path_error_min_speed_scale = gp("path_error_min_speed_scale").value
        self.p_path_error_predict_time  = gp("path_error_predict_time").value
        self.p_rejoin_entry_offset      = gp("rejoin_entry_offset").value
        self.p_rejoin_exit_offset       = gp("rejoin_exit_offset").value
        self.p_rejoin_exit_heading      = gp("rejoin_exit_heading").value
        self.p_rejoin_min_lookahead     = gp("rejoin_min_lookahead").value
        self.p_rejoin_max_lookahead     = gp("rejoin_max_lookahead").value
        self.p_rejoin_step              = gp("rejoin_step").value
        self.p_rejoin_heading_weight    = gp("rejoin_heading_weight").value
        self.p_rejoin_distance_weight   = gp("rejoin_distance_weight").value
        self.p_rejoin_curvature_weight  = gp("rejoin_curvature_weight").value
        self.p_rejoin_clearance_min     = gp("rejoin_clearance_min").value
        self.p_rejoin_clearance_weight  = gp("rejoin_clearance_weight").value
        self.p_rejoin_cross_track_gain_scale = gp("rejoin_cross_track_gain_scale").value
        self.p_dynamic_avoid_enabled    = gp("dynamic_avoid_enabled").value
        self.p_dynamic_path_check_distance = gp(
            "dynamic_path_check_distance").value
        self.p_dynamic_path_corridor_width = gp(
            "dynamic_path_corridor_width").value
        self.p_dynamic_path_corridor_step = gp(
            "dynamic_path_corridor_step").value
        self.p_dynamic_path_min_block_points = gp(
            "dynamic_path_min_block_points").value
        self.p_dynamic_block_enter_ticks = gp("dynamic_block_enter_ticks").value
        self.p_dynamic_block_exit_ticks = gp("dynamic_block_exit_ticks").value
        self.p_dynamic_avoid_min_clearance = gp(
            "dynamic_avoid_min_clearance").value
        dynamic_offsets = gp("dynamic_avoid_lateral_offsets").value
        if isinstance(dynamic_offsets, (list, tuple)):
            self.p_dynamic_avoid_lateral_offsets = [
                float(offset) for offset in dynamic_offsets
            ]
        else:
            self.p_dynamic_avoid_lateral_offsets = [float(dynamic_offsets)]
        self.p_dynamic_avoid_min_lookahead = gp(
            "dynamic_avoid_min_lookahead").value
        self.p_dynamic_avoid_max_lookahead = gp(
            "dynamic_avoid_max_lookahead").value
        self.p_dynamic_avoid_rejoin_distance = gp(
            "dynamic_avoid_rejoin_distance").value
        self.p_dynamic_avoid_step     = gp("dynamic_avoid_step").value
        self.p_dynamic_avoid_side_switch_penalty = gp(
            "dynamic_avoid_side_switch_penalty").value
        self.p_dynamic_avoid_side_hold_sec = gp(
            "dynamic_avoid_side_hold_sec").value
        self.p_dynamic_avoid_cross_track_gain_scale = gp(
            "dynamic_avoid_cross_track_gain_scale").value
        self.p_dynamic_static_filter_enabled = gp(
            "dynamic_static_filter_enabled").value
        self.p_dynamic_static_filter_radius = gp(
            "dynamic_static_filter_radius").value
        self.p_dynamic_static_filter_occupied_threshold = gp(
            "dynamic_static_filter_occupied_threshold").value
        self.p_dynamic_static_filter_unknown_as_static = gp(
            "dynamic_static_filter_unknown_as_static").value
        self.p_dynamic_static_filter_tf_timeout = gp(
            "dynamic_static_filter_tf_timeout").value
        self.p_dynamic_track_cluster_distance = gp(
            "dynamic_track_cluster_distance").value
        self.p_dynamic_track_min_points = gp("dynamic_track_min_points").value
        self.p_dynamic_track_max_radius = gp("dynamic_track_max_radius").value
        self.p_dynamic_track_association_distance = gp(
            "dynamic_track_association_distance").value
        self.p_dynamic_track_timeout = gp("dynamic_track_timeout").value
        self.p_dynamic_motion_min_age = gp("dynamic_motion_min_age").value
        self.p_dynamic_motion_stopped_speed = gp(
            "dynamic_motion_stopped_speed").value
        self.p_dynamic_motion_moving_speed = gp(
            "dynamic_motion_moving_speed").value
        self.p_dynamic_motion_approach_speed = gp(
            "dynamic_motion_approach_speed").value
        self.p_dynamic_motion_recede_speed = gp(
            "dynamic_motion_recede_speed").value
        self.p_dynamic_motion_cpa_horizon = gp(
            "dynamic_motion_cpa_horizon").value
        self.p_dynamic_motion_cpa_margin = gp(
            "dynamic_motion_cpa_margin").value
        self.p_dynamic_approach_reverse_enabled = gp(
            "dynamic_approach_reverse_enabled").value
        self.p_dynamic_approach_reverse_clearance = gp(
            "dynamic_approach_reverse_clearance").value
        self.p_dynamic_approach_reverse_speed = gp(
            "dynamic_approach_reverse_speed").value
        self.p_dynamic_approach_turn_speed = gp(
            "dynamic_approach_turn_speed").value
        self.p_dynamic_layer_enabled = gp("dynamic_layer_enabled").value
        self.p_dynamic_layer_prefer_global_replan = gp(
            "dynamic_layer_prefer_global_replan").value
        self.p_dynamic_layer_topic = gp("dynamic_layer_topic").value
        self.p_dynamic_layer_publish_period = gp(
            "dynamic_layer_publish_period").value
        self.p_dynamic_layer_ttl_sec = gp("dynamic_layer_ttl_sec").value
        self.p_dynamic_layer_min_hold_sec = gp(
            "dynamic_layer_min_hold_sec").value
        self.p_dynamic_layer_clear_confirm_sec = gp(
            "dynamic_layer_clear_confirm_sec").value
        self.p_dynamic_layer_radius_margin = gp(
            "dynamic_layer_radius_margin").value
        self.p_dynamic_layer_min_radius = gp("dynamic_layer_min_radius").value
        self.p_dynamic_layer_max_radius = gp("dynamic_layer_max_radius").value
        self.p_dynamic_layer_observation_range = gp(
            "dynamic_layer_observation_range").value
        self.p_dynamic_layer_clear_range = gp("dynamic_layer_clear_range").value
        self.p_dynamic_layer_prediction_horizon = gp(
            "dynamic_layer_prediction_horizon").value
        self.p_dynamic_layer_prediction_max_distance = gp(
            "dynamic_layer_prediction_max_distance").value
        self.p_dynamic_layer_prediction_speed_max = gp(
            "dynamic_layer_prediction_speed_max").value
        self.p_dynamic_layer_trail_ttl_sec = gp(
            "dynamic_layer_trail_ttl_sec").value
        self.p_dynamic_layer_trail_min_distance = gp(
            "dynamic_layer_trail_min_distance").value
        self.p_dynamic_layer_trail_max_points = gp(
            "dynamic_layer_trail_max_points").value
        self.p_dynamic_layer_escape_distance = gp(
            "dynamic_layer_escape_distance").value
        self.p_dynamic_layer_escape_t_cpa = gp(
            "dynamic_layer_escape_t_cpa").value
        self.p_dynamic_layer_occupied_value = gp(
            "dynamic_layer_occupied_value").value
        self.p_rejoin_predicted_exit_offset = gp(
            "rejoin_predicted_exit_offset").value
        self.p_rejoin_align_angle_thresh = gp("rejoin_align_angle_thresh").value
        self.p_short_lookahead_rejoin_min_distance = gp(
            "short_lookahead_rejoin_min_distance").value
        self.p_short_lookahead_rejoin_ratio = gp(
            "short_lookahead_rejoin_ratio").value
        self.p_short_lookahead_goal_margin = gp(
            "short_lookahead_goal_margin").value
        self.p_max_clearance            = gp("max_clearance").value
        self.p_goal_tolerance           = gp("goal_tolerance").value
        self.p_goal_reached_epsilon     = gp("goal_reached_epsilon").value
        self.p_goal_reached_stopped_speed = gp("goal_reached_stopped_speed").value
        self.p_goal_approach_distance   = gp("goal_approach_distance").value
        self.p_goal_approach_speed      = gp("goal_approach_speed").value
        self.p_goal_align_stop_distance = gp("goal_align_stop_distance").value
        self.p_clearance_slowdown_distance = gp("clearance_slowdown_distance").value
        self.p_clearance_stop_distance  = gp("clearance_stop_distance").value
        self.p_turn_clearance_brake_angle = gp("turn_clearance_brake_angle").value
        self.p_near_wall_creep_speed    = gp("near_wall_creep_speed").value
        self.p_near_wall_creep_min_clearance = gp("near_wall_creep_min_clearance").value
        self.p_rejoin_creep_min_clearance = gp("rejoin_creep_min_clearance").value
        self.p_near_wall_escape_clearance = gp("near_wall_escape_clearance").value
        self.p_near_wall_escape_speed = gp("near_wall_escape_speed").value
        self.p_near_wall_escape_turn = gp("near_wall_escape_turn").value
        self.p_near_wall_escape_max_curvature = gp(
            "near_wall_escape_max_curvature").value
        self.p_align_angle_thresh       = gp("align_angle_thresh").value
        self.p_align_angle_exit         = gp("align_angle_exit").value
        self.p_align_kp                 = gp("align_kp").value
        self.p_align_kd                 = gp("align_kd").value
        self.p_align_v_blend_max        = gp("align_v_blend_max").value
        self.p_align_drive_angle        = gp("align_drive_angle").value
        self.p_align_release_angle      = gp("align_release_angle").value
        self.p_rejoin_align_release_angle = gp("rejoin_align_release_angle").value
        self.p_align_release_clearance  = gp("align_release_clearance").value
        self.p_align_cooldown           = gp("align_cooldown").value
        self.p_align_trigger_ticks      = gp("align_trigger_ticks").value      # P4
        self.p_w_min_rotate             = gp("w_min_rotate").value
        self.p_w_brake_alpha_max        = gp("w_brake_alpha_max").value
        self.p_allow_backward           = gp("allow_backward").value
        self.p_max_path_offset          = gp("max_path_offset").value
        self.p_recovery_path_accept_offset = gp("recovery_path_accept_offset").value
        self.p_recovery_path_accept_duration = gp("recovery_path_accept_duration").value
        self.p_path_lost_offset         = gp("path_lost_offset").value
        self.p_stuck_recovery_sec       = gp("stuck_recovery_sec").value
        self.p_recovery_cooldown        = gp("recovery_cooldown").value
        self.p_spin_duration            = gp("spin_duration").value
        self.p_forward_only_dist        = gp("forward_only_dist").value
        self.p_forward_only_timeout     = gp("forward_only_timeout").value
        self.p_forward_only_speed_scale = gp("forward_only_speed_scale").value
        self.p_forward_only_settle_w    = gp("forward_only_settle_w").value
        self.p_forward_only_min_dist    = gp("forward_only_min_dist").value
        self.p_forward_only_rejoin_offset = gp("forward_only_rejoin_offset").value
        self.p_spin_forward_clearance_margin = gp("spin_forward_clearance_margin").value
        self.p_robot_radius             = gp("robot_radius").value
        self.p_hard_collision_distance  = gp("hard_collision_distance").value
        self.p_safety_distance          = gp("safety_distance").value
        self.p_odom_timeout             = gp("odom_timeout").value
        self.p_scan_range_max           = gp("scan_range_max").value
        self.p_lidar_offset_x           = gp("lidar_offset_x").value
        self.p_lidar_offset_y           = gp("lidar_offset_y").value
        self.p_odom_topic               = gp("odom_topic").value
        self.p_odom_fallback_topic      = gp("odom_fallback_topic").value
        self.p_global_path_topic        = gp("global_path_topic").value
        self.p_goal_pose_topic          = gp("goal_pose_topic").value
        self.p_goal_dedup_dist          = gp("goal_dedup_dist").value
        self.p_goal_dedup_yaw           = gp("goal_dedup_yaw").value
        self.p_reached_new_path_rearm_dist = gp("reached_new_path_rearm_dist").value
        self.p_scan_topic               = gp("scan_topic").value
        self.p_map_topic                = gp("map_topic").value
        self.p_cmd_vel_topic            = gp("cmd_vel_topic").value
        self.p_state_log_period         = gp("state_log_period").value
        self.p_candidate_log_period     = gp("candidate_log_period").value
        self.p_path_tf_timeout          = gp("path_tf_timeout").value

    # ───────────────────────────────────────────────────────────────
    # 콜백 (기존과 동일)
    # ───────────────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry) -> None:
        self._update_state_from_odom(msg, is_fallback=False)
        self._last_odom_time = self._sec_now()

    def _on_odom_fallback(self, msg: Odometry) -> None:
        now = self._sec_now()
        if self._last_odom_time is None:
            self._update_state_from_odom(msg, is_fallback=True)
            self._last_odom_time = now
            return
        if now - self._last_odom_time > self.p_odom_timeout:
            if not self._last_odom_was_fallback:
                self.get_logger().warn(
                    "EKF /odometry/filtered timeout → /odom fallback 사용")
            self._update_state_from_odom(msg, is_fallback=True)
            self._last_odom_time = now

    def _update_state_from_odom(self, msg: Odometry, *, is_fallback: bool) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        if not is_fallback and msg.header.frame_id != self.LOCAL_FRAME:
            self.get_logger().warn(
                f"/odometry/filtered frame_id='{msg.header.frame_id}' ≠ '{self.LOCAL_FRAME}'",
                throttle_duration_sec=10.0)
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        v = twist.linear.x
        w = twist.angular.z
        if self._state is None:
            self._state = RobotState(x=x, y=y, theta=theta, v=v, w=w)
        else:
            self._state.x = x; self._state.y = y; self._state.theta = theta
            self._state.v = v; self._state.w = w
        self._last_odom_was_fallback = is_fallback

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        """새 goal 수신 표시.

        DWA는 path만 따라가지만, 빈 /global_path가 들어왔을 때 이것이
        "새 goal의 계획 실패"인지 "이전/중복 publisher의 stale empty"인지 구분하려면
        goal edge가 필요하다.
        """
        new_goal = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        new_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if self._is_duplicate_goal(new_goal, new_yaw):
            return
        self._last_goal_xy = new_goal
        self._last_goal_yaw = new_yaw
        self._goal_version += 1

    def _on_global_path(self, msg: Path) -> None:
        if not msg.poses:
            if self._should_ignore_empty_path():
                self.get_logger().warn(
                    "빈 /global_path 수신 — 새 goal 없음, 기존 path 유지",
                    throttle_duration_sec=2.0)
                return
            if self._path_local is None or self._path_local.poses:
                self.get_logger().warn("빈 /global_path 수신 — DWA 정지 모드")
            self._path_goal_xy_global = None
            empty = Path()
            empty.header.frame_id = self.LOCAL_FRAME
            empty.header.stamp = self.get_clock().now().to_msg()
            self._path_local = empty
            self._path_goal_version = self._goal_version
            self._reached = False
            return

        src_frame = msg.header.frame_id or self.GLOBAL_FRAME
        if src_frame == self.LOCAL_FRAME:
            if not self._is_path_close_to_state(msg):
                return
            last = msg.poses[-1].pose.position
            rearm_from_reached = should_rearm_reached_with_path(
                self._nav_state == NavState.REACHED,
                self._current_xy(),
                (last.x, last.y),
                self.p_reached_new_path_rearm_dist,
            )
            if self._nav_state == NavState.REACHED and not rearm_from_reached:
                self._path_goal_version = self._goal_version
                self.get_logger().warn(
                    "same-goal local /global_path ignored while REACHED hold is valid",
                    throttle_duration_sec=2.0)
                return
            if rearm_from_reached:
                self.get_logger().info(
                    "REACHED hold released by new local /global_path endpoint")
            self._path_local = msg
            self._reset_path_state()
            self._path_goal_version = self._goal_version
            self._path_goal_xy_global = None
            self.get_logger().info(
                f"/global_path 수신 ({src_frame}) — "
                f"{len(msg.poses)}점, goal=({last.x:.2f},{last.y:.2f})")
            return

        if src_frame != self.GLOBAL_FRAME:
            self.get_logger().warn(
                f"/global_path frame_id='{src_frame}' — 지원 안 됨. 무시.")
            return

        transformed = self._transform_path_to_local(msg)
        if transformed is None:
            return

        if not self._is_path_close_to_state(transformed):
            return

        src_goal = msg.poses[-1].pose.position
        path_goal_xy_global = (src_goal.x, src_goal.y)
        local_goal = transformed.poses[-1].pose.position
        rearm_from_reached = should_rearm_reached_with_path(
            self._nav_state == NavState.REACHED,
            self._current_xy(),
            (local_goal.x, local_goal.y),
            self.p_reached_new_path_rearm_dist,
        )
        if not rearm_from_reached:
            if not self._is_path_goal_close_to_latest_goal(path_goal_xy_global):
                return
            if self._should_ignore_path_while_reached(path_goal_xy_global, transformed):
                return
        else:
            self._adopt_path_goal_if_needed(path_goal_xy_global)
            self.get_logger().info(
                "REACHED hold released by new /global_path endpoint")

        self._path_local = transformed
        self._reset_path_state()
        self._path_goal_version = self._goal_version
        self._path_goal_xy_global = path_goal_xy_global
        last = transformed.poses[-1].pose.position
        self.get_logger().info(
            f"/global_path 수신 ({src_frame} → {self.LOCAL_FRAME}) — "
            f"{len(transformed.poses)}점, "
            f"goal=({last.x:.2f},{last.y:.2f}) [{self.LOCAL_FRAME}]")

    def _reset_path_state(self) -> None:
        """새 path 수신 시 추종 상태 리셋."""
        self._reached = False
        self._path_progress_idx = 0
        self._in_align_mode = False
        # [리뷰 Codex] P4 카운터를 새 path 경계에서 리셋 — 안 하면 직전 path 에서 쌓인
        #   _align_trigger_count(예: 2)가 새 path 로 새어 한 tick 만에 ALIGN 진입 가능.
        #   "N틱 연속" 의미가 path 경계를 넘어가지 않도록.
        self._align_trigger_count = 0
        # EMERGENCY 중 새 path 오면 NORMAL 복귀 (SPIN/FORWARD_ONLY 는 recovery 완료까지 유지).
        # REJOIN 은 1Hz path 갱신 때도 유지한다. 여기서 REJOIN/stuck_counter 를 리셋하면
        # 막힘 상황에서 EMERGENCY status → A* 재계획 → 새 path → counter=0 이 반복되어
        # SPIN recovery 임계치까지 절대 도달하지 못한다.
        if self._nav_state in (NavState.EMERGENCY,):
            self._nav_state = NavState.NORMAL
            self._stuck_counter = 0
        # ── 추가 (2026-05-31 SW · P2 보강): REACHED 도 새 path 오면 해제 ──
        # rationale: P1 dedup 이 '같은 goal' 재계획을 막으므로, 여기 도달하는 새 path 는
        #            사실상 '진짜 다른 goal'. REACHED 를 안 풀면 도착 후 첫 새 goal 에서
        #            로봇이 _control_loop 의 reached_hold 에 갇혀 움직이지 않는다(잠재 버그).
        #            REACHED→NORMAL 복귀로 GOAL_REACHED edge 도 재무장되어 다음 도착 때
        #            다시 한 번 발행됨.
        #            ※ P1 과 짝을 이뤄야 안전 — dedup 없이는 같은 goal 짧은 path 가
        #              REACHED 를 계속 풀어 도착 지점에서 재추종/떨림을 유발할 수 있음.
        if self._nav_state == NavState.REACHED:
            self._nav_state = NavState.NORMAL

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._static_map = msg

    # ───────────────────────────────────────────────────────────────
    # 메인 제어 루프 — NavState 디스패처
    # ───────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        # ── 공통 전제조건 ──────────────────────────────────────────
        if self._state is None:
            return
        if (self._last_odom_time is None or
                self._sec_now() - self._last_odom_time > self.p_odom_timeout):
            self._stop_robot("odom_timeout")
            return

        # ── NavState 디스패치 ─────────────────────────────────────
        # SPIN/FORWARD_ONLY 은 path 없이도 실행 (recovery 우선)
        if self._nav_state == NavState.SPIN:
            self._execute_spin()
            return

        if self._nav_state == NavState.FORWARD_ONLY:
            self._execute_forward_only()
            return

        # path 필요한 상태
        if self._path_local is None or not self._path_local.poses:
            self._stop_robot("no_global_path")
            self._publish_status_value("STOPPED")
            return

        if self._nav_state == NavState.REACHED:
            self._stop_robot("reached_hold")
            return

        # 공통 선행 계산 (NORMAL/ALIGN/EMERGENCY 공유)
        ctx = self._compute_context()
        if ctx is None:
            return   # 내부에서 이미 정지 처리

        # REACHED 전이
        if should_mark_goal_reached(
                ctx["dist_to_goal"],
                self.p_goal_tolerance,
                self.p_goal_reached_epsilon,
                self._state.v,
                self.p_goal_reached_stopped_speed):
            # ── 추가 (2026-05-31 SW · P2): 도착 "순간"에만 GOAL_REACHED edge 신호 ──
            # rationale: NavState.REACHED 진입은 했지만 /dwa/status 에 명시적 도착
            #            신호가 없어 외부(Foxglove·BT·모니터)에서 도착 확인이 어려웠음.
            #            1Hz 타이머(_publish_status)는 이후 "REACHED" 정상 상태를 계속
            #            발행하므로, 여기서는 상승 edge 로 "GOAL_REACHED" 를 한 번만 publish.
            #            (_reset_path_state 가 새 path 시 REACHED 를 풀어 edge 가 재무장됨)
            if self._nav_state != NavState.REACHED:
                self._publish_status_value("GOAL_REACHED")
                self.get_logger().info(
                    f'goal 도착 — 정지 '
                    f'(d={ctx["dist_to_goal"]:.2f}m, '
                    f'pos=({self._state.x:.2f}, {self._state.y:.2f}))'
                )
            self._nav_state = NavState.REACHED
            self._reached = True
            self._stop_robot("goal_reached")
            return

        # 상태별 실행
        if self._nav_state == NavState.EMERGENCY:
            self._execute_emergency(ctx)
        elif self._nav_state == NavState.ALIGN:
            self._execute_align(ctx)
        else:
            self._execute_normal(ctx)

    # ───────────────────────────────────────────────────────────────
    # 공통 선행 계산 — NORMAL/ALIGN/EMERGENCY 가 모두 필요한 값
    # ───────────────────────────────────────────────────────────────
    def _update_dynamic_block_state(self, blocked: bool) -> bool:
        if blocked:
            self._dynamic_block_ticks += 1
            self._dynamic_clear_ticks = 0
        else:
            self._dynamic_clear_ticks += 1
            if self._dynamic_clear_ticks >= max(1, self.p_dynamic_block_exit_ticks):
                self._dynamic_block_ticks = 0
                if self._sec_now() >= self._dynamic_avoid_until:
                    self._dynamic_avoid_side = 0

        if self._dynamic_block_ticks >= max(1, self.p_dynamic_block_enter_ticks):
            return True
        if (self._nav_state == NavState.AVOIDING_DYNAMIC and
                self._dynamic_clear_ticks < max(1, self.p_dynamic_block_exit_ticks)):
            return True
        return False

    def _lookup_local_to_map_transform(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                self.GLOBAL_FRAME, self.LOCAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=self.p_dynamic_static_filter_tf_timeout))
        except (TransformException, tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(
                f"dynamic static-filter TF lookup failed: {e}",
                throttle_duration_sec=2.0)
            return None

        q = t.transform.rotation
        return (
            t.transform.translation.x,
            t.transform.translation.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    @staticmethod
    def _transform_xy(
        xy: Tuple[float, float],
        transform_xy_yaw: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        tx, ty, yaw = transform_xy_yaw
        cos_t = math.cos(yaw)
        sin_t = math.sin(yaw)
        x, y = xy
        return (tx + cos_t * x - sin_t * y,
                ty + sin_t * x + cos_t * y)

    @staticmethod
    def _inverse_transform_xy(
        xy: Tuple[float, float],
        transform_xy_yaw: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        tx, ty, yaw = transform_xy_yaw
        dx = xy[0] - tx
        dy = xy[1] - ty
        cos_t = math.cos(-yaw)
        sin_t = math.sin(-yaw)
        return (cos_t * dx - sin_t * dy,
                sin_t * dx + cos_t * dy)

    def _filter_static_obstacles_for_dynamic(
        self,
        obstacles_local: List[Tuple[float, float]],
    ) -> Tuple[List[Tuple[float, float]], int]:
        if (not self.p_dynamic_static_filter_enabled or
                not obstacles_local or self._state is None or
                self._static_map is None):
            return obstacles_local, 0

        transform = self._lookup_local_to_map_transform()
        if transform is None:
            return obstacles_local, 0

        dynamic_candidates: List[Tuple[float, float]] = []
        static_count = 0
        for obs in obstacles_local:
            local_frame_xy = local_to_world(obs, self._state)
            map_xy = self._transform_xy(local_frame_xy, transform)
            if occupancy_grid_has_static_obstacle_near(
                    self._static_map,
                    map_xy,
                    self.p_dynamic_static_filter_radius,
                    int(self.p_dynamic_static_filter_occupied_threshold),
                    bool(self.p_dynamic_static_filter_unknown_as_static)):
                static_count += 1
                continue
            dynamic_candidates.append(obs)
        return dynamic_candidates, static_count

    def _update_dynamic_tracks(
        self,
        clusters_local: List[DynamicObstacleCluster],
    ) -> List[DynamicObstacleTrack]:
        if self._state is None:
            return []

        now = self._sec_now()
        current_tracks: List[DynamicObstacleTrack] = []
        used_tracks: set[int] = set()
        association = max(0.10, self.p_dynamic_track_association_distance)

        for cluster in clusters_local:
            wx, wy = local_to_world(cluster.center, self._state)
            best_id: Optional[int] = None
            best_dist = float("inf")
            for track_id, track in self._dynamic_tracks.items():
                if track_id in used_tracks:
                    continue
                dist = math.hypot(wx - track.x, wy - track.y)
                gate = association + cluster.radius + track.radius
                if dist < best_dist and dist <= gate:
                    best_dist = dist
                    best_id = track_id

            if best_id is None:
                track = DynamicObstacleTrack(
                    self._next_dynamic_track_id,
                    wx, wy,
                    0.0, 0.0,
                    cluster.radius,
                    cluster.count,
                    1,
                    now,
                    0,
                )
                self._next_dynamic_track_id += 1
                self._dynamic_tracks[track.track_id] = track
            else:
                track = self._dynamic_tracks[best_id]
                dt = max(1e-3, min(0.5, now - track.last_seen))
                meas_vx = (wx - track.x) / dt
                meas_vy = (wy - track.y) / dt
                alpha = 0.45 if track.age >= 2 else 1.0
                track.vx = (1.0 - alpha) * track.vx + alpha * meas_vx
                track.vy = (1.0 - alpha) * track.vy + alpha * meas_vy
                track.x = wx
                track.y = wy
                track.radius = cluster.radius
                track.count = cluster.count
                track.age += 1
                track.last_seen = now
                track.missed = 0

            used_tracks.add(track.track_id)
            current_tracks.append(track)

        timeout = max(0.10, self.p_dynamic_track_timeout)
        stale: List[int] = []
        for track_id, track in self._dynamic_tracks.items():
            if track_id in used_tracks:
                continue
            track.missed += 1
            if now - track.last_seen > timeout:
                stale.append(track_id)
        for track_id in stale:
            self._dynamic_tracks.pop(track_id, None)

        return current_tracks

    def _select_dynamic_motion_estimate(
        self,
        path_xy: List[Tuple[float, float]],
        robot: RobotState,
        projection: PathProjection,
        tracks: List[DynamicObstacleTrack],
    ) -> DynamicMotionEstimate:
        if not path_xy or not tracks:
            return DynamicMotionEstimate("UNKNOWN")

        samples: List[Tuple[float, Tuple[float, float]]] = []
        check_distance = max(0.0, self.p_dynamic_path_check_distance)
        step = max(0.05, self.p_dynamic_path_corridor_step)
        count = int(check_distance / step) + 1
        for i in range(count + 1):
            distance = min(check_distance, i * step)
            sample = sample_path_from_projection(path_xy, projection, distance)
            if sample is None:
                continue
            local_point = world_to_local(sample.point, robot)
            samples.append((distance, local_point))
        if len(samples) < 2:
            return DynamicMotionEstimate("UNKNOWN")

        best_track: Optional[DynamicObstacleTrack] = None
        best_key = (float("inf"), float("inf"))
        best_side = 0
        corridor_width = max(0.05, self.p_dynamic_path_corridor_width)
        for track in tracks:
            local_xy = world_to_local((track.x, track.y), robot)
            if local_xy[0] < -0.20:
                continue
            if math.hypot(*local_xy) > check_distance + corridor_width + track.radius + 0.5:
                continue

            best_dist = float("inf")
            best_along = float("inf")
            side_value = 0
            for (d0, p0), (d1, p1) in zip(samples, samples[1:]):
                dist, t, _ = point_segment_projection(local_xy, p0, p1)
                if dist >= best_dist:
                    continue
                sx = p1[0] - p0[0]
                sy = p1[1] - p0[1]
                side = sx * (local_xy[1] - p0[1]) - sy * (local_xy[0] - p0[0])
                best_dist = dist
                best_along = d0 + t * max(0.0, d1 - d0)
                side_value = 1 if side > 0.0 else -1 if side < 0.0 else 0

            if best_dist - track.radius > corridor_width:
                continue
            key = (best_along, best_dist)
            if key < best_key:
                best_key = key
                best_track = track
                best_side = side_value

        estimate = classify_dynamic_motion(
            best_track,
            robot,
            self.p_robot_radius,
            self.p_dynamic_motion_stopped_speed,
            self.p_dynamic_motion_moving_speed,
            self.p_dynamic_motion_approach_speed,
            self.p_dynamic_motion_recede_speed,
            self.p_dynamic_motion_cpa_horizon,
            self.p_dynamic_motion_cpa_margin,
            int(self.p_dynamic_motion_min_age),
        )
        if best_side != 0:
            estimate.side = best_side
        return estimate

    def _dynamic_layer_radius(self, observed_radius: float) -> float:
        radius = (
            max(0.0, observed_radius)
            + max(0.0, self.p_robot_radius)
            + max(0.0, self.p_dynamic_layer_radius_margin)
        )
        return max(
            self.p_dynamic_layer_min_radius,
            min(self.p_dynamic_layer_max_radius, radius),
        )

    def _append_dynamic_layer_trail(
        self,
        block: DynamicObstacleMapBlock,
        map_xy: Tuple[float, float],
        now: float,
    ) -> None:
        min_distance = max(0.0, self.p_dynamic_layer_trail_min_distance)
        if not block.trail:
            block.trail.append((map_xy[0], map_xy[1], now))
        else:
            last_x, last_y, last_t = block.trail[-1]
            moved = math.hypot(map_xy[0] - last_x, map_xy[1] - last_y)
            if moved >= min_distance or now - last_t >= 1.0:
                block.trail.append((map_xy[0], map_xy[1], now))

        ttl = max(0.0, self.p_dynamic_layer_trail_ttl_sec)
        if ttl > 0.0:
            block.trail = [
                point for point in block.trail
                if now - point[2] <= ttl
            ]
        if not block.trail:
            block.trail.append((map_xy[0], map_xy[1], now))

        max_points = max(2, int(self.p_dynamic_layer_trail_max_points))
        if len(block.trail) > max_points:
            block.trail = block.trail[-max_points:]

    def _upsert_dynamic_layer_block(
        self,
        map_xy: Tuple[float, float],
        radius: float,
        velocity_map: Tuple[float, float],
        now: float,
    ) -> None:
        best_id: Optional[int] = None
        best_dist = float("inf")
        for block_id, block in self._dynamic_layer_blocks.items():
            dist = math.hypot(map_xy[0] - block.x, map_xy[1] - block.y)
            gate = max(block.radius, radius) + 0.60
            if dist < best_dist and dist <= gate:
                best_dist = dist
                best_id = block_id

        expire_at = now + max(1.0, self.p_dynamic_layer_ttl_sec)
        if best_id is None:
            block_id = self._next_dynamic_layer_block_id
            self._next_dynamic_layer_block_id += 1
            self._dynamic_layer_blocks[block_id] = DynamicObstacleMapBlock(
                block_id=block_id,
                x=map_xy[0],
                y=map_xy[1],
                radius=radius,
                vx=velocity_map[0],
                vy=velocity_map[1],
                first_seen=now,
                last_seen=now,
                expire_at=expire_at,
                trail=[(map_xy[0], map_xy[1], now)],
            )
            return

        block = self._dynamic_layer_blocks[best_id]
        self._append_dynamic_layer_trail(block, map_xy, now)
        alpha = 0.60
        block.x = (1.0 - alpha) * block.x + alpha * map_xy[0]
        block.y = (1.0 - alpha) * block.y + alpha * map_xy[1]
        block.radius = max(block.radius, radius)
        block.vx = (1.0 - alpha) * block.vx + alpha * velocity_map[0]
        block.vy = (1.0 - alpha) * block.vy + alpha * velocity_map[1]
        block.last_seen = now
        block.expire_at = expire_at
        block.clear_since = None

    def _dynamic_layer_block_visible(
        self,
        block: DynamicObstacleMapBlock,
        map_to_local_transform: Tuple[float, float, float],
    ) -> bool:
        if self._state is None:
            return False
        odom_xy = self._inverse_transform_xy((block.x, block.y), map_to_local_transform)
        local_xy = world_to_local(odom_xy, self._state)
        if math.hypot(local_xy[0], local_xy[1]) > self.p_dynamic_layer_clear_range:
            return False
        if local_xy[0] < -0.75:
            return False
        return True

    def _update_dynamic_obstacle_layer(
        self,
        tracks: List[DynamicObstacleTrack],
    ) -> int:
        if (not self.p_dynamic_layer_enabled or self._static_map is None or
                self._state is None):
            return len(self._dynamic_layer_blocks)

        transform = self._lookup_local_to_map_transform()
        if transform is None:
            return len(self._dynamic_layer_blocks)

        now = self._sec_now()
        observed_map_xy: List[Tuple[float, float]] = []
        horizon = max(0.0, self.p_dynamic_layer_prediction_horizon)
        max_prediction = max(0.0, self.p_dynamic_layer_prediction_max_distance)

        for track in tracks:
            local_xy = world_to_local((track.x, track.y), self._state)
            if local_xy[0] < -0.50:
                continue
            if math.hypot(local_xy[0], local_xy[1]) > self.p_dynamic_layer_observation_range:
                continue

            center_map = self._transform_xy((track.x, track.y), transform)
            observed_map_xy.append(center_map)
            speed = math.hypot(track.vx, track.vy)
            if speed > 1e-6 and horizon > 0.0 and max_prediction > 0.0:
                travel = min(speed * horizon, max_prediction)
                scale = travel / speed
                end_map = self._transform_xy(
                    (track.x + track.vx * scale,
                     track.y + track.vy * scale),
                    transform,
                )
                velocity_map = (
                    (end_map[0] - center_map[0]) / max(horizon, 1e-3),
                    (end_map[1] - center_map[1]) / max(horizon, 1e-3),
                )
            else:
                velocity_map = (0.0, 0.0)

            self._upsert_dynamic_layer_block(
                center_map,
                self._dynamic_layer_radius(track.radius),
                velocity_map,
                now,
            )

        stale: List[int] = []
        min_hold = max(0.0, self.p_dynamic_layer_min_hold_sec)
        clear_confirm = max(0.0, self.p_dynamic_layer_clear_confirm_sec)
        for block_id, block in self._dynamic_layer_blocks.items():
            if now >= block.expire_at:
                stale.append(block_id)
                continue

            nearest_observed = min(
                (math.hypot(block.x - x, block.y - y) for x, y in observed_map_xy),
                default=float("inf"),
            )
            if nearest_observed <= block.radius + 0.35:
                block.clear_since = None
                continue

            if now - block.first_seen < min_hold:
                continue
            if not self._dynamic_layer_block_visible(block, transform):
                continue

            if block.clear_since is None:
                block.clear_since = now
            elif now - block.clear_since >= clear_confirm:
                stale.append(block_id)

        for block_id in stale:
            self._dynamic_layer_blocks.pop(block_id, None)

        self._publish_dynamic_obstacle_layer()
        return len(self._dynamic_layer_blocks)

    def _paint_dynamic_layer_disc(
        self,
        data: List[int],
        center: Tuple[float, float],
        radius: float,
        value: int,
    ) -> None:
        if self._static_map is None:
            return
        cell = occupancy_grid_world_to_cell(self._static_map, center)
        if cell is None:
            return
        info = self._static_map.info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return
        width = int(info.width)
        height = int(info.height)
        col, row = cell
        radius_cells = max(1, int(math.ceil(radius / resolution)))
        for rr in range(max(0, row - radius_cells),
                        min(height, row + radius_cells + 1)):
            dy = (rr - row) * resolution
            for cc in range(max(0, col - radius_cells),
                            min(width, col + radius_cells + 1)):
                dx = (cc - col) * resolution
                if math.hypot(dx, dy) > radius + 0.5 * resolution:
                    continue
                data[rr * width + cc] = value

    def _paint_dynamic_layer_segment(
        self,
        data: List[int],
        start: Tuple[float, float],
        end: Tuple[float, float],
        radius: float,
        value: int,
    ) -> None:
        if self._static_map is None:
            return
        resolution = max(0.01, float(self._static_map.info.resolution))
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / max(resolution, radius * 0.5))))
        for i in range(steps + 1):
            t = i / steps
            point = (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
            )
            self._paint_dynamic_layer_disc(data, point, radius, value)

    def _publish_dynamic_obstacle_layer(self, force: bool = False) -> None:
        if not self.p_dynamic_layer_enabled or self._static_map is None:
            return

        now = self._sec_now()
        active = bool(self._dynamic_layer_blocks)
        if not active and not self._last_dynamic_layer_active and not force:
            return
        publish_period = max(0.05, self.p_dynamic_layer_publish_period)
        if (not force and active == self._last_dynamic_layer_active and
                now - self._last_dynamic_layer_publish_time < publish_period):
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.GLOBAL_FRAME
        msg.info = self._static_map.info
        width = int(msg.info.width)
        height = int(msg.info.height)
        value = max(1, min(100, int(self.p_dynamic_layer_occupied_value)))
        data = [0] * (width * height)
        horizon = max(0.0, self.p_dynamic_layer_prediction_horizon)
        max_prediction = max(0.0, self.p_dynamic_layer_prediction_max_distance)
        max_speed = max(0.0, self.p_dynamic_layer_prediction_speed_max)

        for block in self._dynamic_layer_blocks.values():
            if len(block.trail) >= 2:
                for start, end in zip(block.trail, block.trail[1:]):
                    self._paint_dynamic_layer_segment(
                        data,
                        (start[0], start[1]),
                        (end[0], end[1]),
                        block.radius,
                        value,
                    )
            elif block.trail:
                self._paint_dynamic_layer_disc(
                    data, (block.trail[-1][0], block.trail[-1][1]),
                    block.radius, value)

            raw_speed = math.hypot(block.vx, block.vy)
            speed = raw_speed
            if max_speed > 0.0:
                speed = min(speed, max_speed)
            travel = min(speed * horizon, max_prediction)
            if travel > 0.05 and raw_speed > 1e-6:
                scale = travel / raw_speed
                end = (block.x + block.vx * scale, block.y + block.vy * scale)
                self._paint_dynamic_layer_segment(
                    data, (block.x, block.y), end, block.radius, value)
            else:
                self._paint_dynamic_layer_disc(
                    data, (block.x, block.y), block.radius, value)

        msg.data = data
        self._dynamic_layer_pub.publish(msg)
        self._last_dynamic_layer_publish_time = now
        self._last_dynamic_layer_active = active
        if active:
            self.get_logger().warn(
                "dynamic obstacle layer published "
                f"(blocks={len(self._dynamic_layer_blocks)}, "
                f"cells={sum(1 for cell in data if cell >= value)}, "
                f"ttl={self.p_dynamic_layer_ttl_sec:.0f}s)",
                throttle_duration_sec=2.0)

    def _compute_context(self) -> Optional[dict]:
        """경로 추종에 필요한 공통 값을 계산해 dict 로 반환.

        실패(lookahead 없음, path_offset 초과 등) 시 None 반환.
        """
        path_xy = self._path_xy(self._path_local)
        gx, gy = path_xy[-1]
        dist_to_goal = math.hypot(gx - self._state.x, gy - self._state.y)

        projection = project_to_path(
            path_xy, (self._state.x, self._state.y), self._path_progress_idx)
        if projection is None:
            self._stop_robot("no_path_projection")
            return None

        nearest_idx = projection.segment_idx
        self._path_progress_idx = nearest_idx

        path_offset = projection.offset
        projection_heading_error = normalize_angle(projection.yaw - self._state.theta)
        predicted_signed_path_offset = predict_signed_path_offset(
            projection.signed_offset,
            projection_heading_error,
            self._state.v,
            self.p_path_error_predict_time,
        )
        predicted_path_offset = abs(predicted_signed_path_offset)
        if path_offset > self.p_path_lost_offset:
            self._stop_robot("path_offset_too_large")
            self._publish_status_value("PATH_LOST")
            self.get_logger().warn(
                f"path 와 {path_offset:.2f}m 떨어짐 (max={self.p_path_lost_offset}m).",
                throttle_duration_sec=2.0)
            return None

        was_rejoining = self._nav_state == NavState.REJOIN
        rejoin_requested = should_use_rejoin(
            path_offset=path_offset,
            heading_error=projection_heading_error,
            was_rejoining=was_rejoining,
            entry_offset=self.p_rejoin_entry_offset,
            exit_offset=self.p_rejoin_exit_offset,
            exit_heading=self.p_rejoin_exit_heading,
            predicted_offset=predicted_path_offset,
            predicted_exit_offset=self.p_rejoin_predicted_exit_offset,
        )
        obstacles_local = self._extract_obstacles_from_scan()
        dynamic_obstacles_local = obstacles_local
        dynamic_static_filtered = 0
        dynamic_blockage = DynamicPathBlockage(
            False, float("inf"), 0, 0.0, 0.0)
        dynamic_blocked = False
        dynamic_avoid_target: Optional[DynamicAvoidTarget] = None
        dynamic_clusters: List[DynamicObstacleCluster] = []
        dynamic_tracks: List[DynamicObstacleTrack] = []
        dynamic_motion = DynamicMotionEstimate("UNKNOWN")
        dynamic_layer_blocks = len(self._dynamic_layer_blocks)
        dynamic_layer_prefer_global_replan = False
        rejoin_target: Optional[RejoinTarget] = None
        if rejoin_requested:
            rejoin_target = choose_rejoin_target(
                path_xy=path_xy,
                robot=self._state,
                projection=projection,
                min_lookahead=self.p_rejoin_min_lookahead,
                max_lookahead=self.p_rejoin_max_lookahead,
                step=self.p_rejoin_step,
                heading_weight=self.p_rejoin_heading_weight,
                distance_weight=self.p_rejoin_distance_weight,
                curvature_weight=self.p_rejoin_curvature_weight,
                effective_offset=predicted_path_offset,
                obstacles_local=obstacles_local,
                robot_radius=self.p_robot_radius,
                clearance_min=self.p_rejoin_clearance_min,
                clearance_weight=self.p_rejoin_clearance_weight,
            )

        # Adaptive lookahead (approach scaling)
        approach_scale = min(1.0, dist_to_goal / max(self.p_lookahead_dist, 1e-3))
        effective_lookahead = max(
            self.p_goal_tolerance * 1.5,
            self.p_lookahead_time * abs(self._state.v),
            self.p_lookahead_dist * approach_scale,
        )
        if rejoin_target is not None:
            effective_lookahead = rejoin_target.distance
            lookahead = rejoin_target.point
            target_path_yaw = rejoin_target.yaw
        else:
            if max(path_offset, predicted_path_offset) > self.p_path_error_slowdown_offset:
                effective_lookahead = min(effective_lookahead, self.p_lookahead_dist)
            lookahead = pick_lookahead_from_projection(
                path_xy, projection, effective_lookahead)
            target_path_yaw = projection.yaw

        if lookahead is None:
            self._stop_robot("no_lookahead")
            return None

        lx, ly = world_to_local(lookahead, self._state)
        L = math.hypot(lx, ly)
        short_lookahead_rejoin = False
        if (rejoin_target is None and
                should_force_rejoin_for_short_lookahead(
                    local_lookahead_distance=L,
                    effective_lookahead=effective_lookahead,
                    dist_to_goal=dist_to_goal,
                    goal_tolerance=self.p_goal_tolerance,
                    min_distance=self.p_short_lookahead_rejoin_min_distance,
                    ratio=self.p_short_lookahead_rejoin_ratio,
                    goal_margin=self.p_short_lookahead_goal_margin)):
            rejoin_target = choose_rejoin_target(
                path_xy=path_xy,
                robot=self._state,
                projection=projection,
                min_lookahead=self.p_rejoin_min_lookahead,
                max_lookahead=self.p_rejoin_max_lookahead,
                step=self.p_rejoin_step,
                heading_weight=self.p_rejoin_heading_weight,
                distance_weight=self.p_rejoin_distance_weight,
                curvature_weight=self.p_rejoin_curvature_weight,
                effective_offset=max(
                    predicted_path_offset,
                    path_offset,
                    self.p_rejoin_entry_offset,
                ),
                obstacles_local=obstacles_local,
                robot_radius=self.p_robot_radius,
                clearance_min=self.p_rejoin_clearance_min,
                clearance_weight=self.p_rejoin_clearance_weight,
            )
            if rejoin_target is not None:
                short_lookahead_rejoin = True
                effective_lookahead = rejoin_target.distance
                lookahead = rejoin_target.point
                target_path_yaw = rejoin_target.yaw
                lx, ly = world_to_local(lookahead, self._state)
                L = math.hypot(lx, ly)

        if (self.p_dynamic_avoid_enabled and
                dist_to_goal > max(self.p_goal_approach_distance,
                                   self.p_goal_tolerance + 0.5)):
            dynamic_obstacles_local, dynamic_static_filtered = (
                self._filter_static_obstacles_for_dynamic(obstacles_local)
            )
            dynamic_clusters = cluster_obstacle_points(
                dynamic_obstacles_local,
                self.p_dynamic_track_cluster_distance,
                int(self.p_dynamic_track_min_points),
                self.p_dynamic_track_max_radius,
            )
            dynamic_tracks = self._update_dynamic_tracks(dynamic_clusters)
            dynamic_layer_blocks = self._update_dynamic_obstacle_layer(dynamic_tracks)
            dynamic_layer_prefer_global_replan = (
                bool(self.p_dynamic_layer_prefer_global_replan)
                and dynamic_layer_blocks > 0
            )
            dynamic_motion = self._select_dynamic_motion_estimate(
                path_xy, self._state, projection, dynamic_tracks)
            dynamic_blockage = detect_path_corridor_blockage(
                path_xy=path_xy,
                robot=self._state,
                projection=projection,
                obstacles_local=dynamic_obstacles_local,
                corridor_width=self.p_dynamic_path_corridor_width,
                check_distance=self.p_dynamic_path_check_distance,
                min_points=int(self.p_dynamic_path_min_block_points),
                step=self.p_dynamic_path_corridor_step,
            )
            dynamic_blocked = self._update_dynamic_block_state(
                dynamic_blockage.blocked)
            if dynamic_blocked and not dynamic_blockage.blocked:
                dynamic_blockage = DynamicPathBlockage(
                    True,
                    min(effective_lookahead, self.p_dynamic_path_check_distance),
                    0,
                    0.0,
                    0.0,
                )
            if (dynamic_blocked and not dynamic_layer_prefer_global_replan and
                    dynamic_motion.state in ("STOPPED", "APPROACHING")):
                # 동적 장애물이 계속 막고 있는 동안에는 직전 우회 side를
                # 유지한다. 한두 tick 후보가 사라졌다고 side를 잊으면
                # DYNAMIC_BLOCKED 대기 상태에 쉽게 갇힌다.
                previous_side = self._dynamic_avoid_side
                dynamic_avoid_target = choose_dynamic_avoid_target(
                    path_xy=path_xy,
                    robot=self._state,
                    projection=projection,
                    blockage=dynamic_blockage,
                    obstacles_local=obstacles_local,
                    robot_radius=self.p_robot_radius,
                    lateral_offsets=self.p_dynamic_avoid_lateral_offsets,
                    min_clearance=self.p_dynamic_avoid_min_clearance,
                    min_lookahead=self.p_dynamic_avoid_min_lookahead,
                    max_lookahead=self.p_dynamic_avoid_max_lookahead,
                    rejoin_distance=self.p_dynamic_avoid_rejoin_distance,
                    step=self.p_dynamic_avoid_step,
                    previous_side=previous_side,
                    side_switch_penalty=self.p_dynamic_avoid_side_switch_penalty,
                )
                if dynamic_avoid_target is not None:
                    lookahead = dynamic_avoid_target.point
                    target_path_yaw = dynamic_avoid_target.yaw
                    effective_lookahead = dynamic_avoid_target.distance
                    self._dynamic_avoid_side = dynamic_avoid_target.side
                    self._dynamic_avoid_until = (
                        self._sec_now() + self.p_dynamic_avoid_side_hold_sec
                    )
                    lx, ly = world_to_local(lookahead, self._state)
                    L = math.hypot(lx, ly)
        alpha = math.atan2(ly, lx)
        path_heading_error = normalize_angle(target_path_yaw - self._state.theta)

        fwd_clear = self._forward_clearance_inline(obstacles_local, kappa=self._last_kappa)

        return {
            "path_xy": path_xy,
            "dist_to_goal": dist_to_goal,
            "nearest_idx": nearest_idx,
            "path_offset": path_offset,
            "signed_path_offset": projection.signed_offset,
            "predicted_path_offset": predicted_path_offset,
            "predicted_signed_path_offset": predicted_signed_path_offset,
            "path_heading_error": path_heading_error,
            "effective_lookahead": effective_lookahead,
            "is_rejoining": rejoin_target is not None,
            "short_lookahead_rejoin": short_lookahead_rejoin,
            "rejoin_distance": rejoin_target.distance if rejoin_target else 0.0,
            "rejoin_alpha": rejoin_target.alpha if rejoin_target else 0.0,
            "rejoin_arrival_error": (
                rejoin_target.arrival_error if rejoin_target else 0.0
            ),
            "rejoin_curvature": rejoin_target.curvature if rejoin_target else 0.0,
            "rejoin_clearance": rejoin_target.clearance if rejoin_target else float("inf"),
            "rejoin_desired_distance": (
                rejoin_target.desired_distance if rejoin_target else 0.0
            ),
            "rejoin_score": rejoin_target.score if rejoin_target else 0.0,
            "dynamic_blocked": dynamic_blocked,
            "dynamic_block_count": dynamic_blockage.count,
            "dynamic_block_distance": dynamic_blockage.distance,
            "dynamic_block_side_bias": dynamic_blockage.side_bias,
            "dynamic_block_margin": dynamic_blockage.min_margin,
            "dynamic_held_side": self._dynamic_avoid_side,
            "dynamic_raw_obstacle_count": len(obstacles_local),
            "dynamic_obstacle_count": len(dynamic_obstacles_local),
            "dynamic_static_filtered": dynamic_static_filtered,
            "dynamic_cluster_count": len(dynamic_clusters),
            "dynamic_track_count": len(dynamic_tracks),
            "dynamic_layer_block_count": dynamic_layer_blocks,
            "dynamic_layer_prefer_global_replan": dynamic_layer_prefer_global_replan,
            "dynamic_motion_state": dynamic_motion.state,
            "dynamic_motion_track_id": dynamic_motion.track_id,
            "dynamic_motion_distance": dynamic_motion.distance,
            "dynamic_motion_speed": dynamic_motion.speed,
            "dynamic_motion_closing": dynamic_motion.closing_speed,
            "dynamic_motion_t_cpa": dynamic_motion.t_cpa,
            "dynamic_motion_d_cpa": dynamic_motion.d_cpa,
            "dynamic_motion_age": dynamic_motion.age,
            "dynamic_motion_confidence": dynamic_motion.confidence,
            "dynamic_motion_side": dynamic_motion.side,
            "dynamic_motion_radius": dynamic_motion.radius,
            "is_dynamic_avoiding": dynamic_avoid_target is not None,
            "dynamic_avoid_side": (
                dynamic_avoid_target.side if dynamic_avoid_target else 0
            ),
            "dynamic_avoid_offset": (
                dynamic_avoid_target.offset if dynamic_avoid_target else 0.0
            ),
            "dynamic_avoid_clearance": (
                dynamic_avoid_target.clearance if dynamic_avoid_target else float("inf")
            ),
            "dynamic_avoid_rejoin_clearance": (
                dynamic_avoid_target.rejoin_clearance
                if dynamic_avoid_target else float("inf")
            ),
            "dynamic_avoid_score": (
                dynamic_avoid_target.score if dynamic_avoid_target else 0.0
            ),
            "dynamic_avoid_mode": (
                dynamic_avoid_target.mode if dynamic_avoid_target else ""
            ),
            "lx": lx, "ly": ly, "L": L, "alpha": alpha,
            "obstacles_local": obstacles_local,
            "fwd_clear": fwd_clear,
        }

    # ───────────────────────────────────────────────────────────────
    # 상태 실행 함수들
    # ───────────────────────────────────────────────────────────────
    @staticmethod
    def _dynamic_wait_nav_state(motion_state: str) -> NavState:
        if motion_state == "APPROACHING":
            return NavState.APPROACHING_DYNAMIC
        if motion_state == "CROSSING":
            return NavState.CROSSING_DYNAMIC
        if motion_state == "RECEDING":
            return NavState.RECEDING_DYNAMIC
        if motion_state == "STOPPED":
            return NavState.STOPPED_DYNAMIC
        return NavState.DYNAMIC_BLOCKED

    def _execute_dynamic_approach_escape(self, ctx: dict) -> bool:
        """Short emergency escape when a tracked dynamic obstacle is closing in."""
        obstacles = ctx["obstacles_local"]
        rear_clear = self._rear_clearance_inline(obstacles)
        turn_bias = self._escape_turn_bias(obstacles)
        turn_dir = 1.0 if turn_bias >= 0.0 else -1.0
        if ctx.get("dynamic_motion_side", 0) > 0:
            turn_dir = -1.0
        elif ctx.get("dynamic_motion_side", 0) < 0:
            turn_dir = 1.0

        if (self.p_dynamic_approach_reverse_enabled and
                rear_clear >= self.p_dynamic_approach_reverse_clearance):
            v_cmd = -abs(self.p_dynamic_approach_reverse_speed)
            w_cmd = turn_dir * min(self.p_w_max * 0.35,
                                   abs(self.p_dynamic_approach_turn_speed))
            self._publish_cmd(VelocityCommand(v=v_cmd, w=w_cmd),
                              allow_backward_override=True)
        else:
            self._publish_cmd(VelocityCommand(
                v=0.0,
                w=turn_dir * min(self.p_w_max * 0.45,
                                 abs(self.p_dynamic_approach_turn_speed)),
            ))

        self._nav_state = NavState.APPROACHING_DYNAMIC
        self._relax_path_acceptance()
        self._publish_status_value("APPROACHING_DYNAMIC")
        self._log_state_throttled()
        self.get_logger().warn(
            "approaching dynamic obstacle; escaping conservatively "
            f"(d={ctx.get('dynamic_motion_distance', float('inf')):.2f}m, "
            f"v={ctx.get('dynamic_motion_speed', 0.0):.2f}, "
            f"closing={ctx.get('dynamic_motion_closing', 0.0):.2f}, "
            f"tcpa={ctx.get('dynamic_motion_t_cpa', float('inf')):.2f}, "
            f"dcpa={ctx.get('dynamic_motion_d_cpa', float('inf')):.2f}, "
            f"rear={rear_clear:.2f})",
            throttle_duration_sec=0.5)
        return True

    def _execute_normal(self, ctx: dict) -> None:
        """NavState.NORMAL — Pure Pursuit + adaptive velocity."""
        alpha       = ctx["alpha"]
        lx          = ctx["lx"]
        ly          = ctx["ly"]
        L           = ctx["L"]
        fwd_clear   = ctx["fwd_clear"]
        dist_to_goal= ctx["dist_to_goal"]
        obstacles   = ctx["obstacles_local"]
        path_offset = ctx["path_offset"]
        signed_path_offset = ctx["signed_path_offset"]
        predicted_path_offset = ctx.get("predicted_path_offset", path_offset)
        path_heading_error = ctx["path_heading_error"]
        is_rejoining = ctx.get("is_rejoining", False)
        dynamic_blocked = ctx.get("dynamic_blocked", False)
        is_dynamic_avoiding = ctx.get("is_dynamic_avoiding", False)
        dynamic_motion_state = ctx.get("dynamic_motion_state", "UNKNOWN")

        if is_dynamic_avoiding:
            self._nav_state = NavState.AVOIDING_DYNAMIC
        elif dynamic_blocked:
            self._nav_state = self._dynamic_wait_nav_state(dynamic_motion_state)
        elif is_rejoining:
            self._nav_state = NavState.REJOIN
        elif self._nav_state in (
                NavState.REJOIN,
                NavState.AVOIDING_DYNAMIC,
                NavState.DYNAMIC_BLOCKED,
                NavState.APPROACHING_DYNAMIC,
                NavState.CROSSING_DYNAMIC,
                NavState.RECEDING_DYNAMIC,
                NavState.STOPPED_DYNAMIC):
            self._nav_state = NavState.NORMAL

        period = 1.0 / max(self.p_control_rate, 1.0)
        dv_max = self.p_a_max * period
        dw_max = self.p_alpha_max * period
        dw_brake_max = self.p_w_brake_alpha_max * period

        if dynamic_blocked and not is_dynamic_avoiding:
            prefer_layer_replan = ctx.get("dynamic_layer_prefer_global_replan", False)
            close_approach_escape = (
                prefer_layer_replan
                and dynamic_motion_state == "APPROACHING"
                and ctx.get("dynamic_motion_distance", float("inf"))
                <= self.p_dynamic_layer_escape_distance
                and ctx.get("dynamic_motion_t_cpa", float("inf"))
                <= self.p_dynamic_layer_escape_t_cpa
            )
            if (dynamic_motion_state == "APPROACHING" and
                    (not prefer_layer_replan or close_approach_escape) and
                    self._execute_dynamic_approach_escape(ctx)):
                return
            status_value = self._dynamic_wait_nav_state(dynamic_motion_state).value
            self._relax_path_acceptance()
            self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
            self._publish_status_value(status_value)
            self._log_state_throttled()
            self.get_logger().warn(
                "dynamic obstacle blocks global path corridor; waiting for "
                "dynamic layer replan or clearance "
                f"(hits={ctx.get('dynamic_block_count', 0)}, "
                f"d={ctx.get('dynamic_block_distance', float('inf')):.2f}m, "
                f"bias={ctx.get('dynamic_block_side_bias', 0.0):+.2f}, "
                f"motion={dynamic_motion_state}, "
                f"v={ctx.get('dynamic_motion_speed', 0.0):.2f}, "
                f"closing={ctx.get('dynamic_motion_closing', 0.0):.2f}, "
                f"tcpa={ctx.get('dynamic_motion_t_cpa', float('inf')):.2f}, "
                f"dcpa={ctx.get('dynamic_motion_d_cpa', float('inf')):.2f}, "
                f"held_side={ctx.get('dynamic_held_side', 0):+d}, "
                f"raw_pts={ctx.get('dynamic_raw_obstacle_count', 0)}, "
                f"dyn_pts={ctx.get('dynamic_obstacle_count', 0)}, "
                f"clusters={ctx.get('dynamic_cluster_count', 0)}, "
                f"tracks={ctx.get('dynamic_track_count', 0)}, "
                f"layer_blocks={ctx.get('dynamic_layer_block_count', 0)}, "
                f"static_filtered={ctx.get('dynamic_static_filtered', 0)})",
                throttle_duration_sec=1.0)
            return

        # ── ALIGN 전이 판정 ──────────────────────────────────────
        # 각도 hysteresis(63° in / 15° out)에 더해, P4 시간 hysteresis:
        #   |alpha|>thresh 가 align_trigger_ticks(기본 3틱 ≈ 150ms) 연속이어야 진입.
        #   AMCL drift 로 한 tick 만 α 폭증해도 즉시 ALIGN 진입하지 않게 함.
        if self._in_align_mode:
            if abs(alpha) < self.p_align_angle_exit:
                self._in_align_mode = False
                self._align_trigger_count = 0
        else:
            align_entry_thresh = (
                self.p_rejoin_align_angle_thresh
                if (is_rejoining or is_dynamic_avoiding)
                else self.p_align_angle_thresh
            )
            if abs(alpha) > align_entry_thresh and L > 0.1:
                self._align_trigger_count += 1
                if self._align_trigger_count >= self.p_align_trigger_ticks:
                    self._in_align_mode = True
                    self._align_trigger_count = 0
            else:
                self._align_trigger_count = 0   # 연속성 끊김 → 리셋

        if self._in_align_mode:
            self._nav_state = NavState.ALIGN
            self._execute_align(ctx)
            return

        # ── Pure Pursuit ─────────────────────────────────────────
        kappa = 2.0 * ly / (L * L) if L >= 1e-3 else 0.0
        self._last_kappa = kappa
        fwd_clear = self._forward_clearance_inline(obstacles, kappa=kappa)

        v_target = self.p_v_max

        # (i) 곡률 기반 감속
        if abs(kappa) > 1e-3:
            v_curve = math.sqrt(self.p_a_lat_max / abs(kappa))
            v_target = min(v_target, v_curve)

        # (ii) arc clearance 기반 감속 — trajectory_clearance 만 사용
        # fwd_clear(부채꼴)는 측면 장애물을 전방으로 오인해 불필요한 감속 유발.
        # trajectory_clearance는 실제 진행 arc 위 장애물만 보므로
        # 측면 벽 옆을 지날 때 감속하지 않고, 진행 방향 장애물에만 반응.
        probe_v = max(0.05, min(self.p_v_max, v_target))
        probe_w = kappa * probe_v
        motion_clear = self._trajectory_clearance_margin(obstacles, probe_v, probe_w)
        cf = self.p_clearance_stop_distance
        cc = max(self.p_clearance_slowdown_distance, cf + 1e-3)
        near_wall_creep = False
        if motion_clear < cc:
            v_clear = self.p_v_max * max(0.0, motion_clear - cf) / max(cc - cf, 1e-3)
            near_wall_creep = self._allow_near_wall_creep(
                motion_clear,
                fwd_clear,
                is_rejoining=is_rejoining,
            )
            if near_wall_creep:
                v_clear = max(v_clear, self.p_near_wall_creep_speed)
            v_target = min(v_target, v_clear)

        dynamic_target_brake = False
        if is_dynamic_avoiding:
            dynamic_target_clear = ctx.get("dynamic_avoid_clearance", float("inf"))
            if dynamic_target_clear < cc:
                v_dynamic_clear = (
                    self.p_v_max
                    * max(0.0, dynamic_target_clear - cf)
                    / max(cc - cf, 1e-3)
                )
                if v_dynamic_clear < v_target - 1e-3:
                    dynamic_target_brake = True
                v_target = min(v_target, v_dynamic_clear)

        # (iii) goal 감속
        v_goal = math.sqrt(2.0 * self.p_a_max *
                           max(0.0, dist_to_goal - self.p_goal_tolerance))
        v_target = min(v_target, v_goal)
        v_goal_approach = goal_approach_speed_limit(
            dist_to_goal,
            self.p_goal_tolerance,
            self.p_goal_approach_distance,
            self.p_goal_approach_speed,
        )
        v_target = min(v_target, v_goal_approach)

        # (iv) heading 감속
        heading_factor = max(0.3, math.cos(alpha))
        v_target *= heading_factor

        # (v) path 이탈 감속 — 경로에서 벌어질수록 속도를 낮춰 복귀 회전을 우선한다.
        path_error_for_speed = max(path_offset, predicted_path_offset)
        if path_error_for_speed > self.p_path_error_slowdown_offset:
            denom = max(
                self.p_path_lost_offset - self.p_path_error_slowdown_offset,
                1e-3,
            )
            scale = 1.0 - (
                path_error_for_speed - self.p_path_error_slowdown_offset
            ) / denom
            scale = max(self.p_path_error_min_speed_scale, min(1.0, scale))
            v_target *= scale

        # ── 최종 한계 (2026-05-31: P3 v_target LPF 제거 — 반응성 회복) ─────────
        # P3 LPF 는 상향(재가속)을 smooth 하느라 직선·감속 후 재가속이 굼떠
        # 주행이 "빠릿"하지 않다는 사용자 피드백 → revert. v_target 즉시 반영.
        # (실제 명령 v_cmd 는 아래에서 가속도 제한 dv_max 로 여전히 부드럽게 변함.)
        # (CLAUDE.md §7.2.0 #4: 이전보다 후퇴한 수정은 revert.)
        v_target = max(0.0, min(self.p_v_max, v_target))
        wall_escape_active = False
        wall_escape_w = 0.0
        if dist_to_goal > max(self.p_goal_approach_distance,
                              self.p_goal_tolerance + 0.5):
            escape_speed_floor, wall_escape_w, wall_escape_active = (
                near_wall_escape_adjustment(
                    motion_clear=motion_clear,
                    forward_clearance=fwd_clear,
                    curvature=kappa,
                    escape_bias=self._escape_turn_bias(obstacles),
                    stop_distance=self.p_clearance_stop_distance,
                    slowdown_distance=self.p_clearance_slowdown_distance,
                    escape_clearance=self.p_near_wall_escape_clearance,
                    escape_speed=self.p_near_wall_escape_speed,
                    escape_turn=self.p_near_wall_escape_turn,
                    escape_max_curvature=self.p_near_wall_escape_max_curvature,
                    acceleration=self.p_a_max,
                )
            )
            if wall_escape_active:
                v_target = max(v_target, escape_speed_floor)

        # w 계산 (v/w 커플링 해제) — 곡률 감속이 반영된 v_target 으로 ω 를 계산해
        #   실행 곡률 w_cmd/v_cmd ≈ κ 정합 유지.
        w_pp = kappa * v_target
        cross_track_gain = self.p_path_cross_track_gain
        if is_rejoining:
            cross_track_gain *= self.p_rejoin_cross_track_gain_scale
        if is_dynamic_avoiding:
            cross_track_gain *= self.p_dynamic_avoid_cross_track_gain_scale
        w_path = (
            self.p_path_heading_gain * path_heading_error
            - cross_track_gain * signed_path_offset
        )
        w_target_raw = w_pp + w_path + wall_escape_w
        if (self.p_w_min_rotate > 0.0
                and abs(alpha) > self.p_align_angle_exit
                and abs(w_target_raw) < self.p_w_min_rotate):
            turn_ref = alpha if abs(alpha) >= abs(path_heading_error) else path_heading_error
            w_target = math.copysign(self.p_w_min_rotate, turn_ref)
        else:
            w_target = w_target_raw

        # 가속도 제한
        w_target = max(-self.p_w_max, min(self.p_w_max, w_target))
        turn_intensity = turn_demand_intensity(
            alpha,
            path_heading_error,
            w_target,
            self.p_w_max,
            self.p_turn_clearance_brake_angle,
        )
        turn_brake_before = v_target
        turn_clearance_ref = motion_clear if is_dynamic_avoiding else fwd_clear
        v_target = turn_clearance_speed_limit(
            v_target,
            turn_clearance_ref,
            self.p_clearance_stop_distance,
            self.p_a_max,
            turn_intensity,
        )
        turn_brake_active = v_target < turn_brake_before - 1e-3
        if turn_brake_active:
            w_pp = kappa * v_target
            w_target_raw = w_pp + w_path + wall_escape_w
            if (self.p_w_min_rotate > 0.0
                    and abs(alpha) > self.p_align_angle_exit
                    and abs(w_target_raw) < self.p_w_min_rotate):
                turn_ref = alpha if abs(alpha) >= abs(path_heading_error) else path_heading_error
                w_target = math.copysign(self.p_w_min_rotate, turn_ref)
            else:
                w_target = w_target_raw
            w_target = max(-self.p_w_max, min(self.p_w_max, w_target))

        dv_brake_max = max(dv_max, self.p_v_brake_a_max * period)
        v_cmd = rate_limit_linear_velocity(
            self._state.v, v_target, dv_max, dv_brake_max, self.p_allow_backward)
        w_cmd = rate_limit_angular_velocity(
            self._state.w, w_target, dw_max, dw_brake_max)

        # 최종 충돌 체크
        sim_state = RobotState(x=0.0, y=0.0, theta=0.0,
                               v=self._state.v, w=self._state.w)
        sim_traj = forward_simulate(sim_state, v_cmd, w_cmd,
                                    self.p_dt, self.p_predict_horizon)
        if sim_traj:
            sim_traj_xy = [(p[0], p[1]) for p in sim_traj[2:]] or \
                          [(p[0], p[1]) for p in sim_traj]
            min_d = min_clearance_distance(sim_traj_xy, obstacles)
        else:
            sim_traj_xy = []; min_d = float("inf")

        collision_imminent = (
            min_d < (self.p_hard_collision_distance + self.p_robot_radius) or
            # fwd_clear 보조: 측면 대각선 동적 장애물 등 arc 밖 위협 감지
            (not is_dynamic_avoiding and
             fwd_clear < (self.p_hard_collision_distance + self.p_robot_radius))
        )

        # stuck/velocity blocked 판정
        # motion_clear: arc 위 장애물 (주 기준)
        # fwd_clear: 부채꼴 장애물 (보조 — arc 밖 측면 벽이 v를 낮춘 경우 커버)
        is_velocity_blocked = (abs(v_cmd) < 0.02 and
                               ((motion_clear < self.p_clearance_stop_distance
                                 and not near_wall_creep) or
                                (not is_dynamic_avoiding and
                                 fwd_clear < self.p_clearance_stop_distance)))
        in_recovery_cooldown = self._sec_now() < self._recovery_cooldown_until

        if collision_imminent or is_velocity_blocked:
            self._relax_path_acceptance()
            if collision_imminent:
                self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                self._publish_status_value("EMERGENCY")
            else:
                self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                self._publish_status_value("STOPPED_NEAR_WALL")

            if not in_recovery_cooldown:
                self._stuck_counter += 1
                stuck_threshold = int(self.p_control_rate * self.p_stuck_recovery_sec)
                if self._stuck_counter > stuck_threshold:
                    self._trigger_recovery(obstacles, motion_clear)
            self._log_state_throttled()
            if sim_traj:
                self._publish_best_trajectory(sim_traj)
            return

        # 정상 발행
        self._stuck_counter = 0
        self._publish_cmd(VelocityCommand(v=v_cmd, w=w_cmd))
        if is_dynamic_avoiding:
            status_value = "AVOIDING_DYNAMIC"
        elif is_rejoining:
            status_value = "REJOIN"
        else:
            status_value = "NORMAL"
        self._publish_status_value(status_value)
        self._log_state_throttled()
        if sim_traj:
            self._publish_best_trajectory(sim_traj)

        # 진단 로그
        dynamic_diag = ""
        if dynamic_blocked or is_dynamic_avoiding:
            dynamic_diag = (
                f"{' dyn=1' if is_dynamic_avoiding else ''}"
                f"{' dynblk=1' if dynamic_blocked else ''}"
                f" dyn_d={ctx.get('dynamic_block_distance', float('inf')):.2f}"
                f" dyn_side={ctx.get('dynamic_avoid_side', 0):+d}"
                f" dyn_off={ctx.get('dynamic_avoid_offset', 0.0):.2f}"
                f" dyn_mode={ctx.get('dynamic_avoid_mode', '')}"
                f" dyn_clr={ctx.get('dynamic_avoid_clearance', float('inf')):.2f}"
                f" dyn_rjc={ctx.get('dynamic_avoid_rejoin_clearance', float('inf')):.2f}"
                f" dyn_motion={ctx.get('dynamic_motion_state', 'UNKNOWN')}"
                f" dyn_v={ctx.get('dynamic_motion_speed', 0.0):.2f}"
                f" dyn_close={ctx.get('dynamic_motion_closing', 0.0):.2f}"
                f" dyn_tcpa={ctx.get('dynamic_motion_t_cpa', float('inf')):.2f}"
                f" dyn_dcpa={ctx.get('dynamic_motion_d_cpa', float('inf')):.2f}"
                f" dyn_age={ctx.get('dynamic_motion_age', 0)}"
                f" dyn_raw={ctx.get('dynamic_raw_obstacle_count', 0)}"
                f" dyn_pts={ctx.get('dynamic_obstacle_count', 0)}"
                f" dyn_clusters={ctx.get('dynamic_cluster_count', 0)}"
                f" dyn_tracks={ctx.get('dynamic_track_count', 0)}"
                f" dyn_layer={ctx.get('dynamic_layer_block_count', 0)}"
                f" dyn_static={ctx.get('dynamic_static_filtered', 0)}"
            )
        self._candidate_log_counter += 1
        target = int(self.p_control_rate * self.p_candidate_log_period)
        if self.p_candidate_log_period > 0.0 and \
           self._candidate_log_counter >= max(1, target):
            self._candidate_log_counter = 0
            self.get_logger().info(
                f"PP[{status_value}]: la=({lx:+.2f},{ly:+.2f}) "
                f"L={L:.2f}(eff={ctx['effective_lookahead']:.2f}) "
                f"α={math.degrees(alpha):+.1f}° κ={kappa:+.2f} "
                f"v={v_cmd:+.2f}/{v_target:.2f} w={w_cmd:+.2f}/{w_target:+.2f} "
                f"cte={path_offset:.2f}/{signed_path_offset:+.2f} "
                f"pcte={predicted_path_offset:.2f} "
                f"rj={ctx['rejoin_distance']:.2f}/"
                f"{math.degrees(ctx['rejoin_arrival_error']):+.1f}° "
                f"rjc={ctx['rejoin_clearance']:.2f} "
                f"ψ={math.degrees(path_heading_error):+.1f}° "
                f"clr={motion_clear:.2f} fwd={fwd_clear:.2f} "
                f"d_goal={dist_to_goal:.2f}"
                f"{' tbrake=1' if turn_brake_active else ''}"
                f"{' dynbrake=1' if dynamic_target_brake else ''}"
                f"{' wesc=1' if wall_escape_active else ''}"
                f"{' sj=1' if ctx.get('short_lookahead_rejoin', False) else ''}"
                f"{dynamic_diag}"
                f"{' creep=1' if near_wall_creep else ''}")

    def _execute_align(self, ctx: dict) -> None:
        """NavState.ALIGN — In-place PD 회전."""
        alpha     = ctx["alpha"]
        fwd_clear = ctx["fwd_clear"]
        obstacles = ctx["obstacles_local"]
        is_rejoining = ctx.get("is_rejoining", False)
        fwd_clear = self._forward_clearance_inline(obstacles, kappa=0.0)

        period = 1.0 / max(self.p_control_rate, 1.0)
        dv_max = self.p_a_max * period
        dw_max = self.p_alpha_max * period
        dw_brake_max = self.p_w_brake_alpha_max * period

        # ALIGN 종료 → NORMAL 전이
        if abs(alpha) < self.p_align_angle_exit:
            self._in_align_mode = False
            self._nav_state = NavState.NORMAL
            self._align_cooldown_until = self._sec_now() + self.p_align_cooldown
            return

        # rotate clearance 체크
        rotate_clear = self._rotation_clearance_inline(obstacles)
        motion_clear = rotate_clear

        if should_release_align(
                alpha,
                is_rejoining,
                self.p_align_release_angle,
                self.p_rejoin_align_release_angle,
                rotate_clear,
                self.p_align_release_clearance):
            self._in_align_mode = False
            self._nav_state = NavState.REJOIN if is_rejoining else NavState.NORMAL
            self._align_trigger_count = 0
            self._align_cooldown_until = self._sec_now() + self.p_align_cooldown
            self._publish_status_value(self._nav_state.value)
            return

        v_target = 0.0
        drive_angle = max(self.p_align_drive_angle, self.p_align_angle_exit + 1e-3)
        if (self.p_align_v_blend_max > 0.0 and
                rotate_clear > self.p_clearance_slowdown_distance and
                fwd_clear > self.p_clearance_slowdown_distance and
                abs(alpha) < drive_angle):
            blend = 1.0 - (abs(alpha) - self.p_align_angle_exit) / \
                max(drive_angle - self.p_align_angle_exit, 1e-3)
            v_target = self.p_align_v_blend_max * max(0.0, min(1.0, blend))
        if ctx["dist_to_goal"] < self.p_goal_align_stop_distance:
            v_target = 0.0

        w_target = (self.p_align_kp * alpha - self.p_align_kd * self._state.w)
        w_target = max(-self.p_w_max * 0.7, min(self.p_w_max * 0.7, w_target))

        dv_brake_max = max(dv_max, self.p_v_brake_a_max * period)
        v_cmd = rate_limit_linear_velocity(
            self._state.v, v_target, dv_max, dv_brake_max, self.p_allow_backward)
        w_cmd = rate_limit_angular_velocity(
            self._state.w, w_target, dw_max, dw_brake_max)

        # align 중 stuck 체크
        is_velocity_blocked = (abs(v_cmd) < 0.02 and
                               motion_clear < self.p_clearance_stop_distance)
        in_recovery_cooldown = self._sec_now() < self._recovery_cooldown_until

        if is_velocity_blocked and not in_recovery_cooldown:
            self._stuck_counter += 1
            stuck_threshold = int(self.p_control_rate * self.p_stuck_recovery_sec)
            if self._stuck_counter > stuck_threshold:
                self._trigger_recovery(obstacles, motion_clear)
                return

        self._publish_cmd(VelocityCommand(v=v_cmd, w=w_cmd))
        self._publish_status_value("ALIGN")
        self._log_state_throttled()

    def _execute_spin(self) -> None:
        """NavState.SPIN — Recovery spin (제자리 회전으로 탈출 방향 확보).

        종료 조건 분리:
          spin_unsafe  : 몸체 반경 기준 회전 공간 없음(충돌 임박) → 즉시 정지(EMERGENCY)
          spin_done    : 직진 arc 가 열렸음 (전진 가능) → FORWARD_ONLY
          그 외         : 계속 회전 (spin_duration 초과 시 EMERGENCY)

        "회전이 안전한가" 기준 = _rotation_clearance_inline (제자리 회전, 실제 명령 v=0 과 일치).
        "전방이 열렸다" 기준 = trajectory_clearance(v>0, w=0).
        측면 벽 때문에 spin이 무한 지속되지 않음(spin_duration 으로도 상한).
        """
        now = self._sec_now()
        obstacles = self._extract_obstacles_from_scan()

        # 회전 자체가 안전한가 — 제자리 회전(실제 명령 v=0)이므로 전진 arc 가 아니라
        # 몸체 반경 기준 clearance 로 판단한다.
        # [Codex T4 P1] 이전엔 전진 0.45m/s arc(_trajectory_clearance_margin)로 검사해,
        #   전방에 가까운 장애물이 있으면 제자리 회전은 안전한데도 EMERGENCY 로 오판했음.
        #   _rotation_clearance_inline 은 명령(v=0)과 일치하는 몸체-반경 기준 여유거리.
        rotate_clear = self._rotation_clearance_inline(obstacles)
        spin_unsafe = rotate_clear < self.p_hard_collision_distance

        # 전진 가능해졌는가 — 직진 arc 기준 (측면 벽 무시)
        fwd_probe_v = max(0.05, self.p_v_max * 0.3)
        forward_clear = self._trajectory_clearance_margin(
            obstacles, fwd_probe_v, 0.0)
        safe_forward_dist = safe_forward_only_distance(
            forward_clear,
            self.p_clearance_stop_distance,
            self.p_spin_forward_clearance_margin,
            self.p_forward_only_dist,
        )
        spin_done = safe_forward_dist >= self.p_forward_only_min_dist

        if now < self._spin_until and not spin_unsafe and not spin_done:
            # 회전 안전 + 전방 아직 막힘 → 계속 회전
            w_spin = self._spin_direction * self.p_w_max
            self._publish_cmd(VelocityCommand(v=0.0, w=w_spin))
            self._publish_status_value("RECOVERY")
            self._log_state_throttled()
            return

        # 종료 — 사유 판단
        self._spin_until = 0.0

        if spin_done:
            # 전진 가능 → FORWARD_ONLY로 전환 (현재 heading으로 짧게 전진)
            # 위치가 바뀐 후 A* 재계획 → rack 모서리 벗어난 경로 생성
            self._forward_only_until   = now + self.p_forward_only_timeout
            self._forward_only_dist    = safe_forward_dist
            self._forward_only_start_x = self._state.x
            self._forward_only_start_y = self._state.y
            self._nav_state  = NavState.FORWARD_ONLY
            self._stuck_counter = 0
            self._path_local = None   # 기존 path 무효화
            self._path_progress_idx = 0
            self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
            self._publish_status_value("FORWARD_ONLY")
            self.get_logger().info(
                f"spin 완료 → FORWARD_ONLY {self._forward_only_dist:.2f}m 전진 후 A* 재계획 "
                f"(forward_clear={forward_clear:.2f}m, margin={self.p_spin_forward_clearance_margin:.2f}m)")
        elif spin_unsafe:
            # 회전 공간 없음(몸체 충돌 임박) → 후진 대신 즉시 정지(EMERGENCY).
            # [Codex T4 P2] 위험 판정 tick 에 같은 tick 으로 정지 명령을 발행한다
            #   (안 하면 다음 tick _execute_emergency 까지 이전 회전 명령이 1 tick 유지됨).
            self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
            self._publish_status_value("EMERGENCY")
            self._nav_state = NavState.EMERGENCY
            self._stuck_counter = 0
            self._recovery_cooldown_until = now + self.p_recovery_cooldown
            self.get_logger().warn(
                f"spin 중 회전 공간 없음 → EMERGENCY (clear={rotate_clear:.2f}m). A* 재계획 대기.")
        else:
            # spin_duration 초과인데 전방 미확보 → 같은 tick 즉시 정지(EMERGENCY) + 재계획 대기.
            # (2026-05-31: 후진 대신 회전만 시도했고 전방이 안 열렸으므로 정지.)
            self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
            self._publish_status_value("EMERGENCY")
            self._nav_state = NavState.EMERGENCY
            self._stuck_counter = 0
            self._recovery_cooldown_until = now + self.p_recovery_cooldown
            self.get_logger().warn(
                f"spin timeout → EMERGENCY (forward={forward_clear:.2f}m). A* 재계획 대기.")

    def _execute_forward_only(self) -> None:
        """NavState.FORWARD_ONLY — spin 완료 후 현재 heading으로 짧게 전진.

        목적: spin으로 heading을 바꾼 후 위치를 이동시켜야 A*가 다른 경로를 생성.
              제자리에서 재계획하면 A*가 또 같은 rack 방향 경로를 줄 수 있음.
        종료: 목표 거리 도달 or 시간 초과 or 전방 장애물 감지 → A* 재계획 대기
        """
        now = self._sec_now()
        obstacles = self._extract_obstacles_from_scan()

        # 전방 안전 체크
        forward_clear = self._trajectory_clearance_margin(
            obstacles, self.p_v_max * 0.3, 0.0)
        collision_near = forward_clear < self.p_clearance_stop_distance

        # 이동 거리 / path 복귀 체크
        dist_moved = math.hypot(
            self._state.x - self._forward_only_start_x,
            self._state.y - self._forward_only_start_y)
        path_offset = (
            self._path_offset_to_state(self._path_local)
            if self._path_local is not None and self._path_local.poses
            else None
        )
        should_finish, reason = should_finish_forward_only(
            now=now,
            until=self._forward_only_until,
            dist_moved=dist_moved,
            target_dist=self._forward_only_dist,
            collision_near=collision_near,
            path_offset=path_offset,
            min_dist_before_path_exit=self.p_forward_only_min_dist,
            path_rejoin_offset=self.p_forward_only_rejoin_offset,
        )

        if not should_finish:
            if abs(self._state.w) > self.p_forward_only_settle_w:
                self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                self._publish_status_value("RECOVERY")
                self._log_state_throttled()
                return
            # 현재 heading 유지하며 직진
            period = 1.0 / max(self.p_control_rate, 1.0)
            dv_max = self.p_a_max * period
            v_target = self.p_v_max * self.p_forward_only_speed_scale
            dv_brake_max = max(dv_max, self.p_v_brake_a_max * period)
            v_cmd = rate_limit_linear_velocity(
                self._state.v, v_target, dv_max, dv_brake_max, self.p_allow_backward)
            self._publish_cmd(VelocityCommand(v=v_cmd, w=0.0))
            self._publish_status_value("RECOVERY")
            self._log_state_throttled()
            return

        # 종료 → A* 재계획 대기
        self._forward_only_until = 0.0
        self._nav_state = NavState.NORMAL
        self._stuck_counter = 0
        self._recovery_cooldown_until = now + self.p_recovery_cooldown

        self._publish_status_value("RECOVERY_DONE")
        self.get_logger().info(
            f"FORWARD_ONLY 완료 → A* 재계획 대기 ({reason})")

    def _execute_emergency(self, ctx: dict) -> None:
        """NavState.EMERGENCY — 완전 정지 + A* 재계획 대기."""
        self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
        self._publish_status_value("EMERGENCY")
        self.get_logger().warn(
            "EMERGENCY: 전후방 막힘. A* 재계획 대기.",
            throttle_duration_sec=2.0)
        # 새 path 수신 시 _reset_path_state 에서 NORMAL 복귀

    # ───────────────────────────────────────────────────────────────
    # Recovery 트리거 — stuck 임계치 초과 시 호출
    # ───────────────────────────────────────────────────────────────
    def _trigger_recovery(self, obstacles_local, motion_clear: float) -> None:
        """Recovery 우선순위: SPIN → EMERGENCY (2026-05-31: 후진 BACKUP 제거).

        stuck 시 제자리 회전(SPIN)으로만 탈출을 시도한다. 회전 공간조차 없으면
        후진하지 않고 즉시 EMERGENCY 정지 → A* 재계획을 기다린다.
        (사용자 지시: 후진 거동이 번거로워 회전만으로 복귀. CLAUDE.md §7.2.0.)
        """
        now = self._sec_now()
        self._relax_path_acceptance()

        # Step 1: spin recovery 시도
        # 제자리 회전은 이동 없음 → 몸체(robot_radius)에 안 닿으면 회전 가능.
        # [Codex T4 P1] _rotation_clearance_inline 은 이미 (중심거리 - robot_radius) 한
        #   '여유 거리'다. robot_radius 와 비교하면 중심이 ~2·robot_radius 밖이어야 해서
        #   과도하게 보수적 → 후진이 없는 지금, 회전 가능한데도 EMERGENCY 로 가버린다.
        #   여유가 hard_collision 마진보다 크면(=몸체가 안 닿으면) SPIN 시도.
        rotate_clear = self._rotation_clearance_inline(obstacles_local)
        if rotate_clear > self.p_hard_collision_distance:
            self._spin_direction = (
                1.0 if self._escape_turn_bias(obstacles_local) >= 0.0 else -1.0
            )
            self._spin_until = now + self.p_spin_duration
            self._nav_state = NavState.SPIN
            self._stuck_counter = 0
            self._recovery_cooldown_until = self._spin_until + self.p_recovery_cooldown
            self.get_logger().warn(
                f"stuck → SPIN recovery {self.p_spin_duration}s "
                f"(rotate_clear={rotate_clear:.2f}m, "
                f"dir={'LEFT' if self._spin_direction > 0 else 'RIGHT'})")
            return

        # Step 2: 회전 공간조차 없음 → 후진 대신 즉시 EMERGENCY 정지
        self._nav_state = NavState.EMERGENCY
        self._stuck_counter = 0
        self._recovery_cooldown_until = now + self.p_recovery_cooldown
        self._relax_path_acceptance()
        self.get_logger().warn(
            f"stuck + 회전 공간 없음 → EMERGENCY "
            f"(rotate_clear={rotate_clear:.2f}m, motion_clear={motion_clear:.2f}m). "
            f"A* 재계획 대기.",
            throttle_duration_sec=2.0)

    # ───────────────────────────────────────────────────────────────
    # Clearance 헬퍼들 (기존과 동일)
    # ───────────────────────────────────────────────────────────────
    def _allow_near_wall_creep(
        self,
        motion_clear: float,
        fwd_clear: float,
        is_rejoining: bool = False,
    ) -> bool:
        """전방은 열려 있고 측면 여유만 낮을 때 최소 전진을 허용한다."""
        if self.p_near_wall_creep_speed <= 0.0:
            return False
        if fwd_clear <= self.p_clearance_slowdown_distance:
            return False
        side_margin_floor = max(
            self.p_clearance_stop_distance,
            self.p_robot_radius + self.p_hard_collision_distance,
            self.p_near_wall_creep_min_clearance,
        )
        if is_rejoining:
            side_margin_floor = max(
                side_margin_floor,
                self.p_rejoin_creep_min_clearance,
            )
        return motion_clear >= side_margin_floor

    def _forward_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
        kappa: float = 0.0,
    ) -> float:
        """진행 arc 방향 기준 동적 부채꼴 (kappa 비례 각도)."""
        if not obstacles_local:
            return float("inf")
        half_angle = min(math.radians(50),
                         math.radians(20) + 0.8 * abs(kappa))
        best = float("inf")
        for ox, oy in obstacles_local:
            if ox < 0.0:
                continue
            ang = math.atan2(oy, ox)
            if abs(ang) > half_angle:
                continue
            d_eff = max(0.0, math.hypot(ox, oy) - self.p_robot_radius)
            if d_eff < best:
                best = d_eff
        return best

    def _trajectory_clearance_margin(
        self,
        obstacles_local: List[Tuple[float, float]],
        v_cmd: float,
        w_cmd: float,
    ) -> float:
        """현재 명령 arc 주변의 최단 여유 거리."""
        if not obstacles_local:
            return float("inf")
        sim_state = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        horizon = max(0.35, self.p_predict_horizon)
        traj = forward_simulate(sim_state, v_cmd, w_cmd, self.p_dt, horizon)
        if not traj:
            return float("inf")
        traj_xy = [(p[0], p[1]) for p in traj[2:]] or [(p[0], p[1]) for p in traj]
        center_dist = min_clearance_distance(traj_xy, obstacles_local)
        if center_dist == float("inf"):
            return float("inf")
        return max(0.0, center_dist - self.p_robot_radius)

    def _rotation_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
    ) -> float:
        """제자리 회전 시 로봇 몸체 반경 기준 최단 장애물 거리.

        기존 전방향 safety_distance 체크의 문제:
          옆 벽(0.4m 거리)도 위험으로 판정 → rotate_clear < safety_distance
          → SPIN 불가 → 데드락(과거엔 즉시 BACKUP 했으나 2026-05-31 후진 제거).

        수정: 제자리 회전은 이동이 없으므로 몸체(robot_radius)에
          실제로 닿는 점만 위험. 옆 벽은 닿지 않으면 회전 가능.
        """
        if not obstacles_local:
            return float("inf")
        # 성능 필터: scan_range_max 대신 합리적 범위만 체크
        # robot_radius(0.2) * 8 = 1.6m — 제자리 회전에 영향 줄 수 있는 범위
        body_range = self.p_robot_radius * 8.0
        best = float("inf")
        for ox, oy in obstacles_local:
            d = math.hypot(ox, oy)
            if d < body_range and d < best:
                best = d
        return max(0.0, best - self.p_robot_radius)

    def _rear_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
    ) -> float:
        """Approximate rear clearance for short dynamic-obstacle retreat."""
        if not obstacles_local:
            return float("inf")
        best = float("inf")
        for ox, oy in obstacles_local:
            if ox > 0.10:
                continue
            if abs(oy) > max(0.45, self.p_robot_radius * 2.5):
                continue
            d = math.hypot(ox, oy)
            if d < best:
                best = d
        if best == float("inf"):
            return float("inf")
        return max(0.0, best - self.p_robot_radius)

    def _escape_turn_bias(self, obstacles_local: List[Tuple[float, float]]) -> float:
        """SPIN 회전 방향 결정용 부호. 양수 → 왼쪽(+), 음수 → 오른쪽(-) 회전.

        전방 근처(1.2m 이내) 장애물의 좌/우 분포를 모아 가까운 벽의 *반대쪽*으로
        도는 방향을 고른다. 크기는 무의미하고 부호만 쓰인다(_trigger_recovery 가
        `>= 0` 으로 LEFT/RIGHT 선택). 2026-05-31: 후진 BACKUP 제거로 backup_turn_*
        파라미터 의존을 끊고 순수 방향 함수로 단순화(이전: _escape_turn_rate).
        """
        if not obstacles_local:
            return 0.0
        side_bias = 0.0
        for ox, oy in obstacles_local:
            if ox < -0.20:
                continue
            d = math.hypot(ox, oy)
            if d < 1e-3 or d > 1.2:
                continue
            side_bias += (1.0 if oy >= 0.0 else -1.0) / d
        # 가까운 벽 반대쪽으로 회전: 벽이 왼쪽(side_bias>0)이면 오른쪽(-)으로 → 부호 반전.
        return -side_bias

    # ───────────────────────────────────────────────────────────────
    # path 변환 (기존과 동일)
    # ───────────────────────────────────────────────────────────────
    def _transform_path_to_local(self, msg: Path) -> Optional[Path]:
        try:
            t = self._tf_buffer.lookup_transform(
                self.LOCAL_FRAME, self.GLOBAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.p_path_tf_timeout))
        except (TransformException, tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            if not self._path_warn_logged:
                self.get_logger().warn(
                    f"path 변환 TF lookup 실패: {e} (이전 path 유지)")
                self._path_warn_logged = True
            return None
        if self._path_warn_logged:
            self.get_logger().info("path 변환 TF 복구")
            self._path_warn_logged = False

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q  = t.transform.rotation
        tyaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        cos_t = math.cos(tyaw)
        sin_t = math.sin(tyaw)

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.LOCAL_FRAME

        from geometry_msgs.msg import PoseStamped
        for p_in in msg.poses:
            p_out = PoseStamped()
            p_out.header.stamp = out.header.stamp
            p_out.header.frame_id = self.LOCAL_FRAME
            px = p_in.pose.position.x
            py = p_in.pose.position.y
            p_out.pose.position.x = tx + cos_t * px - sin_t * py
            p_out.pose.position.y = ty + sin_t * px + cos_t * py
            p_out.pose.position.z = 0.0
            qi_x = p_in.pose.orientation.x
            qi_y = p_in.pose.orientation.y
            qi_z = p_in.pose.orientation.z
            qi_w = p_in.pose.orientation.w
            in_yaw = yaw_from_quaternion(qi_x, qi_y, qi_z, qi_w)
            out_yaw = in_yaw + tyaw
            p_out.pose.orientation.z = math.sin(out_yaw / 2.0)
            p_out.pose.orientation.w = math.cos(out_yaw / 2.0)
            out.poses.append(p_out)
        return out

    # ───────────────────────────────────────────────────────────────
    # 헬퍼
    # ───────────────────────────────────────────────────────────────
    def _sec_now(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    @staticmethod
    def _path_xy(path_msg: Path) -> List[Tuple[float, float]]:
        return [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]

    def _should_ignore_empty_path(self) -> bool:
        """새 goal이 아닌 빈 path가 기존 유효 path를 지우지 않도록 한다."""
        if self._path_local is None or not self._path_local.poses:
            return False
        if self._path_goal_version == self._goal_version:
            return True
        if self._path_goal_matches_last_goal():
            # goal edge가 늦게/중복으로 들어와 version만 어긋난 경우다.
            # 이미 같은 goal의 유효 path가 있으므로 stale empty가 지우지 못하게 동기화한다.
            self._path_goal_version = self._goal_version
            return True
        return False

    def _path_goal_matches_last_goal(self) -> bool:
        """수신한 path의 map-frame goal이 마지막 goal_pose와 같은 목표인지 확인한다."""
        if self._path_goal_xy_global is None or self._last_goal_xy is None:
            return False
        dx = self._path_goal_xy_global[0] - self._last_goal_xy[0]
        dy = self._path_goal_xy_global[1] - self._last_goal_xy[1]
        return math.hypot(dx, dy) < self.p_goal_dedup_dist

    def _current_xy(self) -> Optional[Tuple[float, float]]:
        if self._state is None:
            return None
        return (self._state.x, self._state.y)

    def _goal_match_radius(self) -> float:
        return max(
            self.p_goal_dedup_dist * 3.0,
            self.p_goal_tolerance + 0.10,
            0.75,
        )

    def _adopt_path_goal_if_needed(
        self,
        path_goal_xy_global: Tuple[float, float],
    ) -> None:
        if self._last_goal_xy is not None:
            dx = path_goal_xy_global[0] - self._last_goal_xy[0]
            dy = path_goal_xy_global[1] - self._last_goal_xy[1]
            if math.hypot(dx, dy) <= self._goal_match_radius():
                return
        self._last_goal_xy = path_goal_xy_global
        if self._last_goal_yaw is None:
            self._last_goal_yaw = 0.0
        self._goal_version += 1

    def _is_duplicate_goal(self,
                           new_goal: Tuple[float, float],
                           new_yaw: float) -> bool:
        if self._last_goal_xy is None or self._last_goal_yaw is None:
            return False
        dx = new_goal[0] - self._last_goal_xy[0]
        dy = new_goal[1] - self._last_goal_xy[1]
        # DWA는 최종 yaw를 직접 추종하지 않는다. yaw-only goal을 새 edge로 보면
        # 반복 goal publisher의 미세한 yaw 차이가 stale empty path를 통과시킬 수 있다.
        return (
            math.hypot(dx, dy) < self.p_goal_dedup_dist
        )

    def _should_ignore_path_while_reached(
        self,
        path_goal_xy_global: Tuple[float, float],
        path_local: Path,
    ) -> bool:
        if self._nav_state != NavState.REACHED:
            return False
        if self._state is None or self._last_goal_xy is None or not path_local.poses:
            return False

        goal_dx = path_goal_xy_global[0] - self._last_goal_xy[0]
        goal_dy = path_goal_xy_global[1] - self._last_goal_xy[1]
        goal_match_radius = self._goal_match_radius()
        if math.hypot(goal_dx, goal_dy) >= goal_match_radius:
            return False

        local_goal = path_local.poses[-1].pose.position
        dist_to_path_goal = math.hypot(
            local_goal.x - self._state.x,
            local_goal.y - self._state.y,
        )
        hold_radius = max(self.p_goal_tolerance * 2.0, 0.35)
        if dist_to_path_goal > hold_radius:
            return False

        self._path_goal_version = self._goal_version
        self.get_logger().warn(
            "same-goal /global_path ignored while REACHED hold is valid",
            throttle_duration_sec=2.0)
        return True

    def _is_path_goal_close_to_latest_goal(
        self,
        path_goal_xy_global: Tuple[float, float],
    ) -> bool:
        if self._last_goal_xy is None:
            return True
        dx = path_goal_xy_global[0] - self._last_goal_xy[0]
        dy = path_goal_xy_global[1] - self._last_goal_xy[1]
        tolerance = self._goal_match_radius()
        if math.hypot(dx, dy) <= tolerance:
            return True
        self.get_logger().warn(
            "stale /global_path ignored because goal endpoint differs from latest goal",
            throttle_duration_sec=2.0)
        return False

    def _path_offset_to_state(self, path_msg: Path) -> Optional[float]:
        if self._state is None or not path_msg.poses:
            return None
        path_xy = self._path_xy(path_msg)
        nearest_idx = find_nearest_idx(path_xy, (self._state.x, self._state.y), 0)
        nx, ny = path_xy[nearest_idx]
        return math.hypot(nx - self._state.x, ny - self._state.y)

    def _relax_path_acceptance(self) -> None:
        """복구 직후 현재 pose에서 조금 떨어진 새 global path도 받을 수 있게 한다."""
        duration = max(0.0, float(getattr(self, 'p_recovery_path_accept_duration', 0.0)))
        if duration <= 0.0:
            return
        self._relaxed_path_accept_until = max(
            getattr(self, '_relaxed_path_accept_until', 0.0),
            self._sec_now() + duration,
        )

    def _is_path_close_to_state(self, path_msg: Path) -> bool:
        """현재 pose와 너무 먼 stale path를 수신 단계에서 거부한다."""
        offset = self._path_offset_to_state(path_msg)
        limit = self.p_max_path_offset
        if self._sec_now() < getattr(self, '_relaxed_path_accept_until', 0.0):
            limit = max(limit, self.p_recovery_path_accept_offset)
        if offset is None or offset <= limit:
            return True
        self.get_logger().warn(
            f"/global_path가 현재 pose와 {offset:.2f}m 떨어져 무시 "
            f"(max={limit:.2f}m, 기존 path 유지)",
            throttle_duration_sec=2.0)
        return False

    def _extract_obstacles_from_scan(self) -> List[Tuple[float, float]]:
        scan = self._latest_scan
        if scan is None:
            return []
        points: List[Tuple[float, float]] = []
        angle = scan.angle_min
        increment = scan.angle_increment
        range_max = min(scan.range_max, float(self.p_scan_range_max))
        effective_min = max(scan.range_min, self.p_robot_radius)
        if scan.range_min <= 0:
            effective_min = max(0.05, self.p_robot_radius)
        for r in scan.ranges:
            if r != r or r < effective_min or r > range_max:
                angle += increment
                continue
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            points.append((lx + self.p_lidar_offset_x,
                           ly + self.p_lidar_offset_y))
            angle += increment
        return points

    # ───────────────────────────────────────────────────────────────
    # 진단 로그
    # ───────────────────────────────────────────────────────────────
    def _log_state_throttled(self) -> None:
        if self.p_state_log_period <= 0.0:
            return
        self._state_log_counter += 1
        target = int(self.p_control_rate * self.p_state_log_period)
        if self._state_log_counter >= max(1, target):
            self._state_log_counter = 0
            self.get_logger().info(
                f"DWA state [{self.LOCAL_FRAME}] [{self._nav_state.value}]: "
                f"({self._state.x:.2f}, {self._state.y:.2f}, "
                f"yaw={math.degrees(self._state.theta):.1f}°), "
                f"v={self._state.v:+.2f} w={self._state.w:+.2f}")

    # (2026-05-31 SW) _goal_reached_logged_once() 제거 —
    #   P2 의 REACHED 전이 inline edge 가드(self._nav_state != NavState.REACHED)가
    #   "도착 한 번만 로그 + GOAL_REACHED publish" 를 모두 처리하므로 불필요해짐.

    # ───────────────────────────────────────────────────────────────
    # 시각화
    # ───────────────────────────────────────────────────────────────
    def _publish_best_trajectory(self,
                                 traj: List[Tuple[float, float, float]]) -> None:
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "dwa_best"; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        m.lifetime.nanosec = int(0.3 * 1e9)
        for x, y, _ in traj:
            pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.02
            m.points.append(pt)
        self._best_pub.publish(m)

    # ───────────────────────────────────────────────────────────────
    # cmd_vel / status 발행
    # ───────────────────────────────────────────────────────────────
    def _publish_cmd(
        self,
        cmd: VelocityCommand,
        allow_backward_override: bool = False,
    ) -> None:
        twist = Twist()
        allow_backward = getattr(self, "p_allow_backward", False)
        if allow_backward_override:
            allow_backward = True
        twist.linear.x = float(clamp_forward_velocity(cmd.v, allow_backward))
        twist.angular.z = float(cmd.w)
        self._cmd_pub.publish(twist)

    def _stop_robot(self, reason: str) -> None:
        cmd = Twist()
        self._cmd_pub.publish(cmd)
        self.get_logger().debug(f"정지: {reason}")

    def _publish_status_value(self, value: str) -> None:
        msg = String(); msg.data = value
        self._status_pub.publish(msg)

    def _publish_status(self) -> None:
        """1Hz 정기 status."""
        msg = String()
        if self._state is None:
            msg.data = "WAITING_ODOM"
        elif self._nav_state == NavState.REACHED:
            msg.data = "REACHED"
        elif self._path_local is None or not self._path_local.poses:
            msg.data = "STOPPED"
        else:
            msg.data = self._nav_state.value
        self._status_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DwaPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
