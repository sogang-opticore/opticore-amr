#!/usr/bin/env python3
"""
DWA Local Planner Node — Week 2 (SW 메인)

Opticore AMR — 직접 구현 DWA (Nav2 dwb_local_planner 의존 금지).
디스코드 N-2 작업 산출물.

구현 단계:
    [Week 1] 골격 — 노드/파라미터/토픽 구독·발행/DW/샘플링/안전 정지 ✅
    [Week 2] 본체 — forward_simulate / heading_score / clearance_score
             / _control_loop 평가·선택 / MarkerArray 시각화 ✅

설계 원칙 (sw_context.md):
    1. Nav2의 dwb_local_planner를 import해서 쓰지 말 것 (참조는 OK).
    2. 미확정 값은 # TODO: 팀 합의 필요 명시.
    3. 코드 주석은 한국어 우선.
    4. 파라미터는 모두 declare_parameter로 외부화 (코드 하드코딩 금지).

★ 디스코드 N-2 안전 마진:
    v_max는 팀 합의 전까지 임시값 1.5 m/s (명세 §4.1 = 2.0).
    최대 속도 확정 후 v_max / a_max 파라미터 갱신 필수.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# 데이터 구조 — ROS 비의존 순수 자료형 (단위 테스트 용이)
# ---------------------------------------------------------------------------

@dataclass
class RobotState:
    """현재 로봇 상태 (map frame 기준)."""
    x: float          # [m]
    y: float          # [m]
    theta: float      # [rad], yaw
    v: float          # [m/s], 현재 선속도
    w: float          # [rad/s], 현재 각속도


@dataclass
class DynamicWindow:
    """Dynamic Window — 현재 시점에서 도달 가능한 (v, w) 영역."""
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


# ---------------------------------------------------------------------------
# 순수 알고리즘 함수 — Node 분리, 단위 테스트 가능
# ---------------------------------------------------------------------------

def compute_dynamic_window(
    state: RobotState,
    v_max: float,
    v_min: float,
    w_max: float,
    a_max: float,
    alpha_max: float,
    dt: float,
) -> DynamicWindow:
    """Dynamic Window 계산.

    동역학 한계 ∩ 가속도 한계.

    비유: 자전거를 타는데 "다음 1초 동안 페달과 핸들로 만들 수 있는 속도/회전 범위".
    너무 빠르게 달려서 1초 안에 못 멈추는 속도는 충돌검사 단계에서 별도 처리.
    """
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
    """Dynamic Window 안에서 (v, w) 격자 샘플링.

    n_v=11, n_w=21이면 총 231개 후보.
    각 후보마다 trajectory 시뮬레이션 + 평가 → 최적값 선택.
    """
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


# ---------------------------------------------------------------------------
# Trajectory forward integrate (unicycle model) — Week 2
# ---------------------------------------------------------------------------

def forward_simulate(
    state: RobotState,
    v: float,
    w: float,
    dt: float,
    sim_time: float,
) -> List[Tuple[float, float, float]]:
    """Unicycle model로 (v, w) 명령 유지 시 sim_time 동안의 trajectory 적분.

    비유: 운전대(w)와 액셀(v)을 그대로 유지한 채 sim_time 동안 가만히 두면
    어떻게 가는지 미리 계산하는 것. 적분 step은 dt.

    Args:
        state: 시작 상태 (x, y, theta).
        v, w: 유지할 선속도/각속도.
        dt: 적분 step [s].
        sim_time: 총 예측 시간 [s].

    Returns:
        [(x, y, theta), ...] 길이 = int(sim_time / dt). 빈 리스트일 수도 (sim_time < dt).
    """
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
        # theta를 (-pi, pi] 로 정규화
        theta = math.atan2(math.sin(theta), math.cos(theta))
        traj.append((x, y, theta))
    return traj


# ---------------------------------------------------------------------------
# 평가함수 — Week 2 본체 구현
# ---------------------------------------------------------------------------

def heading_score(traj_end_x: float, traj_end_y: float, traj_end_theta: float,
                  goal_x: float, goal_y: float) -> float:
    """trajectory 끝점에서 goal 방향과의 정렬도 [0, 1].

    Fox 1997 원논문 변형: 1 - |angle_diff| / π
        - 정확히 goal을 바라보면 1.0
        - 정반대 방향이면 0.0
        - 90도 어긋나면 0.5

    Args:
        traj_end_x, traj_end_y, traj_end_theta: trajectory 끝점 상태.
        goal_x, goal_y: 목표 점 (lookahead point, map frame).

    Returns:
        float [0, 1].
    """
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    # 끝점이 정확히 goal 위에 있으면 dx=dy=0 → 방향 의미 없음. 만점 처리.
    if dx == 0.0 and dy == 0.0:
        return 1.0
    angle_to_goal = math.atan2(dy, dx)
    diff = angle_to_goal - traj_end_theta
    # (-pi, pi] 로 정규화
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


def clearance_score(trajectory_xy: List[Tuple[float, float]],
                    obstacle_points: List[Tuple[float, float]],
                    max_clearance: float = 1.0) -> float:
    """trajectory 위 점들에서 가장 가까운 장애물까지의 최소 거리 정규화 [0, 1].

    Args:
        trajectory_xy: trajectory 위 (x, y) 점들.
        obstacle_points: 장애물 (x, y) 점들 (LaserScan을 base_link 기준으로 변환).
        max_clearance: 정규화 분모 [m]. 이 값 이상이면 만점 1.0.

    Returns:
        float [0, 1]. 장애물 없으면 1.0. trajectory 비어 있으면 0.0.

    주의:
        이 함수는 충돌 여부를 판단하지 않음 — 거리만 계산.
        충돌 판정(safety_distance)은 호출 측에서 별도 처리.
    """
    if not trajectory_xy:
        return 0.0
    if not obstacle_points:
        return 1.0  # 장애물 없으면 최대 안전

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
    """선속도가 빠를수록 높은 점수 (정지 회피 효과)."""
    if v_max <= 0.0:
        return 0.0
    return max(0.0, v) / v_max


def min_clearance_distance(trajectory_xy: List[Tuple[float, float]],
                           obstacle_points: List[Tuple[float, float]]) -> float:
    """trajectory 위 점들 중 장애물까지 최단 거리 [m].

    충돌 판정용 (safety_distance와 비교). 장애물 없으면 inf.
    """
    if not trajectory_xy or not obstacle_points:
        return float("inf")
    best = float("inf")
    for tx, ty in trajectory_xy:
        for ox, oy in obstacle_points:
            d_sq = (tx - ox) * (tx - ox) + (ty - oy) * (ty - oy)
            if d_sq < best:
                best = d_sq
    return math.sqrt(best) if best != float("inf") else float("inf")


def pick_lookahead_point(
    path_xy: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    lookahead_dist: float,
) -> Optional[Tuple[float, float]]:
    """Path에서 lookahead_dist 만큼 앞쪽 점을 선택.

    1. 로봇과 가장 가까운 path 점 인덱스를 찾고,
    2. 거기서부터 누적 거리가 lookahead_dist 를 넘는 첫 점을 반환.
    3. 끝까지 가도 lookahead 거리에 못 미치면 마지막 점 반환.

    Args:
        path_xy: nav_msgs/Path의 poses[*].pose.position에서 추출한 (x, y).
        robot_xy: 현재 로봇 위치 (x, y).
        lookahead_dist: 앞쪽 거리 [m].

    Returns:
        (x, y) 또는 None (path 비어있으면).
    """
    if not path_xy:
        return None

    # 1. 가장 가까운 점
    rx, ry = robot_xy
    nearest_idx = 0
    nearest_d_sq = float("inf")
    for i, (px, py) in enumerate(path_xy):
        d_sq = (px - rx) * (px - rx) + (py - ry) * (py - ry)
        if d_sq < nearest_d_sq:
            nearest_d_sq = d_sq
            nearest_idx = i

    # 2. 누적 거리 lookahead 초과하는 첫 점
    cumulative = 0.0
    prev_x, prev_y = path_xy[nearest_idx]
    for j in range(nearest_idx + 1, len(path_xy)):
        x, y = path_xy[j]
        cumulative += math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
        if cumulative >= lookahead_dist:
            return (x, y)
        prev_x, prev_y = x, y

    # 3. 끝까지 가도 부족하면 마지막 점
    return path_xy[-1]


# ---------------------------------------------------------------------------
# DWA 노드 — ROS2 인터페이스
# ---------------------------------------------------------------------------

class DwaPlannerNode(Node):
    """DWA Local Planner ROS2 노드.

    구독:
        /odometry/filtered (EKF) — 우선
        /odom              (DiffDrive) — fallback
        /global_path       (A*가 발행하는 전역 경로)
        /lidar             (장애물 회피용)
    발행:
        /cmd_vel               (Twist, control_rate Hz)
        /dwa/trajectories      (MarkerArray, 후보 시각화)
        /dwa/best_trajectory   (Marker)
        /dwa/status            (String)
    """

    def __init__(self) -> None:
        super().__init__("dwa_planner")

        # -------------------------------------------------------------------
        # 파라미터 선언 — Notion 명세 §4.1 동역학 한계 반영
        # -------------------------------------------------------------------
        # 로봇 동역학 한계 (명세 §4.1)
        # TODO(N-2 디스코드): v_max는 팀 합의 전까지 임시값 1.5 m/s 사용.
        #                    명세는 2.0 m/s이지만 안전 마진 + 가속도 동기화 필요.
        #                    최대 속도 확정 후 v_max / a_max 같이 갱신 필수.
        self.declare_parameter("v_max", 1.5)         # m/s, ⚠ 임시값 (명세: 2.0)
        self.declare_parameter("v_min", -1.0)        # m/s, 후진 운영 정책
        self.declare_parameter("w_max", 1.5)         # rad/s, 명세 §4.1
        self.declare_parameter("a_max", 1.0)         # m/s², 명세 §4.1 (v_max 갱신 시 같이)
        # TODO: 팀 합의 필요 — alpha_max는 명세 미명시. 시뮬에서 측정 후 갱신.
        self.declare_parameter("alpha_max", 1.5)     # rad/s², 임시값

        # 샘플링 / 시뮬레이션
        self.declare_parameter("sample_v_n", 11)
        self.declare_parameter("sample_w_n", 21)
        self.declare_parameter("predict_horizon", 1.0)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("control_rate", 20.0)

        # 평가함수 가중치
        self.declare_parameter("weight_heading", 0.8)
        self.declare_parameter("weight_clearance", 0.4)
        self.declare_parameter("weight_velocity", 0.2)

        # 추종 / 평가 보조
        self.declare_parameter("lookahead_dist", 1.0)        # m, A* path 위 어디를 향할지
        self.declare_parameter("max_clearance", 1.0)         # m, clearance_score 정규화 분모
        self.declare_parameter("goal_tolerance", 0.10)       # m, 도착 판정

        # 안전
        self.declare_parameter("robot_radius", 0.20)      # m, URDF width 0.40 / 2 (TR-05)
        self.declare_parameter("hard_collision_distance", 0.05)  # m, 거부 게이트 (TR-05)
        self.declare_parameter("safety_distance", 0.30)   # m, 명세 §7 — clearance_score 기준
        self.declare_parameter("odom_timeout", 0.5)       # s
        self.declare_parameter("scan_range_max", 25.0)    # m, LaserScan max range cap

        # 토픽명 (계약 README와 일치)
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_fallback_topic", "/odom")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter("scan_topic", "/lidar")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # 파라미터 캐싱
        self._load_params()

        # -------------------------------------------------------------------
        # 상태 변수
        # -------------------------------------------------------------------
        self._state: Optional[RobotState] = None
        self._global_path: Optional[Path] = None
        self._latest_scan: Optional[LaserScan] = None
        self._last_odom_time: Optional[float] = None

        # -------------------------------------------------------------------
        # QoS — sensor data는 BEST_EFFORT, latched는 TRANSIENT_LOCAL
        # -------------------------------------------------------------------
        sensor_qos = QoSProfile(depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # 2026-05-24 통합 패치(SW):
        #   HU의 astar_node.py는 /global_path 를 기본 QoS(RELIABLE / VOLATILE)
        #   로 발행한다 → README 계약(TRANSIENT_LOCAL)과 다름.
        #   매칭 실패로 path를 못 받는 사고를 막기 위해, DWA 구독자는 일단
        #   VOLATILE + depth=10 으로 폭 넓게 받는다 (HU 노드 출력 호환).
        #   향후 HU 노드 QoS를 README 계약대로 TRANSIENT_LOCAL로 정정하면
        #   이쪽도 함께 TRANSIENT_LOCAL로 다시 좁혀야 한다.
        path_qos = QoSProfile(depth=10,
                              reliability=QoSReliabilityPolicy.RELIABLE)

        # -------------------------------------------------------------------
        # 구독자
        # -------------------------------------------------------------------
        self.create_subscription(
            Odometry, self.p_odom_topic, self._on_odom, 10)
        # fallback도 동시에 구독 → EKF 죽으면 자동 전환
        self.create_subscription(
            Odometry, self.p_odom_fallback_topic, self._on_odom_fallback, 10)
        self.create_subscription(
            Path, self.p_global_path_topic, self._on_global_path, path_qos)
        self.create_subscription(
            LaserScan, self.p_scan_topic, self._on_scan, sensor_qos)

        # -------------------------------------------------------------------
        # 발행자
        # -------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(
            Twist, self.p_cmd_vel_topic, 10)
        self._traj_pub = self.create_publisher(
            MarkerArray, "/dwa/trajectories", 10)
        self._best_pub = self.create_publisher(
            Marker, "/dwa/best_trajectory", 10)
        self._status_pub = self.create_publisher(String, "/dwa/status", 10)

        # -------------------------------------------------------------------
        # 타이머
        # -------------------------------------------------------------------
        period = 1.0 / max(self.p_control_rate, 1.0)
        self._control_timer = self.create_timer(period, self._control_loop)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"DWA Planner 시작 — v_max={self.p_v_max} m/s, "
            f"w_max={self.p_w_max} rad/s, a_max={self.p_a_max} m/s², "
            f"control_rate={self.p_control_rate} Hz"
        )

    # -----------------------------------------------------------------------
    # 파라미터 로드
    # -----------------------------------------------------------------------
    def _load_params(self) -> None:
        gp = self.get_parameter
        self.p_v_max = gp("v_max").value
        self.p_v_min = gp("v_min").value
        self.p_w_max = gp("w_max").value
        self.p_a_max = gp("a_max").value
        self.p_alpha_max = gp("alpha_max").value

        self.p_sample_v_n = gp("sample_v_n").value
        self.p_sample_w_n = gp("sample_w_n").value
        self.p_predict_horizon = gp("predict_horizon").value
        self.p_dt = gp("dt").value
        self.p_control_rate = gp("control_rate").value

        self.p_w_heading = gp("weight_heading").value
        self.p_w_clearance = gp("weight_clearance").value
        self.p_w_velocity = gp("weight_velocity").value

        self.p_lookahead_dist = gp("lookahead_dist").value
        self.p_max_clearance = gp("max_clearance").value
        self.p_goal_tolerance = gp("goal_tolerance").value

        self.p_robot_radius = gp("robot_radius").value
        self.p_hard_collision_distance = gp("hard_collision_distance").value
        self.p_safety_distance = gp("safety_distance").value
        self.p_odom_timeout = gp("odom_timeout").value
        self.p_scan_range_max = gp("scan_range_max").value

        self.p_odom_topic = gp("odom_topic").value
        self.p_odom_fallback_topic = gp("odom_fallback_topic").value
        self.p_global_path_topic = gp("global_path_topic").value
        self.p_scan_topic = gp("scan_topic").value
        self.p_cmd_vel_topic = gp("cmd_vel_topic").value

    # -----------------------------------------------------------------------
    # 콜백
    # -----------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._update_state_from_odom(msg)
        self._last_odom_time = self._sec_now()

    def _on_odom_fallback(self, msg: Odometry) -> None:
        # EKF가 살아 있으면 fallback 무시. timeout 초과 시에만 사용.
        now = self._sec_now()
        if self._last_odom_time is None:
            self._update_state_from_odom(msg)
            self._last_odom_time = now
            return
        if now - self._last_odom_time > self.p_odom_timeout:
            self.get_logger().warn(
                "EKF /odometry/filtered timeout → /odom fallback 사용")
            self._update_state_from_odom(msg)
            self._last_odom_time = now

    def _on_global_path(self, msg: Path) -> None:
        # 2026-05-24 통합 패치(SW):
        #   1) frame_id 검사 — README 계약상 항상 'map'.
        #   2) 같은 path 반복 수신 시 로그 폭주 방지: poses 개수 + 마지막 점
        #      좌표가 같으면 INFO 로그 생략(DEBUG로 강등).
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(
                f"/global_path frame_id={msg.header.frame_id!r} ≠ 'map' — 무시"
            )
            return

        if not msg.poses:
            # 빈 path = A* 계획 실패. README 계약대로 즉시 정지.
            if self._global_path is None or self._global_path.poses:
                self.get_logger().warn("빈 /global_path 수신 — DWA 정지 모드")
            self._global_path = msg
            return

        # 중복 path 인지 확인
        last = msg.poses[-1].pose.position
        prev_last = None
        if self._global_path is not None and self._global_path.poses:
            p = self._global_path.poses[-1].pose.position
            prev_last = (p.x, p.y, len(self._global_path.poses))
        new_last = (last.x, last.y, len(msg.poses))

        self._global_path = msg
        if prev_last != new_last:
            self.get_logger().info(
                f"/global_path 수신 — {len(msg.poses)}개 점, "
                f"goal=({last.x:.2f}, {last.y:.2f})"
            )
            # 새 path를 받았으면 도착 1회 로그 플래그 리셋
            if hasattr(self, "_goal_logged"):
                self._goal_logged = False
        else:
            self.get_logger().debug("/global_path 갱신 (변동 없음)")

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # -----------------------------------------------------------------------
    # 헬퍼
    # -----------------------------------------------------------------------
    def _update_state_from_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 쿼터니언 → yaw (2D 평면)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._state = RobotState(
            x=p.x, y=p.y, theta=yaw,
            v=msg.twist.twist.linear.x,
            w=msg.twist.twist.angular.z,
        )

    def _sec_now(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    # -----------------------------------------------------------------------
    # 헬퍼 — LaserScan을 base_link 평면 점 리스트로 변환
    # -----------------------------------------------------------------------
    def _extract_obstacles_from_scan(self) -> List[Tuple[float, float]]:
        """LaserScan ranges를 base_link 기준 (x, y) 점들로 변환.

        Note:
            엄밀히는 lidar_link → base_link TF lookup이 필요하지만, URDF에서
            lidar가 base_footprint 위에 같은 좌표(yaw=0)로 mount되어 있다고
            가정 (TR-02 URDF 패치 결과). 정밀도가 필요해지면 TF lookup 추가.
        """
        scan = self._latest_scan
        if scan is None:
            return []

        points: List[Tuple[float, float]] = []
        angle = scan.angle_min
        increment = scan.angle_increment
        range_max = min(scan.range_max, float(self.p_scan_range_max))
        # TR-05: 로봇 본체 안쪽 점은 자체 frame이라 무시. range_min ≥ robot_radius.
        # 비유: 카메라 시야에 자기 코가 들어오면 장애물로 안 침.
        effective_min = max(scan.range_min, self.p_robot_radius)
        if scan.range_min <= 0:
            effective_min = max(0.05, self.p_robot_radius)

        for r in scan.ranges:
            # NaN / Inf / 범위 밖 / 본체 안쪽은 무시
            if r != r or r < effective_min or r > range_max:
                angle += increment
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
            angle += increment
        return points

    # -----------------------------------------------------------------------
    # 헬퍼 — Path 메시지에서 (x, y) 리스트 추출
    # -----------------------------------------------------------------------
    @staticmethod
    def _path_xy(path_msg: Path) -> List[Tuple[float, float]]:
        return [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]

    # -----------------------------------------------------------------------
    # 메인 제어 루프 — Week 2 본체
    # -----------------------------------------------------------------------
    def _control_loop(self) -> None:
        # 1. 안전 점검 — odom 미수신 또는 timeout
        if self._state is None:
            return
        if (self._last_odom_time is None or
                self._sec_now() - self._last_odom_time > self.p_odom_timeout):
            self._stop_robot(reason="odom_timeout")
            return

        # 2. 전역 경로 점검
        if self._global_path is None or not self._global_path.poses:
            self._stop_robot(reason="no_global_path")
            return

        # 3. 도착 판정 — lookahead 점이 아니라 path의 최종 goal과 거리 비교
        path_xy = self._path_xy(self._global_path)
        if path_xy:
            gx, gy = path_xy[-1]
            dist_to_goal = math.hypot(gx - self._state.x, gy - self._state.y)
            if dist_to_goal < self.p_goal_tolerance:
                self._stop_robot(reason="goal_reached")
                self._goal_reached_logged_once()
                return

        # 4. Lookahead 점 선택 — DWA가 향할 단기 목표
        lookahead = pick_lookahead_point(
            path_xy,
            (self._state.x, self._state.y),
            self.p_lookahead_dist,
        )
        if lookahead is None:
            self._stop_robot(reason="no_lookahead")
            return

        # 5. Dynamic Window 계산
        window = compute_dynamic_window(
            state=self._state,
            v_max=self.p_v_max, v_min=self.p_v_min,
            w_max=self.p_w_max, a_max=self.p_a_max,
            alpha_max=self.p_alpha_max, dt=self.p_dt,
        )

        # 6. 속도 샘플링
        samples = sample_velocities(
            window, self.p_sample_v_n, self.p_sample_w_n)
        if not samples:
            self._stop_robot(reason="no_samples")
            return

        # 7. 장애물 점 추출 (LaserScan → base_link 평면 점)
        #    DWA 내부는 base_link 기준 좌표에서 trajectory를 평가하므로,
        #    state를 (0,0,0) 기준 가상으로 옮긴 후 시뮬한다.
        obstacles_local = self._extract_obstacles_from_scan()

        # 8. 각 (v, w) 후보를 forward_simulate + 평가
        #    base_link 로컬 좌표계 시뮬:
        #      가상 state = (x=0, y=0, theta=0, v=현재v, w=현재w)
        #    lookahead 점도 로컬 좌표로 변환.
        local_state = RobotState(
            x=0.0, y=0.0, theta=0.0,
            v=self._state.v, w=self._state.w,
        )
        local_goal = self._world_to_local(lookahead)

        candidates: List[Tuple[VelocityCommand, List[Tuple[float, float, float]]]] = []
        for v, w in samples:
            traj = forward_simulate(
                local_state, v, w, self.p_dt, self.p_predict_horizon
            )
            if not traj:
                continue
            # 충돌 / 안전거리 침범 검사 — 후보에서 즉시 배제 (TR-05)
            # 첫 2 step은 base_link 원점 근처라 자체 LiDAR 점과 항상 가까움 → skip
            traj_xy = [(p[0], p[1]) for p in traj[2:]]
            if not traj_xy:
                traj_xy = [(p[0], p[1]) for p in traj]
            # 거부 게이트: hard_collision_distance(진짜 충돌 직전) + robot_radius(중심→외곽).
            # safety_distance는 더 큰 값으로 두되 clearance_score로만 영향
            # → 부드러운 회피, 좁은 통로 통과 가능.
            min_d = min_clearance_distance(traj_xy, obstacles_local)
            if min_d < (self.p_hard_collision_distance + self.p_robot_radius):
                continue

            # 평가
            end_x, end_y, end_theta = traj[-1]
            h = heading_score(end_x, end_y, end_theta, local_goal[0], local_goal[1])
            c = clearance_score(traj_xy, obstacles_local, self.p_max_clearance)
            vel_s = velocity_score(v, self.p_v_max)
            score = (
                self.p_w_heading * h
                + self.p_w_clearance * c
                + self.p_w_velocity * vel_s
            )
            candidates.append((VelocityCommand(v=v, w=w, score=score), traj))

        # 9. 모든 후보가 충돌 → 비상 정지
        if not candidates:
            self._stop_robot(reason="all_candidates_blocked")
            self._publish_status_value("EMERGENCY")
            return

        # 10. 최고 점수 선택 + 발행
        candidates.sort(key=lambda c: c[0].score, reverse=True)
        best_cmd, best_traj = candidates[0]
        self._publish_cmd(best_cmd)

        # 11. 시각화 — 후보들 + best 강조
        self._publish_trajectories([t for _, t in candidates])
        self._publish_best_trajectory(best_traj)

    # -----------------------------------------------------------------------
    # 좌표 변환 — world (map) → robot local (base_link)
    # -----------------------------------------------------------------------
    def _world_to_local(self, world_xy: Tuple[float, float]) -> Tuple[float, float]:
        """map frame의 점을 base_link 기준 좌표로.

        DWA는 EKF state(map/odom_filtered frame)를 사용한다고 가정하지만,
        실제로는 EKF가 odom_filtered frame을 발행. 두 frame은 정적 SLAM map과
        SL-1 단계에서는 거의 일치하므로 같다고 가정 (SL-2 통합 시 TF lookup 필요).

        # TODO(SL-4 후 통합): AMCL이 들어오면 map ≠ odom_filtered 가 되므로
        #   `tf2_ros`로 (map → base_footprint) 변환을 lookup해서 정확히
        #   계산해야 한다. 현재는 SL-1 가정에서만 정확.
        """
        dx = world_xy[0] - self._state.x
        dy = world_xy[1] - self._state.y
        cos_t = math.cos(-self._state.theta)
        sin_t = math.sin(-self._state.theta)
        local_x = cos_t * dx - sin_t * dy
        local_y = sin_t * dx + cos_t * dy
        return (local_x, local_y)

    # -----------------------------------------------------------------------
    # 헬퍼 — 한 번만 로그 출력 (goal 도착 같은 이벤트)
    # -----------------------------------------------------------------------
    def _goal_reached_logged_once(self) -> None:
        if not getattr(self, "_goal_logged", False):
            self.get_logger().info("goal 도착 — 정지")
            self._goal_logged = True

    # -----------------------------------------------------------------------
    # 시각화 — MarkerArray로 후보 trajectory 발행
    # -----------------------------------------------------------------------
    def _publish_trajectories(self,
                              trajectories: List[List[Tuple[float, float, float]]]
                              ) -> None:
        """모든 후보 trajectory를 base_link frame의 LINE_STRIP marker로 발행."""
        from geometry_msgs.msg import Point

        array = MarkerArray()
        for i, traj in enumerate(trajectories):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "dwa_candidates"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.01     # 라인 두께
            m.color.r = 0.5
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 0.4
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.2 * 1e9)   # 200ms — 다음 사이클 전에 사라짐
            for x, y, _ in traj:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = 0.01
                m.points.append(pt)
            array.markers.append(m)
        self._traj_pub.publish(array)

    def _publish_best_trajectory(self,
                                 traj: List[Tuple[float, float, float]]
                                 ) -> None:
        """선택된 best trajectory를 base_link frame, 진한 색으로 강조."""
        from geometry_msgs.msg import Point

        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "dwa_best"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.2
        m.color.a = 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.3 * 1e9)
        for x, y, _ in traj:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.02
            m.points.append(pt)
        self._best_pub.publish(m)

    # -----------------------------------------------------------------------
    # status 강제 발행 (EMERGENCY 같은 외부 이벤트)
    # -----------------------------------------------------------------------
    def _publish_status_value(self, value: str) -> None:
        msg = String()
        msg.data = value
        self._status_pub.publish(msg)

    def _stop_robot(self, reason: str) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self._cmd_pub.publish(cmd)
        # 너무 시끄러우므로 debug 레벨로
        self.get_logger().debug(f"정지: {reason}")

    def _publish_cmd(self, cmd: VelocityCommand) -> None:
        twist = Twist()
        twist.linear.x = float(cmd.v)
        twist.angular.z = float(cmd.w)
        self._cmd_pub.publish(twist)

    def _publish_status(self) -> None:
        msg = String()
        if self._state is None:
            msg.data = "WAITING_ODOM"
        elif self._global_path is None or not self._global_path.poses:
            msg.data = "STOPPED"
        else:
            msg.data = "PLANNING"
        self._status_pub.publish(msg)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

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
