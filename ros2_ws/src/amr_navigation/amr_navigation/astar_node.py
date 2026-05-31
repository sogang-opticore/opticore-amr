#!/usr/bin/env python3
"""
astar_node.py — A* 전역 경로계획 노드
담당: HU (메인) · SW, JW (구현 참여)
패키지: amr_navigation
"""

import heapq
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException

from amr_navigation.heuristics import heuristic, movement_cost


def _status_param_to_set(value) -> set[str]:
    """ROS parameter 값(list 또는 comma string)을 status set으로 정규화."""
    if isinstance(value, str):
        return {x.strip() for x in value.split(',') if x.strip()}
    return {str(x).strip() for x in value if str(x).strip()}


def cell_path_length(cells: list[tuple], resolution: float) -> float:
    """Grid cell path의 물리 길이[m]를 계산한다."""
    if len(cells) < 2:
        return 0.0
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
        for a, b in zip(cells, cells[1:])
    )


def remaining_path_metrics(
    cells: list[tuple],
    start_cell: tuple,
    resolution: float,
) -> tuple[float, float]:
    """현재 start에서 path 최근접점 이후 남은 길이와 최근접 offset[m]."""
    if not cells:
        return float('inf'), float('inf')
    nearest_idx = min(
        range(len(cells)),
        key=lambda i: (
            cells[i][0] - start_cell[0]) ** 2 + (cells[i][1] - start_cell[1]) ** 2
    )
    nearest = cells[nearest_idx]
    offset = math.hypot(nearest[0] - start_cell[0],
                        nearest[1] - start_cell[1]) * resolution
    remaining = offset + cell_path_length(cells[nearest_idx:], resolution)
    return remaining, offset


def should_retain_previous_path(
    previous_cells: list[tuple] | None,
    candidate_cells: list[tuple],
    start_cell: tuple,
    resolution: float,
    switch_hysteresis: float,
    max_start_offset: float,
) -> tuple[bool, float, float, float, float]:
    """새 후보가 충분히 좋아지지 않았으면 기존 global path를 유지한다."""
    if not previous_cells or not candidate_cells:
        return False, float('inf'), float('inf'), float('inf'), float('inf')

    prev_remaining, prev_offset = remaining_path_metrics(
        previous_cells, start_cell, resolution)
    cand_remaining, _ = remaining_path_metrics(
        candidate_cells, start_cell, resolution)
    if (not math.isfinite(prev_remaining)
            or not math.isfinite(cand_remaining)
            or prev_offset > max_start_offset):
        return (
            False,
            prev_remaining - cand_remaining,
            prev_remaining,
            cand_remaining,
            prev_offset,
        )

    improvement = prev_remaining - cand_remaining
    return (
        improvement < max(0.0, switch_hysteresis),
        improvement,
        prev_remaining,
        cand_remaining,
        prev_offset,
    )


def cells_on_segment(a: tuple, b: tuple) -> list[tuple]:
    """두 grid cell 사이의 직선 segment를 중복 없이 촘촘한 cell path로 반환."""
    dr = int(b[0]) - int(a[0])
    dc = int(b[1]) - int(a[1])
    steps = max(abs(dr), abs(dc))
    if steps == 0:
        return [a]

    cells: list[tuple] = []
    for i in range(steps + 1):
        cell = (
            int(round(a[0] + dr * i / steps)),
            int(round(a[1] + dc * i / steps)),
        )
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells


def clearance_preference_cost(
    clearance: float,
    *,
    preferred_clearance: float,
    clearance_cost_weight: float,
    inflation_radius: float,
    wall_avoid_clearance: float,
    wall_avoid_cost_weight: float,
    wall_avoid_min_margin: float,
) -> float:
    """벽 근처 free cell을 shortest-path tie에서 밀어내는 clearance 비용."""
    if not math.isfinite(clearance):
        return 0.0

    cost = 0.0
    if clearance_cost_weight > 0.0 and preferred_clearance > 0.0:
        if clearance < preferred_clearance:
            ratio = (preferred_clearance - clearance) / preferred_clearance
            cost += clearance_cost_weight * ratio * ratio

    if wall_avoid_cost_weight > 0.0 and wall_avoid_clearance > inflation_radius:
        if clearance < wall_avoid_clearance:
            min_margin = max(wall_avoid_min_margin, 1.0e-6)
            free_margin = max(clearance - inflation_radius, min_margin)
            desired_margin = max(wall_avoid_clearance - inflation_radius, min_margin)
            barrier = max(0.0, desired_margin / free_margin - 1.0)
            cost += wall_avoid_cost_weight * barrier * barrier

    return cost


def should_apply_path_hysteresis(
    allow_path_hysteresis: bool,
    using_direct_path: bool,
    now: float,
    force_publish_until: float,
) -> bool:
    return (
        allow_path_hysteresis
        and not using_direct_path
        and now >= force_publish_until
    )


def status_allows_path_hysteresis(
    status: str | None,
    stable_statuses: set[str],
) -> bool:
    """DWA가 안정 추종 중일 때만 A* path switch hysteresis를 허용한다."""
    return status is None or status in stable_statuses


class AstarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ── 파라미터 선언 ──────────────────────────────────────────
        self.declare_parameter('heuristic', 'octile')
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('inflation_radius', 0.50)  # robot_radius(0.20) + clearance_stop(0.30)
        self.declare_parameter('preferred_clearance', 1.20)  # robot_radius + DWA slowdown 여유
        self.declare_parameter('clearance_cost_weight', 8.0)
        self.declare_parameter('wall_avoid_clearance', 1.00)
        self.declare_parameter('wall_avoid_cost_weight', 2.5)
        self.declare_parameter('wall_avoid_min_margin', 0.05)
        self.declare_parameter('smoothing', 'catmull_rom')
        self.declare_parameter('smoothing_min_clearance', 0.90)
        # 2026-05-24 보강(SW, HU 보강-1):
        #   goal 셀이 inflation/점유로 막혔을 때 nearest free cell로 자동 보정.
        #   BFS 반경 [cell] = goal_snap_radius / resolution.
        self.declare_parameter('goal_snap_radius', 0.6)   # m, 0 이면 비활성

        # 2026-05-25 추가 (SW · 페어, DWA stuck/벗어남 문제 해결):
        # 주기적 재계획 — 기본 1Hz. DWA가 path를 잃거나 복구가 끝난 경우에는
        # /dwa/status 이벤트로도 즉시 재계획한다.
        self.declare_parameter('replan_period', 1.0)   # s, 0=비활성
        self.declare_parameter('dwa_status_topic', '/dwa/status')
        self.declare_parameter('status_replan_cooldown', 2.0)
        self.declare_parameter(
            'status_replan_states',
            ['EMERGENCY', 'PATH_LOST', 'RECOVERY_DONE', 'STOPPED_NEAR_WALL'],
        )
        self.declare_parameter('status_replan_after_states', ['FORWARD_ONLY', 'RECOVERY'])
        self.declare_parameter(
            'status_replan_reset_states',
            ['NORMAL', 'ALIGN', 'STOPPED', 'REACHED', 'GOAL_REACHED'],
        )

        # 2026-05-31 추가 (SW · dwa-ys, 도착 인식 안정화 P1):
        # goal dedup — 같은 goal 재수신 시 재계획 스킵 임계값 [m].
        # `ros2 topic pub --rate 0.5 /goal_pose ...` 로 같은 goal 을 반복 송신해도
        # 이 거리 이내면 무시하고 기존 path 유지 → DWA 의 도착(REACHED) 신호가 안정됨.
        # 비유: 내비에 같은 목적지를 1초마다 다시 찍어도 경로를 재탐색하지 않는 것.
        # TODO: 0.10 은 미확정 초안(그리드 해상도 ~0.05m 의 2배). 시뮬 측정 후 확정.
        self.declare_parameter('goal_dedup_dist', 0.10)
        # 2026-05-31 리뷰 반영(Codex): dedup 에 도착 방향(yaw) 차이도 포함.
        # 같은 위치에서 yaw 만 바뀐 goal 은 인터페이스상 '새 goal' 이므로 놓치면 안 됨.
        # TODO(미확정): 0.10 rad(≈5.7°) 초안. 시뮬 측정 후 확정.
        self.declare_parameter('goal_dedup_yaw', 0.10)   # rad
        self.declare_parameter('path_switch_hysteresis', 0.35)  # m
        self.declare_parameter('path_switch_max_start_offset', 0.80)  # m
        self.declare_parameter('path_hysteresis_stable_states', ['NORMAL', 'ALIGN'])
        self.declare_parameter('new_goal_force_publish_sec', 5.0)  # s
        self.declare_parameter('goal_direct_distance', 2.0)  # m
        self.declare_parameter('goal_direct_min_clearance', 0.90)  # m

        self.heuristic_type   = self.get_parameter('heuristic').value
        self.allow_diagonal   = self.get_parameter('allow_diagonal').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.preferred_clearance = self.get_parameter('preferred_clearance').value
        self.clearance_cost_weight = self.get_parameter('clearance_cost_weight').value
        self.wall_avoid_clearance = self.get_parameter('wall_avoid_clearance').value
        self.wall_avoid_cost_weight = self.get_parameter('wall_avoid_cost_weight').value
        self.wall_avoid_min_margin = self.get_parameter('wall_avoid_min_margin').value
        self.smoothing        = self.get_parameter('smoothing').value
        self.smoothing_min_clearance = self.get_parameter('smoothing_min_clearance').value
        self.goal_snap_radius = self.get_parameter('goal_snap_radius').value
        self.replan_period    = self.get_parameter('replan_period').value
        self.dwa_status_topic = self.get_parameter('dwa_status_topic').value
        self.status_replan_cooldown = self.get_parameter('status_replan_cooldown').value
        self.status_replan_states = _status_param_to_set(
            self.get_parameter('status_replan_states').value)
        self.status_replan_after_states = _status_param_to_set(
            self.get_parameter('status_replan_after_states').value)
        self.status_replan_reset_states = _status_param_to_set(
            self.get_parameter('status_replan_reset_states').value)
        self.goal_dedup_dist  = self.get_parameter('goal_dedup_dist').value
        self.goal_dedup_yaw   = self.get_parameter('goal_dedup_yaw').value
        self.path_switch_hysteresis = self.get_parameter('path_switch_hysteresis').value
        self.path_switch_max_start_offset = self.get_parameter(
            'path_switch_max_start_offset').value
        self.path_hysteresis_stable_states = _status_param_to_set(
            self.get_parameter('path_hysteresis_stable_states').value)
        self.new_goal_force_publish_sec = self.get_parameter(
            'new_goal_force_publish_sec').value
        self.goal_direct_distance = self.get_parameter('goal_direct_distance').value
        self.goal_direct_min_clearance = self.get_parameter(
            'goal_direct_min_clearance').value

        # ── 내부 상태 ──────────────────────────────────────────────
        self.map_data: OccupancyGrid | None = None
        self.inflated_grid: np.ndarray | None = None
        self.clearance_grid: np.ndarray | None = None
        self.goal: PoseStamped | None = None
        self._last_goal_xy: tuple[float, float] | None = None  # P1 dedup (2026-05-31)
        self._last_goal_yaw: float | None = None               # P1 dedup yaw (리뷰 반영)
        self._has_valid_path_for_goal = False
        self._status_replan_armed = True
        self._last_status_replan_time = -float('inf')
        self._last_dwa_status: str | None = None
        self._last_path_cells: list[tuple] | None = None
        self._force_publish_until = -float('inf')

        # ── TF ────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── QoS 설정 ───────────────────────────────────────────────
        # /map은 slam_toolbox가 TRANSIENT_LOCAL(latched)로 발행.
        # 구독 QoS도 맞춰야 노드 시작 시 맵을 즉시 받을 수 있음.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── 구독 ───────────────────────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10,
        )
        self.status_sub = self.create_subscription(
            String, self.dwa_status_topic, self._on_dwa_status, 10,
        )

        # ── 발행 ───────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # ── 주기적 재계획 타이머 (2026-05-25 추가) ─────────────────
        # replan_period > 0 이면 그 주기로 _plan() 자동 호출. goal 이 있을 때만 동작.
        if self.replan_period > 0.0:
            self.create_timer(self.replan_period, self._replan_timer)
            self.get_logger().info(
                f'AstarPlanner 주기적 재계획 활성 — {self.replan_period}s 마다')
        else:
            self.get_logger().info(
                'AstarPlanner 주기적 재계획 비활성 — DWA 상태 이벤트 기반 재계획')

        self.get_logger().info('AstarPlanner 노드 시작 — 맵과 goal 대기 중')

    # ══════════════════════════════════════════════════════════════
    # 주기적 재계획 (2026-05-25 추가)
    # ══════════════════════════════════════════════════════════════
    def _sec_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_dwa_status(self, msg: String):
        """DWA 상태 이벤트를 받아 필요할 때만 A* 재계획한다."""
        status = msg.data.strip()
        if not status:
            return

        prev_status = self._last_dwa_status
        self._last_dwa_status = status

        if status in self.status_replan_reset_states:
            self._status_replan_armed = True
            if prev_status in self.status_replan_after_states:
                self._request_status_replan(f'{prev_status}->{status}')
            return

        if status in self.status_replan_states:
            self._request_status_replan(status)

    def _request_status_replan(self, reason: str) -> bool:
        """DWA가 막힘/복구 이벤트를 보냈을 때 현재 pose 기준 path를 갱신한다."""
        if self.goal is None or self.map_data is None:
            return False
        if not self._status_replan_armed:
            return False

        now = self._sec_now()
        if now - self._last_status_replan_time < self.status_replan_cooldown:
            return False

        self._last_status_replan_time = now
        self.get_logger().warn(
            f'DWA 상태 {reason} 감지 — 현재 pose 기준 A* 이벤트 재계획')
        success = self._plan(clear_on_failure=False, allow_path_hysteresis=False)
        if success:
            self._status_replan_armed = False
        return success

    def _replan_timer(self):
        """주기적 재계획 — DWA stuck / path 벗어남 자동 복구.

        조건:
            (1) goal 없음 / 맵 없음 → skip
            (2) 자기 위치가 goal 근처 (0.30m) → skip (DWA REACHED 상태 유지)
            (3) 그 외 → _plan() 호출 (= 새 path 발행)
        """
        if self.goal is None or self.map_data is None:
            return

        # 자기 위치가 goal 근처면 replan skip — DWA 의 REACHED 상태 보존.
        # 그렇지 않으면 새 path 가 self._reached 를 False 로 리셋하고 다시 추종 시작.
        # 단, 주기 재계획 실패가 기존 성공 path 를 빈 path 로 덮어쓰면
        # DWA status 가 STOPPED/NORMAL 로 출렁이므로 성공 캐시가 있을 때는 보존한다.
        start_world = self._get_robot_position()
        if start_world is None:
            if self._has_valid_path_for_goal:
                self.get_logger().warn(
                    '주기 재계획 TF lookup 실패 — 기존 /global_path 유지',
                    throttle_duration_sec=2.0)
                return
            self._handle_plan_failure(
                'TF lookup 실패 — 경로 계획 중단',
                clear_on_failure=True)
            return

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        dist_to_goal = math.hypot(gx - start_world[0], gy - start_world[1])
        if dist_to_goal < 0.30:   # DWA goal_tolerance(0.20) + 마진
            return

        self._plan(
            clear_on_failure=False,
            allow_path_hysteresis=status_allows_path_hysteresis(
                self._last_dwa_status,
                self.path_hysteresis_stable_states,
            ),
        )

    # ══════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════

    def _map_callback(self, msg: OccupancyGrid):
        """
        /map 수신 시 호출.
        맵을 캐시하고 inflation 그리드를 즉시 빌드.
        맵이 바뀔 때마다 재빌드됨 (slam_toolbox가 계속 업데이트).
        """
        self.map_data = msg
        self.inflated_grid = self._build_inflated_grid(msg)
        self.get_logger().info(
            f'맵 수신: {msg.info.width}×{msg.info.height}, '
            f'해상도={msg.info.resolution:.3f} m/cell'
        )

    def _goal_callback(self, msg: PoseStamped):
        """
        /goal_pose 수신 시 호출.
        맵이 없으면 goal 무시, 있으면 즉시 경로 계획 시작.
        같은 goal(goal_dedup_dist 이내) 재수신 시 재계획 스킵 — dedup (2026-05-31 SW).
        """
        if self.map_data is None:
            self.get_logger().warn('맵 미수신 — goal 무시')
            return

        new_goal = (msg.pose.position.x, msg.pose.position.y)
        # 도착 방향(yaw) — 평면 quaternion → yaw
        q = msg.pose.orientation
        new_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # ── dedup (2026-05-31 SW · P1, 리뷰 반영: XY + yaw) ──────────────
        # rationale: `--rate 0.5` 로 같은 goal 을 반복 송신하면 매번 _plan() 이 호출돼
        #            2초마다 새(짧은) path 가 발행되고, DWA 의 도착 신호가 흔들린다.
        #            직전 goal 과 위치·방향 모두 임계값 이내면 재계획을 건너뛴다.
        # [리뷰 Codex] XY 만 비교하면 같은 위치에서 yaw 만 바뀐 goal 을 놓침(계약 위반).
        #            → yaw 차이도 함께 본다.
        # 주의: _replan_timer(1Hz) 의 주기적 재계획과는 별개 — 그쪽은 goal 0.30m 이내면
        #       skip 하지만, 이 콜백은 goal 위치와 무관하게 매번 _plan() 했었음.
        # [미확정] 현재 A*/DWA 는 최종 yaw 를 적극 추종하지 않음(도착 판정은 거리 기반).
        #          yaw 는 dedup '새 goal 판별' 용으로만 사용. 최종 yaw 추종은 향후 과제. (README §3.0)
        if self._last_goal_xy is not None and self._last_goal_yaw is not None:
            dx = new_goal[0] - self._last_goal_xy[0]
            dy = new_goal[1] - self._last_goal_xy[1]
            dyaw = abs(math.atan2(math.sin(new_yaw - self._last_goal_yaw),
                                  math.cos(new_yaw - self._last_goal_yaw)))
            if math.hypot(dx, dy) < self.goal_dedup_dist and dyaw < self.goal_dedup_yaw:
                return  # 같은 goal(위치+방향) — 재계획 스킵, 기존 path 유지

        self._last_goal_xy = new_goal
        self._last_goal_yaw = new_yaw
        self.goal = msg
        self._has_valid_path_for_goal = False
        self._status_replan_armed = True
        self._last_path_cells = None
        self._force_publish_until = (
            self._sec_now() + max(0.0, float(self.new_goal_force_publish_sec)))
        self.get_logger().info(
            f'Goal 수신: ({new_goal[0]:.2f}, {new_goal[1]:.2f}, yaw={new_yaw:.2f})'
        )
        self._plan(clear_on_failure=True, allow_path_hysteresis=False)

    # ══════════════════════════════════════════════════════════════
    # 경로 계획 메인
    # ══════════════════════════════════════════════════════════════

    def _plan(
        self,
        *,
        clear_on_failure: bool = True,
        allow_path_hysteresis: bool = False,
    ) -> bool:
        """
        A* 경로 계획 메인 함수.
        성공 시 /global_path 발행.

        clear_on_failure=True 이면 실패 시 빈 Path를 발행해 DWA를 정지시킨다.
        새 goal 최초 계획처럼 기존 경로를 더 이상 믿으면 안 되는 경우에 쓴다.
        clear_on_failure=False 이면 기존 성공 경로가 있을 때 빈 Path를 발행하지 않는다.
        주기 재계획의 일시적 실패가 DWA의 정상 추종을 STOPPED로 흔드는 것을 막기 위함이다.
        """
        # 현재 로봇 위치 TF lookup (base_footprint → map)
        start_world = self._get_robot_position()
        if start_world is None:
            self._handle_plan_failure(
                'TF lookup 실패 — 경로 계획 중단',
                clear_on_failure=clear_on_failure)
            return False

        goal_world = (self.goal.pose.position.x, self.goal.pose.position.y)

        start_cell = self._world_to_cell(start_world)
        goal_cell  = self._world_to_cell(goal_world)

        # 2026-05-25 보강(SW · 페어): start 셀이 inflation/점유 영역이면 인근
        # free 셀로 보정. 좁은 통로에서 로봇이 inflation 안쪽으로 살짝 들어가면
        # A* 가 첫 노드부터 막혀 "경로 없음" 무한 반복 → DWA STOPPED 무한 루프.
        # 비유: 발이 진흙에 잠긴 채로는 길 찾기 불가 → 발 먼저 자유 지반으로 옮기기.
        if not self._is_free_cell(start_cell):
            snapped = self._snap_to_nearest_free(start_cell)
            if snapped is None:
                self._handle_plan_failure(
                    f'Start {start_cell}이 점유/맵-밖이고 인근 자유공간 없음',
                    clear_on_failure=clear_on_failure)
                return False
            self.get_logger().info(
                f'Start 보정: {start_cell} (점유/inflation) → {snapped} (인근 free)')
            start_cell = snapped

        # 2026-05-24 보강(SW, HU 보강-1): goal 셀이 막혔으면 nearest free cell 보정.
        # 비유: 우체부가 "그 주소엔 우체통이 없네요" 라고 그냥 돌아가지 않고
        #       가장 가까운 우체통을 찾아 거기에 두는 것.
        if not self._is_free_cell(goal_cell):
            snapped = self._snap_to_nearest_free(goal_cell)
            if snapped is None:
                self._handle_plan_failure(
                    f'Goal {goal_cell}이 점유/맵-밖이고 인근 자유공간 없음',
                    clear_on_failure=clear_on_failure)
                return False
            self.get_logger().info(
                f'Goal 보정: {goal_cell} (점유/inflation) → {snapped} (인근 free)')
            goal_cell = snapped

        direct_path = self._try_goal_direct_path(
            start_cell, goal_cell, start_world, goal_world)
        using_direct_path = direct_path is not None
        if direct_path is not None:
            cell_path = direct_path
            self.get_logger().info(
                f'goal 직선 접근 path 사용: {len(cell_path)} cells')
        else:
            cell_path = self._astar(start_cell, goal_cell)

        if cell_path is None:
            self._handle_plan_failure(
                '경로 없음',
                clear_on_failure=clear_on_failure)
            return False

        if self.smoothing == 'catmull_rom':
            cell_path = self._smooth_catmull_rom(cell_path)

        now = self._sec_now()
        if (should_apply_path_hysteresis(
                allow_path_hysteresis,
                using_direct_path,
                now,
                self._force_publish_until)
                and self._path_is_still_free(self._last_path_cells)):
            keep, improvement, prev_len, cand_len, offset = should_retain_previous_path(
                previous_cells=self._last_path_cells,
                candidate_cells=cell_path,
                start_cell=start_cell,
                resolution=self.map_data.info.resolution,
                switch_hysteresis=self.path_switch_hysteresis,
                max_start_offset=self.path_switch_max_start_offset,
            )
            if keep:
                self._has_valid_path_for_goal = True
                self.get_logger().info(
                    '기존 경로 유지: '
                    f'개선={improvement:.2f}m < {self.path_switch_hysteresis:.2f}m, '
                    f'prev={prev_len:.2f}m cand={cand_len:.2f}m offset={offset:.2f}m',
                    throttle_duration_sec=2.0)
                return True

        path_min_clearance = self._path_min_clearance(cell_path)
        path_msg = self._cells_to_path(cell_path)
        self.path_pub.publish(path_msg)
        self._has_valid_path_for_goal = True
        self._last_path_cells = list(cell_path)
        self.get_logger().info(
            f'경로 발행: {len(path_msg.poses)} 웨이포인트, '
            f'min_clear={path_min_clearance:.2f}m')
        return True

    def _handle_plan_failure(self, reason: str, *, clear_on_failure: bool) -> None:
        """계획 실패 처리.

        주기 재계획 실패가 이미 발행된 성공 경로를 덮어쓰면 DWA status가
        NORMAL/STOPPED로 출렁인다. 기존 경로가 유효하고 호출자가 보존을 허용한
        경우에는 빈 Path를 발행하지 않는다.
        """
        if not clear_on_failure and self._has_valid_path_for_goal:
            self.get_logger().warn(
                f'{reason} — 기존 /global_path 유지(빈 path 미발행)',
                throttle_duration_sec=2.0)
            return

        self._has_valid_path_for_goal = False
        self._last_path_cells = None
        self.get_logger().warn(f'{reason} — 빈 path 발행')
        self._publish_empty_path()

    # ══════════════════════════════════════════════════════════════
    # A* 알고리즘
    # ══════════════════════════════════════════════════════════════

    def _astar(self, start: tuple, goal: tuple) -> list | None:
        """
        A* 탐색 메인 로직.

        open_set: (f값, 셀) 형태의 min-heap
        g_score: 시작점에서 각 셀까지의 실제 비용
        closed_set: 이미 처리한 셀 (재방문 방지)
        came_from: 경로 역추적용 부모 셀 기록

        반환: 셀 좌표 리스트 [(row, col), ...] 또는 None (실패)
        """
        open_set: list = []
        heapq.heappush(open_set, (0.0, start))

        came_from: dict = {}
        g_score: dict   = {start: 0.0}
        closed_set: set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            # heapq는 같은 셀이 여러 번 들어갈 수 있으므로
            # closed_set으로 중복 처리 방지
            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set:
                    continue

                # g(n) = 현재까지의 실제 이동 비용
                step = movement_cost(current, neighbor)
                tentative_g = g_score[current] + step * (1.0 + self._clearance_cost(neighbor))

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    # f(n) = g(n) + h(n)
                    f = tentative_g + heuristic(neighbor, goal, self.heuristic_type)
                    heapq.heappush(open_set, (f, neighbor))

        return None  # 경로 없음

    def _reconstruct_path(self, came_from: dict, current: tuple) -> list:
        """came_from dict를 역추적해 start → goal 셀 리스트 반환."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # ══════════════════════════════════════════════════════════════
    # 그리드 헬퍼
    # ══════════════════════════════════════════════════════════════

    def _get_neighbors(self, cell: tuple) -> list:
        """
        8-connected 또는 4-connected 이웃 셀 반환.
        맵 밖이거나 점유된 셀은 제외.
        """
        row, col = cell
        deltas = (
            [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
            if self.allow_diagonal
            else [(-1,0),(0,-1),(0,1),(1,0)]
        )
        neighbors = []
        for dr, dc in deltas:
            candidate = (row + dr, col + dc)
            if not self._is_free_cell(candidate):
                continue
            if dr != 0 and dc != 0:
                # 대각선으로 벽 모서리를 스치며 통과하지 않도록 양 옆 직교 셀도 확인.
                if (not self._is_free_cell((row + dr, col)) or
                        not self._is_free_cell((row, col + dc))):
                    continue
            neighbors.append(candidate)
        return neighbors

    def _clearance_at_cell(self, cell: tuple) -> float:
        """raw obstacle 기준 셀 중심 clearance[m]. 없으면 inf."""
        if self.clearance_grid is None:
            return float('inf')
        row, col = cell
        h, w = self.clearance_grid.shape
        if row < 0 or col < 0 or row >= h or col >= w:
            return 0.0
        return float(self.clearance_grid[row, col])

    def _clearance_cost(self, cell: tuple) -> float:
        """벽 가까운 free 셀에 부드러운 비용을 부여해 중앙 경로를 선호한다."""
        clearance = self._clearance_at_cell(cell)
        return clearance_preference_cost(
            clearance,
            preferred_clearance=getattr(self, 'preferred_clearance', 0.0),
            clearance_cost_weight=getattr(self, 'clearance_cost_weight', 0.0),
            inflation_radius=getattr(self, 'inflation_radius', 0.0),
            wall_avoid_clearance=getattr(self, 'wall_avoid_clearance', 0.0),
            wall_avoid_cost_weight=getattr(self, 'wall_avoid_cost_weight', 0.0),
            wall_avoid_min_margin=getattr(self, 'wall_avoid_min_margin', 0.05),
        )

    def _snap_to_nearest_free(self, cell: tuple) -> tuple | None:
        """막힌 셀에 대해 BFS로 인근 자유공간 셀 찾기 (HU 보강-1, 2026-05-24).

        반경: goal_snap_radius / resolution [cells]. 0이면 비활성.
        BFS는 4-conn 또는 8-conn 어떤 거든 거의 차이 없음 → 8-conn으로.

        반환: 가장 가까운 free cell (row, col) 또는 None.
        """
        if self.goal_snap_radius <= 0 or self.inflated_grid is None or self.map_data is None:
            return None

        res = self.map_data.info.resolution
        max_radius = int(math.ceil(self.goal_snap_radius / res))
        if max_radius <= 0:
            return None

        from collections import deque
        h, w = self.inflated_grid.shape
        r0, c0 = cell
        visited = {(r0, c0)}
        q = deque([(r0, c0, 0)])
        deltas = (
            (-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)
        )
        best_cell = None
        best_key = (-1.0, -float('inf'))
        while q:
            r, c, d = q.popleft()
            if 0 <= r < h and 0 <= c < w and self.inflated_grid[r, c] == 0:
                clearance = min(self._clearance_at_cell((r, c)), self.preferred_clearance)
                dist = math.hypot(r - r0, c - c0) * res
                key = (clearance, -dist)
                if key > best_key:
                    best_key = key
                    best_cell = (r, c)
            if d >= max_radius:
                continue
            for dr, dc in deltas:
                nr, nc = r + dr, c + dc
                if (nr, nc) in visited:
                    continue
                visited.add((nr, nc))
                q.append((nr, nc, d + 1))
        return best_cell

    def _is_free_cell(self, cell: tuple) -> bool:
        """
        셀이 맵 범위 내이고 통과 가능한지 확인.
        inflated_grid 기준 사용 (0=free, 1=blocked).

        # 보강 필요: inflation_radius=0.30m가 창고 통로(~3m)에서
        # 너무 보수적으로 막히지 않는지 실제 주행 후 튜닝 권장.
        """
        if self.inflated_grid is None:
            return False
        row, col = cell
        h, w = self.inflated_grid.shape
        if row < 0 or col < 0 or row >= h or col >= w:
            return False
        return self.inflated_grid[row, col] == 0

    def _build_inflated_grid(self, msg: OccupancyGrid) -> np.ndarray:
        """
        OccupancyGrid → 2D numpy 배열 변환 + 거리장 기반 inflation 적용.
        로봇이 벽/선반에 너무 가깝게 붙지 않도록 장애물 주변을 팽창.

        점유(>=50) 또는 unknown(-1) 셀을 blocked으로 처리.

        clearance_grid는 raw obstacle 중심까지의 거리[m]다. A* 비용에서
        preferred_clearance 안쪽 free 셀에 페널티를 줘 벽 경계 대신 통로 중앙을 선호한다.
        """
        from scipy.ndimage import distance_transform_edt

        w   = msg.info.width
        h   = msg.info.height
        res = msg.info.resolution

        raw     = np.array(msg.data, dtype=np.int8).reshape((h, w))
        raw_blocked = (raw >= 50) | (raw == -1)

        clearance = distance_transform_edt(~raw_blocked) * res
        self.clearance_grid = clearance.astype(np.float32)
        blocked = raw_blocked | (clearance <= self.inflation_radius)

        return blocked.astype(np.uint8)

    def _segment_is_free(self, a: tuple, b: tuple) -> bool:
        """두 셀 사이 직선 구간이 inflated obstacle을 지나지 않는지 확인."""
        dr = int(b[0]) - int(a[0])
        dc = int(b[1]) - int(a[1])
        steps = max(abs(dr), abs(dc))
        if steps == 0:
            return self._is_free_cell(a)

        prev = None
        for i in range(steps + 1):
            r = int(round(a[0] + dr * i / steps))
            c = int(round(a[1] + dc * i / steps))
            cell = (r, c)
            if not self._is_free_cell(cell):
                return False
            if prev is not None:
                pr, pc = prev
                if abs(r - pr) == 1 and abs(c - pc) == 1:
                    if (not self._is_free_cell((r, pc)) or
                            not self._is_free_cell((pr, c))):
                        return False
            prev = cell
        return True

    def _segment_min_clearance(self, a: tuple, b: tuple) -> float:
        """두 셀 사이 직선 구간의 최소 raw obstacle clearance[m]."""
        return min(self._clearance_at_cell(c) for c in cells_on_segment(a, b))

    def _segment_is_safe_for_smoothing(self, a: tuple, b: tuple) -> bool:
        """스무딩 segment가 free이고 최소 clearance 기준도 만족하는지 확인."""
        if not self._segment_is_free(a, b):
            return False
        min_clearance = getattr(self, 'smoothing_min_clearance', 0.0)
        if min_clearance <= 0.0:
            return True
        return self._segment_min_clearance(a, b) >= min_clearance

    # ══════════════════════════════════════════════════════════════
    # 경로 스무딩
    # ══════════════════════════════════════════════════════════════

    def _path_is_still_free(self, cells: list[tuple] | None) -> bool:
        """이전 path가 현재 map에서도 통과 가능한지 확인."""
        if not cells:
            return False
        stride = max(1, len(cells) // 80)
        sampled = cells[::stride]
        if sampled[-1] != cells[-1]:
            sampled.append(cells[-1])
        if any(not self._is_free_cell(c) for c in sampled):
            return False
        return all(
            self._segment_is_free(a, b)
            for a, b in zip(sampled, sampled[1:])
        )

    def _path_min_clearance(self, cells: list[tuple]) -> float:
        if not cells:
            return 0.0
        return min(self._clearance_at_cell(c) for c in cells)

    def _try_goal_direct_path(
        self,
        start_cell: tuple,
        goal_cell: tuple,
        start_world: tuple,
        goal_world: tuple,
    ) -> list[tuple] | None:
        """목표 근처에서 안전한 직선 segment가 열려 있으면 최종 접근 path로 사용."""
        if self.goal_direct_distance <= 0.0:
            return None
        dist_to_goal = math.hypot(
            goal_world[0] - start_world[0],
            goal_world[1] - start_world[1],
        )
        if dist_to_goal > self.goal_direct_distance:
            return None
        if not self._segment_is_free(start_cell, goal_cell):
            return None

        direct_cells = cells_on_segment(start_cell, goal_cell)
        min_clearance = self._path_min_clearance(direct_cells)
        if min_clearance < self.goal_direct_min_clearance:
            return None
        return direct_cells

    def _smooth_catmull_rom(self, cells: list, samples: int = 5) -> list:
        """
        Catmull-Rom 스플라인으로 경로 스무딩.
        인접 4점을 이용해 곡선 보간, 각 구간을 samples개 점으로 분할.

        스무딩 후 inflation 재검증:
          blocked 셀을 단순 제거하지 않고 해당 구간을 raw A* 경로로 대체.
          이유: 단순 제거 시 앞뒤 free 점 사이 연결선이 여전히 벽을 뚫음.
          예) A(free)→B(blocked)→C(free) 에서 B 제거하면
              A→C 직선이 벽을 가로지르는 문제 그대로 남음.
        """
        if len(cells) < 4:
            return cells

        def _cr(p0, p1, p2, p3, t):
            """Catmull-Rom 보간 단일 점 계산."""
            t2, t3 = t*t, t*t*t
            r = 0.5 * (2*p1[0] + (-p0[0]+p2[0])*t
                       + (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2
                       + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
            c = 0.5 * (2*p1[1] + (-p0[1]+p2[1])*t
                       + (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2
                       + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
            return (int(round(r)), int(round(c)))

        padded   = [cells[0]] + cells + [cells[-1]]
        smoothed = [cells[0]]

        for i in range(1, len(padded) - 2):
            p0, p1, p2, p3 = padded[i-1], padded[i], padded[i+1], padded[i+2]
            for s in range(1, samples + 1):
                smoothed.append(_cr(p0, p1, p2, p3, s / samples))

        # ── 스무딩 후 inflation 재검증 ────────────────────────────────
        if self.inflated_grid is None:
            return smoothed

        validated = []
        for si, cell in enumerate(smoothed):
            if self._is_free_cell(cell) and (
                    not validated or self._segment_is_safe_for_smoothing(validated[-1], cell)):
                validated.append(cell)
            else:
                # blocked → 해당 구간의 raw A* 점으로 대체
                # si=0은 cells[0], si=1..samples는 cells[0~1] 구간에 대응
                raw_seg   = max(0, (si - 1) // samples) if si > 0 else 0
                raw_start = min(raw_seg, len(cells) - 1)
                raw_end   = min(raw_seg + 1, len(cells) - 1)
                fallback_ok = False
                for raw_cell in cells[raw_start:raw_end + 1]:
                    if validated and validated[-1] == raw_cell:
                        fallback_ok = True
                        continue
                    if (self._is_free_cell(raw_cell) and
                            (not validated or self._segment_is_free(validated[-1], raw_cell))):
                        validated.append(raw_cell)
                        fallback_ok = True
                    else:
                        fallback_ok = False
                        break
                if not fallback_ok:
                    return cells

        return validated if len(validated) > 1 else cells

    # ══════════════════════════════════════════════════════════════
    # 좌표 변환
    # ══════════════════════════════════════════════════════════════

    def _world_to_cell(self, world: tuple) -> tuple:
        """월드 좌표 (x, y) → 그리드 셀 (row, col)."""
        info = self.map_data.info
        col  = int((world[0] - info.origin.position.x) / info.resolution)
        row  = int((world[1] - info.origin.position.y) / info.resolution)
        return (row, col)

    def _cell_to_world(self, cell: tuple) -> tuple:
        """그리드 셀 (row, col) → 월드 좌표 (x, y) — 셀 중심점."""
        info = self.map_data.info
        x = cell[1] * info.resolution + info.origin.position.x + info.resolution / 2
        y = cell[0] * info.resolution + info.origin.position.y + info.resolution / 2
        return (x, y)

    def _get_robot_position(self) -> tuple | None:
        """
        TF lookup으로 현재 로봇 위치 **(map frame 기준)** 반환.
        실패 시 None 반환.

        2026-05-25 수정 (SW · 페어):
          기존 `lookup_transform('odom_filtered', 'base_footprint', ...)` 는
          odom_filtered frame 기준 좌표를 돌려준다. AMCL 통합 후
          map ≠ odom_filtered 이므로 그 좌표를 map 좌표인 양 _world_to_cell()에
          넣으면 start_cell이 엉뚱한 위치(범위 밖 또는 점유 셀)로 계산되어
          A* 가 "경로 없음 — 빈 path" 를 반복 발행한다.
          → target='map', source='base_footprint' 로 정정. map → odom_filtered →
            base_footprint 체인을 TF 가 자동으로 합성해 map 기준 좌표를 돌려줌.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as e:
            self.get_logger().warn(f'TF lookup 실패: {e}')
            return None

    # ══════════════════════════════════════════════════════════════
    # Path 메시지 변환 / 발행
    # ══════════════════════════════════════════════════════════════

    def _cells_to_path(self, cells: list) -> Path:
        """
        셀 리스트 → nav_msgs/Path (frame_id=map).
        인접 점 방향으로 heading(yaw → quaternion) 채움.
        """
        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        world_points = [self._cell_to_world(c) for c in cells]

        for i, (wx, wy) in enumerate(world_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0

            # heading: 다음 점 방향으로 yaw 계산
            if i < len(world_points) - 1:
                nx, ny = world_points[i + 1]
                yaw = math.atan2(ny - wy, nx - wx)
            else:
                # 마지막 점: 직전 방향 유지
                if len(world_points) >= 2:
                    px, py = world_points[-2]
                    yaw = math.atan2(wy - py, wx - px)
                else:
                    yaw = 0.0

            # yaw → quaternion (z축 회전만, 2D)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(pose)

        return path

    def _publish_empty_path(self):
        """실패 시 빈 Path 발행 — DWA 정지 트리거."""
        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        self.path_pub.publish(path)


# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AstarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
