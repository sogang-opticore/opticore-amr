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

import tf2_ros
from tf2_ros import TransformException

from amr_navigation.heuristics import heuristic, movement_cost


class AstarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ── 파라미터 선언 ──────────────────────────────────────────
        self.declare_parameter('heuristic', 'octile')
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('inflation_radius', 0.30)
        self.declare_parameter('smoothing', 'catmull_rom')
        # 2026-05-24 보강(SW, HU 보강-1):
        #   goal 셀이 inflation/점유로 막혔을 때 nearest free cell로 자동 보정.
        #   BFS 반경 [cell] = goal_snap_radius / resolution.
        self.declare_parameter('goal_snap_radius', 0.6)   # m, 0 이면 비활성

        # 2026-05-25 추가 (SW · 페어, DWA stuck/벗어남 문제 해결):
        # 주기적 재계획 — 로봇이 path 벗어났을 때 현재 위치에서 goal 까지 새 path.
        # 0 = 비활성 (goal 받을 때만 1회), >0 = 그 주기로 자동 재계획.
        # 비유: GPS 내비가 한 번만 길 안내하지 않고, 잘못 빠지면 "재탐색" 하는 것.
        self.declare_parameter('replan_period', 1.0)   # s, 0=비활성

        self.heuristic_type   = self.get_parameter('heuristic').value
        self.allow_diagonal   = self.get_parameter('allow_diagonal').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.smoothing        = self.get_parameter('smoothing').value
        self.goal_snap_radius = self.get_parameter('goal_snap_radius').value
        self.replan_period    = self.get_parameter('replan_period').value

        # ── 내부 상태 ──────────────────────────────────────────────
        self.map_data: OccupancyGrid | None = None
        self.inflated_grid: np.ndarray | None = None
        self.goal: PoseStamped | None = None

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

        # ── 발행 ───────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # ── 주기적 재계획 타이머 (2026-05-25 추가) ─────────────────
        # replan_period > 0 이면 그 주기로 _plan() 자동 호출. goal 이 있을 때만 동작.
        if self.replan_period > 0.0:
            self.create_timer(self.replan_period, self._replan_timer)
            self.get_logger().info(
                f'AstarPlanner 주기적 재계획 활성 — {self.replan_period}s 마다')

        self.get_logger().info('AstarPlanner 노드 시작 — 맵과 goal 대기 중')

    # ══════════════════════════════════════════════════════════════
    # 주기적 재계획 (2026-05-25 추가)
    # ══════════════════════════════════════════════════════════════
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
        start_world = self._get_robot_position()
        if start_world is not None:
            gx = self.goal.pose.position.x
            gy = self.goal.pose.position.y
            dist_to_goal = math.hypot(gx - start_world[0], gy - start_world[1])
            if dist_to_goal < 0.30:   # DWA goal_tolerance(0.20) + 마진
                return

        self._plan()

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
        """
        if self.map_data is None:
            self.get_logger().warn('맵 미수신 — goal 무시')
            return
        self.goal = msg
        self.get_logger().info(
            f'Goal 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        self._plan()

    # ══════════════════════════════════════════════════════════════
    # 경로 계획 메인
    # ══════════════════════════════════════════════════════════════

    def _plan(self):
        """
        A* 경로 계획 메인 함수.
        성공 시 /global_path 발행, 실패 시 빈 Path 발행 (DWA 정지 트리거).
        """
        # 현재 로봇 위치 TF lookup (base_footprint → map)
        start_world = self._get_robot_position()
        if start_world is None:
            self.get_logger().warn('TF lookup 실패 — 경로 계획 중단')
            self._publish_empty_path()
            return

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
                self.get_logger().warn(
                    f'Start {start_cell}이 점유/맵-밖이고 인근 자유공간 없음 — 빈 path')
                self._publish_empty_path()
                return
            self.get_logger().info(
                f'Start 보정: {start_cell} (점유/inflation) → {snapped} (인근 free)')
            start_cell = snapped

        # 2026-05-24 보강(SW, HU 보강-1): goal 셀이 막혔으면 nearest free cell 보정.
        # 비유: 우체부가 "그 주소엔 우체통이 없네요" 라고 그냥 돌아가지 않고
        #       가장 가까운 우체통을 찾아 거기에 두는 것.
        if not self._is_free_cell(goal_cell):
            snapped = self._snap_to_nearest_free(goal_cell)
            if snapped is None:
                self.get_logger().warn(
                    f'Goal {goal_cell}이 점유/맵-밖이고 인근 자유공간 없음 — 빈 path')
                self._publish_empty_path()
                return
            self.get_logger().info(
                f'Goal 보정: {goal_cell} (점유/inflation) → {snapped} (인근 free)')
            goal_cell = snapped

        cell_path = self._astar(start_cell, goal_cell)

        if cell_path is None:
            self.get_logger().warn('경로 없음 — 빈 path 발행')
            self._publish_empty_path()
            return

        if self.smoothing == 'catmull_rom':
            cell_path = self._smooth_catmull_rom(cell_path)

        path_msg = self._cells_to_path(cell_path)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'경로 발행: {len(path_msg.poses)} 웨이포인트')

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
                tentative_g = g_score[current] + movement_cost(current, neighbor)

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
        return [
            (row + dr, col + dc)
            for dr, dc in deltas
            if self._is_free_cell((row + dr, col + dc))
        ]

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
        while q:
            r, c, d = q.popleft()
            if 0 <= r < h and 0 <= c < w and self.inflated_grid[r, c] == 0:
                return (r, c)
            if d >= max_radius:
                continue
            for dr, dc in deltas:
                nr, nc = r + dr, c + dc
                if (nr, nc) in visited:
                    continue
                visited.add((nr, nc))
                q.append((nr, nc, d + 1))
        return None

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
        OccupancyGrid → 2D numpy 배열 변환 + 원형 커널 inflation 적용.
        로봇이 벽/선반에 너무 가깝게 붙지 않도록 장애물 주변을 팽창.

        점유(>=50) 또는 unknown(-1) 셀을 blocked으로 처리.

        # 보강 필요: 현재는 binary dilation (단순 팽창).
        # 거리 기반 비용 그라디언트로 바꾸면 DWA clearance와 더 잘 맞음.
        """
        from scipy.ndimage import binary_dilation

        w   = msg.info.width
        h   = msg.info.height
        res = msg.info.resolution

        raw     = np.array(msg.data, dtype=np.int8).reshape((h, w))
        blocked = (raw >= 50) | (raw == -1)

        radius_cells = int(math.ceil(self.inflation_radius / res))
        if radius_cells > 0:
            y, x    = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
            kernel  = (x*x + y*y <= radius_cells*radius_cells)
            blocked = binary_dilation(blocked, structure=kernel)

        return blocked.astype(np.uint8)

    # ══════════════════════════════════════════════════════════════
    # 경로 스무딩
    # ══════════════════════════════════════════════════════════════

    def _smooth_catmull_rom(self, cells: list, samples: int = 5) -> list:
        """
        Catmull-Rom 스플라인으로 경로 스무딩.
        인접 4점을 이용해 곡선 보간, 각 구간을 samples개 점으로 분할.

        # 보강 필요:
        # 1) samples 튜닝 — 5~10 사이 권장.
        # 2) 스무딩 후 inflated 셀 통과 여부 재검증 없음.
        #    좁은 통로에서 경로가 장애물을 뚫을 수 있음. 추후 검토.
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

        return smoothed

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
