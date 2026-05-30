#!/usr/bin/env python3
"""
frontier_explorer.py — A*+DWA를 활용한 frontier-based 자율 매핑 explorer

⚠ 사용 시점: SLAM **mapping 모드** 일 때만 의미가 있다.
   저장된 맵 + AMCL(localization 모드)에서는 unknown(-1)이 거의 없으므로
   frontier가 즉시 비고 FINISHED 상태로 전이된다. 이미 만들어진 맵을
   주행하는 시나리오에서는 본 노드를 띄울 필요가 없다.

비유:
    안갯속에서 손전등(LiDAR)을 들고 걷는 사람이 있다. 시야 끝에 "안개 가장자리"
    가 보이면 그쪽으로 걸어가서 더 본다. 가장자리가 모두 사라지면 다 본 것.
    우리 A*+DWA는 "어디로 갈지만 알려주면 거기로 데려다주는 운전자"이고,
    frontier_explorer는 "가장자리만 골라서 운전자에게 알려주는 길잡이" 다.

알고리즘:
    1. /map (OccupancyGrid) 받는다.
    2. frontier 셀 검출 — free(0~free_threshold) 셀 중 unknown(-1)과 8-인접.
    3. BFS로 cluster화 → min_cluster_size 미만은 제외.
    4. 점수 = cluster_size / distance(robot, centroid) — 크고 가까운 cluster 선호.
    5. 이미 방문 시도한 centroid 근처(visit_radius)는 제외.
    6. best cluster centroid를 /goal_pose 로 발행 → A*+DWA가 알아서 이동.
    7. /dwa/status 모니터 + goal_timeout 으로 도착/실패 판단.
    8. 모든 frontier 소진 시 종료.

입력:
    /map         OccupancyGrid (TRANSIENT_LOCAL)
    /dwa/status  String — STOPPED/PLANNING/...
    TF           map -> base_footprint

출력:
    /goal_pose                 PoseStamped — A* 노드 입력
    /exploration/frontiers     MarkerArray — Foxglove 시각화
    /exploration/status        String      — EXPLORING/WAITING/FINISHED

설계 원칙 (CLAUDE.md):
    1. ROS 비의존 순수 함수와 ROS 노드 분리 (단위 테스트 용이).
    2. /cmd_vel을 직접 발행하지 않고 A*+DWA를 그대로 활용.
    3. 모든 튜닝 값은 declare_parameter 외부화.
    4. 한국어 주석 우선.
"""

from __future__ import annotations

import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# heuristics.py 재사용 — 우리가 통합한 모듈
from amr_navigation.heuristics import (
    GridInfo, world_to_grid, grid_to_world, in_bounds,
)


# ---------------------------------------------------------------------------
# 순수 함수 — frontier 검출·클러스터링·goal 선택 (ROS 비의존)
# ---------------------------------------------------------------------------

Cell = Tuple[int, int]   # (col, row), OccupancyGrid 인덱스 컨벤션


def _is_free(v: int, free_threshold: int = 50) -> bool:
    """OccupancyGrid 셀이 자유 공간인가? (0 <= v < free_threshold)"""
    return 0 <= v < free_threshold


def _is_unknown(v: int) -> bool:
    return v < 0


# 8-방향 이웃 (heuristics.py와 동일 컨벤션 — (col, row) 가산)
_NEIGHBORS_8 = (
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)


def find_frontier_cells(
    data: List[int], w: int, h: int,
    free_threshold: int = 50,
) -> List[Cell]:
    """OccupancyGrid에서 frontier 셀 (col, row) 리스트.

    frontier = free 셀 중 8-이웃 하나라도 unknown인 셀.
    비유: '안개 가장자리' — 빛이 닿은 곳(free)인데 바로 옆이 안갯속(unknown).
    """
    out: List[Cell] = []
    for row in range(h):
        for col in range(w):
            v = data[row * w + col]
            if not _is_free(v, free_threshold):
                continue
            for dx, dy in _NEIGHBORS_8:
                nx, ny = col + dx, row + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if _is_unknown(data[ny * w + nx]):
                    out.append((col, row))
                    break
    return out


def cluster_frontiers(
    cells: List[Cell],
    min_size: int = 8,
) -> List[List[Cell]]:
    """frontier 셀을 BFS로 cluster화. min_size 미만은 제외.

    하나의 안개 가장자리(긴 곡선)는 보통 연결된 셀들의 띠다. 그걸 cluster로
    묶어 centroid를 잡으면 goal 한 점으로 환원할 수 있다.
    """
    cell_set = set(cells)
    visited: set = set()
    clusters: List[List[Cell]] = []
    for start in cells:
        if start in visited:
            continue
        cluster: List[Cell] = []
        q: deque = deque([start])
        while q:
            c = q.popleft()
            if c in visited:
                continue
            visited.add(c)
            cluster.append(c)
            cx, cy = c
            for dx, dy in _NEIGHBORS_8:
                n = (cx + dx, cy + dy)
                if n in cell_set and n not in visited:
                    q.append(n)
        if len(cluster) >= min_size:
            clusters.append(cluster)
    return clusters


def cluster_centroid(cluster: List[Cell]) -> Tuple[float, float]:
    """클러스터 중심 (col_mean, row_mean) — float."""
    if not cluster:
        return (0.0, 0.0)
    sx = sum(c[0] for c in cluster) / len(cluster)
    sy = sum(c[1] for c in cluster) / len(cluster)
    return (sx, sy)


def select_best_cluster(
    clusters: List[List[Cell]],
    robot_cell: Cell,
    visited_centroids: List[Cell],
    min_dist_cells: float,
    visit_radius_cells: float,
) -> Optional[Tuple[List[Cell], Cell]]:
    """best cluster + 그 centroid 셀 반환. 없으면 None.

    점수 = cluster_size / max(distance, 1.0)
        - 큰 cluster일수록 좋고 (정보 이득↑)
        - 가까울수록 좋음 (이동 시간↓)
    """
    if not clusters:
        return None

    rx, ry = robot_cell
    best: Optional[Tuple[List[Cell], Cell]] = None
    best_score = -float("inf")

    for cluster in clusters:
        cx_f, cy_f = cluster_centroid(cluster)
        cx, cy = int(round(cx_f)), int(round(cy_f))
        d = math.hypot(cx - rx, cy - ry)
        if d < min_dist_cells:
            continue   # 너무 가까움 (현재 위치 근처)
        # 이미 방문 시도한 centroid 근처는 스킵
        if any(
            math.hypot(cx - vx, cy - vy) < visit_radius_cells
            for vx, vy in visited_centroids
        ):
            continue
        score = len(cluster) / max(d, 1.0)
        if score > best_score:
            best_score = score
            best = (cluster, (cx, cy))

    return best


# ---------------------------------------------------------------------------
# ROS 노드
# ---------------------------------------------------------------------------

class FrontierExplorerNode(Node):
    """A*+DWA를 활용한 자율 매핑 explorer.

    /cmd_vel을 직접 발행하지 않고, /goal_pose로 A* 노드를 지휘한다.
    """

    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        # ── 파라미터 ────────────────────────────────────────────────
        self.declare_parameter("min_cluster_size", 8)
        self.declare_parameter("free_threshold", 50)         # 0 <= v < threshold = free
        self.declare_parameter("min_goal_dist", 0.5)         # m
        self.declare_parameter("visit_radius", 0.8)          # m
        self.declare_parameter("replan_period", 3.0)         # s, 주기 평가
        self.declare_parameter("goal_timeout", 30.0)         # s, 한 goal 최대 시간
        self.declare_parameter("arrival_settle_time", 2.0)   # s, STOPPED 가 이 시간 유지되면 도착으로 판정
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("status_publish_period", 1.0)

        self.p_min_cluster_size = self.get_parameter("min_cluster_size").value
        self.p_free_threshold = self.get_parameter("free_threshold").value
        self.p_min_goal_dist = self.get_parameter("min_goal_dist").value
        self.p_visit_radius = self.get_parameter("visit_radius").value
        self.p_replan_period = self.get_parameter("replan_period").value
        self.p_goal_timeout = self.get_parameter("goal_timeout").value
        self.p_arrival_settle_time = self.get_parameter("arrival_settle_time").value
        self.p_robot_frame = self.get_parameter("robot_frame").value
        self.p_map_frame = self.get_parameter("map_frame").value
        self.p_status_publish_period = self.get_parameter("status_publish_period").value

        # ── 상태 ────────────────────────────────────────────────────
        self._map: Optional[OccupancyGrid] = None
        self._dwa_status: str = "UNKNOWN"
        self._dwa_status_since: Optional[float] = None
        self._current_goal_cell: Optional[Cell] = None
        self._current_goal_world: Optional[Tuple[float, float]] = None
        self._goal_sent_time: Optional[float] = None
        self._visited_centroids: List[Cell] = []
        self._finished: bool = False

        # ── TF ──────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── QoS ─────────────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── 구독 ────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(String, "/dwa/status", self._on_dwa_status, 10)

        # ── 발행 ────────────────────────────────────────────────────
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, "/exploration/frontiers", 10
        )
        self.status_pub = self.create_publisher(String, "/exploration/status", 10)

        # ── 타이머 ──────────────────────────────────────────────────
        self.create_timer(self.p_replan_period, self._tick)
        self.create_timer(self.p_status_publish_period, self._publish_status)

        self.get_logger().info(
            f"FrontierExplorer 시작 — "
            f"min_cluster_size={self.p_min_cluster_size}, "
            f"replan_period={self.p_replan_period}s, "
            f"goal_timeout={self.p_goal_timeout}s"
        )

    # ── 콜백 ────────────────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_dwa_status(self, msg: String) -> None:
        if msg.data != self._dwa_status:
            self._dwa_status = msg.data
            self._dwa_status_since = self._now()

    # ── 메인 루프 (replan_period 마다) ──────────────────────────────
    def _tick(self) -> None:
        if self._finished:
            return
        if self._map is None:
            self.get_logger().warn("맵 미수신 — 대기 중")
            return

        # 현재 진행 중인 goal 상태 확인
        if self._current_goal_cell is not None:
            if self._is_goal_done():
                self._mark_current_visited()
                self._current_goal_cell = None
            else:
                # 아직 진행 중 — 다음 tick에서 다시
                return

        # 로봇 현재 셀
        robot_cell = self._lookup_robot_cell()
        if robot_cell is None:
            return

        # frontier 검출 + 클러스터링
        m = self._map
        w, h = m.info.width, m.info.height
        cells = find_frontier_cells(
            list(m.data), w, h, free_threshold=self.p_free_threshold
        )
        clusters = cluster_frontiers(cells, min_size=self.p_min_cluster_size)

        self.get_logger().info(
            f"frontier: {len(cells)} 셀, {len(clusters)} 클러스터, "
            f"방문기록={len(self._visited_centroids)}"
        )
        self._publish_frontier_markers(clusters)

        # best cluster 선택
        res = m.info.resolution
        min_dist_cells = self.p_min_goal_dist / res
        visit_radius_cells = self.p_visit_radius / res

        best = select_best_cluster(
            clusters, robot_cell,
            self._visited_centroids,
            min_dist_cells, visit_radius_cells,
        )

        if best is None:
            self.get_logger().info("남은 frontier 없음 — 매핑 종료 ✓")
            self._finished = True
            self._publish_status_value("FINISHED")
            return

        _, centroid_cell = best
        self._send_goal(centroid_cell)

    def _is_goal_done(self) -> bool:
        """현재 goal이 도착(또는 실패)했는지 판정.

        판정 규칙 (2026-05-31 리뷰 반영, DWA P2 GOAL_REACHED 도입 대응):
          (성공) /dwa/status 가 'GOAL_REACHED'(도착 edge, 1회) → 즉시 완료.
          (성공) 'REACHED'(도착 후 1Hz 정상 상태)가 arrival_settle_time 이상 유지 → 완료.
          (실패) 'STOPPED'(path 없음/계획 실패, README §3.3)가 settle 이상 유지 → 이 frontier 포기.
          (실패) goal_timeout 초과.
        ※ 이전 구현은 'STOPPED' 만 봤는데, P2 이후 DWA 는 도착 시 GOAL_REACHED→REACHED 를
           발행(STOPPED 아님) → 도착을 못 알아채고 timeout 까지 대기하던 문제(Codex 리뷰) 수정.
        """
        if self._goal_sent_time is None:
            return False
        elapsed = self._now() - self._goal_sent_time
        if elapsed > self.p_goal_timeout:
            self.get_logger().warn(
                f"Goal timeout ({elapsed:.1f}s ≥ {self.p_goal_timeout:.1f}s) — 스킵"
            )
            return True

        # 성공성 도착 — GOAL_REACHED 는 edge 라 즉시 완료
        if self._dwa_status == "GOAL_REACHED":
            self.get_logger().info("Goal 도착(GOAL_REACHED) — 완료")
            return True

        settled = (
            self._dwa_status_since is not None
            and (self._now() - self._dwa_status_since) >= self.p_arrival_settle_time
            and elapsed > self.p_arrival_settle_time  # goal 발행 직후 잔존 상태 제외
        )
        if self._dwa_status == "REACHED" and settled:
            self.get_logger().info("Goal 도착(REACHED 유지) — 완료")
            return True
        if self._dwa_status == "STOPPED" and settled:
            # STOPPED = path 없음/계획 실패(성공 아님) → 이 frontier 는 도달 불가로 보고 스킵
            self.get_logger().warn("DWA STOPPED 유지(계획 실패 추정) — frontier 스킵")
            return True
        return False

    # ── 헬퍼 ────────────────────────────────────────────────────────
    def _lookup_robot_cell(self) -> Optional[Cell]:
        if self._map is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.p_map_frame, self.p_robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup 실패 ({self.p_map_frame}->{self.p_robot_frame}): {e}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        info_dc = self._info_dc()
        cell = world_to_grid((x, y), info_dc)
        if not in_bounds(cell, info_dc):
            self.get_logger().warn(
                f"로봇이 맵 밖에 있음: cell={cell}, world=({x:.2f},{y:.2f})"
            )
            return None
        return cell

    def _info_dc(self) -> GridInfo:
        info = self._map.info
        return GridInfo(
            width=info.width, height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
        )

    def _send_goal(self, centroid_cell: Cell) -> None:
        info_dc = self._info_dc()
        wx, wy = grid_to_world(centroid_cell, info_dc)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p_map_frame
        msg.pose.position.x = float(wx)
        msg.pose.position.y = float(wy)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        self._current_goal_cell = centroid_cell
        self._current_goal_world = (wx, wy)
        self._goal_sent_time = self._now()
        # 새 goal 발행 직후엔 status 변화 카운터 리셋 (직전 STOPPED 잔존 방지)
        self._dwa_status_since = self._now()

        self.get_logger().info(
            f"새 goal 발행 → cell={centroid_cell}, world=({wx:.2f}, {wy:.2f})"
        )

    def _mark_current_visited(self) -> None:
        if self._current_goal_cell is not None:
            self._visited_centroids.append(self._current_goal_cell)
            self.get_logger().info(
                f"visited+ {self._current_goal_cell} "
                f"(총 {len(self._visited_centroids)})"
            )

    # ── 시각화 ──────────────────────────────────────────────────────
    def _publish_frontier_markers(self, clusters: List[List[Cell]]) -> None:
        if self._map is None:
            return
        info_dc = self._info_dc()

        array = MarkerArray()
        # 이전 마커 정리
        clear = Marker()
        clear.header.frame_id = self.p_map_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, cluster in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.p_map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "frontier_cells"
            m.id = i
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = info_dc.resolution * 1.4
            m.scale.y = info_dc.resolution * 1.4
            # cluster마다 다른 색
            m.color.r = float((i * 47) % 255) / 255.0
            m.color.g = float((i * 113) % 255) / 255.0
            m.color.b = float((i * 197) % 255) / 255.0
            m.color.a = 0.85
            for c in cluster:
                wx, wy = grid_to_world(c, info_dc)
                pt = Point()
                pt.x = float(wx)
                pt.y = float(wy)
                pt.z = 0.02
                m.points.append(pt)
            array.markers.append(m)

        # 방문한 centroid 마커
        if self._visited_centroids:
            vm = Marker()
            vm.header.frame_id = self.p_map_frame
            vm.header.stamp = self.get_clock().now().to_msg()
            vm.ns = "visited_centroids"
            vm.id = 9999
            vm.type = Marker.POINTS
            vm.action = Marker.ADD
            vm.scale.x = info_dc.resolution * 4.0
            vm.scale.y = info_dc.resolution * 4.0
            vm.color.r = 0.7
            vm.color.g = 0.7
            vm.color.b = 0.7
            vm.color.a = 0.6
            for c in self._visited_centroids:
                wx, wy = grid_to_world(c, info_dc)
                pt = Point()
                pt.x = float(wx)
                pt.y = float(wy)
                pt.z = 0.05
                vm.points.append(pt)
            array.markers.append(vm)

        # 현재 goal centroid 강조
        if self._current_goal_world is not None:
            gm = Marker()
            gm.header.frame_id = self.p_map_frame
            gm.header.stamp = self.get_clock().now().to_msg()
            gm.ns = "current_goal"
            gm.id = 10000
            gm.type = Marker.SPHERE
            gm.action = Marker.ADD
            gm.scale.x = info_dc.resolution * 8.0
            gm.scale.y = info_dc.resolution * 8.0
            gm.scale.z = info_dc.resolution * 8.0
            gm.color.r = 1.0
            gm.color.g = 0.3
            gm.color.b = 0.0
            gm.color.a = 0.8
            gm.pose.position.x = float(self._current_goal_world[0])
            gm.pose.position.y = float(self._current_goal_world[1])
            gm.pose.position.z = 0.1
            gm.pose.orientation.w = 1.0
            array.markers.append(gm)

        self.frontier_marker_pub.publish(array)

    # ── 상태 발행 ───────────────────────────────────────────────────
    def _publish_status(self) -> None:
        if self._finished:
            self._publish_status_value("FINISHED")
        elif self._map is None:
            self._publish_status_value("WAITING_MAP")
        elif self._current_goal_cell is not None:
            self._publish_status_value("EXPLORING")
        else:
            self._publish_status_value("WAITING")

    def _publish_status_value(self, v: str) -> None:
        msg = String()
        msg.data = v
        self.status_pub.publish(msg)

    def _now(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
