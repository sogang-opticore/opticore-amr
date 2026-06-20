"""P-10 — CCTV no-go 오버레이 (바닥투영 → /dynamic_obstacle_layer 발행).

PLAN §1.1/§1.4 계약:
  - /cctv/detections(Detection2DArray, frame_id=카메라) 구독 → foot pixel 투영 → world → map → 누적.
  - /map(TRANSIENT_LOCAL) 구독 → info(해상도/origin/dims) 캐시. 발행 grid 는 /map.info 통째 복제(full-size).
  - publish_rate_hz 타이머: decay_timeout 지난 점 만료 → 클러스터 → min_persist 충족분만 disc 페인트(=100)
    → OccupancyGrid(frame=map, stamp=sim-now, RELIABLE/VOLATILE) 를 layer_topic 으로 발행.
  - 활성→비활성 전환 시 clear grid 짧게 발행 후 침묵(DWA 정상 장애물 안 지움).
  - world→map: map = world + (map_offset_x,map_offset_y) (실측 (−2.96,−15.12)).
"""
from __future__ import annotations

import array
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from amr_perception.cctv_projection import (
    project_pixel_to_ground, world_to_map, map_to_cell)

GLOBAL_FRAME = 'map'

# 4대 카메라 기본 extrinsics (warehouse.world, world 프레임) [x,y,z,roll,pitch,yaw]
DEFAULT_POSES = {
    'corridor_1n': [14.5, 18.5, 4.49, 0.0, 1.1345, -1.5708],
    'corridor_1s': [14.5, 5.5, 4.49, 0.0, 1.1345, 1.5708],
    'corridor_2n': [49.5, 24.5, 4.49, 0.0, 1.1345, -1.5708],
    'corridor_2s': [49.5, 11.5, 4.49, 0.0, 1.1345, 1.5708],
}


class CctvNogoOverlay(Node):
    def __init__(self):
        super().__init__('cctv_nogo_overlay')

        self.declare_parameter('detections_topic', '/cctv/detections')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('layer_topic', '/dynamic_obstacle_layer')
        self.declare_parameter('camera_names', ['corridor_1n', 'corridor_1s'])
        self.declare_parameter('default_intrinsics', [293.244, 293.244, 320.0, 240.0])
        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('map_offset_x', -2.96)
        self.declare_parameter('map_offset_y', -15.12)
        self.declare_parameter('nogo_radius_m', 0.9)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('decay_timeout_sec', 1.5)
        self.declare_parameter('min_persist_count', 3)
        self.declare_parameter('cluster_radius_m', 1.0)
        self.declare_parameter('clear_hold_sec', 1.2)
        # 카메라별 pose override (기본은 DEFAULT_POSES)
        for cam, pose in DEFAULT_POSES.items():
            self.declare_parameter(f'pose_{cam}', pose)

        self.cams = list(self.get_parameter('camera_names').value)
        det_topic = self.get_parameter('detections_topic').value
        map_topic = self.get_parameter('map_topic').value
        layer_topic = self.get_parameter('layer_topic').value
        self.def_K = tuple(self.get_parameter('default_intrinsics').value)
        self.use_cinfo = bool(self.get_parameter('use_camera_info').value)
        self.off_x = float(self.get_parameter('map_offset_x').value)
        self.off_y = float(self.get_parameter('map_offset_y').value)
        self.radius = float(self.get_parameter('nogo_radius_m').value)
        self.value = int(self.get_parameter('occupied_value').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.decay = float(self.get_parameter('decay_timeout_sec').value)
        self.min_persist = int(self.get_parameter('min_persist_count').value)
        self.cluster_r = float(self.get_parameter('cluster_radius_m').value)
        self.clear_hold = max(1, int(self.get_parameter('clear_hold_sec').value * rate))

        self.poses = {}
        for cam in self.cams:
            try:
                self.poses[cam] = tuple(self.get_parameter(f'pose_{cam}').value)
            except rclpy.exceptions.ParameterNotDeclaredException:
                if cam in DEFAULT_POSES:
                    self.poses[cam] = tuple(DEFAULT_POSES[cam])
                else:
                    self.get_logger().warn(f'{cam}: pose 파라미터 없음 — 투영 불가')

        self.K = {c: self.def_K for c in self.cams}
        self.map_info = None
        self.points = deque()          # (sim_t, world_x, world_y)
        self._was_active = False
        self._clear_remaining = 0

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        layer_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST)
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.create_subscription(Detection2DArray, det_topic, self._on_det, sub_qos)
        if self.use_cinfo:
            for cam in self.cams:
                self.create_subscription(
                    CameraInfo, f'/cctv/{cam}/camera_info',
                    lambda m, c=cam: self._on_cinfo(m, c), sub_qos)

        self.layer_pub = self.create_publisher(OccupancyGrid, layer_topic, layer_qos)
        self.timer = self.create_timer(1.0 / max(0.5, rate), self._publish)
        self.get_logger().info(
            f'CctvNogoOverlay 시작 | layer={layer_topic} map={map_topic} '
            f'offset=({self.off_x},{self.off_y}) radius={self.radius} rate={rate}Hz '
            f'cams={self.cams}')

    # ---- callbacks ----
    def _on_map(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.get_logger().info(
            f'/map 수신: {msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f} '
            f'origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f})',
            once=True)

    def _on_cinfo(self, msg: CameraInfo, cam: str):
        k = msg.k
        if k and k[0] > 0.0:
            self.K[cam] = (k[0], k[4], k[2], k[5])

    def _on_det(self, msg: Detection2DArray):
        cam = msg.header.frame_id
        pose = self.poses.get(cam)
        if pose is None or not msg.detections:
            return
        K = self.K.get(cam, self.def_K)
        t = self._stamp_sec(msg.header.stamp)
        if t <= 0.0:
            t = self._now()
        for d in msg.detections:
            foot_u = d.bbox.center.position.x
            foot_v = d.bbox.center.position.y + d.bbox.size_y / 2.0  # bbox 하단중앙 = 발
            world = project_pixel_to_ground(foot_u, foot_v, pose, K)
            if world is None:
                continue
            self.points.append((t, world[0], world[1]))

    # ---- publish ----
    def _publish(self):
        if self.map_info is None:
            return
        now = self._now()
        # 만료 제거
        cutoff = now - self.decay
        while self.points and self.points[0][0] < cutoff:
            self.points.popleft()

        clusters = self._cluster([(x, y) for (_, x, y) in self.points])
        active_centroids = [c for c in clusters if c[2] >= self.min_persist]
        active = len(active_centroids) > 0

        if active:
            grid = self._build_grid(active_centroids, now)
            self.layer_pub.publish(grid)
            self._was_active = True
            self._clear_remaining = self.clear_hold
            self.get_logger().info(
                f'no-go 발행: {len(active_centroids)} 클러스터 '
                f'cells>0 @ {[(round(c[0],2),round(c[1],2)) for c in active_centroids]}',
                throttle_duration_sec=2.0)
        elif self._was_active:
            # 소멸 전환 — clear grid 짧게 발행 후 침묵(DWA 정상 장애물 보호)
            self.layer_pub.publish(self._build_grid([], now))
            self._clear_remaining -= 1
            if self._clear_remaining <= 0:
                self._was_active = False
                self.get_logger().info('no-go 소멸 → 좌핀치 재개방 (clear 발행 후 침묵)')

    def _cluster(self, pts):
        """근접점 그리디 클러스터링 → [(cx, cy, count), ...] (world 프레임)."""
        clusters = []  # [sx, sy, count]
        r2 = self.cluster_r * self.cluster_r
        for (x, y) in pts:
            placed = False
            for c in clusters:
                cx, cy = c[0] / c[2], c[1] / c[2]
                if (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                    c[0] += x; c[1] += y; c[2] += 1
                    placed = True
                    break
            if not placed:
                clusters.append([x, y, 1])
        return [(c[0] / c[2], c[1] / c[2], c[2]) for c in clusters]

    def _build_grid(self, centroids, stamp_sec: float) -> OccupancyGrid:
        info = self.map_info
        w, h, res = int(info.width), int(info.height), float(info.resolution)
        arr = np.zeros((h, w), dtype=np.int8)
        r_cells = max(1, int(math.ceil(self.radius / res)))
        for (wx, wy, _cnt) in centroids:
            mx, my = world_to_map(wx, wy, self.off_x, self.off_y)
            cell = map_to_cell(mx, my, info.origin.position.x, info.origin.position.y, res)
            if cell is None:
                continue
            col, row = cell
            if col < 0 or row < 0 or col >= w or row >= h:
                continue
            self._paint_disc(arr, col, row, r_cells, w, h)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = GLOBAL_FRAME
        msg.info = info
        msg.data = array.array('b', arr.tobytes())
        return msg

    def _paint_disc(self, arr, col, row, r_cells, w, h):
        y0, y1 = max(0, row - r_cells), min(h, row + r_cells + 1)
        x0, x1 = max(0, col - r_cells), min(w, col + r_cells + 1)
        if y0 >= y1 or x0 >= x1:
            return
        ys = np.arange(y0, y1)[:, None]
        xs = np.arange(x0, x1)[None, :]
        mask = (ys - row) ** 2 + (xs - col) ** 2 <= r_cells * r_cells
        sub = arr[y0:y1, x0:x1]
        sub[mask] = self.value

    # ---- time helpers ----
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CctvNogoOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
