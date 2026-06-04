#!/usr/bin/env python3
"""
fused_tracker — LiDAR + 로봇 카메라 YOLO + CCTV YOLO 융합 semantic 트래커

P-5 스캐폴드 + P-6(LiDAR 자체 클러스터링, 옵션 A) + P-7(multi-source 칼만).

P-7 추가분:
  predict(등속) → 소스별 순차 association(거리+클래스 게이트) → update(source별 R 차등)
  → unmatched 관측은 새 트랙, unmatched 트랙은 miss_count 증가 후 소멸.
  위치는 칼만이 공분산으로 자동 가중(LiDAR R 작게 → 신뢰), 클래스는 YOLO가 채움.
  velocity는 칼만 state[vx,vy]에서 나옴. _publish_tracks가 KF 트랙 → TrackedObject 변환.

구현 단계:
  P-6   LiDAR 자체 클러스터링 → _process_lidar
  P-7   multi-source 칼만 + association + 클래스 → _update_tracks (이번)
  P-8   TrackedObject MarkerArray 컴패니언 → self._marker_pub
  P-9/10  CCTV 디텍션 입력 → _cctv_det_cb (지금은 보관만)

TODO(P-8): /perception/tracked_objects/markers (TrackedObject 3D 박스).
  self._marker_pub 자리는 그대로 비워둔다 (cluster 마커와 별개).
"""

import math
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, CameraInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from tf2_ros import Buffer, TransformListener, TransformException

from amr_msgs.msg import TrackedObject, TrackedObjectArray

from amr_perception.lidar_clustering import (
    scan_to_points,
    cluster_points,
    has_static_obstacle_near,
)
# --- P-7: 융합 칼만 코어 (ROS 비의존 순수 모듈) ---
from amr_perception.multisource_kalman import (
    Observation,
    FusedKalmanTrack,
    associate,
    SOURCE_LIDAR,
    SOURCE_ROBOT_CAM,
)


OUTPUT_FRAME = 'map'
PUBLISH_RATE_HZ = 10.0


@dataclass
class LidarObservation:
    """P-6 산출물: map 프레임 동적 관측 (클래스/속도 없음 — source=lidar)."""
    x: float
    y: float
    radius: float
    stamp: object  # builtin_interfaces/Time (scan.header.stamp)


class FusedTracker(Node):
    def __init__(self):
        super().__init__('fused_tracker')

        # --- 파라미터 (launch/yaml에서 주입) ---
        self.declare_parameter('output_topic', '/perception/tracked_objects')
        self.declare_parameter('lidar_topic', '/lidar')
        self.declare_parameter('robot_detections_topic', '/perception/detections')
        self.declare_parameter('cctv_detections_topic', '/cctv/detections')  # [미확정] P-9 확정 시 갱신
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('lidar_frame', 'lidar_link')  # scan.header.frame_id 비었을 때 fallback
        self.declare_parameter('camera_frame', 'camera_optical_link')  # robot cam 역투영 프레임
        self.declare_parameter('cluster_markers_topic', '/perception/lidar_clusters')

        # P-6 클러스터링 (DWA dynamic_track_* 출발값 재사용)
        self.declare_parameter('cluster_distance', 0.35)
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('cluster_max_radius', 0.85)
        # P-6 static wall 필터 (DWA dynamic_static_filter_* 재사용)
        self.declare_parameter('static_filter_enabled', True)
        self.declare_parameter('static_filter_radius', 0.60)
        self.declare_parameter('static_filter_occupied_threshold', 65)
        self.declare_parameter('static_filter_unknown_as_static', True)
        # P-7 association/소멸 (object_tracker 출발값 재사용)
        self.declare_parameter('max_association_dist', 4.0)
        self.declare_parameter('max_miss_count', 90)

        gp = self.get_parameter
        output_topic = gp('output_topic').value
        lidar_topic = gp('lidar_topic').value
        robot_det_topic = gp('robot_detections_topic').value
        cctv_det_topic = gp('cctv_detections_topic').value
        cam_info_topic = gp('camera_info_topic').value
        map_topic = gp('map_topic').value
        cluster_markers_topic = gp('cluster_markers_topic').value

        self._lidar_frame = gp('lidar_frame').value
        self._camera_frame = gp('camera_frame').value
        self._cluster_distance = float(gp('cluster_distance').value)
        self._cluster_min_points = int(gp('cluster_min_points').value)
        self._cluster_max_radius = float(gp('cluster_max_radius').value)
        self._static_filter_enabled = bool(gp('static_filter_enabled').value)
        self._static_filter_radius = float(gp('static_filter_radius').value)
        self._static_filter_occupied_threshold = int(gp('static_filter_occupied_threshold').value)
        self._static_filter_unknown_as_static = bool(gp('static_filter_unknown_as_static').value)
        self._max_assoc_dist = float(gp('max_association_dist').value)
        self._max_miss = int(gp('max_miss_count').value)

        # --- QoS ---
        track_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        lidar_qos = qos_profile_sensor_data
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- 최신 메시지 보관 ---
        self._latest_scan = None
        self._latest_map = None
        self._latest_robot_det = None
        self._latest_cctv_det = None
        self._camera_info = None

        # --- P-6 산출물: map 프레임 LiDAR 동적 관측 ---
        self._lidar_observations = []  # List[LidarObservation]

        # --- P-7 트랙 상태: 융합 칼만 필터 리스트 (publish 시 TrackedObject로 변환) ---
        self._kf_tracks = []  # List[FusedKalmanTrack]

        # --- TF (map ← odom_filtered ← ... ← {lidar_link, camera_optical_link}) ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- 구독 ---
        self.create_subscription(LaserScan, lidar_topic, self._lidar_cb, lidar_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(Detection2DArray, robot_det_topic, self._robot_det_cb, reliable_qos)
        self.create_subscription(Detection2DArray, cctv_det_topic, self._cctv_det_cb, reliable_qos)
        self.create_subscription(CameraInfo, cam_info_topic, self._camera_info_cb, reliable_qos)

        # --- 발행 ---
        self._track_pub = self.create_publisher(TrackedObjectArray, output_topic, track_qos)
        self._cluster_marker_pub = self.create_publisher(MarkerArray, cluster_markers_topic, 10)
        self._marker_pub = None  # TODO(P-8): TrackedObject MarkerArray 컴패니언

        # --- 타이머: 클러스터링 → 융합 → 트랙 발행 ---
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"fused_tracker 기동 (P-7). 출력={output_topic} (frame={OUTPUT_FRAME}). "
            f"cluster(d={self._cluster_distance}, min={self._cluster_min_points}, "
            f"rmax={self._cluster_max_radius}) static_filter={self._static_filter_enabled} "
            f"assoc_dist={self._max_assoc_dist} max_miss={self._max_miss}. "
            f"구독: {lidar_topic}, {map_topic}, {robot_det_topic}, {cctv_det_topic}, {cam_info_topic}"
        )

    # ----- 구독 콜백 (보관만) -----
    def _lidar_cb(self, msg: LaserScan):
        self._latest_scan = msg

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _robot_det_cb(self, msg: Detection2DArray):
        self._latest_robot_det = msg  # P-7: _tick에서 소비(consume-once)

    def _cctv_det_cb(self, msg: Detection2DArray):
        self._latest_cctv_det = msg  # TODO(P-10): CCTV ground projection → map

    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_info = msg  # K matrix (robot cam 역투영용)

    # ----- 타이머 -----
    def _tick(self):
        self._process_lidar()
        self._update_tracks()   # P-7
        self._publish_tracks()

    # ----- P-6: LiDAR scan → map 프레임 동적 관측 -----
    def _process_lidar(self):
        self._lidar_observations = []
        scan = self._latest_scan
        if scan is None:
            return

        points = scan_to_points(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
        )
        clusters = cluster_points(
            points, self._cluster_distance,
            self._cluster_min_points, self._cluster_max_radius,
        )
        if not clusters:
            self._publish_cluster_markers([], [])
            return

        src_frame = scan.header.frame_id or self._lidar_frame
        tf = self._lookup_tf(src_frame, scan.header.stamp)
        if tf is None:
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        grid = self._latest_map
        if self._static_filter_enabled and grid is None:
            self.get_logger().warn(
                '/map 미수신 — static 필터 비활성 상태로 동작(모든 cluster 동적 후보).',
                throttle_duration_sec=5.0,
            )

        dynamic_obs = []
        static_clusters = []
        for c in clusters:
            cx, cy = c.center
            mx = tx + cx * cos_y - cy * sin_y
            my = ty + cx * sin_y + cy * cos_y

            if (self._static_filter_enabled and grid is not None
                    and has_static_obstacle_near(
                        grid, (mx, my),
                        self._static_filter_radius,
                        self._static_filter_occupied_threshold,
                        self._static_filter_unknown_as_static)):
                static_clusters.append((mx, my, c.radius))
                continue

            dynamic_obs.append(LidarObservation(mx, my, c.radius, scan.header.stamp))

        self._lidar_observations = dynamic_obs
        self._publish_cluster_markers(dynamic_obs, static_clusters)

    # ----- P-7: multi-source 칼만 융합 -----
    def _update_tracks(self):
        dt = 1.0 / PUBLISH_RATE_HZ

        # 1) predict + miss_count 증가 (update 시 0으로 리셋)
        for t in self._kf_tracks:
            t.predict(dt)
            t.miss_count += 1

        # 2) 소스별 순차 association/update.
        #    LiDAR 먼저(위치 정밀화) → robot YOLO(refine된 트랙에 클래스 스탬프).
        self._associate_and_update(self._lidar_to_obs(), dt)
        self._associate_and_update(self._robot_cam_to_obs(), dt)

        # 3) 오래 못 본 트랙 소멸
        self._kf_tracks = [t for t in self._kf_tracks if t.miss_count < self._max_miss]

    def _associate_and_update(self, observations, dt):
        if not observations:
            return
        matches, unmatched_obs, _ = associate(
            self._kf_tracks, observations, self._max_assoc_dist)
        for ti, oi in matches:
            self._kf_tracks[ti].update(observations[oi])
        for oi in unmatched_obs:
            self._kf_tracks.append(FusedKalmanTrack(observations[oi], dt))

    def _lidar_to_obs(self):
        return [Observation(o.x, o.y, SOURCE_LIDAR) for o in self._lidar_observations]

    def _robot_cam_to_obs(self):
        """robot YOLO Detection2DArray → 핀홀 역투영(지면 z=0) → map Observation. consume-once."""
        det = self._latest_robot_det
        self._latest_robot_det = None  # stale 재투영 방지
        if det is None:
            return []
        if self._camera_info is None:
            self.get_logger().warn(
                'camera_info 미수신 — robot YOLO 투영 불가(LiDAR-only 동작). '
                'camera_info_topic 파라미터 확인.',
                throttle_duration_sec=5.0,
            )
            return []

        obs = []
        for d in det.detections:
            u = d.bbox.center.position.x
            v = d.bbox.center.position.y + d.bbox.size_y * 0.5  # 바닥 닿는 하단 중앙점
            world = self._pixel_to_world(u, v, det.header)
            if world is None:
                continue
            cls, conf = None, 0.0
            if d.results:
                cls = d.results[0].hypothesis.class_id or None
                conf = float(d.results[0].hypothesis.score)
            obs.append(Observation(world[0], world[1], SOURCE_ROBOT_CAM, cls, conf))
        return obs

    def _pixel_to_world(self, u, v, header):
        """픽셀 (u,v) → camera_optical_link ray → map 지면(z=0) 교차점."""
        info = self._camera_info
        K = info.k  # row-major 9
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        if fx == 0.0 or fy == 0.0:
            return None

        # optical frame ray (x right, y down, z forward)
        ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)

        tf = self._lookup_tf(self._camera_frame, header.stamp)
        if tf is None:
            return None
        tr = tf.transform.translation
        origin = np.array([tr.x, tr.y, tr.z], dtype=float)
        ray_map = self._quat_to_rot(tf.transform.rotation) @ ray_cam

        # 지면 z=0 교차: 카메라는 지면 위(origin_z>0), ray는 아래로 향해야 함
        if ray_map[2] >= -1e-6:
            return None
        s = -origin[2] / ray_map[2]
        if s <= 0.0:
            return None
        p = origin + s * ray_map
        return float(p[0]), float(p[1])

    @staticmethod
    def _quat_to_rot(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ], dtype=float)

    def _lookup_tf(self, src_frame: str, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                OUTPUT_FRAME, src_frame, stamp, timeout=Duration(seconds=0.05))
        except TransformException:
            pass
        try:
            return self._tf_buffer.lookup_transform(OUTPUT_FRAME, src_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'TF {src_frame}->{OUTPUT_FRAME} 실패: {e}',
                throttle_duration_sec=2.0,
            )
            return None

    # ----- 검증용 cluster 마커 (동적=초록, static 필터=빨강) -----
    def _publish_cluster_markers(self, dynamic_obs, static_clusters):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        mid = 0
        for o in dynamic_obs:
            arr.markers.append(self._cyl_marker(mid, o.x, o.y, o.radius, 0.0, 1.0, 0.0))
            mid += 1
        for (mx, my, r) in static_clusters:
            arr.markers.append(self._cyl_marker(mid, mx, my, r, 1.0, 0.0, 0.0))
            mid += 1

        self._cluster_marker_pub.publish(arr)

    def _cyl_marker(self, mid, x, y, radius, cr, cg, cb):
        m = Marker()
        m.header.frame_id = OUTPUT_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lidar_clusters'
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(x=float(x), y=float(y), z=0.15)
        m.pose.orientation.w = 1.0
        d = max(0.1, 2.0 * float(radius))
        m.scale.x = d
        m.scale.y = d
        m.scale.z = 0.3
        m.color.a = 0.6
        m.color.r = cr
        m.color.g = cg
        m.color.b = cb
        return m

    # ----- 출력: KF 트랙 → TrackedObjectArray (P-7) -----
    def _publish_tracks(self):
        out = TrackedObjectArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = OUTPUT_FRAME
        out.tracks = [self._to_msg(t) for t in self._kf_tracks]
        self._track_pub.publish(out)

    def _to_msg(self, t: FusedKalmanTrack) -> TrackedObject:
        m = TrackedObject()
        m.id = int(t.track_id) & 0xFFFFFFFF
        m.class_name = t.class_name
        m.class_confidence = float(t.class_confidence)
        px, py = t.position
        m.position = Point(x=px, y=py, z=0.0)
        vx, vy = t.velocity
        m.velocity = Vector3(x=vx, y=vy, z=0.0)
        m.source = int(t.source) & 0xFF
        m.position_covariance = [float(v) for v in t.position_covariance]
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FusedTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
