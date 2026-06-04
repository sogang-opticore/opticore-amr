#!/usr/bin/env python3
"""
fused_tracker — LiDAR + 로봇 카메라 YOLO + CCTV YOLO 융합 semantic 트래커

P-5 스캐폴드 + P-6(LiDAR 자체 클러스터링, 옵션 A).

P-6 추가분:
  /lidar scan → 센서프레임 직교좌표 → scan 순서 jump 클러스터링
  → cluster center를 map 프레임으로 TF → /map 기반 static wall 필터
  → 동적 후보를 (위치, 반경, 관측시각) LidarObservation으로 보관 (source=lidar).
  P-7 칼만이 self._lidar_observations를 소비한다.
  검증용 /perception/lidar_clusters(MarkerArray): 동적 후보=초록, static 필터=빨강.

아직 안 하는 것(=P-7): 칼만/association/velocity/class. P-6 관측엔 velocity/class 없음.
  → _publish_tracks는 P-5처럼 빈 TrackedObjectArray 발행 그대로.

구현 단계:
  P-6   LiDAR 자체 클러스터링 → _process_lidar (이번)
  P-7   multi-source 칼만 + association + 클래스 → 융합 코어 (_lidar_observations 소비)
  P-8   map 프레임 출력 표준화 → _publish_tracks 채움
  P-9/10  CCTV 디텍션 입력 → _cctv_det_cb 채움

TODO(나중, P-8): /perception/tracked_objects/markers (TrackedObject 3D 박스).
  self._marker_pub 자리는 그대로 비워둔다 (cluster 마커와 별개).
"""

import math
from dataclasses import dataclass

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
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from tf2_ros import Buffer, TransformListener, TransformException

from amr_msgs.msg import TrackedObject, TrackedObjectArray  # noqa: F401 (TrackedObject는 P-7에서 사용)

from amr_perception.lidar_clustering import (
    scan_to_points,
    cluster_points,
    has_static_obstacle_near,
)


OUTPUT_FRAME = 'map'
PUBLISH_RATE_HZ = 10.0


@dataclass
class LidarObservation:
    """P-6 산출물: map 프레임 동적 관측 (클래스/속도 없음 — source=lidar)."""
    x: float
    y: float
    radius: float
    stamp: object  # builtin_interfaces/Time (scan.header.stamp) — P-7 칼만이 사용


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

        gp = self.get_parameter
        output_topic = gp('output_topic').value
        lidar_topic = gp('lidar_topic').value
        robot_det_topic = gp('robot_detections_topic').value
        cctv_det_topic = gp('cctv_detections_topic').value
        cam_info_topic = gp('camera_info_topic').value
        map_topic = gp('map_topic').value
        cluster_markers_topic = gp('cluster_markers_topic').value

        self._lidar_frame = gp('lidar_frame').value
        self._cluster_distance = float(gp('cluster_distance').value)
        self._cluster_min_points = int(gp('cluster_min_points').value)
        self._cluster_max_radius = float(gp('cluster_max_radius').value)
        self._static_filter_enabled = bool(gp('static_filter_enabled').value)
        self._static_filter_radius = float(gp('static_filter_radius').value)
        self._static_filter_occupied_threshold = int(gp('static_filter_occupied_threshold').value)
        self._static_filter_unknown_as_static = bool(gp('static_filter_unknown_as_static').value)

        # --- QoS ---
        # 출력: 트랙은 떨어지면 안 되니 RELIABLE, 최신 상태만 쓰니 VOLATILE
        track_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # YOLO 디텍션 / camera_info: 퍼블리셔(yolo_detector 등) 기본값(RELIABLE)과 맞춤
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # LiDAR: 센서 데이터 = BEST_EFFORT. 퍼블리셔와 안 맞추면 연결 자체가 안 붙음.
        lidar_qos = qos_profile_sensor_data
        # /map: slam_toolbox가 TRANSIENT_LOCAL(latched)로 발행 → 맞춰야 시작 시 즉시 수신.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- 최신 메시지 보관 (P-6/P-7에서 소비) ---
        self._latest_scan = None
        self._latest_map = None
        self._latest_robot_det = None
        self._latest_cctv_det = None
        self._camera_info = None

        # --- P-6 산출물: map 프레임 LiDAR 동적 관측 (P-7 칼만이 소비) ---
        self._lidar_observations = []  # List[LidarObservation]

        # --- 트랙 상태 (P-7 칼만이 채움) ---
        self._tracks = []  # List[amr_msgs/TrackedObject]

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
        self._marker_pub = None  # TODO(P-8): TrackedObject MarkerArray 컴패니언 (cluster 마커와 별개)

        # --- 타이머: 클러스터링 → 트랙 발행 ---
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"fused_tracker 기동 (P-6). 출력={output_topic} (frame={OUTPUT_FRAME}). "
            f"cluster(d={self._cluster_distance}, min={self._cluster_min_points}, "
            f"rmax={self._cluster_max_radius}) static_filter={self._static_filter_enabled}. "
            f"구독: {lidar_topic}, {map_topic}, {robot_det_topic}, {cctv_det_topic}, {cam_info_topic}"
        )

    # ----- 구독 콜백 (보관만) -----
    def _lidar_cb(self, msg: LaserScan):
        self._latest_scan = msg

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _robot_det_cb(self, msg: Detection2DArray):
        self._latest_robot_det = msg  # TODO(P-7): bbox → ray, LiDAR 트랙과 association, 클래스 채움

    def _cctv_det_cb(self, msg: Detection2DArray):
        self._latest_cctv_det = msg  # TODO(P-10): CCTV ground projection → map

    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_info = msg  # K matrix (robot cam 역투영용)

    # ----- 타이머 -----
    def _tick(self):
        self._process_lidar()
        self._publish_tracks()

    # ----- P-6: LiDAR scan → map 프레임 동적 관측 -----
    def _process_lidar(self):
        self._lidar_observations = []
        scan = self._latest_scan
        if scan is None:
            return

        # 1) scan → 센서프레임 직교좌표
        points = scan_to_points(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
        )
        # 2) scan 순서 jump 클러스터링 (센서 프레임)
        clusters = cluster_points(
            points, self._cluster_distance,
            self._cluster_min_points, self._cluster_max_radius,
        )
        if not clusters:
            self._publish_cluster_markers([], [])
            return

        # 3) TF: 센서프레임 → map (scan stamp 기준, 실패 시 최신으로 fallback)
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

        # 4) center를 map으로 변환 + static 필터
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

    def _lookup_tf(self, src_frame: str, stamp):
        """src_frame → map TF. scan stamp로 시도 후 실패하면 최신(Time())으로 재시도."""
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

    # ----- 출력 (P-6 범위 밖: 빈 트랙 그대로. P-8에서 채움) -----
    def _publish_tracks(self):
        out = TrackedObjectArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = OUTPUT_FRAME
        out.tracks = self._tracks  # P-6까지는 항상 빈 리스트
        self._track_pub.publish(out)


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
