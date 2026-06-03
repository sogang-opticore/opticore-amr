#!/usr/bin/env python3
"""
fused_tracker — LiDAR + 로봇 카메라 YOLO + CCTV YOLO 융합 semantic 트래커 (P-5 스캐폴드)

이 노드는 P-5 단계에서 '인터페이스 골격'만 세운다. 융합 로직은 비어 있고
(콜백은 최신 메시지만 보관), 10Hz 타이머로 빈 TrackedObjectArray를 map 프레임으로 발행한다.
빈 노드라도 모든 토픽이 떠 있어야 다운스트림(P-6/7/8/10, F-3)이 미리 연결/개발 가능하다.

구현 단계:
  P-6   LiDAR 자체 클러스터링 → _lidar_cb 채움
  P-7   multi-source 칼만 + association + 클래스 → 융합 코어
  P-8   map 프레임 출력 표준화 → _publish_tracks 채움
  P-9/10  CCTV 디텍션 입력 → _cctv_det_cb 채움

TODO(나중, P-5 범위 밖): /perception/tracked_objects/markers (visualization_msgs/MarkerArray)
  컴패니언 발행 → Foxglove 3D 박스 시각화. object_tracker.py의 마커 생성 패턴 참고.
  여기서는 self._marker_pub 자리만 비워둔다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray

from tf2_ros import Buffer, TransformListener

from amr_msgs.msg import TrackedObject, TrackedObjectArray  # noqa: F401 (TrackedObject는 P-7에서 사용)


OUTPUT_FRAME = 'map'
PUBLISH_RATE_HZ = 10.0


class FusedTracker(Node):
    def __init__(self):
        super().__init__('fused_tracker')

        # --- 파라미터 (launch에서 주입; 토픽 이름은 노드 코드 수정 없이 바꿀 수 있게) ---
        self.declare_parameter('output_topic', '/perception/tracked_objects')
        self.declare_parameter('lidar_topic', '/lidar')
        self.declare_parameter('robot_detections_topic', '/perception/detections')
        self.declare_parameter('cctv_detections_topic', '/cctv/detections')  # [미확정] P-9 확정 시 갱신
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        output_topic = self.get_parameter('output_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        robot_det_topic = self.get_parameter('robot_detections_topic').value
        cctv_det_topic = self.get_parameter('cctv_detections_topic').value
        cam_info_topic = self.get_parameter('camera_info_topic').value

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

        # --- 최신 메시지 보관 (P-6/P-7에서 소비) ---
        self._latest_scan = None
        self._latest_robot_det = None
        self._latest_cctv_det = None
        self._camera_info = None

        # --- 트랙 상태 (P-7 칼만이 채움) ---
        self._tracks = []  # List[amr_msgs/TrackedObject]

        # --- TF (map ← odom_filtered ← ... ← {lidar_link, camera_optical_link}) ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- 구독 (빈 노드여도 토픽은 떠 있어야 함) ---
        self.create_subscription(LaserScan, lidar_topic, self._lidar_cb, lidar_qos)
        self.create_subscription(Detection2DArray, robot_det_topic, self._robot_det_cb, reliable_qos)
        self.create_subscription(Detection2DArray, cctv_det_topic, self._cctv_det_cb, reliable_qos)
        self.create_subscription(CameraInfo, cam_info_topic, self._camera_info_cb, reliable_qos)

        # --- 발행 ---
        self._track_pub = self.create_publisher(TrackedObjectArray, output_topic, track_qos)
        self._marker_pub = None  # TODO(나중): MarkerArray 컴패니언

        # --- 발행 타이머 (빈 메시지라도 토픽 살아있게 → Foxglove echo 확인용) ---
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_tracks)

        self.get_logger().info(
            f"fused_tracker 스캐폴드 기동. 출력={output_topic} (frame={OUTPUT_FRAME}). "
            f"구독: {lidar_topic}, {robot_det_topic}, {cctv_det_topic}, {cam_info_topic}"
        )

    # ----- 구독 콜백 (P-5: 보관만. 융합 로직은 P-6/P-7/P-10) -----
    def _lidar_cb(self, msg: LaserScan):
        self._latest_scan = msg  # TODO(P-6): scan → cluster → static 필터 → map 변환

    def _robot_det_cb(self, msg: Detection2DArray):
        self._latest_robot_det = msg  # TODO(P-7): bbox → ray, LiDAR 트랙과 association, 클래스 채움

    def _cctv_det_cb(self, msg: Detection2DArray):
        self._latest_cctv_det = msg  # TODO(P-10): CCTV ground projection → map

    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_info = msg  # K matrix (robot cam 역투영용)

    # ----- 출력 -----
    def _publish_tracks(self):
        out = TrackedObjectArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = OUTPUT_FRAME
        out.tracks = self._tracks  # P-5에서는 항상 빈 리스트
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
