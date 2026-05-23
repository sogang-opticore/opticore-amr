#!/usr/bin/env python3
"""
object_tracker.py — P-3 좌표 변환 + P-4 Kalman Filter 추적
담당: JW (메인) · HU (협업)
패키지: amr_perception

P-3: Detection2DArray → 핀홀 카메라 역투영 → 3D 좌표 (map frame)
P-4: Kalman Filter 추적 → /detected_obstacles 발행
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import tf2_ros
from tf2_ros import TransformException

import time


# ── Kalman Filter 추적 객체 ──────────────────────────────────────────────────

class KalmanTrack:
    """
    단일 객체 추적용 Kalman Filter.
    상태벡터: [x, y, vx, vy] — 등속 모델 (person 0.8m/s 기준)
    """

    _next_id = 0

    def __init__(self, x: float, y: float, class_name: str):
        self.track_id = KalmanTrack._next_id
        KalmanTrack._next_id += 1
        self.class_name = class_name
        self.miss_count = 0  # 탐지 못한 프레임 수

        dt = 1.0 / 15.0  # 카메라 15Hz 기준

        # 상태 전이 행렬 (등속 모델)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # 관측 행렬 (x, y만 관측)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # 프로세스 노이즈
        self.Q = np.eye(4) * 0.1

        # 관측 노이즈
        self.R = np.eye(2) * 0.5

        # 초기 상태
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)

        # 초기 공분산
        self.P = np.eye(4) * 1.0

    def predict(self):
        """predict 단계 — 다음 위치 예측."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        """update 단계 — 관측값으로 보정."""
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.miss_count = 0

    @property
    def position(self):
        return self.x[0], self.x[1]

    @property
    def velocity(self):
        return self.x[2], self.x[3]


# ── ObjectTracker 노드 ───────────────────────────────────────────────────────

class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('object_tracker')

        # 파라미터
        self.declare_parameter('max_miss_count', 10)
        self.declare_parameter('max_association_dist', 2.0)  # m
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('markers_topic', '/perception/tracked_markers')

        self.max_miss = self.get_parameter('max_miss_count').value
        self.max_dist = self.get_parameter('max_association_dist').value

        # 카메라 내부 파라미터 (K 행렬)
        self.K = None  # /camera/camera_info 수신 후 채워짐
        self.camera_frame = None

        # 추적 목록
        self.tracks: list[KalmanTrack] = []

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 구독
        self.create_subscription(
            CameraInfo, '/camera_info', self._on_camera_info, 10)
        self.create_subscription(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            self._on_detections, qos)

        # 발행 — 추적 시각화 (MarkerArray)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('markers_topic').value,
            qos)

        self.get_logger().info('ObjectTracker 노드 시작')

    # ── 카메라 정보 수신 ────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        """K 행렬 한 번만 저장."""
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.camera_frame = 'camera_link'
            self.get_logger().info(
                f'카메라 K 행렬 수신 완료 | frame: {self.camera_frame}')

    # ── 탐지 결과 수신 ─────────────────────────────────────────────────────

    def _on_detections(self, msg: Detection2DArray):
        """Detection2DArray → 3D 좌표 변환 → Kalman Filter 업데이트."""
        if self.K is None:
            return

        # 1. 각 탐지 결과를 3D 좌표로 변환
        measurements = []
        for det in msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y

            # 핀홀 역투영 (지면 z=0 가정)
            world_pos = self._pixel_to_world(u, v, msg.header)
            if world_pos is None:
                continue

            class_name = ''
            if det.results:
                class_name = det.results[0].hypothesis.class_id

            measurements.append((world_pos[0], world_pos[1], class_name))

        # 2. Kalman Filter predict
        for track in self.tracks:
            track.predict()

        # 3. 측정값과 트랙 매칭 (greedy 방식)
        matched_track_ids = set()
        matched_meas_ids = set()

        for mi, (mx, my, cls) in enumerate(measurements):
            best_dist = self.max_dist
            best_tid = None
            for ti, track in enumerate(self.tracks):
                if ti in matched_track_ids:
                    continue
                tx, ty = track.position
                dist = np.hypot(mx - tx, my - ty)
                if dist < best_dist:
                    best_dist = dist
                    best_tid = ti

            if best_tid is not None:
                self.tracks[best_tid].update(np.array([mx, my]))
                matched_track_ids.add(best_tid)
                matched_meas_ids.add(mi)

        # 4. 매칭 안 된 측정값 → 새 트랙 생성
        for mi, (mx, my, cls) in enumerate(measurements):
            if mi not in matched_meas_ids:
                self.tracks.append(KalmanTrack(mx, my, cls))
                self.get_logger().info(
                    f'새 트랙 생성: id={self.tracks[-1].track_id} cls={cls}')

        # 5. 매칭 안 된 트랙 → miss_count 증가, 임계 초과 시 삭제
        for ti, track in enumerate(self.tracks):
            if ti not in matched_track_ids:
                track.miss_count += 1
        self.tracks = [t for t in self.tracks if t.miss_count < self.max_miss]

        # 6. 시각화 발행
        self._publish_markers(msg.header)

    # ── P-3: 핀홀 역투영 ───────────────────────────────────────────────────

    def _pixel_to_world(self, u: float, v: float, header) -> tuple | None:
        """
        픽셀 (u, v) → map frame (x, y) 변환.
        지면(z=0) 가정으로 역투영.
        """
        if self.K is None:
            return None

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # 카메라 좌표계에서 방향벡터 (정규화)
        x_c = (u - cx) / fx
        y_c = (v - cy) / fy
        z_c = 1.0

        # TF로 카메라 → map 변환
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            # map 없으면 odom 시도
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom_filtered',
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except TransformException as e:
                self.get_logger().warn(f'TF lookup 실패: {e}')
                return None

        # 카메라 위치 (map frame)
        cam_x = tf.transform.translation.x
        cam_y = tf.transform.translation.y
        cam_z = tf.transform.translation.z

        # 간단한 지면 교차점 계산
        # 카메라가 지면 위 cam_z 높이에 있고, 광선이 아래를 향한다고 가정
        # t = cam_z / y_c (y축이 아래 방향)
        if abs(y_c) < 1e-6:
            return None

        t = cam_z / y_c
        if t < 0:
            return None

        world_x = cam_x + x_c * t
        world_y = cam_y + t  # 전방 방향

        return (world_x, world_y)

    # ── 시각화 ─────────────────────────────────────────────────────────────

    def _publish_markers(self, header):
        """추적 결과를 MarkerArray로 발행."""
        array = MarkerArray()

        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        array.markers.append(delete_marker)

        for track in self.tracks:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = header.stamp
            m.ns = 'tracked_objects'
            m.id = track.track_id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = track.position[0]
            m.pose.position.y = track.position[1]
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 1.0
            m.color.a = 0.7
            if track.class_name == 'person':
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
            else:
                m.color.r = 1.0
                m.color.g = 0.5
                m.color.b = 0.0
            array.markers.append(m)

        self.marker_pub.publish(array)


# ── main ────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
