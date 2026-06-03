#!/usr/bin/env python3
"""
object_tracker.py — P-3 좌표 변환 + P-4 Kalman Filter 추적
담당: JW (메인) · HU (협업)
패키지: amr_perception

P-3: Detection2DArray → 핀홀 카메라 역투영 → 3D 좌표 (map frame)
P-4: Kalman Filter 추적 → /detected_obstacles 발행
"""

import time
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

    def __init__(self, x: float, y: float, class_name: str, bbox=None):
        self.track_id = KalmanTrack._next_id
        KalmanTrack._next_id += 1
        self.class_name = class_name
        self.bbox = bbox
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

    def update(self, z: np.ndarray, bbox=None):
        """update 단계 — 관측값으로 보정."""
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        if bbox is not None:
            self.bbox = bbox
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
        self.declare_parameter('max_miss_count', 90)
        self.declare_parameter('max_association_dist', 4.0)  # m
        self.declare_parameter('min_iou_threshold', 0.1)
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('markers_topic', '/perception/tracked_markers')
        self.declare_parameter('target_frame', 'odom_filtered')
        self.declare_parameter('camera_frame', 'camera_optical_link')

        self.max_miss = self.get_parameter('max_miss_count').value
        self.max_dist = self.get_parameter('max_association_dist').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_iou = self.get_parameter('min_iou_threshold').value

        # 카메라 내부 파라미터 (K 행렬)
        self.K = None  # /camera/camera_info 수신 후 채워짐
        # 픽셀 ray는 camera optical frame 기준으로 계산한다.

        # 추적 목록
        self.tracks: list[KalmanTrack] = []

        # Foxglove 렌더링 부하를 줄이기 위해 MarkerArray 발행은 10Hz로 제한
        self.last_marker_pub_time = self.get_clock().now()
        self.marker_pub_period_sec = 0.1
        self.debug_pixel_fail_count = 0

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
            self.get_logger().info(
                f'카메라 K 행렬 수신 완료 | frame: {self.camera_frame}')

    # ── 탐지 결과 수신 ─────────────────────────────────────────────────────

    def _on_detections(self, msg: Detection2DArray):
        """Detection2DArray → 3D 좌표 변환 → Kalman Filter 업데이트."""
        if self.K is None:
            return

        # TF 프레임당 한 번만 lookup (캐싱)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01),
            )
        except TransformException:
            return

        # 1. 각 탐지 결과를 3D 좌표로 변환
        measurements = []
        for det in msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y + det.bbox.size_y * 0.5
            bbox = self._bbox_to_xyxy(det.bbox)
            world_pos = self._pixel_to_world_with_tf(u, v, tf)
            if world_pos is None:
                continue
            class_name = ''
            if det.results:
                class_name = det.results[0].hypothesis.class_id
            measurements.append((world_pos[0], world_pos[1], class_name, bbox))

        # 2. Kalman Filter predict
        for track in self.tracks:
            track.predict()

        # 3. 측정값과 트랙 매칭 (bbox IoU 기반 greedy 방식)
        matched_track_ids = set()
        matched_meas_ids = set()
        existing_track_count = len(self.tracks)

        for mi, (mx, my, cls, bbox) in enumerate(measurements):
            best_tid = None
            best_iou = self.min_iou
            best_dist = self.max_dist

            for ti, track in enumerate(self.tracks):
                if ti in matched_track_ids:
                    continue
                if track.class_name and cls and track.class_name != cls:
                    continue

                tx, ty = track.position
                dist = np.hypot(mx - tx, my - ty)
                if dist > self.max_dist:
                    continue

                iou = self._bbox_iou(bbox, track.bbox) if track.bbox is not None else 0.0

                # IoU를 우선 사용하고, IoU가 약할 때는 3D 거리로 fallback 매칭한다.
                if iou >= self.min_iou:
                    if best_tid is None or iou > best_iou:
                        best_iou = iou
                        best_dist = dist
                        best_tid = ti
                elif best_tid is None and dist < best_dist:
                    best_dist = dist
                    best_tid = ti

            if best_tid is not None:
                self.tracks[best_tid].update(np.array([mx, my]), bbox)
                matched_track_ids.add(best_tid)
                matched_meas_ids.add(mi)

        # 4. 매칭 안 된 측정값 → 새 트랙 생성
        for mi, (mx, my, cls, bbox) in enumerate(measurements):
            if mi not in matched_meas_ids:
                self.tracks.append(KalmanTrack(mx, my, cls, bbox))
                if self.tracks[-1].track_id < 10 or self.tracks[-1].track_id % 20 == 0:
                    self.get_logger().info(
                        f'새 트랙 생성: id={self.tracks[-1].track_id} cls={cls}')

        # 5. 매칭 안 된 기존 트랙 → miss_count 증가, 임계 초과 시 삭제
        for ti, track in enumerate(self.tracks[:existing_track_count]):
            if ti not in matched_track_ids:
                track.miss_count += 1
        self.tracks = [t for t in self.tracks if t.miss_count < self.max_miss]

        # 6. 시각화 발행
        self._publish_markers(msg.header)

    def _bbox_to_xyxy(self, bbox):
        """Detection2D bbox를 (x1, y1, x2, y2) 픽셀 좌표로 변환."""
        cx = bbox.center.position.x
        cy = bbox.center.position.y
        half_w = bbox.size_x * 0.5
        half_h = bbox.size_y * 0.5
        return (cx - half_w, cy - half_h, cx + half_w, cy + half_h)

    def _bbox_iou(self, a, b) -> float:
        """두 bbox의 IoU를 계산."""
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b

        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)

        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter

        if union <= 0.0:
            return 0.0
        return inter / union

    def _pixel_fail(self, reason: str):
        """pixel_to_world 실패 원인을 가끔만 로그로 출력."""
        self.debug_pixel_fail_count += 1
        if self.debug_pixel_fail_count % 30 == 0:
            self.get_logger().warn(f'pixel_to_world 실패: {reason}')
        return None

    def _quat_to_rot_matrix(self, q):
        """geometry_msgs Quaternion을 3x3 회전 행렬로 변환."""
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=float)

    # ── P-3: 핀홀 역투영 ───────────────────────────────────────────────────

    def _pixel_to_world_with_tf(self, u: float, v: float, tf) -> tuple | None:
        """TF를 외부에서 받아 역투영 — 프레임당 한 번만 TF lookup."""
        fx = self.K[0, 0]; fy = self.K[1, 1]
        cx = self.K[0, 2]; cy = self.K[1, 2]

        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return None

        ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)
        ray_norm = np.linalg.norm(ray_camera)
        if ray_norm < 1e-6:
            return None
        ray_camera /= ray_norm

        origin = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=float)

        rot = self._quat_to_rot_matrix(tf.transform.rotation)
        ray_world = rot @ ray_camera

        if abs(ray_world[2]) < 1e-6:
            return None

        t = -origin[2] / ray_world[2]
        if t < 0:
            return None

        point = origin + t * ray_world
        return (float(point[0]), float(point[1]))

        def _pixel_to_world(self, u: float, v: float, header) -> tuple | None:
        """
        픽셀 (u, v)을 target_frame 기준 지면(z=0) 좌표로 변환.
        카메라 ray를 TF rotation으로 target_frame에 회전시킨 뒤 지면과 교차시킨다.
        """
        if self.K is None or self.camera_frame is None:
            return self._pixel_fail('K 또는 camera_frame 없음')

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return self._pixel_fail('fx/fy invalid')

        ray_camera = np.array([
            (u - cx) / fx,
            (v - cy) / fy,
            1.0,
        ], dtype=float)

        ray_norm = np.linalg.norm(ray_camera)
        if ray_norm < 1e-6:
            return self._pixel_fail('ray norm invalid')
        ray_camera = ray_camera / ray_norm

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01),
            )
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup 실패: {self.target_frame} <- {self.camera_frame}: {e}')
            return self._pixel_fail('TF lookup 실패')

        origin = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=float)

        rot = self._quat_to_rot_matrix(tf.transform.rotation)
        ray_world = rot @ ray_camera

        if abs(ray_world[2]) < 1e-6:
            return self._pixel_fail(f'ray_world z 거의 0: {ray_world}')

        t = -origin[2] / ray_world[2]
        if t < 0:
            return self._pixel_fail(f'지면 교차가 카메라 뒤쪽: t={t:.3f}, origin={origin}, ray={ray_world}')

        point = origin + t * ray_world
        return (float(point[0]), float(point[1]))

    # ── 시각화 ─────────────────────────────────────────────────────────────

    def _publish_markers(self, header):
        """추적 결과를 MarkerArray로 발행."""
        now = self.get_clock().now()
        elapsed_ns = (now - self.last_marker_pub_time).nanoseconds
        if elapsed_ns < int(self.marker_pub_period_sec * 1e9):
            return
        self.last_marker_pub_time = now

        array = MarkerArray()

        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        array.markers.append(delete_marker)

        for track in self.tracks:
            m = Marker()
            m.header.frame_id = self.target_frame
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
