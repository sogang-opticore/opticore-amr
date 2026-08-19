"""P-9 — CCTV person 검출기 (BG-subtraction).

EXPLORE 락(PLAN §1.3): YOLOv8n 은 25° top-down·23px person 을 "bird" 로 오분류 → BG-sub 사용.
빈 복도 기준배경(warmup 평균, 고정) 대비 grayscale abs-diff → threshold → morphology → contour.
출력: /cctv/detections (vision_msgs/Detection2DArray, header.frame_id=카메라명; 신규 메시지 X).

배경 취득: 노드 기동 후 카메라별 warmup_frames 평균 → 고정.
  - dynamic 쇼케이스: 사람 입장 전이라 자연 빈배경.
  - static 게이트: 게이트 스크립트가 warmup 동안 person 텔레포트 아웃 후 (14,18) 복귀.
검출은 cv_bridge 없이 np.frombuffer 디코드(NumPy 2.x 충돌 회피).
"""
from __future__ import annotations

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


def _decode_rgb(msg: Image) -> np.ndarray:
    """sensor_msgs/Image(rgb8|bgr8) → HxWx3 uint8 (cv_bridge 미사용)."""
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return arr.reshape(msg.height, msg.width, 3)


class CctvDetector(Node):
    def __init__(self):
        super().__init__('cctv_detector')

        self.declare_parameter('camera_names', ['corridor_1n', 'corridor_1s'])
        self.declare_parameter('image_topic_template', '/cctv/{cam}/image')
        self.declare_parameter('detections_topic', '/cctv/detections')
        self.declare_parameter('detector_type', 'bgsub')          # bgsub|yolo(미지원→bgsub)
        self.declare_parameter('warmup_frames', 15)
        self.declare_parameter('diff_threshold', 30)
        self.declare_parameter('min_area', 60.0)
        self.declare_parameter('max_area', 50000.0)
        self.declare_parameter('morph_open', 3)
        self.declare_parameter('morph_dilate', 5)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_topic_template', '/cctv/{cam}/debug')

        self.cams = list(self.get_parameter('camera_names').value)
        img_tpl = self.get_parameter('image_topic_template').value
        det_topic = self.get_parameter('detections_topic').value
        self.det_type = self.get_parameter('detector_type').value
        self.warmup = int(self.get_parameter('warmup_frames').value)
        self.thr = int(self.get_parameter('diff_threshold').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area = float(self.get_parameter('max_area').value)
        self.k_open = int(self.get_parameter('morph_open').value)
        self.k_dil = int(self.get_parameter('morph_dilate').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        dbg_tpl = self.get_parameter('debug_topic_template').value

        if self.det_type != 'bgsub':
            self.get_logger().warn(
                f"detector_type='{self.det_type}' 미지원(EXPLORE 락=bgsub) → bgsub 사용")

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.det_pub = self.create_publisher(Detection2DArray, det_topic, qos)

        # per-camera 상태
        self._bg = {c: None for c in self.cams}        # float32 grayscale 배경
        self._acc = {c: None for c in self.cams}        # warmup 누적
        self._n = {c: 0 for c in self.cams}             # warmup 프레임 수
        self._dbg_pub = {}
        for c in self.cams:
            self.create_subscription(
                Image, img_tpl.format(cam=c),
                lambda m, c=c: self._on_image(m, c), qos)
            if self.publish_debug:
                self._dbg_pub[c] = self.create_publisher(
                    Image, dbg_tpl.format(cam=c), qos)
        self.get_logger().info(
            f'CctvDetector 시작 | cams={self.cams} warmup={self.warmup} thr={self.thr} '
            f'min_area={self.min_area}')

    def _on_image(self, msg: Image, cam: str):
        try:
            rgb = _decode_rgb(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'{cam} 디코드 실패: {e}', throttle_duration_sec=5.0)
            return
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)

        # warmup: 배경 누적
        if self._bg[cam] is None:
            self._acc[cam] = gray.copy() if self._acc[cam] is None else self._acc[cam] + gray
            self._n[cam] += 1
            if self._n[cam] >= self.warmup:
                self._bg[cam] = self._acc[cam] / float(self._n[cam])
                self.get_logger().info(f'{cam} 배경 고정 ({self._n[cam]} 프레임 평균)')
            return

        boxes = self._detect(gray, self._bg[cam])

        out = Detection2DArray()
        out.header = msg.header
        out.header.frame_id = cam  # ← 어느 카메라인지 overlay 가 식별
        for (x, y, w, h, area) in boxes:
            d = Detection2D()
            d.header = out.header
            d.bbox.center.position.x = float(x + w / 2.0)
            d.bbox.center.position.y = float(y + h / 2.0)
            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = float(min(1.0, area / 500.0))
            d.results.append(hyp)
            out.detections.append(d)
        self.det_pub.publish(out)

        if self.publish_debug and cam in self._dbg_pub:
            self._publish_debug(rgb, boxes, msg, cam)

    def _detect(self, gray: np.ndarray, bg: np.ndarray):
        diff = cv2.absdiff(gray, bg).astype(np.uint8)
        _, mask = cv2.threshold(diff, self.thr, 255, cv2.THRESH_BINARY)
        if self.k_open > 0:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, np.ones((self.k_open, self.k_open), np.uint8))
        if self.k_dil > 0:
            mask = cv2.dilate(mask, np.ones((self.k_dil, self.k_dil), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            out.append((x, y, w, h, area))
        out.sort(key=lambda b: -b[4])  # 큰 것 우선
        return out

    def _publish_debug(self, rgb, boxes, msg, cam):
        img = rgb.copy()
        for (x, y, w, h, area) in boxes:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img, (int(x + w / 2), int(y + h)), 3, (0, 0, 255), -1)  # foot
        out = Image()
        out.header = msg.header
        out.header.frame_id = cam
        out.height, out.width = img.shape[0], img.shape[1]
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = img.shape[1] * 3
        out.data = img.tobytes()
        self._dbg_pub[cam].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CctvDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
