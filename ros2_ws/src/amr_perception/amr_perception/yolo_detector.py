# TODO: YOLOv8 detector node placeholder

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from cv_bridge import CvBridge
import cv2
import numpy as np
import time

from ultralytics import YOLO


# COCO index 기준. forklift는 COCO에 없으므로 truck(7)으로 대체 확정
# 시뮬레이션 환경 한정 (커스텀 모델 미사용 확정 — 팀 합의 2026-05-21)
TARGET_CLASS_IDS = {
    0: 'person',
    7: 'forklift',  # truck(class_id=7)으로 대체 확정
}


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # 파라미터 선언
        self.declare_parameter('model_path', '/workspace/models/yolov8n.pt')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('debug_image_topic', '/perception/debug_image')

        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value

        # YOLOv8 모델 로드
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info('YOLO model loaded successfully')

        self.bridge = CvBridge()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 구독
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos
        )

        # 발행
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            detections_topic,
            qos
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            debug_image_topic,
            qos
        )

        # FPS 측정용
        self._last_time = time.time()
        self._frame_count = 0

        self.get_logger().info(
            f'YoloDetector started | '
            f'image_topic={image_topic} | '
            f'detections_topic={detections_topic}'
        )

    def image_callback(self, msg: Image):
        t_start = time.time()

        # sensor_msgs/Image → OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        # YOLOv8 추론
        results = self.model(
            cv_image,
            conf=self.conf_threshold,
            verbose=False
        )

        # Detection2DArray 구성
        det_array = Detection2DArray()
        det_array.header = msg.header

        debug_image = cv_image.copy()

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                if class_id not in TARGET_CLASS_IDS:
                    continue

                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = TARGET_CLASS_IDS[class_id]
                hyp.hypothesis.score = conf
                det.results.append(hyp)

                det_array.detections.append(det)

                # 디버그 이미지 bbox 그리기
                label = f"{TARGET_CLASS_IDS[class_id]} {conf:.2f}"
                cv2.rectangle(debug_image,
                              (int(x1), int(y1)), (int(x2), int(y2)),
                              (0, 255, 0), 2)
                cv2.putText(debug_image, label,
                            (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.detections_pub.publish(det_array)

        # 디버그 이미지 발행
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

        # FPS 로깅 (10프레임마다)
        self._frame_count += 1
        if self._frame_count % 10 == 0:
            elapsed = time.time() - self._last_time
            fps = 10.0 / elapsed
            inference_ms = (time.time() - t_start) * 1000
            self.get_logger().info(
                f'FPS: {fps:.1f} | inference: {inference_ms:.1f}ms | '
                f'detections: {len(det_array.detections)}'
            )
            self._last_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()