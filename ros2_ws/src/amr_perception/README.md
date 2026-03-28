# amr_perception

YOLOv8 기반 인식 패키지

## 설명
- YOLOv8 객체 인식 ROS2 노드
- 장애물 감지 및 분류
- 포인트 클라우드 처리
- 센서 퓨전 (Camera + LiDAR)

## 구조 (예정)
```
amr_perception/
├── amr_perception/
│   ├── __init__.py
│   ├── yolo_node.py
│   └── obstacle_detector.py
├── launch/
├── config/
├── models/
└── package.xml
```
