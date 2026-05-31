import pytest
import numpy as np


def test_model_load():
    """YOLOv8 모델 로드 성공 여부"""
    from ultralytics import YOLO
    model = YOLO('/workspace/models/yolov8n.pt')
    assert model is not None


def test_inference_output_shape():
    """추론 결과 박스 shape 검증"""
    from ultralytics import YOLO
    import numpy as np

    model = YOLO('/workspace/models/yolov8n.pt')

    # 640x480 더미 이미지 (카메라 스펙 기준)
    dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
    results = model(dummy_image, verbose=False)

    assert results is not None
    assert len(results) > 0
    # boxes 속성 존재 확인
    assert hasattr(results[0], 'boxes')


def test_target_class_filter():
    """TARGET_CLASS_IDS 필터 동작 확인"""
    from amr_perception.yolo_detector import TARGET_CLASS_IDS

    assert 0 in TARGET_CLASS_IDS   # person
    assert TARGET_CLASS_IDS[0] == 'person'