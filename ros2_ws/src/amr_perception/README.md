# amr_perception

인지 패키지 (ament_python). LiDAR 클러스터링 기반 트래커, CCTV no-go 오버레이, YOLOv8 검출기를 담는다.

> 2026-10-13 갱신: 실제 노드 구성 기준으로 재작성.
> **현재 주행(A\*/DWA)은 이 패키지의 출력을 구독하지 않는다.** DWA는 LiDAR로 자체 클러스터링·트래킹을 하고 `dynamic_obstacle_layer`를 직접 발행한다.
> 서버 부하·구조 처방은 [`docs/server-run-prescription.md`](../../../docs/server-run-prescription.md), [`docs/multi-robot-architecture.md`](../../../docs/multi-robot-architecture.md) 참고.

## 노드 (`setup.py` console_scripts)

| 실행 파일 | 입력 → 출력 | 기동 |
|---|---|---|
| `fused_tracker` | LiDAR(+로봇 카메라 검출, CCTV 검출은 보관만) → `/perception/tracked_objects` (`amr_msgs/TrackedObjectArray`, map 프레임) | `multi_robot.launch.py`가 +50 s에 1개 기동 (**amr1 LiDAR만 입력**). 단독: `fused_tracker.launch.py` |
| `yolo_detector` | `/camera` → `/perception/detections` (`vision_msgs/Detection2DArray`), `/perception/debug_image` | `perception.launch.py` (네임스페이스·remap 없음 → 멀티로봇 토픽 `/amrN/camera`와 연결되지 않음). `multi_robot.launch.py`는 YOLO를 띄우지 않는다 |
| `cctv_detector` | `/cctv/<cam>/image` → `/cctv/detections` (배경차분, YOLO 미사용) | `cctv.launch.py` |
| `cctv_nogo_overlay` | `/cctv/detections` + map → `layer_topic` (OccupancyGrid no-go, A*가 합성) | `cctv.launch.py` |
| `object_tracker` | (구버전 카메라 트래커, 현재 어떤 launch에서도 쓰지 않음) | — |

보조 모듈: `lidar_clustering.py`(P-6), `multisource_kalman.py`(P-7), `cctv_projection.py`(픽셀→바닥 투영, 단위테스트 `test/test_cctv_projection.py`).

## 설정

| 파일 | 요점 |
|---|---|
| `config/fused_tracker_params.yaml` | 기본 `tracking_frame: odom_filtered`(단일로봇). 멀티로봇은 launch에서 `map` + amr1 토픽으로 덮어씀 |
| `config/yolo_params.yaml` | `yolov8n.pt`(`/workspace/models/yolov8n.pt`), `device: cuda`. `inference_target_fps`는 코드에서 선언되지 않아 효과 없음 → 모든 프레임 추론 |
| `config/cctv_nogo_params.yaml` | 처리 카메라 `corridor_1n` 1대, `nogo_radius_m: 0.9`(1.5 m 복도 기준 값), 로봇 자기검출 제외는 `/amr1/amcl_pose`만 |

## 실행 예

```bash
# CCTV no-go 오버레이 (amr1에 연결). 멀티로봇 기본값(/map, /dynamic_obstacle_layer)은 존재하지 않으므로 반드시 override
ros2 launch amr_perception cctv.launch.py map_topic:=/amr1/map layer_topic:=/amr1/dynamic_obstacle_layer
```

## 빌드 주의

- `--symlink-install`과 최신 setuptools 조합에서 이 패키지 빌드가 실패한 기록이 있다(`scripts/clean_build.sh`가 기본으로 skip). 일반 `colcon build --packages-select amr_perception`은 PASS 기록이 있다([`docs/cctv/PROGRESS.md`](../../../docs/cctv/PROGRESS.md)).
- `multi_robot.launch.py`는 이 패키지의 share 경로를 요구하므로, 이 패키지가 빌드·설치되지 않으면 멀티로봇 launch 자체가 실패한다.
- `cv_bridge`와 NumPy 2.x 충돌 기록이 있다. CCTV 노드는 `np.frombuffer`로 디코드해 회피했지만 `yolo_detector`는 `cv_bridge`를 쓴다.
- CCTV 설계·검증 기록: [`docs/cctv/`](../../../docs/cctv/)
