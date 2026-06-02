# LiDAR-Camera Fusion 알고리즘 설계

> 브랜치 분기 전 사전 작업 #2
> 작성: HU
> 목적: YOLO bbox + 2D LiDAR scan 융합 → scaled object/occlusion에 강건한 3D 객체 위치 추정
> 의존성: JW의 `object_tracker.py` 머지 후 그 위에서 확장 (별도 노드)

---

## 1. 동기 — 현재 방식의 한계

### 1.1 현재 (object_tracker.py)

**방식**: Ground plane assumption (z=0 가정)

```python
# bbox 하단 중앙 픽셀 → 카메라 ray
ray_camera = [(u-cx)/fx, (v-cy)/fy, 1.0]
# TF로 world frame 회전
ray_world = rot @ ray_camera
# z=0 평면과 교차
t = -origin[2] / ray_world[2]
point = origin + t * ray_world
```

객체 크기 가정 없음 (size prior 아님). bbox 하단 = 객체 발 위치라는 가정만 사용.

### 1.2 Ground plane 가정의 failure modes

dynamic_obstacles_v2 작업에서 실측된 / 예측 가능한 실패 케이스:

| Failure mode | 원인 | 영향 |
|---|---|---|
| 떠있는 객체 | forklift v1 z=0.5m | bbox 하단 ≠ 바퀴 → 거리 오추정 (실측됨) |
| 발/바닥 접점 가림 | 박스 뒤 사람 상반신 | bbox 하단이 박스 윗면 → 위치 오차 큼 |
| Bbox 노이즈 | YOLO 추론 지터 | 픽셀 1~2줄 차이가 멀리 있을수록 큰 거리 오차로 amplify |
| 카메라 pitch 진동 | IMU 노이즈, EKF 지터 | 평면 교점 떨림 |
| 비평면 지면 | (시뮬은 OK, 실물 환경 대비) | 사용 불가 |

**LiDAR fusion은 위 가정 자체가 필요 없음.** 거리를 직접 측정.

---

## 2. 알고리즘 — 4단계

### 2.1 입력
- YOLO bbox: `vision_msgs/Detection2DArray` (픽셀 좌표 + 클래스)
- LiDAR: `sensor_msgs/LaserScan` (2D 거리 측정, 720 samples, 360°, max 25m)
- Camera info: `K matrix` (fx≈fy≈381.4, cx=320, cy=240)
- TF: `camera_optical_link ↔ lidar_link ↔ base_footprint ↔ target_frame`

### 2.2 알고리즘 흐름

```
[YOLO bbox] (픽셀 u, v)
    │
    │ ① bbox → frustum (3D cone in camera frame)
    ↓
[Camera frustum]
    │
    │ ② LiDAR scan → 2D points in lidar frame → camera frame 변환
    ↓
[LiDAR points in camera frame]
    │
    │ ③ frustum 안에 들어가는 점들만 필터링
    ↓
[Filtered LiDAR points]
    │
    │ ④ 클러스터링 (DBSCAN or 거리 threshold)
    │   대표 거리/위치 추출 (median or centroid)
    ↓
[3D object position (x, y) in target_frame]
```

### 2.3 각 단계 상세

#### ① bbox → frustum

bbox 4 모서리 픽셀을 카메라 ray 4개로 변환:

```python
def bbox_to_frustum_rays(bbox, K):
    fx, fy = K[0,0], K[1,1]
    cx_K, cy_K = K[0,2], K[1,2]

    corners_pixel = [
        (bbox.x1, bbox.y1),  # top-left
        (bbox.x2, bbox.y1),  # top-right
        (bbox.x2, bbox.y2),  # bottom-right
        (bbox.x1, bbox.y2),  # bottom-left
    ]

    rays = []
    for u, v in corners_pixel:
        ray = np.array([(u - cx_K) / fx, (v - cy_K) / fy, 1.0])
        ray /= np.linalg.norm(ray)
        rays.append(ray)
    return rays  # 4개의 unit vector (camera optical frame)
```

#### ② LiDAR → camera frame 변환

2D LaserScan을 (x, y, z=0) 3D 점으로 확장 후 TF 적용:

```python
def lidar_to_camera_frame(scan, tf_lidar_to_camera):
    # 1) LaserScan → 2D points in lidar frame
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)
    valid = (ranges > scan.range_min) & (ranges < scan.range_max)

    xs = ranges[valid] * np.cos(angles[valid])
    ys = ranges[valid] * np.sin(angles[valid])
    points_lidar = np.stack([xs, ys, np.zeros_like(xs), np.ones_like(xs)], axis=1)  # homogeneous

    # 2) lidar_frame → camera_optical_frame
    T = tf_to_matrix(tf_lidar_to_camera)  # 4x4
    points_camera = (T @ points_lidar.T).T[:, :3]
    return points_camera  # (N, 3) in camera optical frame
```

#### ③ Frustum 필터링

camera optical frame에서 각 점이 4개 frustum ray로 둘러싸인 영역 안에 있는지 검사. 단순한 방법:

```python
def point_in_frustum(point, frustum_rays):
    # 카메라 optical frame: z=forward
    if point[2] <= 0:  # 카메라 뒤
        return False

    # 점을 image plane(z=1)에 투영
    u_normalized = point[0] / point[2]
    v_normalized = point[1] / point[2]

    # 각 frustum ray도 z=1 평면에 투영해 2D bbox 형성
    ray_uvs = [(r[0]/r[2], r[1]/r[2]) for r in frustum_rays]
    u_min = min(r[0] for r in ray_uvs)
    u_max = max(r[0] for r in ray_uvs)
    v_min = min(r[1] for r in ray_uvs)
    v_max = max(r[1] for r in ray_uvs)

    return u_min <= u_normalized <= u_max and v_min <= v_normalized <= v_max
```

#### ④ 클러스터링 + 거리 추출

frustum 안에 점이 여러 개면 가장 가까운 점 그룹만 사용 (앞 객체가 뒤 객체 가릴 때).

```python
def cluster_and_extract(points_in_frustum, eps=0.3):
    if len(points_in_frustum) == 0:
        return None

    # 거리 기준 정렬, 가까운 점부터 클러스터
    distances = np.linalg.norm(points_in_frustum, axis=1)
    sorted_idx = np.argsort(distances)
    sorted_pts = points_in_frustum[sorted_idx]
    sorted_dist = distances[sorted_idx]

    # 가장 가까운 점부터 eps 내 점들 모음 (단일 클러스터)
    cluster = [sorted_pts[0]]
    for i in range(1, len(sorted_pts)):
        if sorted_dist[i] - sorted_dist[0] < eps:
            cluster.append(sorted_pts[i])
        else:
            break  # 거리 점프 → 다른 객체

    # 대표 위치: median (outlier-robust)
    cluster = np.array(cluster)
    centroid = np.median(cluster, axis=0)
    return centroid
```

**참고**: 더 정교한 방법으로 DBSCAN(sklearn) 사용 가능. 일단 단순 거리 클러스터링으로 시작.

### 2.4 출력

target_frame 기준 (x, y) 좌표. JW의 object_tracker가 받는 `(world_pos[0], world_pos[1])`와 동일한 형식.

---

## 3. 신규 노드 — `lidar_fusion_tracker.py`

### 3.1 위치
```
ros2_ws/src/amr_perception/amr_perception/
├── yolo_detector.py          (기존)
├── object_tracker.py         (기존, ground plane 방식 — baseline)
└── lidar_fusion_tracker.py   (신규, LiDAR fusion 방식)
```

### 3.2 설계 원칙

**`object_tracker.py`를 거의 그대로 복제 + `_pixel_to_world` 교체.**

- KF 추적 로직 (`KalmanTrack`): 동일 사용 가능 → 별도 모듈로 추출 권장 (`kalman_track.py`)
- IoU 매칭, miss_count: 동일
- 발행 토픽만 다르게 (baseline과 동시 가동을 위해)

### 3.3 토픽 인터페이스

**구독**:
- `/perception/detections` (YOLO 결과)
- `/lidar` (LaserScan)
- `/camera_info`

**발행**:
- `/perception/tracked_markers_lidar_fusion` (MarkerArray, Foxglove 시각화)
- `/perception/objects_3d` (커스텀 메시지 — 자세한 건 03_master_interface_spec.md 참조)

**파라미터**:
- `target_frame` (default: `odom_filtered` — JW 코드와 동일)
- `lidar_frame` (default: `lidar_link`)
- `camera_frame` (default: `camera_optical_link`)
- `frustum_eps` (default: 0.3m — 클러스터링 거리 임계)
- `max_lidar_range` (default: 25.0m)

### 3.4 시간 동기화 처리

LaserScan과 Detection2DArray의 timestamp가 다름 (LiDAR 10Hz, Camera 15Hz).

**전략**: TF buffer 활용. detection 시점의 LaserScan을 찾기:
```python
# detection의 timestamp 기준으로 가장 가까운 LaserScan을 보관 buffer에서 가져옴
# 또는 message_filters.ApproximateTimeSynchronizer 사용
```

복잡도 vs 정확성 tradeoff:
- 단순: 가장 최근 LaserScan 사용 (대략 ±50ms 오차, AMR 0.6m/s × 50ms = 3cm 위치 오차)
- 정확: ApproximateTimeSynchronizer (구현 복잡)

**1차안**: 단순 (최근 scan). 충돌 회피용으론 3cm 오차 무시 가능. 정량 평가에서 문제되면 업그레이드.

---

## 4. Baseline 보존 전략

**JW의 `object_tracker.py`는 절대 수정 안 함.**

이유:
- Ablation 실험: "Ground plane (baseline) vs LiDAR fusion (ours)" 정량 비교
- 머지 conflict 회피
- 안전망 (LiDAR fusion이 실패해도 baseline 살아있음)

두 노드 동시 가동 → 같은 detection 입력에 대해 두 가지 위치 추정 비교 가능.

```
        ┌─→ object_tracker.py (baseline)        → /perception/tracked_markers
YOLO ───┤
        └─→ lidar_fusion_tracker.py (ours)      → /perception/tracked_markers_lidar_fusion
                ↑
            /lidar
```

---

## 5. 정량 평가 시나리오

### 5.1 검증 환경
- `/ground_truth` (nav_msgs/Odometry, world frame) → 동적 객체의 진짜 위치
- person_1, forklift_1의 실제 좌표는 `dynamic_obstacle_mover.py`에서 set_pose로 강제됨 → ground truth 정확함

### 5.2 비교 지표

| 시나리오 | 측정 | 기대 결과 |
|---|---|---|
| 평지 정상 보행 | 두 방식의 위치 오차 (vs ground truth) | 비슷함 |
| 떠있는 forklift (z>0) | 두 방식의 거리 오차 | LiDAR fusion 우세 |
| 발 가림 occlusion (박스 뒤 사람) | 두 방식의 위치 오차 + 검출 여부 | LiDAR fusion 우세 |
| 멀리 있는 객체 (>10m) | bbox 노이즈 amplify 효과 | LiDAR fusion 우세 (예상) |
| Static person 군집 | 클러스터링 안정성 | 검증 필요 |

### 5.3 발표용 슬라이드 후보

> "Ground plane assumption은 객체가 바닥에 정확히 닿아있을 때만 정확. 떠있는 객체, 발 가림, bbox 노이즈에서 위치 오차 발생. LiDAR fusion은 가정 없이 직접 측정 — 같은 시나리오에서 평균 X cm vs Y cm 오차."

---

## 6. 알려진 한계 (정직하게 명시)

| 한계 | 영향 | 완화책 |
|---|---|---|
| 2D LiDAR (z 정보 없음) | 평면 위치만 추정 | 우리 시뮬은 평지 — 문제 없음 |
| LiDAR FOV (360°) vs 카메라 FOV (80°) | LiDAR가 본 객체를 카메라가 못 봄 | bbox 없이는 클래스 모름 — 의도된 한계 |
| 가까운 두 객체 클러스터 분리 실패 | 사람 옆 사람 | DBSCAN 등 정교한 클러스터링 (P-7+ 검토) |
| 반사/투명 객체 | LiDAR 못 잡음 | 시뮬 환경에선 문제 없음 |
| 시간 동기화 오차 | 50ms × 속도 = 위치 오차 | 단순 방식으로 시작, 정량 평가 후 결정 |

---

## 7. 결정 필요 사항

- [ ] `target_frame` 통일 — JW는 `odom_filtered` 사용 중. Cooperative master에서는 multi-AMR 통합 위해 `world` 권장. 토픽 발행은 `odom_filtered`로 호환 유지, master 내부에서 `world` 변환?
- [ ] LiDAR 클러스터링 방식 — 단순 거리 vs DBSCAN
- [ ] 시간 동기화 — 단순 최근 scan vs ApproximateTimeSynchronizer
- [ ] `kalman_track.py` 분리 여부 (코드 중복 회피)
- [ ] JW와 협의: 신규 노드로 갈지 / `object_tracker.py`에 LiDAR fusion 옵션 추가 (파라미터 토글)
  - HU 입장: 신규 노드 권장 (ablation, conflict 회피)

---

## 8. 작업 순서 (브랜치 분기 후)

1. `kalman_track.py` 추출 (object_tracker.py에서 KalmanTrack 클래스만 별도 모듈로) — JW 합의 필요
2. `lidar_fusion_tracker.py` 골격 — KalmanTrack 재사용, _pixel_to_world만 새 알고리즘으로
3. ① bbox → frustum 함수 단위 테스트
4. ② LiDAR → camera frame 변환 단위 테스트
5. ③ frustum 필터링 함수 단위 테스트
6. ④ 클러스터링 함수 단위 테스트
7. 통합 + Foxglove에서 두 tracker 동시 확인
8. 정량 평가 시나리오 5.2 실행
9. PR: `feat(perception): LiDAR-camera fusion tracker (baseline preserving)`
