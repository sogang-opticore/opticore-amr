# Cooperative Master Node — 인터페이스 명세

> 브랜치 분기 전 사전 작업 #3
> 작성: HU
> 목적: 다수 AMR + CCTV의 detection을 통합하는 master 노드 인터페이스 정의
> 의존성: lidar_fusion_tracker 머지 후 그 위에서 구현 / DWA 측 인터페이스는 별도 합의 필요

---

## 1. 역할

```
[AMR_1 fusion tracker]       /amr_1/perception/objects_3d  ─┐
[AMR_2 fusion tracker]       /amr_2/perception/objects_3d  ─┤
[CCTV_1 detector]            /cctv_1/perception/objects_3d ─┼─→ [cooperative_master] ─→ /global_object_map
[CCTV_2 detector]            /cctv_2/perception/objects_3d ─┤
[CCTV_3 detector (옵션)]     /cctv_3/perception/objects_3d ─┘
                                                                       ↓
                                                              각 AMR의 DWA가 구독
```

**처리**:
1. 각 source의 detection을 world frame으로 통합
2. Data association (어느 detection이 같은 객체인지)
3. 같은 객체의 multi-source 측정 fusion (Kalman 또는 weighted mean)
4. 객체 lifecycle 관리 (생성/유지/제거)
5. `/global_object_map` publish

---

## 2. 메시지 스펙

### 2.1 단일 detection 메시지 — `Object3D.msg`

`amr_msgs` 패키지 신설 or `amr_perception/msg/` 추가.

```
# Object3D.msg — 한 sensor가 본 단일 객체

std_msgs/Header header           # frame_id = world (또는 sender frame), stamp

uint32 source_id                 # 발신 sensor 식별
                                 #   0~99: AMR (예: 1=amr_1, 2=amr_2)
                                 #   100~199: CCTV (예: 101=cctv_1, 102=cctv_2)

string class_name                # "person" or "forklift"
float32 class_confidence         # YOLO confidence

geometry_msgs/Point position     # world frame 좌표 (z 무시 가능, 2D)
geometry_msgs/Vector3 velocity   # 추정 속도 (tracker 단계까지 거쳤다면)

float64[9] position_covariance   # 3x3 row-major
                                 # CCTV는 평면 정확/depth 큰 covariance
                                 # AMR LiDAR fusion은 모든 방향 비슷한 covariance
                                 # AMR ground plane은 distance에 따라 anisotropic
```

### 2.2 통합 객체 메시지 — `GlobalObject.msg`

```
# GlobalObject.msg — master가 통합한 객체

uint32 global_id                 # master가 부여한 영속 ID (track ID 아님)
string class_name
float32 class_confidence         # 통합 confidence

geometry_msgs/Point position     # world frame, fused
geometry_msgs/Vector3 velocity   # world frame, fused
float64[9] position_covariance
float64[9] velocity_covariance

uint32[] contributing_sources    # 이 객체에 기여한 source_id 리스트
                                 # 발표/디버깅용 (예: [1, 101] = AMR_1 + CCTV_1)

builtin_interfaces/Duration time_since_last_update
                                 # 마지막 갱신 후 경과 — stale 판단용
```

### 2.3 토픽 메시지 — `GlobalObjectMap.msg`

```
# GlobalObjectMap.msg — master 통합 출력

std_msgs/Header header           # frame_id = world
GlobalObject[] objects
```

---

## 3. 토픽 인터페이스

### 3.1 Master 입력 (구독)

| 토픽 | 타입 | 발신자 | 주기 |
|---|---|---|---|
| `/amr_1/perception/objects_3d` | `Object3DArray` | AMR_1 fusion tracker | ~15Hz |
| `/amr_2/perception/objects_3d` | `Object3DArray` | AMR_2 fusion tracker | ~15Hz |
| `/cctv_N/perception/objects_3d` | `Object3DArray` | CCTV detector | ~15Hz |

**참고**: `Object3DArray.msg`는 단순히 `Header + Object3D[] objects`.

### 3.2 Master 출력 (발행)

| 토픽 | 타입 | 주기 | 구독자 |
|---|---|---|---|
| `/global_object_map` | `GlobalObjectMap` | 10Hz (예상) | 각 AMR의 DWA |
| `/global_object_markers` | `visualization_msgs/MarkerArray` | 5Hz | Foxglove |
| `/risk_alerts` | `RiskAlert[]` (옵션) | event-driven | 각 AMR |

### 3.3 좌표계 통일 — World frame

**모든 입력은 world frame으로 받음.** 각 sender의 책임:
- AMR fusion tracker: `lidar_link → world` 변환 후 발행
- CCTV detector: SDF에 명시된 정적 좌표 사용 (master 파라미터로 별도 주입 가능)

**근거**:
- multi-AMR에서 각자의 `odom_filtered`가 SLAM drift로 어긋남
- world frame은 시뮬에서 `/ground_truth` frame과 동일 → 검증 정확
- 실물 환경 한정: "캘리브레이션된 공통 frame 필요" 명시 (향후 과제)

---

## 4. Data Association — 핵심 알고리즘

### 4.1 문제

서로 다른 source의 detection이 들어옴:
- AMR_1이 본 person at (12.3, 17.8)
- AMR_2가 본 person at (12.5, 17.9)
- CCTV_1이 본 person at (12.4, 17.85)

**같은 사람인가? 다른 사람인가?**

### 4.2 알고리즘 — Greedy nearest-neighbor + class match

JW의 `object_tracker.py` 매칭 로직 (IoU + 거리 fallback) 참고. 다만 IoU는 single-camera 가정이라 못 씀. 거리만 사용.

```python
def associate(new_detections, existing_globals):
    """
    new_detections: 이번 cycle에 들어온 Object3D 리스트
    existing_globals: master가 유지 중인 GlobalObject 리스트

    return: (matched_pairs, unmatched_new, unmatched_existing)
    """
    DIST_THRESHOLD = 1.5  # m — person 폭의 ~2배

    matched_new = set()
    matched_existing = set()
    pairs = []

    # 모든 (new, existing) 쌍의 거리 계산
    candidates = []
    for ni, new in enumerate(new_detections):
        for ei, existing in enumerate(existing_globals):
            if new.class_name and existing.class_name and new.class_name != existing.class_name:
                continue
            dist = euclidean(new.position, existing.position)
            if dist < DIST_THRESHOLD:
                candidates.append((dist, ni, ei))

    # 가까운 순으로 greedy 매칭
    candidates.sort()
    for dist, ni, ei in candidates:
        if ni in matched_new or ei in matched_existing:
            continue
        pairs.append((ni, ei))
        matched_new.add(ni)
        matched_existing.add(ei)

    unmatched_new = [n for i, n in enumerate(new_detections) if i not in matched_new]
    unmatched_existing = [e for i, e in enumerate(existing_globals) if i not in matched_existing]

    return pairs, unmatched_new, unmatched_existing
```

**복잡도**: O(N×M log(N×M)). 객체 수 적으니 (몇십 개) 충분.

**대안**: Hungarian algorithm (scipy.optimize.linear_sum_assignment) — 최적해 보장. 일단 greedy로 시작, 정량 평가에서 문제되면 업그레이드.

### 4.3 Threshold 결정

`DIST_THRESHOLD = 1.5m`:
- 너무 작으면 (예: 0.5m): 같은 사람을 두 객체로 봄 (SLAM drift)
- 너무 크면 (예: 3m): 가까이 있는 두 사람을 한 객체로 봄
- 1.5m가 person 폭(~0.5m)의 3배, 시뮬 SLAM drift 보다 큰 수준

이 값은 시뮬에서 검증하면서 튜닝.

---

## 5. Fusion — Multi-source 결합

### 5.1 두 가지 모드

#### 모드 A: 단순 가중평균 (내분)

HU 직관: "둘 다 보면 내분"

```python
def fuse_simple(measurements):
    """
    measurements: [(position, covariance), ...]
    return: (fused_position, fused_covariance)
    """
    # Information form: 공분산 역수 가중
    info_sum = np.zeros((3, 3))
    info_pos_sum = np.zeros(3)
    for pos, cov in measurements:
        info = np.linalg.inv(cov)
        info_sum += info
        info_pos_sum += info @ pos
    fused_cov = np.linalg.inv(info_sum)
    fused_pos = fused_cov @ info_pos_sum
    return fused_pos, fused_cov
```

**근거**: 공분산 작은 sensor에 자동으로 더 큰 가중치. 두 sensor 공분산 동일하면 정확히 평균(=중점). 가중치를 covariance로 표현하면 "내분의 일반화".

#### 모드 B: Kalman Filter (시간축 포함)

각 GlobalObject가 KF 상태 유지 → 새 measurement로 update.

```python
class GlobalObjectKF:
    def __init__(self, initial_obj):
        # 상태: [x, y, vx, vy]
        # JW의 KalmanTrack과 동일한 구조
        ...

    def update(self, measurement):
        # measurement.covariance를 R로 사용
        # → 각 source의 신뢰도가 자동 반영
        ...

    def predict(self, dt):
        # 새 detection 없어도 시간 진행 → 객체 위치 예측
        # DWA가 "예측 위치" 활용 가능
        ...
```

### 5.2 추천 — 모드 B (Kalman)

이유:
- 시간축 포함 → velocity 추정 → DWA의 predictive avoidance에 직결
- JW의 KalmanTrack 거의 그대로 재사용 가능
- "Detection 안 들어와도 predict로 객체 위치 유지" 효과 (occlusion 직후 0.5초 정도 유지)

모드 A는 fallback. 일단 모드 B로 구현.

### 5.3 Covariance 설계 — sensor별

| Sensor | position_covariance 패턴 |
|---|---|
| CCTV 천장 카메라 | xy 작음 (평면 정확), z 큼 (의미 없음, 무시) |
| AMR LiDAR fusion | xy 모두 ~0.1m 수준 (LiDAR noise σ=0.03 기반) |
| AMR ground plane (baseline) | 가까운 객체 작음, 먼 객체 큼 (anisotropic) |

발표용 한 줄: "센서 특성을 covariance로 모델링 → KF가 자동 가중"

---

## 6. 객체 Lifecycle

| 상태 | 조건 | 동작 |
|---|---|---|
| Spawn | 어느 source든 새 객체 detection (unmatched) | global_id 부여, GlobalObject 생성 |
| Active | 1초 이내 갱신 | predict + update 매 cycle |
| Stale | 1~3초 갱신 없음 | predict만 (위치 예측 유지) |
| Dead | 3초 이상 갱신 없음 | 제거 |

**참고**: JW의 `max_miss_count = 90` (6초 @15Hz)는 객체 단일 추적용. master는 multi-source라 한 source 끊겨도 다른 source가 봄 → 더 짧게 (3초) 설정.

### 6.1 영속 global_id

- 한 번 부여된 global_id는 객체가 사라질 때까지 유지
- Stale → Active 복귀 시에도 동일 ID
- 발표용: "fleet 차원 ID 일관성" 강조 가능

---

## 7. DWA 인터페이스 — DWA 담당자 합의 필요 ★

### 7.1 두 옵션

#### 옵션 A: DWA가 `/global_object_map` 직접 구독

- 우리 출력 → DWA가 받아 cost function에 추가
- "예측 기반 회피" 가능 (velocity + predict)
- DWA 코드 수정 필요 → DWA 담당자 작업

#### 옵션 B: 우리가 costmap layer로 publish

- 표준 Nav2 layer plugin 작성
- DWA 코드 수정 없음
- 예측 정보 활용 어려움 (costmap은 정적 grid)

**HU 결정: 옵션 A.** A*/DWA 직접 구현이라는 우리 차별점과 예측 기반 회피라는 추가 차별점 살리기 위해.

### 7.2 합의 필요 항목

DWA 담당자랑 만나서 결정:
- [ ] DWA가 어느 토픽 구독할지 (`/global_object_map` 그대로 vs 단순화한 사본)
- [ ] DWA cost function에 객체를 어떻게 반영할지
  - 객체 위치에 inflation cost?
  - 객체 예측 궤적(1초 후)도 cost?
  - velocity 기반 TTC 계산?
- [ ] 메시지 타입 — 우리 커스텀 (`GlobalObjectMap`) vs 표준 (`MarkerArray`, `Detection3DArray`)
  - 커스텀이 정보 완전. 표준이 DWA 측 변경 적음
- [ ] DWA가 어떤 frame에서 동작 — `world`인지 `map`인지 `odom_filtered`인지
- [ ] 위험 알림 별도 토픽 (`/risk_alerts`) 필요한가, `/global_object_map`만으로 충분한가

### 7.3 인터페이스 합의 문서 양식

```
[NAV ↔ Perception Cooperative 인터페이스 합의서]

- 토픽: /global_object_map
- 타입: amr_msgs/GlobalObjectMap (또는 합의된 표준 타입)
- 주기: 10Hz
- Frame: world
- DWA 측 처리:
  - 각 객체 위치에 cost N (inflation radius M)
  - 예측 위치 (1초 후, velocity 기반) 에 cost N/2
- DWA 측 수정 PR: feature/dwa-cooperative-integration
- Perception 측 PR: feature/cooperative-perception
- 머지 순서: DWA 측 인터페이스 stub 먼저 → 우리 publish → DWA 측 활용 로직
```

---

## 8. 패키지 구조 — 결정 필요

### 8.1 옵션 A: `amr_perception`에 통합

```
amr_perception/
├── amr_perception/
│   ├── yolo_detector.py
│   ├── object_tracker.py        (baseline)
│   ├── lidar_fusion_tracker.py  (per-AMR)
│   ├── cooperative_master.py    (신규, system-wide)
│   └── kalman_track.py          (공통 모듈)
└── msg/
    ├── Object3D.msg
    ├── Object3DArray.msg
    ├── GlobalObject.msg
    └── GlobalObjectMap.msg
```

장점: 패키지 하나로 관리, perception 영역 응집

### 8.2 옵션 B: `amr_msgs` 신설 + `amr_fleet` 신설

```
amr_msgs/                  (메시지 전용)
├── msg/
│   ├── Object3D.msg
│   └── ...

amr_perception/            (per-AMR, 기존 + lidar_fusion_tracker)

amr_fleet/                 (system-wide, cooperative_master)
└── amr_fleet/
    └── cooperative_master.py
```

장점: SW 패키지 분리 명확, 향후 Fleet 확장 시 자연스러움
단점: 패키지 3개 동시 작업

**HU 추천: 옵션 A로 시작 → 6월 데모 후 옵션 B로 리팩터링.** 일정상 옵션 B는 부담.

다만 메시지 타입은 다른 팀(SW)이 쓸 가능성 있어 `amr_msgs` 분리도 합리적. **별도 의견 수렴 필요.**

---

## 9. 시각화

- `/global_object_markers` (MarkerArray) Foxglove에서 표시
- 색상으로 source 구분:
  - 빨강: AMR_1만 본 객체
  - 파랑: AMR_2만 본 객체
  - 노랑: CCTV만 본 객체
  - 초록: 다중 source (cooperative effect)
- 객체 옆에 `contributing_sources` 텍스트 (예: `[AMR_1, CCTV_1]`)

발표용 효과: cooperative perception이 실제로 동작 중임을 한눈에.

---

## 10. 결정 필요 사항 (정리)

- [ ] **DWA 담당자 합의** — 인터페이스 (§7.2 체크리스트)
- [ ] **JW 합의** — `kalman_track.py` 추출 + 신규 노드 방향
- [ ] 패키지 구조 (§8 옵션 A vs B)
- [ ] Fusion 모드 (Kalman 모드 B 권장)
- [ ] CCTV 수량 (CCTV SDF 문서 참조)
- [ ] Stale → Dead 시간 (1초 / 3초 — 시뮬 테스트로 결정)
- [ ] Risk alert 별도 토픽 도입 여부

---

## 11. 작업 순서 (브랜치 분기 후, lidar_fusion_tracker 완료 후)

1. 메시지 정의 (`amr_msgs` 신설 또는 `amr_perception/msg/` 추가)
2. `lidar_fusion_tracker.py` → `Object3DArray` 발행 추가
3. `cooperative_master.py` 골격 — 입력 구독 + 빈 발행
4. Data association 구현 (§4)
5. KF fusion 구현 (§5)
6. Object lifecycle 관리 (§6)
7. `/global_object_map` publish
8. Foxglove 시각화 확인
9. DWA 측 합의된 인터페이스로 검증 (DWA 담당자와 통합 테스트)
10. 정량 평가 + ablation
11. PR: `feat(perception): cooperative master for multi-source object fusion`
