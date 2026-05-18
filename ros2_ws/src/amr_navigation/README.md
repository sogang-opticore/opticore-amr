# amr_navigation

> Opticore AMR Navigation 패키지 — A\* Global Planner + DWA Local Planner (직접 구현).
>
> ⚠ **Nav2의 `dwb_local_planner` / `nav2_navfn` / `nav2_smac_planner`를 사용하지 않는다.**
> Notion 명세 §7 "DWA Local Planner와 A\* Global Planner의 핵심 로직은 직접 구현한다"를 따른다.
>
> 본 README는 동시에 **A\* ↔ DWA 토픽 계약(Interface Contract)** 문서다.
> 양쪽 노드 작성 전에 합의해야 한다. 변경 절차는 §11 참고.
>
> **상태**: 잠정 합의 (2026-05-15) — A\*은 Python으로 확정, 토픽명·타입은 잠정. 양쪽 구현 진행하며 보강.

---

## 0. 디스코드 N-0 작업 매핑 (2026-05-15)

본 문서는 디스코드 `#navigation` 채널의 **N-0. A↔DWA 인터페이스 설계** 작업의 산출물이다.

| 디스코드 요구 항목 | 본 문서 위치 |
|---|---|
| 전역 경로 토픽명 / 타입 / frame_id 명시 (잠정: `/global_path`, `nav_msgs/Path`, `map`) | §2.2, §3.1 |
| `/cmd_vel` 발행 주체 명확화 (DWA 단독 — A\*는 직접 발행하지 않음) | §1 원칙, §2.2, §3.2 |
| goal 입력 토픽 정의 (`/goal_pose`, `geometry_msgs/PoseStamped`) | §2.1, §3.0 |
| 좌표계 정책 (모든 path는 `map` frame 기준) | §2.3 |
| stamp 정책 명시 | §3.4 |

**담당**: SW(지상원, 작성) · HU(양현욱, 리뷰)
**리뷰 결과 (2026-05-15)**: 양현욱 — "리뷰 OK, A\*은 Python으로 작성하기로 결정"

---

## 1. 패키지 책임

| 노드 | 파일 | 언어 | 담당자 | 책임 |
|---|---|---|---|---|
| `astar_planner` | `amr_navigation/astar_node.py` (예정) | **Python** ✅ (확정 2026-05-15) | HU(메인) · SW(휴리스틱 분담) | 전역 경로 계획 — 정적 맵 기반 |
| `dwa_planner` | `amr_navigation/dwa_node.py` | Python | **SW(메인)** | 지역 경로 계획 + 속도 명령 발행 |

**원칙**:
- A\*은 경로만 만들고 `cmd_vel`을 **직접 발행하지 않는다**.
- **DWA만 `cmd_vel`을 발행한다** (단일 발행 주체).
- A\*과 DWA는 **별도 노드**로 분리한다.
- 휴리스틱 / 비용 함수는 ROS 의존성 없는 순수 모듈(`amr_navigation/heuristics.py`)에 둔다 → 단위 테스트 용이.
- `include/amr_navigation/astar_planner.hpp`는 placeholder만 두고 사용하지 않는다 (C++ 구현 미채택).

---

## 2. 토픽 계약 (Interface Contract)

> 표 안의 토픽명은 **잠정** 결정. 변경 시 §11 변경 절차 따름.

### 2.1 입력 (구독)

| 토픽 | 타입 | 발행 주체 | 구독자 | QoS | 비고 |
|---|---|---|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 / BT / 사용자 | **A\*** | Reliable, depth 1 | frame_id = `map` |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | **A\*** | Reliable, depth 1, **TRANSIENT_LOCAL** | frame_id = `map`, latched |
| `/global_path` | `nav_msgs/Path` | A\* | **DWA** | Reliable, depth 1, **TRANSIENT_LOCAL** | frame_id = `map`, latched |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (robot_localization) | DWA (1순위) | Reliable | frame_id = `odom_filtered` |
| `/odom` | `nav_msgs/Odometry` | DiffDrive plugin | DWA (fallback, timeout > 0.5s) | Reliable | frame_id = `odom` |
| `/lidar` | `sensor_msgs/LaserScan` | ros_gz_bridge | DWA | **BEST_EFFORT**, depth 10 | frame_id = `lidar_link`, ~10Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 다수 | 모든 노드 | 표준 | 좌표 변환 |

### 2.2 출력 (발행)

| 토픽 | 타입 | 발행 주체 | 주기 | QoS | 용도 |
|---|---|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | **A\*** | goal 입력 시 1회 + 동적 재계획 시 | Reliable, depth 1, TRANSIENT_LOCAL | DWA가 추종할 전역 경로 |
| `/cmd_vel` | `geometry_msgs/Twist` | **DWA 단독** ★ | 20 Hz (control_rate) | Reliable, depth 10 | Ignition DiffDrive 입력 |
| `/dwa/trajectories` | `visualization_msgs/MarkerArray` | DWA | 5 Hz | Reliable | 후보 trajectory 시각화 (Foxglove) |
| `/dwa/best_trajectory` | `visualization_msgs/Marker` | DWA | 5 Hz | Reliable | 선택된 trajectory 강조 |
| `/dwa/status` | `std_msgs/String` | DWA | 1 Hz | Reliable | "WAITING_ODOM" / "STOPPED" / "PLANNING" / "EMERGENCY" |

> ★ **`/cmd_vel`은 DWA만 발행한다.** A\*은 경로만 만들고 운동 명령은 만들지 않는다. 이중 발행자가 생기면 Twist가 충돌하므로 절대 금지.

### 2.3 좌표계 정책

- **모든 Path는 `map` frame 기준**으로 발행한다 (`/goal_pose`, `/global_path` 모두).
- DWA는 `/global_path`를 받으면 내부에서 현재 로봇 위치(`base_footprint`)와의 변환을 TF로 lookup.
- `lidar` raw scan은 `lidar_link` frame이지만, 장애물 회피 시 `base_footprint` 기준으로 변환해 사용.
- 모든 노드는 `use_sim_time: true` 강제 (Notion 명세 §8 · CLAUDE.md §6.1).
- TF 트리: `map → odom_filtered → base_footprint → base_link → {lidar_link, camera_link, imu_link, wheel_links, ...}`

---

## 3. 메시지 상세 스펙

### 3.0 `/goal_pose` (외부 → A\*)

```yaml
header:
  stamp: <ros::Time>           # 발행 시각 (use_sim_time)
  frame_id: "map"              # 항상 map frame
pose:
  position:    { x, y, z=0.0 } # z는 항상 0 (2D 운영)
  orientation: { x, y, z, w }  # 도착 시 로봇 방향 (도킹 정밀도 명세 §4.2)
```

**발행 트리거**: RViz2 "2D Goal Pose" 버튼, BT의 NavigateTo 노드, 또는 Mission node가 발행.
**A\* 동작**: 새 `/goal_pose` 수신 시 즉시 이전 계획을 폐기하고 재계획 시작.

### 3.1 `/global_path` (A\* → DWA)

```yaml
header:
  stamp: <ros::Time>           # 경로 생성 완료 시각
  frame_id: "map"              # 항상 map frame
poses:                         # PoseStamped 배열
  - header:
      stamp: <ros::Time>       # header.stamp와 동일 (§3.4 참고)
      frame_id: "map"
    pose:
      position:    { x, y, z=0.0 }
      orientation: { x, y, z, w }  # heading은 다음 점 방향으로 채움 (마지막은 goal orientation)
```

**규칙**:
- `poses[0]` = 현재 로봇 위치(`base_footprint`)와 가까운 셀의 중심 (start).
- `poses[-1]` = 목적지 (goal).
- 점 간격: **0.05 ~ 0.20 m** (DWA가 lookahead 점 선택하기 쉬운 밀도).
- 빈 path(`poses=[]`)는 **"계획 실패"** 컨벤션 → DWA는 즉시 정지, `status="STOPPED"`.
- 점 수 상한: **500개 권장** (계산 비용·메시지 크기 관점). 초과 시 down-sampling.

### 3.2 `/cmd_vel` (DWA → Ignition)

```yaml
linear:
  x: <float>      # 전진 속도 [m/s], 범위 [v_min, v_max] = [-1.0, 2.0]
  y: 0.0          # 디퍼렌셜이라 항상 0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: <float>      # 회전 속도 [rad/s], 범위 [-w_max, w_max] = [-1.5, 1.5]
```

> `Twist`는 `Header`가 없는 메시지라 stamp가 없다. 발행자(DWA)만 책임지며, 발행 주체가 단 하나임을 §2.2에서 명시.

### 3.3 `/dwa/status` (DWA → 모니터링)

`std_msgs/String`의 `data` 값으로 다음 4개 상태 중 하나를 1Hz로 발행:

| 값 | 조건 |
|---|---|
| `WAITING_ODOM` | 초기 부팅 직후 — `/odometry/filtered` 미수신 |
| `STOPPED` | odom 수신 완료, 그러나 `/global_path` 없음 또는 빈 path |
| `PLANNING` | path 수신 완료, 정상 평가 루프 실행 중 |
| `EMERGENCY` | 모든 trajectory 후보가 충돌 또는 안전거리(0.30m) 침범 → 즉시 정지 |

### 3.4 stamp 정책 ⭐

본 패키지의 모든 메시지의 `header.stamp` 운영 정책:

| 메시지 | `header.stamp` 의미 | `poses[*].header.stamp` (Path 전용) |
|---|---|---|
| `/goal_pose` | 발행자(RViz/BT)가 입력한 시각 | — |
| `/global_path` | A\*이 **경로 계산을 완료한** 시각 | header.stamp와 **동일** 값으로 채움 (모든 점) |
| `/odometry/filtered`, `/odom` | EKF / DiffDrive plugin이 메시지를 만든 시각 | — |
| `/lidar` | Gazebo가 scan을 측정한 시각 | — |
| `/cmd_vel` | `Twist` — stamp 필드 없음 | — |
| `/dwa/status` | `String` — stamp 필드 없음 | — |
| `/dwa/trajectories`, `/dwa/best_trajectory` | DWA가 후보를 평가한 시각 | — |

**규칙 요약**:
1. 모든 stamp는 **ROS2 시뮬레이션 시간** (`use_sim_time: true`). wall clock 금지.
2. `nav_msgs/Path`의 `header.stamp`와 `poses[*].header.stamp`는 **동일 값**으로 채운다 (개별 점에 시간 정보 부여 안 함).
3. DWA의 `sensor_timeout = 0.1s` 는 **odom/imu state 입력에만** 적용. `/global_path`는 stamp 무관(latched).
4. DWA의 `odom_timeout = 0.5s` 초과 시 fallback 또는 정지.

---

## 4. 파라미터 (DWA / A\*)

### 4.1 DWA — 로봇 동역학 한계 (Notion 명세 §4.1 확정값)

| 파라미터 | 값 | 출처 |
|---|---|---|
| `v_max` | **2.0 m/s** | 명세 "최대 속도 2.0 m/s" |
| `v_min` | -1.0 m/s | 후진 제한 (운영 정책) |
| `w_max` | **1.5 rad/s** | 명세 "최대 회전 속도 1.5 rad/s" |
| `a_max` | **1.0 m/s²** | 명세 "최대 가속도 1.0 m/s²" |
| `alpha_max` | 1.5 rad/s² | 명세 미명시 — **TODO 시뮬레이션 측정 후 갱신** |

> ⚠ `alpha_max`는 명세에 없음. 임시값 사용. `cmd_vel` 스텝 응답으로 측정 후 확정.

### 4.2 DWA — 샘플링 / 시뮬레이션

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `sample_v_n` | 11 | 선속도 샘플 수 |
| `sample_w_n` | 21 | 각속도 샘플 수 (총 11×21 = 231 후보) |
| `predict_horizon` | 1.0 s | trajectory 예측 시간 |
| `dt` | 0.1 s | 적분 step |
| `control_rate` | 20.0 Hz | 제어 주기 |

### 4.3 DWA — 평가함수 가중치

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `weight_heading` | 0.8 | goal 방향 정렬도 |
| `weight_clearance` | 0.4 | 장애물 여유 거리 |
| `weight_velocity` | 0.2 | 빠른 속도 선호 |

> 좁은 통로(< 1.5 m)에서는 `weight_clearance`를 0.6으로 상향 권장.

### 4.4 DWA — 안전

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `safety_distance` | 0.30 m | 명세 §7 안전거리 |
| `odom_timeout` | 0.5 s | 이 시간 안에 `/odometry/filtered` 없으면 `/odom` fallback |

### 4.5 A\* — 그리드 / 휴리스틱 (제안, HU 확정 대기)

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `heuristic` | `"octile"` | `manhattan` / `euclidean` / `octile` (heuristics.py 등록됨) |
| `allow_diagonal` | `true` | 8-connected |
| `inflation_radius` | 0.30 m | 로봇 폭 0.40 m + 여유 0.10 m |
| `smoothing` | `"catmull_rom"` | `none` / `catmull_rom` / `bezier` |
| `replan_period` | 0.0 s | 0이면 goal 입력 시에만 1회, > 0이면 주기적 재계획 |

---

## 5. 실패 / 안전 정책

| 상황 | A\* 동작 | DWA 동작 |
|---|---|---|
| goal이 점유 셀 | 빈 path 발행 + 로그 | — |
| goal 도달 불가 (장애물로 막힘) | 빈 path 발행 | 정지 |
| `/global_path` empty 수신 | — | 즉시 정지, `status="STOPPED"` |
| 모든 trajectory 후보 충돌 | — | 정지, `status="EMERGENCY"` |
| 장애물 거리 < `safety_distance` (0.30 m) | — | 즉시 정지 (명세 §7 안전거리) |
| `/odometry/filtered` 미수신 > 0.5 s | — | `/odom`으로 fallback |
| `/odom`마저 미수신 | — | 정지, `status="WAITING_ODOM"` |
| `/global_path` frame_id ≠ `map` | — | 경고 로그 + path 무시 |

---

## 6. 성능 목표 (Notion 명세 §4 발췌)

| 지표 | 목표 |
|---|---|
| 경로 계획 성공률 | ≥ 98% (50쌍) |
| 경로 재계획 시간 | ≤ 500 ms |
| Cross Track Error (직선) | ≤ 5 cm |
| Cross Track Error (곡선) | ≤ 10 cm |
| 동적 회피 30회 충돌 | 0건 |
| 회피 후 경로 이탈 | ≤ 1 m |
| 회피 후 복귀 시간 | ≤ 5 s |
| 응답 시간 (cmd → 첫 움직임) | ≤ 200 ms |
| 단위 테스트 커버리지 | ≥ 70% |

---

## 7. 디렉터리 구조

```
amr_navigation/
├── package.xml
├── CMakeLists.txt
├── README.md                   # 본 문서 (인터페이스 계약)
├── include/amr_navigation/
│   └── astar_planner.hpp       # placeholder (C++ 미채택)
├── amr_navigation/             # Python 패키지 (★ 실제 구현 영역)
│   ├── __init__.py
│   ├── dwa_node.py             # ✅ DWA 노드 (Week 1 골격 완료)
│   ├── heuristics.py           # ✅ A* 휴리스틱·비용함수 (Week 1)
│   └── astar_node.py           # 🟡 A* 노드 (Week 2, HU 작성 예정)
├── launch/
│   └── dwa_only.launch.py      # DWA 단독 검증용
├── config/
│   ├── dwa_params.yaml         # DWA 파라미터 외부화
│   └── astar_params.yaml       # 🟡 A* 파라미터 (HU 작성 예정)
└── test/
    └── test_heuristics.py      # 단위 테스트 (21건 통과)
```

---

## 8. 빌드 & 실행

```bash
# 빌드
cd /workspace/opticore-amr/ros2_ws
colcon build --symlink-install --packages-select amr_navigation
source install/setup.bash

# 단위 테스트
colcon test --packages-select amr_navigation --event-handlers console_direct+
colcon test-result --verbose

# DWA 단독 실행 (Week 1 검증용)
ros2 launch amr_navigation dwa_only.launch.py

# 토픽 확인
ros2 topic echo /cmd_vel       # 빈 평가함수라 (0,0)이 나와야 함 (Week 1 의도)
ros2 topic echo /dwa/status    # WAITING_ODOM → STOPPED → PLANNING 상태 변화

# 통합 검증 (warehouse.launch.py가 같이 떠 있을 때)
ros2 topic pub --once /global_path nav_msgs/msg/Path \
  "{header: {frame_id: 'map'}, poses: [
     {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]}"
# → /dwa/status가 STOPPED → PLANNING 으로 전환되면 통합 OK
```

---

## 9. 합의 체크리스트 (N-0 리뷰 결과 2026-05-15)

- [x] `/global_path` 토픽명·타입 OK (잠정, 변경 시 §11)
- [x] `/cmd_vel` 발행 주체가 **DWA 단독**임에 동의 ★
- [x] `/goal_pose` 토픽명·타입 OK
- [x] 좌표계 정책(`map` frame) OK
- [x] stamp 정책 OK (§3.4)
- [x] 빈 path = 실패 컨벤션 OK
- [x] DWA 파라미터 기본값 OK (특히 `alpha_max` TODO 인지)
- [x] A\* 휴리스틱 옵션 3종 OK (실제 default는 추후 결정)
- [x] A\* 직접 구현 — Nav2 NavFn 사용 안 함에 동의
- [x] **A\* 언어 = Python 확정** ✅ (양현욱 결정, 2026-05-15)
- [x] 안전 정책 §5 OK
- [x] QoS 프로파일 (BEST_EFFORT for sensor, TRANSIENT_LOCAL for latched) OK

> 모든 항목 합의 완료. 양현욱 리뷰 메시지(디스코드 `#navigation`, 2026-05-15 01:11):
> *"리뷰는 했으나 커밋은 따로 안했습니다~ branch에 이미 올라와져 있었고, 이상 없음 확인 했습니다. A\*는 파이썬으로 작성하기로 결정했습니다"*

---

## 10. 미해결 / 미확정 사항 (Week 2 진입 후 확정)

- [ ] A\* 재계획 트리거 정책: 동적 장애물 발견 시 자동 재계획? 일정 주기 강제?
- [ ] `/global_path` 너무 길 때 (> 500 points) DWA의 lookahead 정책
- [ ] A\* 실패(경로 못 찾음) 시 발행 형식: 빈 Path? 별도 status 토픽?
- [ ] `alpha_max` 실측값 (시뮬레이션 step response 측정 후 갱신)
- [ ] A\* 사용 맵 소스: SLAM의 `/map` (동적) vs 미리 저장된 `warehouse_map.yaml` (정적)

---

## 11. 변경 절차

본 계약을 변경하려면:
1. 변경 제안을 디스코드 `#navigation` 채널에 포스팅
2. 양쪽 담당(A\*: HU, DWA: SW)이 동의
3. 본 README.md 갱신 + §12 변경 이력 한 줄 추가
4. 양쪽 코드 (`dwa_node.py`, `astar_node.py`, `dwa_params.yaml`, `astar_params.yaml`) 반영
5. 통합 테스트(mock `/global_path` → DWA `PLANNING` 전이) 통과 후 `dev` 머지

---

## 12. 변경 이력

| 일자 | 변경 | 작성자 | 리뷰 |
|---|---|---|---|
| 2026-05-10 | 초안 작성 — Nav2 기반 → 직접 구현으로 정정, 토픽 계약 추가, `v_max=2.0` 명세 반영 | SW(지상원) | — |
| 2026-05-15 | **N-0 작업 (디스코드 `#navigation`) 반영** — A\* Python 확정, stamp 정책 §3.4 신설, QoS 프로파일 명시, 잠정 결정 표시 강화, `/goal_pose` 스펙 §3.0 추가, 통합 검증 명령 §8 추가 | SW(지상원) | HU(양현욱) 리뷰 OK |
