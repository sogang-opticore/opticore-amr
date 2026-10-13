# 멀티로봇 구조 처방 — "로봇마다 풀스택"에서 "공유 서비스 + 얇은 로봇별 루프"로

> **작성** 2026-10-13 · **상태** 제안(Proposal), 팀 결정 필요 · **관련** [`server-run-prescription.md`](server-run-prescription.md) §5.4
>
> **목적**: 로봇 수 N에 비례해 늘어나는 중복 계산과 프로세스를 끊는다. 또 로봇끼리 서로를 "모르는 장애물"로 다루는 구조를 바꾼다.
>
> **전제**: 팀 명세는 "A\*와 DWA 핵심 로직은 직접 구현"이다([`amr_navigation/README.md`](../ros2_ws/src/amr_navigation/README.md)). 이 문서는 알고리즘을 갈아엎자는 제안이 아니다. **배치**(어디서 몇 번 돌리나)와 **실행 방식**(언어·스레드·이벤트)을 바꾸자는 제안이다.

---

## 1. 현재 구조 (as-is)

```text
Ignition Gazebo (1) ── 렌더링 센서 12개(lidar 4, 로봇 카메라 4, CCTV 4), 물리 250 Hz
   │  ▲
   │  └─ dynamic_obstacle_mover (Py) ── 0.2 s마다 `ign service` 프로세스 생성
   │
   ├─ amr1 ┐  ros_gz_bridge (/clock, lidar, imu, odom, camera…)
   ├─ amr2 │  RSP · static TF ×2 · IMU/odom 공분산 주입기 (Py ×2)
   ├─ amr3 │  EKF · map_server · AMCL · lifecycle_manager
   └─ amr4 ┘  A* (Py, 1M 셀 격자, 1 Hz + 이벤트 재계획) · DWA (Py, 20 Hz, 자체 LiDAR 트래킹 + no-go 레이어)
                       × 4 = 48 프로세스
전역: fleet_localization_init (Py) · fused_tracker (Py, amr1 입력만) · deadlock_manager (Py)
선택: f3_priority_node (Py) · CCTV 검출/오버레이 (Py, amr1에만 연결) · [YOLO (Py+torch) × N]
```

### 구조적 문제

| # | 문제 | 구체적 모습 |
|---|---|---|
| P1 | **같은 일을 N번 한다** | 같은 맵을 map_server 4개가 4번 로드하고, A\* 4개가 inflation·distance transform을 4번 계산한다. `/clock`을 4번 브리지하고, Python TF listener 여러 개가 4대 전체 `/tf`를 각자 역직렬화한다 |
| P2 | **무거운 계산을 Python 단일 스레드 콜백 안에서 한다** | A\* 계획 한 번에 수 초, 경로가 없으면 14 s(로컬 측정). DWA 틱 50–75 ms(예산 50 ms). 그동안 TF·goal·센서 처리가 멈춘다 |
| P3 | **로봇끼리 서로를 모른다** | 각 DWA가 다른 AMR을 LiDAR 점으로만 보고 멈춘다. no-go를 칠하면 각 A\*가 그 구역을 30 s~5 min 벽으로 취급하고 각자 우회한다. 조율은 사후 개입(deadlock_manager의 goal 덮어쓰기)뿐이다 |
| P4 | **인지 결과가 주행에 쓰이지 않는다** | `fused_tracker`·YOLO·CCTV 출력을 구독하는 주행 노드가 없다. DWA가 4대분 트래킹을 각자 다시 한다 |
| P5 | **기동이 벽시계 타이머 기반이다** | `TimerAction`(10 s 간격, +20/+40/+55 s)은 느린 서버에서 순서를 보장하지 못한다. lifecycle 타임아웃을 사후 복구 노드로 메운다 |
| P6 | **실행 계약이 검사되지 않는다** | YAML 키와 네임스페이스가 맞지 않아 튜닝이 조용히 빠졌다. QoS 불일치(F-3 경로)와 고정 경로(모델)도 있다 |

로봇 1대를 추가할 때마다 Python 프로세스 4개(A\*, DWA, 주입기 2)와 C++ 프로세스 8개가 늘어난다. 특히 A\*·DWA는 각각 코어 1개 가까이를 쓴다. 그래서 **4대에서 CPU가 포화되고 RTF가 약 0.3으로 떨어지는 것은 구조의 직접적인 결과**다.

---

## 2. 설계 원칙

1. **로봇별로는 "그 로봇만의 것"만 둔다.** 센서 I/O, 위치추정(EKF·AMCL), 저지연 안전 제어(속도 명령·충돌 정지)가 여기에 해당한다.
2. **모든 로봇에 공통인 것은 한 번만 계산한다.** 정적 맵과 비용층, 전역 계획, 장애물 트래킹, 플릿 조율이 여기에 해당한다.
3. **무거운 계산은 콜백 밖에서, 빠른 코어로 한다.** C++·numpy·numba를 쓰고, 워커 스레드나 액션 서버로 분리한다. 주기 폴링 대신 이벤트 기반으로 동작한다.
4. **로봇은 서로를 "AMR"로 안다.** 공유 pose와 경로로 양보·예약한다. LiDAR 덩어리로 추정하지 않는다.
5. **계약은 문서화하고 시작 시 검사한다.** 토픽, QoS, 프레임, 시간 기준, 파라미터 적용 여부가 대상이다.
6. **N = 1에서도 같은 구조로 동작한다.** 단일로봇 개발 흐름을 깨지 않는다.

---

## 3. 목표 구조 (to-be)

```text
[시뮬레이션]
  Ignition Gazebo — EGL headless(GPU), 필요한 센서만 (로봇 카메라·CCTV는 시나리오별 on/off)
  global_bridge ×1 — /clock (+ CCTV는 CCTV 시나리오에서만)
  amrN sensor_bridge — lidar, imu, odom, cmd_vel 만

[플릿 공유 서비스 ×1]
  fleet_map_server ── /map (latched, 1개)
  fleet_tracker    ── 모든 로봇 LiDAR (+ CCTV) → /fleet/obstacles (map 프레임, 로봇 자신 제외)
                                               → /fleet/dynamic_layer (필요 구역만, 해제도 발행)
  fleet_planner    ── 정적 비용층(inflation·clearance) 1회 계산·공유
                      요청 시 계획: /amrN/goal_pose → /amrN/global_path (기존 계약 유지) 또는 액션
                      동적층 = /fleet/obstacles 기반 '높은 비용'(벽 아님), 다른 로봇 경로는 예약/비용으로 반영
  fleet_manager    ── F-3(선제 우선순위) + F-2(교착 해소) 통합, 로봇 상태 집계
  (선택) perception_server ── YOLO 1개 (GPU, half, 로봇별 최신 프레임 배치, 2–5 Hz, 필요할 때만)

[로봇별 ×N]  component container 1개
  robot_state_publisher · EKF · AMCL(map은 /map 공유) · local_controller · cmd_vel watchdog
  local_controller 입력: /amrN/global_path, /amrN/odometry/filtered,
                         /amrN/lidar (근접 안전 정지 전용), /fleet/obstacles (회피 판단)
```

### 무엇이 달라지나

- **A\* 4개 → fleet_planner 1개.** 정적 비용층은 한 번만 계산한다. 계획은 요청이 있을 때만 하고, 빠른 코어(C++ 또는 벡터화)와 워커에서 돈다. 주기 재계획 대신 이벤트(새 goal, 경로 차단, 장애물 변화)로 동작한다.
  - **주의**: Python 한 프로세스 안의 스레드는 GIL 때문에 4대 계획을 직렬로 처리한다. 공유 플래너의 탐색 코어는 C++/numba이거나 프로세스 풀이어야 한다.
- **DWA 내장 트래킹·레이어 4벌 → fleet_tracker 1개.** DWA 틱의 대부분이던 정적 필터, 클러스터링, 레이어 재생성이 제어기에서 빠진다. 제어기는 경로 추종과 근접 안전만 맡아 "얇아진다".
- **로봇 자신을 장애물 목록에서 뺀다.** 다른 AMR은 장애물이 아니라 "예약·우선순위 대상"으로 다룬다. 앞 로봇 때문에 멈추거나 통로를 수 분간 닫는 일이 사라진다.
- **map_server 4개 → 1개.** AMCL `map_topic`은 `/map`으로 remap한다.
- **`/clock` 4중 → 1개, 로봇 카메라 기본 off.**

---

## 4. 컴포넌트 배치표

| 컴포넌트 | 현재 | 목표 | 근거 | 기대 효과 |
|---|---|---|---|---|
| `/clock` 브리지 | 로봇별 4 | 전역 1 | 시간 발행자는 하나여야 한다 | clock 트래픽 75% 감소, 시간 역행 제거 |
| 로봇 카메라 | 4대 렌더 + 브리지 | 기본 off, 시나리오별 on | 주행 소비자 없음 | 렌더링 60회/s, 약 55 MB/s 제거 |
| `map_server` | 로봇별 4 | 전역 1 | 같은 맵 | 프로세스 3개 감소 |
| 전역 계획(A\*) | 로봇별 4, Python, 1 Hz 폴링 | `fleet_planner` 1, 빠른 코어, 이벤트 기반 | 정적층 공유, 계획은 요청형 | 코어 약 4개 상시 점유 → 요청 시에만 |
| 장애물 트래킹·no-go 레이어 | DWA 내장 ×4 + `fused_tracker`(amr1만) | `fleet_tracker` 1 (전 로봇 LiDAR + CCTV, 로봇 제외) | 전체 시야, 중복 제거 | DWA 틱의 주 비용 제거, CCTV가 주행에 실제로 기여 |
| 지역 제어 | DWA Python 약 5,900줄 ×4 | 로봇별 유지하되 얇게 (numpy PP 또는 C++) | 저지연 안전은 로봇별이 맞다 | 틱 50–75 ms → 수 ms (목표) |
| AMCL·EKF·RSP | 로봇별 별도 프로세스 | 로봇별, 컨테이너 1개로 묶음 | 로봇 고유 상태 | 프로세스·IPC 감소 |
| IMU·odom 공분산 주입 | 로봇별 Python 2개 | 컨테이너 안 C++ 컴포넌트 1개 또는 제거 | 단순 통과 처리 | Python 프로세스 8개 감소 |
| YOLO | (수동 시) 로봇별 | 공유 1개(GPU 배치, on-demand) 또는 미사용 | 주행이 소비하지 않음 | CUDA 컨텍스트 4 → 1 |
| 교착·우선순위 | F-2(사후 goal 덮어쓰기) + F-3(별도, 경로 QoS 불일치) | `fleet_manager` 1 + 계획 단계 예약 | 사후 개입은 진동·동결을 낳는다 | livelock 제거 |
| 기동 | 벽시계 `TimerAction` | 준비 신호 기반(lifecycle 전이·TF 가용성 대기) | 느린 서버에서도 순서 보장 | 미국지화 상태에서 주행 시작 방지 |
| 파라미터 | 노드 이름 키 YAML (네임스페이스에서 무시됨) | `/**/<node>:` 키 규약 + 시작 시 핵심 값 로그·검사 | 조용한 실패 방지 | 튜닝이 항상 적용됨 |

---

## 5. 인터페이스 계약 (초안)

기존 메시지를 최대한 재사용한다. 새 메시지 타입은 꼭 필요할 때만 `amr_msgs`에 추가한다.

| 인터페이스 | 타입 | QoS | 프레임 / 시간 | 비고 |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 기본 | 발행자 1개 | 전역 브리지 |
| `/map` | `nav_msgs/OccupancyGrid` | reliable, transient_local, depth 1 | `map` | 로봇 공통 |
| `/amrN/goal_pose` | `geometry_msgs/PoseStamped` | reliable | `map` | 기존 유지 |
| `/amrN/global_path` | `nav_msgs/Path` | reliable, **transient_local**, depth 1 | `map`, stamp = 계획 시작 pose 시각 | 현재 VOLATILE이다. 구독 측(F-3 등)과 함께 맞춘다. 시작점은 현재 로봇 투영점, 점 간격 0.1–0.2 m |
| 계획 요청(선택) | `nav2_msgs/action/ComputePathToPose` 재사용 | 액션 | `map` | 로봇 id는 네임스페이스로 구분 |
| `/fleet/obstacles` | `amr_msgs/TrackedObjectArray` (기존) | reliable, depth 1 | `map` | 로봇 자신 제외 |
| `/fleet/dynamic_layer` | `nav_msgs/OccupancyGrid` | reliable, transient_local, depth 1 | `map` | 해제(빈 레이어)도 발행한다. 계획에서는 높은 비용으로 쓴다 |
| `/fleet/priorities` | `std_msgs/Int32MultiArray` | latched | — | 기존 유지 |
| `/amrN/cmd_vel` | `geometry_msgs/Twist` | 기본 | — | watchdog: 0.3 s 무명령이면 정지 |
| 파라미터 파일 | `/**/<node_name>:` 키 | — | — | 노드 시작 시 핵심 값 1줄 로그 |
| 시간 | 모든 노드 `use_sim_time` | — | — | 판단 로직에 벽시계 금지, 틱 수 대신 초 단위 |

---

## 6. 이행 계획 — 단계마다 되돌릴 수 있게

각 단계는 **게이트(정량 기준)**를 통과해야 다음으로 넘어간다. 단일로봇 경로(N = 1)는 매 단계 함께 확인한다. 기간은 규모를 가늠하기 위한 감이다.

| 단계 | 내용 | 게이트 | 규모 |
|---|---|---|---|
| **Phase 0 — 계측과 계약** | A\* 계획 ms·확장 수 로그, DWA 틱 ms 로그, RTF 기록 스크립트, 파라미터 시작 로그, 검증 하네스(clean_restart, gate 스크립트) 저장소 이전 | 1·2·4대 기준선 수치 확보 | 1–2일 |
| **Phase 1 — 중복 제거 (동작 불변)** | YAML 키 수정, `/clock` 단일화, 로봇 카메라 off, map_server 1개, `num_robots` 인자, EGL 렌더링, mover 경량화 | RTF 개선 확인. F-1 GATE-GOAL 4/4·COLLISION 0 유지 | 2–3일 |
| **Phase 2 — 로봇별 계산 경량화** | A\* 사전계산 비용·격자 0.10 m·weighted·탐색 상한·워커 스레드. DWA numpy화·스캔 1회 처리·큐 depth 1·watchdog·멀티스레드 executor. deadlock_manager 버그 수정 | 계획 p95 ≤ 0.5 s, 틱 p95 ≤ 10 ms, `cmd_vel` ≥ 0.9 × 주기 × RTF | 1–2주 |
| **Phase 3 — 공유 서비스화** | `fleet_tracker`(전 LiDAR, 로봇 제외) 도입 → DWA·A\*가 소비. `fleet_planner`(정적층 공유, 요청형) 도입. 로봇별 component container | 동적 장애물 ON에서도 4대 GATE-GOAL PASS, `DYNAMIC_*` 대기 시간·우회 횟수 감소 | 2–3주 |
| **Phase 4 — 플릿 조율 고도화** | 계획 단계 예약(우선순위 기반 순차 계획 또는 시공간 예약), F-2·F-3를 `fleet_manager`로 통합, CCTV를 `fleet_tracker`에 합류 | 핀치 교차 시나리오(S-9 단일 병목 맵)에서 교착·동결 0 | 2주+ |

> 캡스톤 일정상 **Phase 0–2만으로도 측정 가능한 개선의 대부분**을 얻을 수 있다. Phase 3–4는 Phase 2 결과를 보고 결정한다.

---

## 7. 대안 비교

| 안 | 내용 | 장점 | 단점 |
|---|---|---|---|
| **A. Nav2 표준 스택 (로봇별, composition)** | 로봇별 `planner_server`(Smac/NavFn) + `controller_server`(RPP/MPPI) + costmap, `use_composition` | C++ 성능, 검증된 기능, 적은 개발량 | "A\*·DWA 직접 구현" 명세와 충돌. costmap은 여전히 로봇별 중복. 로봇 간 조율은 별도로 필요 |
| **B. 자체 구현 유지 + 공유 서비스 (본 문서)** | §3 구조 | 명세 유지, 중복 제거, 로봇 간 상호 인지 | 개발량이 가장 크다(fleet_planner·fleet_tracker) |
| **C. 자체 알고리즘을 Nav2 플러그인(C++)으로 이식** | 팀 A\*를 `nav2_core::GlobalPlanner`로, PP 제어기를 `nav2_core::Controller`로 구현 | 명세(핵심 로직 직접 구현) 유지 + Nav2 인프라(costmap layer, lifecycle, composition) 활용 | C++ 이식 비용. costmap 공유는 별도 설계 필요 |

**권장**: B를 목표로 Phase 0–2를 먼저 진행한다. Phase 3 착수 시점에 팀 역량과 일정을 보고 B와 C 중 하나를 고른다. 어느 쪽이든 Phase 0–2의 계측·경량화는 그대로 쓸모가 있다.

---

## 8. 리스크와 대응

| 리스크 | 대응 |
|---|---|
| 공유 서비스가 단일 장애점이 된다 (planner·tracker가 죽으면 전 로봇 영향) | 로봇별 제어기는 로컬 LiDAR 근접 정지를 항상 유지한다. 공유 서비스는 lifecycle로 감시하고, 장애 시 마지막 경로를 저속 추종한다 |
| 공유 트래커를 거치면 장애물 정보가 늦다 | 충돌 직전 판단은 로컬 LiDAR(로봇별), 회피·계획 판단은 공유 정보로 역할을 나눈다 |
| Python GIL 때문에 공유 플래너가 직렬화된다 | 탐색 코어를 C++/numba로 쓰거나 프로세스 풀을 쓴다. 정적층은 공유 메모리로 둔다 |
| 단일로봇 흐름 회귀 | N = 1 구성을 매 단계 게이트에 포함한다 |
| 일정 대비 개발량 | Phase 0–2 우선. 공유 서비스는 결과를 보고 결정한다 |

---

## 9. 개발 루프에 대한 처방

- **알고리즘 튜닝**(A\* 비용, PP 이득, 회피 규칙)은 가벼운 구성에서 빠르게 반복한다. 로봇 1대, 카메라·CCTV off, `dynamic_obstacles:=false`로 시작해 필요한 요소만 켠다. 순수 함수 단위테스트와 ROS 없는 2D 시뮬레이터(예: 팀의 AMR-Lite 2D 환경)도 이 단계에 쓸 수 있다.
- **4대 Gazebo 런은 통합 검증용**으로만 쓴다. 매 런마다 [runbook §5](multi-robot-runbook.md)의 지표를 기록하고, 단계별 게이트로 판정한다.
- 튜닝 값은 반드시 "적용 확인(`ros2 param get`) → 측정 → 비교" 순서로 다룬다.
