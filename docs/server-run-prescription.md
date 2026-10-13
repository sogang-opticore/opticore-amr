# 서버 구동 진단·처방전 — "느리고, 자율주행이 제대로 안 된다"

> **작성** 2026-10-13 · **대상** `dev` @ 447d08c (트리 = PR #27 머지 a4d5110) · **범위** `warehouse.launch.py` + `multi_robot.launch.py` 4대 구동
>
> **근거**
> - 코드 정적 분석
> - 순수 함수 단독 벤치마크: 로컬 Apple M2, CPython 3.11, ROS는 스텁으로 대체
> - 기존 런 기록: `docs/fleet_f1~f3`, `docs/cctv`
> - upstream 소스 확인: rclpy Humble 파라미터 매칭(직접 확인). Fortress 센서·렌더링 동작, tf2_ros 시간 역행 처리는 분석 과정에서 확인
>
> **서버 런 로그는 없다.** 그래서 진단마다 서버에서 확정하는 방법을 붙였다(§6 검진표).
> 이 문서는 진단과 처방만 담는다. 코드·설정은 바꾸지 않았다. 처방 중 코드 변경이 필요한 항목은 담당자 리뷰를 거쳐 별도 브랜치에서 진행한다.
>
> 관련 문서
> - 구조 처방: [`multi-robot-architecture.md`](multi-robot-architecture.md)
> - 실행·측정 절차: [`multi-robot-runbook.md`](multi-robot-runbook.md)
> - 서버 환경: [`environment-setup.md`](environment-setup.md)

---

## 0. 처방 요약

### 주호소

서버(RunPod GPU 파드, Docker, headless Ignition Fortress)에서 4대 멀티로봇 스택을 띄웠다. 크래시는 없었지만 두 가지 문제가 있었다.

- **S1 너무 느림.** 시뮬레이션과 반응이 모두 느리다. 과거 실측 RTF는 1대일 때 0.3–0.5, 4대일 때 약 0.33이다.
- **S2 자율주행 불량.** 이상한 우회, 멈춤, 제자리 회전, 도착 실패가 나타난다.

### 진단 (영향이 큰 순서)

| # | 진단 | 관련 증상 | 확신도 | 서버에서 확정하는 법 |
|---|---|---|---|---|
| **D1** | **4대 launch에서 튜닝 YAML(DWA·A\*·AMCL)이 통째로 무시된다.** YAML 최상위 키는 `dwa_planner:` 같은 노드 이름인데, 실제 노드는 `/amr1/dwa_planner`처럼 네임스페이스 아래에 있어 매칭되지 않는다. 그 결과 `robot_radius` 0.20, A\* clearance 1.25 같은 **예전 코드 기본값**으로 주행한다 | **S2**, S1 | 코드와 rclpy upstream 소스로 확인 | `ros2 param get /amr1/dwa_planner robot_radius`가 `0.2`면 확진 |
| **D2** | **로봇마다 복제된 순수 Python 계산.** A\*는 100만 셀을 Python으로 탐색해 계획 한 번에 수 초, 경로가 없으면 14 s가 걸린다. DWA는 틱당 50–75 ms(예산 50 ms), 막힘 시 0.35 s다. 이것이 4벌 돌아 CPU가 포화된다 | **S1**, S2 | 코드 + 로컬 벤치 | `ps`/`top`에서 astar·dwa가 각각 100% 근처 |
| **D3** | **단일 스레드 executor 안에서 계획과 TF 대기를 한다.** 그동안 TF·goal·스캔 처리가 밀리고, 오래된 입력으로 제어한다. 제어가 멈춰도 마지막 속도 명령이 계속 적용되며 watchdog이 없다 | **S2** | 코드 | `ros2 topic hz /amr1/cmd_vel`이 20 × RTF보다 낮음 |
| **D4** | **시뮬레이터 렌더링·브리지 과부하.** Fortress는 센서 렌더링이 끝날 때까지 물리 루프를 기다리므로 렌더 비용이 곧 RTF가 된다. 그런데 아무도 쓰지 않는 로봇 카메라 4대를 렌더링·브리지하고(약 55 MB/s), CCTV 4대를 항상 켜 두며, 그림자도 켜져 있다. 서버에서 렌더링이 CPU(llvmpipe)로 떨어졌을 가능성이 크다. 여기에 `/clock` 4중 브리지(물리 스텝마다), `-v4` 로그, 0.2 s마다 `ign` 프로세스를 새로 만드는 mover가 더해진다 | **S1** | 구조는 코드·upstream 소스로 확인. llvmpipe 여부는 미확인 | RTF 측정, `ogre2.log`의 `GL_RENDERER`, `ros2 topic info /clock` |
| **D5** | **로봇끼리 서로를 "모르는 장애물"로 보고 멈추고 통로를 닫는다.** DWA는 대부분 상태에서 v = 0으로 대기하고, no-go 레이어를 30 s~5 min 유지한다. 그러면 A\*가 그 구역을 벽으로 취급해 우회하거나 NO PATH를 반복한다 | **S2**, S1 | 코드 | `/amrN/dwa/status`에서 `DYNAMIC_*` 상태가 오래 지속됨 |
| **D6** | **플릿 노드 부작용.** livelock 영구 동결, 의도된 정지를 stuck으로 오판, 관리 중 goal 유실이 있다. F-3는 경로 구독 QoS가 맞지 않아 경로를 받지 못한다 | **S2** | 코드 | `/fleet/deadlock_status`에 `"livelock": true` |
| **D7** | **서버 환경 함정.** 기본 브랜치 `main`이 스텁이고, 워크스페이스·모델 경로가 하드코딩되어 있으며, 재빌드 누락으로 예전 설치본이 실행될 수 있다. 잔여 프로세스와 `/dev/shm` 잔재, 소프트웨어 렌더링 폴백, CPU 쿼터·스레드 과다 가능성도 있다 | S1, S2 | 기록 + 코드 | §6 검진표 |

> **한 줄 결론**
> - **느린 이유**: 로봇 수에 비례해 늘어나는 Python 계산(D2)과 시뮬레이터 부하(D4)가 CPU를 포화시킨다. 시뮬레이션이 느려지는 것으로 그 부하를 흡수하다 보니 RTF가 약 0.3에서 멈춘 것으로 보인다(추정, §2).
> - **자율주행이 안 된 이유**: 단일로봇에서 맞춘 튜닝이 4대 launch에서는 적용되지 않았다(D1). 여기에 지연(D3), 로봇 상호 차단(D5), 플릿 부작용(D6)이 겹쳤다.

### "YOLO·A\*·DWA를 로봇마다 띄워서 무거웠다"에 대하여

- **맞는 부분**: `multi_robot.launch.py`는 A\*·DWA를 포함해 로봇당 12개 노드를 띄운다(브리지, RSP, 정적 TF 2, 공분산 주입기 2, EKF, map_server, AMCL, lifecycle). 그중 A\*·DWA가 가장 무겁다(D2). 같은 맵 처리, `/clock`, TF 수신도 4번씩 반복된다.
- **YOLO는 다르다**: `dev`의 `multi_robot.launch.py`는 YOLO를 띄우지 않는다. 서버에서 따로 로봇별로 띄웠다면 다음 부하가 더해진다.
  - CUDA 컨텍스트 4개. 프로세스당 RAM 1–2.5 GB, VRAM 0.5–1 GB로 추정된다.
  - 프레임 스킵 없이 15 Hz 전 프레임을 추론한다. `inference_target_fps`는 선언되지 않아 효과가 없다.
  - debug 이미지를 항상 발행한다.
  - 카메라 렌더링이 강제된다.
- **결정적 사실**: 주행(A\*·DWA)은 YOLO 출력은 물론 `fused_tracker`·CCTV 출력도 구독하지 않는다. 주행 검증 런에서 이것들은 순수 부하다. CCTV 오버레이만 예외로, amr1의 A\* 레이어를 덮어쓴다.
- 그래서 처방은 "로봇별 스택을 가볍게"에서 끝나지 않는다. "로봇마다 둘 것과 한 번만 둘 것을 나누는" 구조 처방이 필요하다(§5.4, [`multi-robot-architecture.md`](multi-robot-architecture.md)).

### 처방 단계

| 단계 | 내용 | 변경 범위 | 기대 효과 |
|---|---|---|---|
| **1. 즉시** (§5.1) | 환경 점검, 클린 재시작, 주행과 무관한 기능 끄기, 정지·주행 상태 RTF 비교, YAML 적용 여부 확인 | 없음 (실행 방법만) | 원인 분리, 기준선 확보 |
| **2. 설정** (§5.2) | YAML 키 수정, GPU(EGL) 렌더링 보장, `/clock` 단일 브리지, 카메라 브리지 제거, A\* 재계획 축소, DWA 주기·동적 레이어 보존시간 축소, TF 대기 제거, 로그 레벨 | yaml·launch 수십 줄 | 튜닝이 실제로 적용되고, 렌더링과 CPU 부하가 크게 줄어든다 |
| **3. 코드 경량화** (§5.3) | A\* 벡터화·격자 조정·탐색 상한·워커 스레드, DWA numpy화·스캔 1회 처리·watchdog, 플릿 버그 수정 | 노드 내부 | 계획 0.5 s 이하, 틱 10 ms대 |
| **4. 구조** (§5.4) | 공유 서비스(맵·계획·트래킹·조율)와 로봇별 얇은 루프(위치추정·제어)로 재편, C++/composition | 아키텍처 | 로봇 수에 비례하던 비용 제거, 로봇 간 상호 인지 |

---

## 1. 전제와 근거

- **구동 구성**: T1 `warehouse.launch.py`(Gazebo 서버 + 동적 장애물 mover) + T2 `multi_robot.launch.py`(4대). F-3, CCTV, YOLO는 별도 launch다.
- **서버**: RunPod RTX A5000 / A40 파드, Docker 컨테이너. 시뮬레이터는 `ign gazebo -v4 -s -r`(headless)다.
- **벤치 수치의 의미**: 분석 중 각 노드의 순수 함수(A\* 탐색, DWA 정적 필터 등)를 ROS 없이 단독으로 실행해 잰 값이다(Apple M2, CPython 3.11). 서버(CPython 3.10, 공유 vCPU)에서는 **1.5–3배 느릴 것으로 추정**한다. 절대값보다 "예산 대비 몇 배인가"로 읽는다.
- **기존 기록에 남은 증거**
  - RTF: 1대(CCTV 런) 0.3–0.5 ([`cctv/PROGRESS.md`](cctv/PROGRESS.md)), 4대 약 0.33 ([`fleet_f1/PLAN.md`](fleet_f1/PLAN.md), [`fleet_f2/PLAN.md`](fleet_f2/PLAN.md)).
  - sim 50 Hz로 발행되는 odom 계열 토픽이 F-1·F-2 게이트에서 19–24 Hz로 측정되었다(벽시계 기준이면 측정 당시 RTF 약 0.4–0.5).
  - CPU 경합: `fleet_localization_init.py` 주석에 "lifecycle_manager(autostart)가 CPU 경합으로 change_state 응답 타임아웃 → amcl 'unconfigured'로 남는 문제(특히 마지막 스폰 amr4)"라고 적혀 있다. 이를 우회하려고 self-heal 노드를 만들었다.
  - 환경 함정(F-1): 예전 실행 파일이 그대로 실행됨(stale exe), 월드 2개 겹침("jump back in time"), `/dev/shm` FastRTPS 잔재로 DDS 저하.
  - 장거리 주행 문제(F-2): "prepos 긴 드라이브는 보행자/정적 클러터에 amr2가 길잃음 → `dynamic_obstacles:=false`로 청정화."

---

## 2. 부하 지도 — 4대 구동 시 실제로 떠 있는 것

| 구분 | 노드 | 언어 | 개수 | 비고 |
|---|---|---|---|---|
| 시뮬 | Ignition Gazebo 서버 | C++ (ruby 프로세스) | 1 | 물리 250 Hz(0.004 s 스텝). 렌더링 센서 12개(gpu_lidar 4, 로봇 카메라 4, CCTV 4) |
| 로봇별 | `ros_gz_bridge` | C++ | 4 | 각각 `/clock`과 카메라 이미지 포함 9개 토픽 |
| 로봇별 | RSP, static TF ×2, `map_server`, `amcl`, `lifecycle_manager`, `ekf_node` | C++ | 7 × 4 = 28 | `map_server` 4개가 같은 맵을 4번 로드 |
| 로봇별 | IMU·odom 공분산 주입기 | Python | 2 × 4 = 8 | IMU 100 Hz, odom 50 Hz를 그대로 통과 |
| 로봇별 | `astar_planner` | Python | 4 | 1207 × 840 격자, 1 Hz 주기 + 이벤트 재계획 |
| 로봇별 | `dwa_node.py` | Python | 4 | 20 Hz 제어, 약 5,900줄 상태기계 |
| 전역 | `fleet_localization_init`, `fused_tracker`, `deadlock_manager`, `dynamic_obstacle_mover` | Python | 4 | mover는 0.2 s(sim)마다 `ign service` 프로세스 생성 |
| 선택 | F-3, CCTV(검출+오버레이+브리지), YOLO | Python | 0~ | YOLO는 launch에 없음 |

- 상주 프로세스는 약 50개이고, 그중 Python이 약 20개다. `use_sim_time` 노드는 모두 `/clock`을 구독한다.
- **CPU 수지(추정)**
  - DWA: 틱 50–75 ms × 20 Hz × RTF. RTF가 1이면 로봇당 1–1.5코어다.
  - A\*: (계획 시간 × RTF)가 1 s를 넘는 순간부터 쉬지 않고 계획한다. 로봇당 약 1코어다.
  - 4대면 내비게이션 Python만으로 4–8코어가 필요하다. 여기에 Gazebo 물리·렌더링, AMCL·EKF ×4, 브리지 ×4가 더해진다.
  - 파드의 vCPU가 이보다 적으면 시뮬레이션이 느려지면서(RTF 저하) 부하가 줄어드는 지점에서 평형이 생긴다. **4대 RTF가 약 0.33에서 멈춘 것과 일치하는 설명이지만, 서버에서 확인이 필요한 추정이다.**

---

## 3. 진단 상세 ① — 왜 느린가 (S1)

### 3.1 A\*: 100만 셀 Python 탐색 (D2)

- **구현** (`astar_node.py:1177-1215`)
  - `heapq`와, tuple 셀을 키로 쓰는 dict/set으로 탐색한다.
  - 확장마다 이웃 8개에 대해 `_clearance_cost()`와 `_dynamic_side_preference_cost()`를 다시 계산한다.
  - 확장마다 numpy 스칼라 접근(`_is_free_cell`)이 최대 16회 일어난다.
  - 확장 수나 시간에 상한이 없다.
- **측정** (확장당 약 17 µs):

| 직선거리 | 실제 적용되는 코드 기본값 (D1) | YAML 값이 적용될 경우 |
|---|---|---|
| 5 m | 92k 확장, 1.6 s | 1k, 0.02 s |
| 21 m | 371k, 6.5 s | 50k, 0.8 s |
| 45 m | 245k, 4.5 s | 76k, 1.3 s |
| 55 m | 244k, 4.1 s | 177k, 2.8 s |
| 도달 불가 goal | **756,080(도달 가능 셀 전부), 14.2 s** | (같은 양상) |

  README가 정한 목표는 500 ms 이하다(`amr_navigation/README.md` §6). 서버에서는 40 m 계획에 8–14 s, NO PATH 판정에 30–40 s가 걸릴 것으로 추정한다.
- **확장이 폭증하는 이유**: 벽 근처 clearance 페널티가 step 비용을 최대 약 300배까지 키우는데, 휴리스틱은 octile 거리 그대로다. 그 결과 도달 가능 셀의 12–49%를 확장한다. `preferred_clearance`가 1.25(코드 기본값, D1)면 페널티 범위가 넓어져 더 나빠진다.
- **병목이 아닌 것**: distance transform(맵당 1회, scipy, 88 ms), 동적 레이어 합성(1 ms 미만), 스무딩(3–27 ms), 경로 전환 지표 계산(1–15 ms).
- **재계획 빈도** (로봇당, sim 초 기준)
  - 1 Hz 타이머(`replan_period: 1.0`).
  - DWA가 동적 상태 7종 중 하나면 1 s 쿨다운으로 추가 계획. DWA는 상태를 매 틱(20 Hz) 발행한다.
  - EMERGENCY / PATH_LOST / STOPPED_NEAR_WALL 등이면 2 s 쿨다운으로 추가 계획.
  - 새 goal마다 쿨다운 없이 즉시 계획. deadlock_manager는 goal을 최대 2 Hz로 재발행한다.
  - 도착 후에도 goal이 해제되지 않는다. 0.30 m 이내만 건너뛰는데 multi_robot의 `goal_tolerance`는 0.4다. 그래서 **정지한 로봇도 매 sim 초마다 재계획**한다.

### 3.2 DWA: 틱당 계산이 예산 초과 (D2)

- 노드에 numpy가 전혀 없다(`dwa_node.py` import부). 측정 결과(예산 50 ms/틱 @20 Hz):

| 작업 | 시간 | 비고 |
|---|---|---|
| 정적 벽 필터 (`occupancy_grid_has_static_obstacle_near`) | 47–71 ms | 점 약 720개 각각에 대해 25 × 25 셀 원판을 순수 Python 이중 루프로 검사 (`dwa_node.py:1868-1874`) |
| 같은 필터, 맵과 0.6 m 이상 어긋난 점이 많을 때 | 127–150 ms | AMCL 오차나 맵에 없는 물체가 있을 때 |
| 막힌 2 m 통로에서 회피 후보 탐색 | **343 ms** | 후보 약 96개 × arc 32점 × 전체 점 |
| 동적 레이어 재생성(발행마다, 활성 시 최대 1 Hz) | 98 ms + 블록당 약 16 ms | `data = [0] * (1207*840)` 리스트 생성 후 전체 `enumerate` (`dwa_node.py:3840, 3874`) |
| REJOIN + goal shortcut | 12 ms | |

- 평상시 틱만 50–75 ms라 20 Hz를 유지할 수 없다. 10 Hz 스캔을 20 Hz 틱이 두 번씩 다시 처리한다.
- 비용은 스캔 점 수, 필터 반경², 맵 크기에 비례해 커진다. `scan_range_max` 25 m와 `dynamic_static_filter_radius` 0.6 m가 그대로 비용이 된다.

### 3.3 실행 구조: 단일 스레드 executor (D3)

- 모든 Python 노드가 `rclpy.spin(node)`(기본 SingleThreadedExecutor)만 쓴다. 저장소 전체에 `MultiThreadedExecutor`나 callback group이 없다.
- **A\***
  - `_plan()`을 goal 콜백·status 콜백·타이머에서 직접 호출한다(`astar_node.py:614-615, 672, 847`).
  - TF listener도 같은 executor를 쓴다(`astar_node.py:578`).
  - 계획이 수 초 걸리는 동안 `/tf`, goal, status, 동적 레이어가 모두 대기한다. 계획이 끝나도 subscription당 메시지 하나씩만 처리되므로 TF가 계속 밀린다. 그 결과 오래된 시작 pose나 TF extrapolation 오류가 생긴다.
- **DWA**
  - 제어 틱 안에서 0.05 s 타임아웃 TF 조회를 여러 번 한다(`dwa_node.py:2796-2800`).
  - 새 경로 수신 콜백 안에서는 최대 1.0 s 블로킹 조회를 한다(`dwa_node.py:5561-5564`).
- `fused_tracker`도 정확한 stamp로 50 ms 대기 조회를 틱마다 2회 한다(`fused_tracker.py:416-417`).

### 3.4 시뮬레이터·브리지 (D4)

**렌더링 비용이 곧 RTF다.**
- Fortress의 Sensors 시스템은 진행 중인 렌더링이 끝날 때까지 시뮬레이션 루프를 기다린다(upstream 소스 확인).
- 렌더링 센서는 gpu_lidar 4개(10 Hz), 로봇 카메라 4대(15 Hz), 그리고 CCTV 4대(15 Hz, 구독되면 렌더)다. RTF 1을 유지하려면 sim 1초에 렌더링 100–160회를 소화해야 한다.
- 물리(4 ms 스텝 250회)에 0.1–0.25 s를 쓰고 나면 렌더 1회에 남는 시간은 약 5–7 ms다.
- GPU 렌더링(1회 1–4 ms)이면 RTF 0.7–1 수준이 나온다. CPU 렌더링(llvmpipe, 1회 15–50 ms)이면 **RTF 0.1–0.3**까지 떨어진다(추정).

| 항목 | 근거 | 영향 |
|---|---|---|
| **로봇 카메라 4대**(640×480, 15 Hz) 렌더링 + 브리지 | URDF `rgb_camera` `always_on`, `multi_robot.launch.py:227-228` 브리지 | ROS 쪽 구독자가 없다. 브리지가 ign 쪽에서 항상 구독하므로, 구독자 유무로 렌더링을 건너뛰는 최적화가 있어도 무력화된다. 초당 60회 렌더링과 약 55 MB/s 변환·복사가 생긴다 |
| **`/clock` 4중 브리지** | 로봇별 브리지마다 `'/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'` (`:221`) | Gazebo는 clock을 throttle 없이 물리 스텝마다(최대 250 Hz) 발행한다. 4배가 된 clock 메시지가 `/clock`을 구독하는 모든 노드(Python은 콜백이 같은 executor에서 실행됨)에 전달된다. 역순 도착에 따른 시간 역행은 §4.6 |
| **CCTV 카메라 4대** 상시 | `warehouse.world` CCTV 센서 `always_on`, 15 Hz, 640×480, 노이즈 | CCTV 파이프라인을 안 쓰는 런에서도 월드에 들어 있다. CCTV launch를 쓰면 4대를 모두 브리지하지만 실제 처리는 `corridor_1n` 1대뿐이다 |
| **그림자** | `sun_main` `cast_shadows true` | 카메라 렌더링마다 그림자 패스가 추가된다 |
| **`-v4` 로그** | `warehouse.launch.py:25` | 디버그 수준 로그 |
| **mover의 프로세스 생성** | `dynamic_obstacle_mover.py:166-177`: 0.2 s(sim)마다 `subprocess.Popen(['ign','service',...])` | 매번 ruby 기반 `ign` CLI와 transport 노드가 새로 뜨고 discovery가 반복된다 |
| 쓰지 않는 구독 | deadlock_manager가 odom 4개(50 Hz)를 구독하지만 값을 쓰지 않는다. Python TF listener 여러 개가 4대 전체 `/tf`를 역직렬화한다 | 작지만 누적된다 |

### 3.5 서버 환경 (D7, 느림 관련)

- **소프트웨어 렌더링 폴백 가능성 (느림의 1순위 의심).**
  - ign-rendering은 `DISPLAY`가 열리면 GLX를 쓰고, 디스플레이를 열 수 없을 때만 EGL headless로 전환한다(upstream 소스 확인). 그래서 `DISPLAY`가 Xvfb를 가리키면 Mesa **llvmpipe(CPU)**로 렌더링한다.
  - EGL로 가더라도 컨테이너에 NVIDIA EGL/GL 라이브러리가 있어야 GPU를 쓴다. 이는 `NVIDIA_DRIVER_CAPABILITIES`에 `graphics`가 있어야 하고, 미설정 시 기본값은 `compute,utility`다.
  - 개인 Dockerfile은 `ubuntu:22.04` 기반에 xvfb를 설치하고, `warehouse.launch.py`는 `--headless-rendering`을 쓰지 않는다. 즉 **CPU 렌더링 조건이 갖춰져 있다.**
  - 2026-06 CCTV 런은 "GPU 렌더 정상(util>0, libEGL 정상)"으로 기록되었다. 하지만 GPU util>0은 다른 GPU 작업(YOLO 프레임 테스트 등)으로도 나올 수 있어 렌더러를 직접 확인한 근거는 아니다. 파드나 이미지가 바뀌면 `ogre2.log`의 `GL_RENDERER`로 반드시 다시 확인한다([`environment-setup.md`](environment-setup.md) §6).
- **정리 alias의 함정.** 개인 Dockerfile의 `killros`는 `pkill -f ign && pkill -f foxglove && pkill -f ros2 && pkill Xvfb`처럼 `&&`로 이어져 있다. 중간에 대상이 없는 명령(예: foxglove 미실행)이 있으면 **뒤의 kill이 실행되지 않는다.** 이전 런의 브리지·DWA가 살아남아 `/clock`·`cmd_vel` 발행자가 중복될 수 있다. `;`로 잇거나 [runbook §1](multi-robot-runbook.md) 스크립트를 쓴다.
- **CPU 쿼터와 스레드 과다.** 컨테이너에서는 `nproc`가 호스트 코어 수를 보고할 수 있다. 그러면 torch·BLAS·OpenCV가 과다하게 스레드를 만든다(YOLO를 띄운 경우 특히 큼).
- **잔여 프로세스와 `/dev/shm` 잔재.** 이전 런의 `ros2 launch` 부모가 남으면 월드가 2개 겹친다. `pkill -9`를 반복하면 FastRTPS 공유메모리가 쌓여 DDS가 느려진다(F-1에서 실제 발생).

### 3.6 YOLO를 로봇별로 띄운 경우

| 항목 | 내용 |
|---|---|
| 구성 | `perception.launch.py`는 네임스페이스·remap이 없는 단일 노드다. 입력 `/camera`는 멀티로봇에 존재하지 않는다(`/amrN/camera`). 로봇별로 띄우려면 별도 remap이 필요하다 |
| 처리량 | 프레임 스킵 없이 수신 프레임마다 추론한다(`yolo_detector.py:97`). debug 이미지도 항상 발행한다(`:148-150`) |
| 비용 추정 | CUDA는 프로세스당 RAM 1–2.5 GB, VRAM 0.5–1 GB, CPU 전처리 5–10 ms/프레임이다. CPU로 돌리면 50–150 ms/프레임이라 4대 × 15 Hz면 어느 파드든 전 코어를 점유한다 |
| 주행 기여 | 없다. 주행 노드 중 `/perception/*`를 구독하는 것이 없다 |

---

## 4. 진단 상세 ② — 왜 자율주행이 안 되나 (S2)

### 4.1 ★ 튜닝 YAML이 4대 launch에서 적용되지 않는다 (D1)

- **원리**
  - ROS 2 파라미터 파일의 최상위 키는 노드의 **전체 이름**과 정규식으로 매칭된다.
  - rclpy Humble `_populate_node_parameters_from_rcl_params`는 키 `dwa_planner`를 `/dwa_planner`로 바꾼 뒤, 노드 FQN `/amr1/dwa_planner`와 전체 일치(`regex_match`)를 검사한다. 일치하지 않으면 건너뛴다. rclcpp(AMCL)도 같은 규칙이다.
  - `multi_robot.launch.py`는 A\*·DWA·AMCL을 `namespace=rn`으로 띄우고 YAML 파일과 인라인 dict를 함께 넘긴다. 인라인 dict는 launch_ros가 `/**` 키로 쓰므로 적용되지만, **YAML 파일 쪽은 통째로 무시된다**.
  - 같은 launch의 EKF는 노드 이름에 맞춘 임시 YAML을 생성하므로 정상이다(`multi_robot.launch.py:59-64`).
- **실제로 적용되는 값** (YAML과 코드 기본값이 다른 항목만)

| 노드 | 파라미터 | YAML (의도) | 4대 launch에서 실제 | 영향 |
|---|---|---|---|---|
| DWA | `robot_radius` | 0.36 | **0.20** | 풋프린트를 0.16 m 작게 본다. eaeca45("풋프린트 과소로 인한 동적장애물 접촉 수정")가 무효가 된다 |
| DWA | `predict_horizon` | 0.5 | 1.0 | 충돌 검사 적분 구간이 2배다 |
| DWA | `weight_heading` / `weight_path_tangent` / `weight_velocity` | 0.3 / 2.0 / 0.3 | 0.6 / 0.8 / 0.2 | 14차 이후 미사용이라 영향 없음(yaml 주석) |
| A\* | `preferred_clearance` | 0.50 | **1.25** | 2 m 핀치 회피가 되살아난다. c459871의 C4 수정이 무효가 되고, 탐색이 폭증한다(§3.1) |
| A\* | `wall_avoid_clearance` | 0.55 | 1.05 | 같은 문제 |
| A\* | `smoothing_min_clearance` | 0.45 | 0.90 | 스무딩 결과가 달라진다 |
| AMCL | `update_min_d` / `update_min_a` | 0.05 / 0.05 | 0.25 / 0.2 (Nav2 기본) | 보정이 드물어진다 |
| AMCL | `recovery_alpha_slow/fast` | 0.001 / 0.1 | 0.0 / 0.0 | **오정합 시 복구용 랜덤 파티클 주입이 꺼진다** |
| AMCL | `max_particles`, `laser_max_range`, `set_initial_pose` | 3000, 12.0, true | 2000, 100.0, false | |

- **왜 지금까지 드러나지 않았나**: 단일로봇 launch(`nav_full`, `astar_dwa`)는 네임스페이스가 없어 YAML이 정상 적용된다. 그래서 **단일로봇에서 맞춘 튜닝이 4대에서는 사라지는 구조**다. 0.36 반경 수정이나 핀치 clearance 수정처럼 YAML만 바꾼 개선은 멀티로봇과 CCTV 검증 런에 반영되지 않았다. CCTV 검증 런은 `make_robot_nodes`로 같은 네임스페이스 구성을 썼다. c459871 이후 C4 재검증 기록이 없는 것도 이 문제와 연관되어 있을 수 있다.
- **확정법**: `ros2 param get /amr1/dwa_planner robot_radius`(0.2면 확진), `/amr1/astar_planner preferred_clearance`(1.25), `/amr1/amcl update_min_d`(0.25).

### 4.2 지연이 주행을 망가뜨리는 방식 (D2 + D3)

| 현상 | 근거 | 결과 |
|---|---|---|
| 오래된 입력으로 제어 | scan·odom 큐 depth 10(`dwa_node.py:2190-2215`). rclpy는 준비된 subscription마다 1개씩만 처리한다. 메시지 stamp를 확인하지 않고 scan 타임아웃도 없다 | 틱이 밀리면 가장 오래된 샘플(odom 약 0.2 s, scan 최대 약 1 s)로 판단한다. LiDAR가 끊기면 마지막 스캔을 계속 쓴다 |
| watchdog 없음 | Ignition DiffDrive는 명령 타임아웃이 없다 | 긴 틱(0.35 s)이나 계획 중에도 로봇이 마지막 Twist로 계속 움직인다 |
| 가속 제한 왜곡 | `dv_max = a_max * period`를 측정 속도 기준으로 적용한다(`dwa_node.py:4714, 5002`) | 틱을 건너뛰면 실효 가속이 줄어 기어가거나 제동이 늦어진다 |
| 틱 수로 센 타이머 | stuck, ALIGN 진입, 동적 진입·해제 카운터가 틱 단위다 | 틱이 밀리면 실제 시간이 늘어나 복구가 늦어진다 |
| TF 실패 시 벽을 동적 장애물로 판단 | 정적 필터 TF 실패 시 전체 점을 dynamic으로 반환(`:2850-2852`). 위험도 1.0이면 제자리 회전(`:3373-3375`) | `map→amrN/odom_filtered`가 끊기면 **벽이 동적 장애물이 되어** 멈추거나 제자리에서 돈다 |
| A\*가 오래된 pose에서 계획 | 경로 stamp는 발행 시각(`astar_node.py:1839`). DWA는 경로가 현재 위치에서 1.0 m 넘게 떨어지면 거부한다(`dwa_node.py:5745-5757`) | 경로를 따라 전진한 경우는 괜찮다. 옆으로 비킨 뒤(회피·REJOIN·복구)에는 새 경로가 거부되어 DYNAMIC_BLOCKED가 되고, 다시 긴 계획이 시작되는 고리가 생긴다 |
| odom 프레임 경로 캐시 | 경로를 받을 때 한 번만 map→odom_filtered로 변환한다(`:2632-2661`). A\*가 같은 경로를 유지하면 재발행하지 않는다 | AMCL 보정이 제어기에 반영되지 않은 채 경로 전체를 주행할 수 있다 |
| 스캔 재처리로 생기는 유령 속도 | 같은 스캔을 매 틱 최신 pose로 다시 처리한다(`:2883, 2910`) | 정지 물체에 ±0.3 m/s(회전 중 ±0.9 m/s)의 가짜 속도가 생겨 CROSSING/APPROACHING으로 오분류된다 |

### 4.3 로봇끼리 서로 막는 구조 (D5)

- **판정**: 정적 필터를 통과한 LiDAR 점이 앞쪽 3.2 m 경로의 0.5 m 폭 안에 2개 이상 있으면 "막힘"이다. 다른 AMR도 여기에 걸린다. A\*·DWA는 다른 로봇의 pose·경로를 모른다.
- **반응**
  - 국소 우회는 상대가 STOPPED, APPROACHING, UNKNOWN일 때만 시도한다(`dwa_node.py:4237-4238`). 그 밖의 경우는 `v=0` 대기다(`:4742-4754`). **앞 로봇이 멀어지는 중이어도 뒤 로봇은 멈춘다.**
  - 2 m 통로에서는 측면 오프셋 후보가 모두 0.55 m 여유를 만족하지 못해 대기만 한다.
- **no-go 레이어 보존**
  - 블록은 마지막 관측 후 30 s 유지된다(`dynamic_layer_min_hold_sec`). 시야 밖 블록은 TTL 300 s까지 남고, 궤적(trail)은 60 s다.
  - 빈 레이어는 발행하지 않는다(`:3879-3887`). 그래서 A\*는 마지막 레이어를 35 s 타임아웃까지 쓴다.
  - A\*는 동적 셀을 벽(hard block)으로 취급한다(`astar_node.py:788-789`).
  - 결과: **한 번 본 로봇 때문에 2 m 통로가 30 s~5 min 닫힌다.** goal이 그 너머에 있으면 NO PATH 전역 탐색(§3.1의 14 s)이 트리거마다 반복된다.
- **정면 조우**: 두 로봇이 서로 `reverse_away`를 선택하면 진동할 수 있다(코드상 가능성).
- F-2 기록에서도 "A\*가 상대 로봇을 미리 우회해 2.66 m 안으로 접근조차 안 함"이 관찰되었다.

### 4.4 플릿 노드 부작용 (D6)

| 문제 | 근거 | 결과 |
|---|---|---|
| **livelock 영구 동결** | `flip_count`가 런 전체에서 누적되고 리셋이 없다(`deadlock_manager.py:68-69, 349-351`). livelock이면 hold 위치만 계속 재발행한다(`:378-381`) | HOLD→RETREAT가 런 전체에서 4번째 일어나면 그 로봇은 **런이 끝날 때까지 정지**한다. 새 goal도 0.5 s마다 덮어쓰인다 |
| stuck 오판 | 4 s(sim) 동안 변위 < 0.10 m면 stuck이다(`:255-260`). 구독 중인 `lin_speed`·`dwa_status`는 판정에 쓰지 않는다 | 제자리 회전, 보행자 대기, goal 경계 정지도 stuck이 된다. 앞에 고우선 AMR이 2 m 이내에 있으면 HOLD→RETREAT 사이클이 시작된다 |
| 관리 중 goal 유실 | NORMAL에서만 mission goal을 채택한다(`:182`) | HOLD/RETREAT/RESUMING 중 사용자 goal은 무시되고 덮어쓰인다 |
| RETREAT 재계획 폭주 | RETREAT goal을 현재 yaw로 재발행한다(`:404`). yaw가 0.1 rad 이상 바뀌면 A\* dedup을 통과한다 | 최대 2 Hz로 전체 A\* 재계획이 일어난다 |
| F-3 경로 미수신 | `f3_priority_node`는 `/amrN/global_path`를 TRANSIENT_LOCAL로 구독하고, A\*는 VOLATILE로 발행한다 | DDS 호환 불가로 경로가 오지 않는다. 로봇이 정지 상태로 투영되어 선제 우선순위 예측이 사실상 동작하지 않는다 |

### 4.5 경로·goal 처리 세부

- **goal snap이 너무 멀리 옮긴다**
  - A\*는 막힌 goal을 BFS로 최대 약 0.85 m 옮긴다. 이때 "가장 가까운 셀"이 아니라 "clearance가 가장 큰 셀"(`min(clearance, preferred_clearance)`)을 고른다(`astar_node.py:1346-1348`).
  - 코드 기본값 1.25가 적용되면(D1) 랙 근처 goal 400개 실험에서 이동 거리 중앙값이 0.62 m였고, **30%가 0.75 m를 넘었다.**
  - DWA는 경로 끝점이 최신 goal에서 0.75 m 넘게 떨어지면 `stale /global_path ignored because goal endpoint differs`로 경로를 버린다(`dwa_node.py:5646-5651, 5719-5724`). 결과적으로 **그 goal로는 영영 출발하지 못하고 A\*만 재계획을 반복한다.** 14.5%는 빈 셀을 찾지 못해 빈 경로가 된다.
- **경로 점 중복**: 셀 1,051개짜리 경로가 5,251 pose로 발행된다(80%가 중복, `astar_node.py:1748-1761`). DWA가 이 점들을 전부 Python으로 변환한다.
- **도착 후 재계획**: §3.1 참고. 정지한 로봇이 CPU를 계속 쓴다.

### 4.6 위치추정·시간

- **`/clock` 역행(추정, 로그 확인 필요)**
  - 브리지 4개가 같은 clock tick을 각자 재발행하므로, 구독자가 "B의 t₂ 다음에 C의 t₁"처럼 역순으로 받을 수 있다.
  - tf2_ros C++ Buffer(AMCL·EKF 등)는 시간이 조금이라도 역행하면 "Detected jump back in time. Clearing TF buffer."를 남기고 버퍼를 비운다. 그 결과 TF 조회가 간헐적으로 실패한다.
  - F-1에서는 월드 2개가 겹쳤을 때 같은 경고를 겪었다.
- **AMCL 기본값으로 동작**(§4.1). 복구 주입이 꺼져 있어 반복 구조의 랙 통로에서 오정합되면 스스로 회복하기 어렵다.
- **기동 순서가 벽시계 타이머에 의존**: launch `TimerAction`(10 s 간격, A\*/DWA +20 s, loc_init +40 s …)은 벽시계 기준이다. CPU 경합으로 lifecycle 전이가 타임아웃되는 것을 loc_init이 사후 복구하는 구조다. 느린 서버에서는 "준비 전에 주행 노드 기동"이 일어날 수 있다.
  - loc_init 전에는 `map → amrN/...` TF가 없다. 이 사이에 보낸 goal은 A\*가 1 Hz로 재시도하므로 늦어질 뿐이다. 다만 `/amrN/map`을 아직 받지 못한 시점의 goal은 버려진다.
  - initialpose는 한 번만 발행된다. AMCL이 나중에 재구성되면(아래 bond 리셋) 다시 받을 길이 없다.
  - **lifecycle bond 타임아웃(추정)**: Nav2 lifecycle_manager의 bond는 벽시계 4 s 타임아웃이다. CPU가 멈칫하면 "CRITICAL FAILURE: SERVER amcl IS DOWN" 후 재기동될 수 있다. D1 때문에 `set_initial_pose`도 꺼져 있어, 재기동된 AMCL은 초기 pose 없이 미국지화 상태로 남는다.
- **EKF 입력 일부 누락(정확도 문제, 주원인 아님)**
  - odom 주입기는 pose frame을 `amrN/odom`으로 쓰는데, 이 프레임은 TF 트리에 없다(DiffDrive TF는 브리지하지 않는 `tf_gz_diff`로 감).
  - IMU 메시지 frame(`amrN/base_footprint/imu_sensor`로 추정)에는 lidar·camera와 달리 정적 TF 보정이 없다.
  - robot_localization은 변환에 실패한 측정을 조용히 버린다. 결과적으로 EKF는 사실상 휠 속도(vx, vyaw)만 쓰는 dead-reckoning이다. 무미끄럼 시뮬이라 당장 치명적이지는 않다. F-1 EXPLORE에서도 "injector frame_id ≠ EKF world_frame"으로 지적된 바 있다.

### 4.7 시나리오·맵 함정

- **person_1이 벽을 통과한다**: 경로 (14, 18) → (17, 18)이 핀치 동쪽 벽(x 15.5~15.75)을 관통한다(`dynamic_obstacle_mover.py:123`). 2 m 핀치 안의 보행자 + no-go 보존(§4.3) 조합이면 핀치가 장시간 닫힌다.
- **모델 경로 하드코딩**
  - `warehouse.launch.py:7-8`은 `IGN_GAZEBO_RESOURCE_PATH`를 `/workspace/ros2_ws/src/amr_bringup/models`로 덮어쓰고, `models/`는 install 대상이 아니다.
  - 소스가 다른 경로에 있으면 `model://` include가 실패한다. Fortress 서버는 SDF 오류가 있으면 월드를 아예 띄우지 않는다.
  - 이번 런은 시뮬레이션이 돌았으므로 해당하지 않았을 것으로 본다. 다만 새 서버로 옮기면 바로 걸린다.
  - `standing_person` 메시는 첫 로드 때 Fuel URL에서 받아오므로, 인터넷이 막힌 서버에서는 첫 기동이 지연되거나 실패할 수 있다.
- **맵 회색 픽셀**: `free_thresh: 0.25` 때문에 벽 바깥의 회색(205) 띠가 free로 로드된다. 이 섬에 goal을 찍으면 NO PATH 전역 탐색이 일어난다.
- **CCTV 오버레이와 DWA 레이어 충돌**: 같은 토픽에 두 발행자가 있다. A\*는 마지막 메시지만 쓴다(`astar_node.py:741`).

---

## 5. 처방

### 5.0 원칙 (재발 방지)

1. **측정 없이 튜닝하지 않는다.** 런마다 RTF, `cmd_vel` Hz, 계획 시간, 프로세스 CPU를 기록한다([runbook §5](multi-robot-runbook.md)).
2. **파라미터 변경은 적용 확인까지가 한 단위다.** `ros2 param get`으로 실제 값을 확인한다(D1 재발 방지).
3. **1대 → 2대 → 4대 순서로 늘린다.** 각 단계의 통과 기준은 RTF ≥ 0.8, `cmd_vel` ≥ 0.9 × 20 × RTF Hz, 계획 ≤ 0.5 s다.
4. **한 번에 하나만 바꾸고 A/B로 비교한다.**
5. **주행 검증 런에서는 주행에 쓰이지 않는 기능을 끈다.** YOLO, CCTV, F-3, 로봇 카메라, (기준선에서는) 동적 장애물과 deadlock_manager가 해당한다.

### 5.1 1단계 — 오늘 바로 (코드·설정 변경 없음)

| 순서 | 할 일 | 방법 | 판정 |
|---|---|---|---|
| 1 | 올바른 코드가 도는지 확인 | `dev` 브랜치, `/workspace/ros2_ws` 링크, 재빌드, `diff -r src/... install/...` ([runbook §0](multi-robot-runbook.md)) | stale 설치본 배제 |
| 2 | 클린 재시작 | [runbook §1](multi-robot-runbook.md) 스크립트 | 잔여 프로세스·shm 0 |
| 3 | GPU 렌더링 확인 | `echo $DISPLAY $NVIDIA_DRIVER_CAPABILITIES`, `nvidia-smi`, `ogre2.log`의 `GL_RENDERER` ([environment §6](environment-setup.md)) | llvmpipe면 이것부터 해결한다. Gazebo 셸에서 `unset DISPLAY`를 하고, 지원되면 `ign gazebo ... --headless-rendering`으로 직접 실행해 비교 |
| 4 | 스레드 과다 방지 | `export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1` | |
| 5 | 기준선 런 | `dynamic_obstacles:=false`, `enable_deadlock_manager:=false`, YOLO·CCTV·F-3 미실행 | |
| 6 | **정지 RTF와 주행 RTF 비교** | goal 없이 1분간 RTF를 기록한 뒤 4대에 goal을 주고 다시 기록 | 정지 상태부터 낮으면 시뮬레이터·센서(D4·렌더링) 문제다. goal 후 급락하면 계획기·제어기(D2) 문제다 |
| 7 | D1 확진 | `ros2 param get /amr1/dwa_planner robot_radius` 등 3개 | 0.2 / 1.25 / 0.25면 2단계 첫 항목 |
| 8 | 기능을 하나씩 켜며 비교 | 동적 장애물 → deadlock_manager → (필요 시) F-3·CCTV | 어느 기능에서 주행이 무너지는지 확인 |

> 현재 launch는 로봇 수가 4대로 고정(`ROBOTS` 리스트)이라 1·2대 측정을 하려면 2단계의 `num_robots` 인자가 필요하다.

### 5.2 2단계 — 설정·launch 최소 수정 (담당자 리뷰 후 1–2일)

| # | 대상 파일 | 변경 | 효과 | 주의 |
|---|---|---|---|---|
| 2-1 | `amr_navigation/config/dwa_params.yaml`, `astar_params.yaml`, `amr_slam/config/amcl_params.yaml` | 최상위 키를 `dwa_planner:` → `/**/dwa_planner:`(A\*: `/**/astar_planner:`, AMCL: `/**/amcl:`)로 변경. 네임스페이스 유무와 관계없이 매칭된다 | **D1 해소.** 의도한 튜닝이 4대에 적용된다 | AMCL yaml의 `set_initial_pose: true` + (0,0)이 4대 모두에 적용된다. 인라인 dict에 로봇별 `initial_pose`(= world − (3.0, 15.0))를 넣거나(2-13) `set_initial_pose: False`로 둔다. `update_min_d/a 0.05`는 AMCL CPU를 늘리니 4대에서는 0.1 정도도 검토한다. 변경 후 `ros2 param get`으로 확인한다 |
| 2-2 | `multi_robot.launch.py`, `warehouse.launch.py` | 로봇별 브리지에서 `/clock`을 빼고 전역 브리지 1개에서만 `/clock`을 브리지한다 | `/clock` 트래픽 75% 감소, 시간 역행 원천 제거 | |
| 2-3 | `multi_robot.launch.py` (+ URDF) | 로봇별 브리지에서 `camera`, `camera_info`를 제거한다. 장기적으로는 xacro 인자 `enable_camera`로 카메라 센서 자체를 끈다 | 렌더링 60회/s, 약 55 MB/s 제거 | 카메라가 필요한 실험(YOLO)만 켠다 |
| 2-4 | `warehouse.world` / `warehouse.launch.py` | CCTV 없는 런용 월드 변형(또는 CCTV `update_rate` 5 Hz), `sun_main` `cast_shadows false`, `-v4` → `-v1` | 렌더링·로그 부하 감소 | CCTV 데모에서는 원복 |
| 2-5 | `astar_params.yaml` (2-1 적용 후) | `replan_period: 0`(이벤트 기반) 또는 3 s 이상. `status_replan_states`에서 동적 7종과 `STOPPED_NEAR_WALL`을 빼거나 쿨다운을 3–5 s로 늘린다 | A\* 상시 점유 해소 | 동적 회피는 DWA 국소 우회와 레이어로 처리 |
| 2-6 | `dwa_params.yaml` (2-1 적용 후) | 4대 런에서 `control_rate: 10.0`, `scan_range_max: 8.0`, `dynamic_static_filter_radius: 0.30`, `dynamic_static_filter_tf_timeout: 0.0`, `path_tf_timeout: 0.05` 이하 | DWA 틱 비용 수분의 1, 블로킹 제거 | 10 Hz에서도 `v_max` 1.5 m/s 제동거리가 괜찮은지 확인 |
| 2-7 | `dwa_params.yaml` | 동적 레이어 보존 축소: `dynamic_layer_min_hold_sec` 30 → 3–5, `dynamic_layer_ttl_sec` 300 → 15–30, `dynamic_layer_trail_ttl_sec` 60 → 10. 플릿 기준선에서는 `dynamic_layer_enabled: false`도 비교한다 | 로봇 한 번 관측으로 통로가 수 분간 닫히는 현상 완화(D5) | 보행자 시나리오 회귀 확인 |
| 2-8 | `multi_robot.launch.py` | `num_robots` 인자(1–4) 추가 | 1→2→4대 단계 측정 가능 | `fleet_localization_init`·`deadlock_manager`의 `robots` 목록도 함께 줄인다 |
| 2-9 | `f3_priority_node.py` | `/amrN/global_path` 구독 QoS를 VOLATILE로 변경(1줄) | F-3 경로 입력 복구 | |
| 2-10 | `deadlock_manager` 파라미터 | 근본 수정(3단계) 전 임시 조치: 기준선에서는 끈다. 켜야 하면 `livelock_max_n`을 크게 잡아 영구 동결을 피한다 | 영구 정지 회피 | 진동 가능성은 남는다 |
| 2-11 | `warehouse.launch.py`, `amr_bringup/CMakeLists.txt` | 모델 경로를 패키지 share에서 계산해 기존 값 앞에 덧붙이고(prepend), `models/`를 install에 추가. `standing_person` 메시 URI를 저장소의 `meshes/standing.dae`로 변경 | 서버 경로·인터넷과 무관하게 보행자·지게차 로드 | |
| 2-12 | `warehouse.launch.py` + 실행 환경 | Gazebo 실행 셸에서 `DISPLAY`를 해제하고 `--headless-rendering`(EGL)을 사용한다. 파드는 `NVIDIA_DRIVER_CAPABILITIES=all`(또는 `graphics` 포함)로 띄운다 | CPU 렌더링(llvmpipe)이었다면 **가장 큰 RTF 개선** | `ogre2.log`의 `GL_RENDERER`로 확인 |
| 2-13 | `multi_robot.launch.py` (lifecycle_manager 인라인) | 시뮬 런에서는 `bond_timeout: 0.0`(bond 비활성)을 검토. AMCL 인라인 dict에 로봇별 `initial_pose`를 넣어 재기동돼도 위치를 잃지 않게 한다 | CPU 멈칫 시 AMCL 리셋으로 국지화를 잃는 위험 제거 | |

### 5.3 3단계 — 코드 경량화 (1–2주)

**A\*** (`astar_node.py`)

| 처방 | 효과 |
|---|---|
| 정적 비용(clearance·wall-avoid)을 셀 배열로 사전계산하고 flat index로 탐색 | 약 2배(추정) |
| 0.10 m 계획 격자 | 3.5–7배(실측). 경로 길이 증가는 최대 0.4 m |
| Weighted A\* (w = 1.5) | 실측 확장 245k → 22k, 244k → 5k. 경로 증가는 최대 1.4 m. 우회가 많은 21 m 경로에서는 효과가 작다 |
| 확장 수·시간 상한, 연결 성분(`ndimage.label`)으로 도달성 사전검사 | **NO PATH 14 s → 즉시**. 동적 셀은 hard block 대신 높은 비용으로 |
| 계획을 워커 스레드로 분리(최신 요청만 유지, 새 goal이면 취소) + `TransformListener(..., spin_thread=True)` + 계획 ms·확장 수 로그 | executor가 막히지 않는다(D3) |
| 도착 시 goal 해제, 경로 중복 제거와 0.1–0.2 m 리샘플, 시작점을 현재 투영점으로 다듬기, stamp를 pose 시각으로 | 불필요한 재계획·변환 비용 제거, 오래된 경로 거부 감소 |
| goal snap을 최근접 free 셀로 바꾸고 이동 상한을 DWA 수용 반경(0.75 m) 미만으로 | 출발하지 못하는 goal 제거 |
| (장기) C++·numba 탐색 또는 공유 플래너(§5.4) | 50–100배(추정) |

**DWA** (`dwa_node.py`)

| 처방 | 효과 |
|---|---|
| numpy 도입. 정적 필터는 "맵 1회 distance transform → 점마다 O(1) 조회"로 바꾼다 | 47–150 ms → 1 ms 미만(예상) |
| 스캔은 수신 시 1회만, 스캔 시각의 pose로 처리 | 중복 연산과 유령 속도 제거 |
| 동적 레이어는 numpy 배열로 bbox만 칠하고, 빈 레이어도 발행(해제)한다. 트래킹·레이어는 별도 노드로(§5.4) | 98 ms 재생성 제거, 통로 해제 지연 제거 |
| 회피 후보 clearance를 KD-tree나 거리 격자로 계산 | 343 ms → 수 ms(예상) |
| 큐 depth 1, 메시지 나이 검사(scan이 0.3 s보다 오래되면 정지), `cmd_vel` watchdog(소형 C++ 노드 또는 타임아웃 mux) | 오래된 입력과 폭주 제거 |
| `MultiThreadedExecutor` + callback group 분리(수신과 제어), 틱 수 대신 실측 dt·초 단위 타이머 | 지연에 강한 제어 |
| 블로킹 TF 조회를 틱당 1회 비블로킹 캐시로 | TF 실패 시 벽을 동적으로 보는 문제 제거 |
| 다른 AMR은 공유 pose로 식별하고, 멀어지는 대상에는 정지 대신 속도를 맞춘다 | 앞 로봇 때문에 멈추는 현상 해소 |

**플릿·인프라**

- `deadlock_manager`
  - 진행이 확인되면 `flip_count`를 리셋한다.
  - stuck 판정에 `dwa_status`·`lin_speed`를 반영하고, TF stamp로 오래된 pose를 거른다.
  - 관리 중에도 새 goal을 수용하고, RETREAT yaw는 진입 시 한 번만 고정한다.
  - F-2 유닛테스트(`f2_unit_test.py`)를 저장소에 커밋한다.
- dynamic_obstacle_mover
  - person_1 경로를 수정해 벽을 관통하지 않게 한다.
  - `ign service` 프로세스 생성 대신 상주 클라이언트나 Gazebo actor 궤적을 쓴다.
- 서버 하네스(`clean_restart.sh`, gate 스크립트)를 저장소 `scripts/`로 옮긴다.
- 노드 시작 시 핵심 파라미터를 한 줄 로그로 남겨 적용 여부를 바로 볼 수 있게 한다.

### 5.4 4단계 — 구조 처방 (요약)

로봇 수 N에 비례해 늘어나는 비용을 구조적으로 끊는다. 상세는 [`multi-robot-architecture.md`](multi-robot-architecture.md)에 있다.

- **로봇별로 남길 것**
  - 센서 I/O(브리지), 위치추정(EKF·AMCL)
  - **얇은** 지역 제어기(경로 추종 + 로컬 LiDAR 안전 정지)
  - `cmd_vel` watchdog
  - 이것들을 로봇당 component container 1개로 묶는다.
- **한 번만 둘 것 (fleet 서비스)**
  - `/clock` 브리지, `map_server`
  - **fleet planner**: 정적 비용층을 한 번 계산해 공유하고, 요청이 있을 때만 계획한다(로봇별 액션).
  - **fleet tracker**: 모든 로봇의 LiDAR와 CCTV를 합치고 로봇 자신은 제외한 장애물 목록과 레이어를 만든다.
  - **fleet manager**: F-2와 F-3를 통합하고 계획 단계 예약을 맡는다.
  - (선택) **YOLO 서버**: GPU 1개 컨텍스트, 배치, 2–5 Hz, 필요할 때만.
- 기동은 벽시계 타이머 대신 준비 신호(lifecycle/이벤트) 기반으로 바꾼다. 파라미터 키 규약(`/**/node:`)은 시작 시 검사한다.

---

## 6. 검진표 — 서버에서 원인 확정하기

순서대로 진행한다. 각 항목이 어느 진단을 확정·기각하는지 표시했다.

| # | 검사 | 명령 | 이상 소견 | 진단 |
|---|---|---|---|---|
| 1 | 파라미터 적용 | `ros2 param get /amr1/dwa_planner robot_radius`<br>`ros2 param get /amr1/astar_planner preferred_clearance`<br>`ros2 param get /amr1/amcl update_min_d` | 0.2 / 1.25 / 0.25 | D1 |
| 2 | `/clock` 발행자 | `ros2 topic info /clock` | Publisher count 4 | D4, §4.6 |
| 3 | RTF (정지 → 주행) | `ign topic -e -t /world/warehouse/stats \| grep real_time_factor` | 정지 상태부터 낮음 → D4·렌더링. 주행 시 급락 → D2 | D2, D4 |
| 4 | 렌더러 | `nvidia-smi`, `grep -E "GL_VENDOR\|GL_RENDERER" ~/.ignition/rendering/ogre2.log` | llvmpipe, GPU 사용 0 | D7 |
| 5 | 프로세스 CPU | `ps -eo pid,pcpu,rss,args --sort=-pcpu \| head -25`<br>(가능하면 `py-spy top --pid <pid>`) | astar·dwa 각 100% 근처. py-spy에서 `_astar`, `occupancy_grid_has_static_obstacle_near`, `_publish_dynamic_obstacle_layer`가 상위 | D2 |
| 6 | 제어·계획 주기 | `ros2 topic hz /amr1/cmd_vel`, `ros2 topic hz /amr1/global_path` | cmd_vel이 20 × RTF보다 낮음. 경로 주기가 매우 낮음 | D2, D3 |
| 7 | 센서·TF 지연 | `lag_diag.py`(runbook §5의 remap 포함), `ros2 run tf2_ros tf2_monitor map amr1/base_footprint` | 1 s 이상 lag | D3 |
| 8 | 로그 패턴 | `grep -E "jump back in time\|Message Filter dropping\|extrapolation\|TF lookup\|경로 없음\|stale /global_path\|무시\|LIVELOCK\|blocks global path corridor\|center-risk reentry"` | 시간 역행·TF 실패 / NO PATH 반복 / 경로 거부 / 동결 | §4.6, D3, D5, §4.5, D6 |
| 9 | 주행 상태 | `ros2 topic echo /amr1/dwa/status`, `ros2 topic echo /fleet/deadlock_status` | `DYNAMIC_*`가 수십 초 지속, `"livelock": true` | D5, D6 |
| 10 | A/B | `dynamic_obstacles:=false`, `enable_deadlock_manager:=false` 각각 끄고 켜기 | 끄면 주행이 회복됨 | D5, D6 |
| 11 | 환경 | `nproc; cat /sys/fs/cgroup/cpu.max`, `df -h /dev/shm`, `ps -ef \| grep -E "ign\|ros2"`(런 전), `echo $DISPLAY $NVIDIA_DRIVER_CAPABILITIES` | 쿼터 < 필요 코어 수, shm 부족, 잔여 프로세스, DISPLAY=Xvfb + capabilities에 graphics 없음 | D7, D4 |
| 12 | 중복 노드·발행자 | `ros2 node list \| sort \| uniq -d`, `ros2 topic info /amr1/cmd_vel` | 중복 노드 이름, cmd_vel 발행자가 2 이상 | D7 (이전 런 잔존) |
| 13 | AMCL 생존·국지화 | `ros2 lifecycle get /amr4/amcl`, 로그 grep `CRITICAL FAILURE\|Please set the initial pose` | inactive/unconfigured, bond 끊김 | §4.6 |

---

## 7. 성공 기준 (처방 후 목표값)

| 지표 | 현재 (기록·추정) | 2단계 후 목표 | 3·4단계 후 목표 |
|---|---|---|---|
| RTF (4대 주행 중) | 약 0.33 (기록) | 0.6 이상 | 0.9 이상 |
| `cmd_vel` 주기 | 미측정 | 0.9 × 20 × RTF Hz 이상 | 같은 기준 |
| A\* 계획 시간 (약 40 m) | 서버 추정 8–14 s | 1.5 s 이하 (yaml 적용, 이벤트 재계획) | 0.3 s 이하 |
| NO PATH 판정 | 서버 추정 30–40 s | 상한 적용 시 1 s 이하 | 즉시 (사전검사) |
| 파라미터 적용 | 미적용 (D1) | 3개 노드 모두 yaml 값 | 시작 시 자동 검사 |
| GATE-GOAL / COLLISION | F-1 run9 PASS (동적 장애물 OFF 기준) | 동적 장애물 ON에서도 PASS | 핀치 교차 시나리오 PASS |
| livelock 동결 | 발생 가능 | 0 (임시 조치) | 0 (근본 수정) |

> 목표값은 처방 우선순위를 정하려고 세운 것이다. 예측치가 아니다. 단계마다 실측으로 갱신한다.

---

## 부록 A. 근거 위치

| 진단 | 파일:라인 |
|---|---|
| D1 YAML 키 | `amr_navigation/config/dwa_params.yaml:30`, `astar_params.yaml:1`, `amr_slam/config/amcl_params.yaml:1`, `amr_bringup/launch/multi_robot.launch.py:120-132, 146-193` · 코드 기본값 `dwa_node.py:1932, 2103`, `astar_node.py:391-397` · EKF는 정상(`multi_robot.launch.py:49-64`) · rclpy Humble `src/rclpy/node.cpp` `_populate_node_parameters_from_rcl_params` |
| D2 A\* | `astar_node.py:1177-1215, 1201-1206, 1244-1249, 614-615, 708` |
| D2 DWA | `dwa_node.py:1868-1874, 3840, 3874-3877, 1008-1040, 4175-4177, 4243-4248` |
| D3 | `astar_node.py:578, 672, 847, 1883` · `dwa_node.py:2186-2215, 2796-2800, 5561-5564, 5866` · `fused_tracker.py:416-417` |
| D4 | `multi_robot.launch.py:220-230` · `amr_robot.urdf.xacro:197-210` · `warehouse.world` (조명 60-66, CCTV 797-925) · `warehouse.launch.py:7-8, 25` · `dynamic_obstacle_mover.py:116, 166-177` |
| D5 | `dwa_node.py:4237-4238, 4742-4754, 3708-3712, 3879-3887` · `dwa_params.yaml:326-365` · `astar_node.py:788-789` |
| D6 | `deadlock_manager.py:68-69, 182, 255-260, 349-351, 378-381, 404` · `f3_priority_node.py:108-120` · `astar_node.py:610` |
| §4.5 | `astar_node.py:1336-1348, 1748-1761, 1839` · `dwa_node.py:5646-5651, 5719-5757` |
| §4.7 | `dynamic_obstacle_mover.py:121-127` · `warehouse.world` `wall_pinch_L2` · `warehouse_map.yaml` |

## 부록 B. 확인되지 않은 것 (서버 로그로 확정 필요)

- 서버 런에서 실제로 YOLO를 띄웠는지, 띄웠다면 몇 개를 어떤 device로 띄웠는지
- 해당 서버의 렌더링 모드(EGL/GLX/llvmpipe)와 vCPU 쿼터
- `/clock` 중복이 실제로 시간 역행과 TF 버퍼 초기화를 일으켰는지 (`jump back in time` 로그 여부)
- 벤치 수치(M2)가 서버에서 몇 배로 커지는지 (추정 1.5–3배)
- C4(핀치 리루트) 재검증 결과: D1 수정 후 다시 확인해야 한다
