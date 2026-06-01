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
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 / BT / 사용자 | **A\***, DWA | Reliable, depth 1 | frame_id = `map`. DWA는 새 goal edge 확인용으로만 구독하며 반복 goal은 dedup |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | **A\***, DWA | Reliable, depth 1, **TRANSIENT_LOCAL** | frame_id = `map`, latched. DWA는 동적 장애물 판정에서 static wall LiDAR 점을 제외하는 데 사용 |
| `/dynamic_obstacle_layer` | `nav_msgs/OccupancyGrid` | DWA | **A\*** | Reliable, depth 1, **TRANSIENT_LOCAL** | LiDAR로 관찰한 동적 장애물을 임시 점유 영역으로 표시하는 overlay. A\*는 `/map` inflation 위에 합성해 우회 경로를 만든다 |
| `/dwa/status` | `std_msgs/String` | DWA | **A\*** | Reliable, depth 10 | 경로 재계획 이벤트 입력 (`EMERGENCY`/`PATH_LOST`/`RECOVERY_DONE` 등) |
| `/global_path` | `nav_msgs/Path` | A\* | **DWA** | Reliable, depth 1, **TRANSIENT_LOCAL** | frame_id = `map`, latched |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (robot_localization) | DWA (1순위) | Reliable | frame_id = `odom_filtered` |
| `/odom` | `nav_msgs/Odometry` | DiffDrive plugin | DWA (fallback, timeout > 0.5s) | Reliable | frame_id = `odom` |
| `/lidar` | `sensor_msgs/LaserScan` | ros_gz_bridge | DWA | **BEST_EFFORT**, depth 10 | frame_id = `lidar_link`, ~10Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 다수 | 모든 노드 | 표준 | 좌표 변환 |

### 2.2 출력 (발행)

| 토픽 | 타입 | 발행 주체 | 주기 | QoS | 용도 |
|---|---|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | **A\*** | goal 입력 시 1회 + 1Hz 주기 재계획 + DWA 상태 이벤트 시 | Reliable, depth 1, TRANSIENT_LOCAL | DWA가 추종할 전역 경로 |
| `/cmd_vel` | `geometry_msgs/Twist` | **DWA 단독** ★ | 20 Hz (control_rate) | Reliable, depth 10 | Ignition DiffDrive 입력 |
| `/dwa/trajectories` | `visualization_msgs/MarkerArray` | DWA | 5 Hz | Reliable | 후보 trajectory 시각화 (Foxglove) |
| `/dwa/best_trajectory` | `visualization_msgs/Marker` | DWA | 5 Hz | Reliable | 선택된 trajectory 강조 |
| `/dwa/status` | `std_msgs/String` | DWA | 1 Hz + edge 이벤트 | Reliable | 상태 문자열 — 상세는 §3.3 (`NORMAL`/`ALIGN`/`REJOIN`/`AVOIDING_DYNAMIC`/`DYNAMIC_BLOCKED`/`INSIDE_DYNAMIC_ZONE`/`RECOVERY`/`EMERGENCY`/`PATH_LOST`/`REACHED` 등) |
| `/dynamic_obstacle_layer` | `nav_msgs/OccupancyGrid` | DWA | 0.5 s 기본, 변경/해제 시 | Reliable, depth 1, TRANSIENT_LOCAL | A\*가 static map에 overlay하는 임시 no-go layer |

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

> **[미확정] 최종 yaw**: 현재 A\*/DWA 는 goal 의 최종 yaw 를 **적극 추종하지 않는다**(도착 판정은 거리 기반).
> A\* 의 goal dedup(2026-05-31)은 yaw 차이를 '새 goal' 판별에만 사용한다(같은 위치 yaw-only 변경을 놓치지 않기 위함).
> DWA 의 `/goal_pose` 구독은 stale empty path 방어용 goal edge 확인이 목적이므로 위치 기준 dedup만 사용한다.
> 최종 yaw 추종(정밀 도킹)은 향후 도킹 단계 과제.

**발행 트리거**: RViz2 "2D Goal Pose" 버튼, BT의 NavigateTo 노드, 또는 Mission node가 발행.
**A\* 동작**: 새 `/goal_pose` 수신 시 즉시 이전 계획을 폐기하고 재계획 시작(단, dedup 임계 이내 동일 goal 은 스킵).

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
- A\*는 binary inflation 바깥 free 셀도 장애물 거리장 비용으로 다시 평가해,
  가능한 경우 벽 경계보다 통로 중앙에 가까운 경로를 선호한다.
- 빈 path(`poses=[]`)는 **"계획 실패"** 컨벤션 → DWA는 즉시 정지, `status="STOPPED"`.
- 기본값에서는 A\*가 `/global_path`를 1초마다 현재 pose 기준으로 다시 발행한다.
  DWA의 `EMERGENCY`/`PATH_LOST`/`RECOVERY_DONE` 이벤트도 즉시 재계획 trigger로 사용한다.
- 단, A\*의 **주기 재계획** 실패가 이미 성공한 경로를 가진 상태에서 발생하면 빈 path를 발행하지 않고
  기존 `/global_path`를 유지한다. 일시적 TF/맵 흔들림이 DWA의 정상 추종을 `STOPPED/NORMAL`로
  떨리게 만들지 않기 위한 정책이다. 새 goal의 최초 계획 실패처럼 기존 경로를 믿으면 안 되는 경우에는
  기존처럼 빈 path를 발행한다.
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

`std_msgs/String`의 `data` 값으로 상태를 1Hz로 발행(도착 순간엔 `GOAL_REACHED` edge 1회 추가 발행):

| 값 | 조건 |
|---|---|
| `WAITING_ODOM` | 초기 부팅 직후 — `/odometry/filtered` 미수신 |
| `STOPPED` | odom 수신 완료, 그러나 `/global_path` 없음 또는 빈 path (계획 실패) |
| `NORMAL` | path 수신 완료, Pure Pursuit 정상 추종 루프 실행 중 (코드가 발행하는 실제 값; 구 문서 `PLANNING`) |
| `REJOIN` | path 이탈 상태. 가장 가까운 점 대신 미래 path 후보를 골라 작은 조향각으로 재합류 중 |
| `DYNAMIC_BLOCKED` | LiDAR 동적 장애물이 global path corridor를 막고 있으며, 기본값에서는 dynamic layer 기반 A\* 우회 재계획을 기다림 |
| `INSIDE_DYNAMIC_ZONE` | 로봇 현재 pose가 `/dynamic_obstacle_layer` no-go 내부 또는 가장자리에 있어 DWA가 정상 Pure Pursuit를 중단하고 가장 가까운 no-go block/trail/prediction의 반대 방향으로 저속 탈출하며, A\*가 start escape corridor를 열어 탈출 경로를 재계획해야 하는 상태 |
| `APPROACHING_DYNAMIC` | 추적된 동적 장애물의 CPA/closing speed가 위험해 정지, 짧은 후퇴, 또는 제자리 회피 회전을 우선 |
| `CROSSING_DYNAMIC` | 동적 장애물이 움직이며 path를 가로지르는 중으로 판단되어 우회보다 감속 대기를 우선 |
| `RECEDING_DYNAMIC` | 동적 장애물이 로봇/경로에서 멀어지는 중으로 판단되어 path가 clear될 때까지 감속 대기 |
| `STOPPED_DYNAMIC` | 추적된 동적 장애물이 정지 상태로 path를 막고 있으며 안전 후보가 없으면 대기 |
| `AVOIDING_DYNAMIC` | 정적 global path는 유지하되 LiDAR 기반 side-offset 또는 close-sidestep 목표점으로 동적 장애물을 우회 중 |
| `GOAL_REACHED` | **도착 순간 1회(edge)** — goal_tolerance 진입 시 (2026-05-31 P2 추가) |
| `REACHED` | 도착 후 정지 유지 상태 (1Hz 정상 발행) |
| `EMERGENCY` | 모든 trajectory 후보가 충돌 또는 안전거리(0.30m) 침범 → 즉시 정지 |
| `RECOVERY` | SPIN/FORWARD_ONLY 복구 동작 중 |
| `RECOVERY_DONE` | FORWARD_ONLY 복구 완료. A\*가 현재 pose 기준으로 1회 재계획해야 하는 edge 이벤트 |
| `PATH_LOST` | 현재 pose가 path와 너무 멀어 기존 path를 믿기 어려운 상태 |

> 그 외 내부 NavState(`ALIGN`/`RECOVERY`/`STOPPED_NEAR_WALL`/`PATH_LOST`/`EMERGENCY` 등)도 해당 상태일 때 그 값이 그대로 발행된다. (`RECOVERY` = SPIN/FORWARD_ONLY 복구 중. 2026-05-31 후진 제거 → 회전 전용)
> 도착 판정 소비자(예: `frontier_explorer`)는 **성공 = `GOAL_REACHED`/`REACHED`**, **실패·대기 = `STOPPED`** 로 구분한다.

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
| `v_brake_a_max` | 3.0 m/s² | goal/장애물 앞 감속 반응을 빠르게 하는 감속 전용 상한. **TODO 시뮬레이션 튜닝** |
| `alpha_max` | 1.5 rad/s² | 명세 미명시 — **TODO 시뮬레이션 측정 후 갱신** |

> ⚠ `alpha_max`는 명세에 없음. 임시값 사용. `cmd_vel` 스텝 응답으로 측정 후 확정.

### 4.2 DWA — 샘플링 / 시뮬레이션

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `sample_v_n` | 11 | 선속도 샘플 수 |
| `sample_w_n` | 21 | 각속도 샘플 수 (총 11×21 = 231 후보) |
| `predict_horizon` | 0.5 s | trajectory 예측 시간 |
| `dt` | 0.1 s | 적분 step |
| `control_rate` | 20.0 Hz | 제어 주기 |

### 4.3 DWA — Path 추종

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `lookahead_dist` | 0.65 m | path 선분 투영점 기준 최소 lookahead. 짧은 주기 흔들림을 줄이기 위해 완만하게 조향 |
| `lookahead_time` | 0.55 s | 속도 비례 lookahead. v=1.5 m/s에서 약 0.825 m |
| `path_heading_gain` | 0.6 | path 접선 방향 heading 오차 보정 |
| `path_cross_track_gain` | 0.8 | path 횡오차 복귀 보정 |
| `path_error_slowdown_offset` | 0.25 m | 이 이상 path에서 벌어지면 속도 감속 시작 |
| `path_error_min_speed_scale` | 0.42 | path 복귀 중 최소 속도 스케일 |
| `path_error_predict_time` | 0.55 s | 현재 heading/speed로 미래 횡오차를 예측해 선제 REJOIN/감속 |
| `goal_reached_epsilon` | 0.03 m | goal tolerance 경계에서 멈춘 경우 REACHED로 latch하는 추가 거리 band |
| `goal_reached_stopped_speed` | 0.03 m/s | 추가 band를 적용할 때 요구하는 정지 속도 |
| `goal_approach_distance` | 1.20 m | 목표 근처에서 선형 속도 상한을 추가로 낮춰 goal 주변 배회를 줄임 |
| `goal_approach_speed` | 0.80 m/s | `goal_approach_distance` 지점의 접근 속도 상한 |
| `goal_align_stop_distance` | 1.50 m | 목표 근처 ALIGN에서는 turn-in-motion을 막고 먼저 자세를 정렬 |
| `rejoin_entry_offset` | 0.30 m | 이 이상 path에서 벗어나면 `REJOIN` 후보 선택 |
| `rejoin_exit_offset` | 0.22 m | `REJOIN` 해제 hysteresis 거리 |
| `rejoin_exit_heading` | 0.52 rad | path heading 오차가 남아 있으면 `REJOIN` 유지 |
| `rejoin_min_lookahead` | 0.80 m | 너무 가까운 복귀점 배제 |
| `rejoin_max_lookahead` | 3.50 m | 미래 path 후보 탐색 상한 |
| `rejoin_step` | 0.20 m | 후보 arc-length 간격 |
| `rejoin_heading_weight` | 1.2 | 합류 지점 path heading mismatch 비용 |
| `rejoin_distance_weight` | 0.12 | 동적 적정 합류거리와의 차이 비용 |
| `rejoin_curvature_weight` | 0.18 | 큰 곡률의 급합류 후보 억제 비용 |
| `rejoin_clearance_min` | 0.80 m | LiDAR 기준 로봇-합류점 직선 구간의 목표 최소 여유 |
| `rejoin_clearance_weight` | 2.8 | 합류선이 벽/장애물에 가까운 후보를 밀어내는 비용 |
| `rejoin_cross_track_gain_scale` | 0.42 | `REJOIN` 중 nearest CTE 보정 완화 |
| `rejoin_heading_boost_gain` / `rejoin_heading_boost_max` | 1.0 / 0.45 rad/s | `REJOIN` 중 path heading 차이가 클수록 추가 회전 명령을 더해 큰 각도 복귀를 빠르게 처리 |
| `rejoin_predicted_exit_offset` | 0.42 m | `REJOIN` 유지용 예측 offset hysteresis. 진입은 민감하게, 해제는 현재/예측 offset을 분리해 판단 |
| `rejoin_align_angle_thresh` | 1.75 rad | `REJOIN` 중 ALIGN 진입 완화(약 100도) |
| `short_lookahead_rejoin_min_distance` | 0.35 m | path가 로봇 근처에서 접혀 실제 local lookahead가 너무 짧아지면 `REJOIN`으로 승격 |
| `short_lookahead_rejoin_ratio` | 0.55 | 실제 local lookahead가 effective lookahead 대비 이 비율보다 짧으면 접힌 lookahead로 판단 |
| `short_lookahead_goal_margin` | 1.0 m | goal 근처 final approach에서는 short-lookahead REJOIN을 비활성화 |
| `goal_shortcut_enabled` | true | `REJOIN` 후보가 있어도 goal 방향 local corridor가 LiDAR와 dynamic layer 기준 안전하면 path 합류점 대신 goal-bearing shortcut을 선택 |
| `goal_shortcut_max_lookahead` | 3.60 m | goal을 한 번에 찍지 않고 이 거리 안의 bounded target으로 잘라 매 tick 재검사 |
| `goal_shortcut_min_clearance` | 0.90 m | shortcut 직선 segment와 초기 Pure Pursuit arc가 동시에 만족해야 하는 최소 LiDAR clearance |
| `goal_shortcut_max_angle` | 0.70 rad | 큰 U-turn shortcut을 막고, 비교적 작은 조향으로 goal 방향을 탈 수 있을 때만 사용 |
| `goal_shortcut_path_error_speed_scale` / `goal_shortcut_cross_track_gain_scale` | 0.35 / 0.0 | shortcut 중에는 path 복귀 감속과 CTE 복귀 조향을 낮춰 불필요하게 global path 위로 돌아가려는 힘을 줄임 |
| `dynamic_avoid_enabled` | true | LiDAR 기반 동적 장애물 local bypass 활성화 |
| `dynamic_path_check_distance` | 3.20 m | 현재 path projection부터 앞쪽으로 동적 차단을 검사할 거리 |
| `dynamic_path_corridor_width` | 0.50 m | global path 주변 이 폭 안의 LiDAR point cluster를 차단 후보로 판단 |
| `dynamic_path_min_block_points` | 2 | corridor 차단으로 인정할 최소 LiDAR point 수 |
| `dynamic_avoid_lateral_offsets` | [0.55, 0.75, 0.95, 1.15] | 좌우 side-offset 우회 목표 후보 거리 |
| `dynamic_avoid_min_clearance` | 0.55 m | side-offset 우회 목표까지 이동하는 직선 segment와 초기 Pure Pursuit arc의 기본 LiDAR clearance. fallback 후보도 stop margin 근처(기본 약 0.37 m) 아래로는 허용하지 않음 |
| `dynamic_avoid_rejoin_distance` | 1.55 m | 차단 지점 뒤쪽 global path로 재합류할 기본 거리 |
| `dynamic_avoid_side_switch_penalty` | 2.00 | 동적 장애물 우회 중 좌우 side 전환 비용. 장애물 옆에서 `+1/-1` 목표가 번갈아 선택되는 oscillation을 억제 |
| `dynamic_static_filter_enabled` | true | `/map`의 정적 장애물 근처 LiDAR 점은 동적 차단 후보에서 제외 |
| `dynamic_static_filter_radius` | 0.60 m | LiDAR 점과 static occupied cell을 같은 정적 장애물로 볼 반경 |
| `dynamic_static_filter_occupied_threshold` | 65 | 정적 장애물로 인정할 OccupancyGrid 점유값 |
| `dynamic_static_filter_unknown_as_static` | true | unknown cell도 정적 구조물 쪽으로 보수적으로 보고 동적 후보에서 제외 |
| `dynamic_track_cluster_distance` | 0.35 m | scan 순서상 인접한 dynamic LiDAR 점을 같은 obstacle cluster로 묶는 jump 거리 |
| `dynamic_track_min_points` | 3 | dynamic obstacle track을 만들 최소 cluster point 수 |
| `dynamic_track_max_radius` | 0.85 m | 너무 긴 벽/선형 구조물을 track으로 보지 않기 위한 cluster 반경 상한 |
| `dynamic_track_association_distance` | 0.90 m | 이전 track과 새 cluster를 같은 장애물로 연결하는 거리 gate |
| `dynamic_motion_min_age` | 2 tick | 속도/CPA 판단 전에 필요한 최소 관측 tick |
| `dynamic_motion_stopped_speed` | 0.08 m/s | 이 속도 이하 track은 정지 동적 장애물로 판단 |
| `dynamic_motion_approach_speed` | 0.18 m/s | 상대 closing speed가 이 값 이상이고 CPA가 위험하면 `APPROACHING_DYNAMIC` |
| `dynamic_motion_cpa_horizon` | 2.50 s | closest point of approach 예측 시간 창 |
| `dynamic_motion_cpa_margin` | 0.35 m | CPA 기준 robot+obstacle 반경을 뺀 잔여 여유가 이 값 이하면 접근 위험 |
| `dynamic_approach_reverse_enabled` | true | 접근 동적 장애물에서 후방 LiDAR 여유가 충분하면 짧은 후퇴 허용 |
| `dynamic_approach_reverse_clearance` | 0.80 m | 접근 장애물 후퇴에 필요한 후방 clearance |
| `dynamic_approach_reverse_speed` | 0.16 m/s | 접근 장애물 후퇴 기본 속도. 일반 recovery 후진이 아니라 접근 위험 전용 짧은 escape |
| `dynamic_approach_reverse_max_speed` | 0.45 m/s | 접근 장애물 closing speed에 비례해 후퇴 속도를 올릴 때의 상한 |
| `dynamic_approach_reverse_speed_gain` | 0.18 | `reverse_speed = base + gain * closing_speed`로 접근 속도에 대응하는 계수 |
| `dynamic_layer_enabled` | true | 동적 장애물을 `/dynamic_obstacle_layer` 임시 점유 grid로 발행 |
| `dynamic_layer_prefer_global_replan` | true | 동적 layer block이 있을 때 DWA 즉석 우회/접근 escape보다 A\* 우회 재계획을 우선 |
| `dynamic_layer_local_fallback_ticks` | 8 | global replan을 기다려도 계속 막히면 DWA local avoid target을 다시 허용하는 control tick 수 |
| `dynamic_layer_path_corridor_width` | 1.20 m | 동적 track이 현재 global path 근처에 있을 때만 no-go layer 후보로 올리는 corridor 폭 |
| `dynamic_layer_path_lookahead` | 5.0 m | no-go layer 후보 판정에 사용할 현재 path 전방 거리. 후방 접근 장애물은 현재 위치가 아니라 속도 기반 swept path가 이 구간을 침범하는지 별도 검사 |
| `dynamic_layer_max_blocks` | 3 | false positive 누적으로 layer가 커지지 않도록 유지할 목표 dynamic block 수. 단, `dynamic_layer_min_hold_sec` 안의 block은 조기 삭제하지 않는다 |
| `dynamic_layer_ttl_sec` | 300.0 s | 한 번 관찰한 동적 장애물 영역을 임시 no-go로 유지할 최대 시간 |
| `dynamic_layer_min_hold_sec` | 30.0 s | 마지막 관측 이후 최소 이 시간 동안 block 유지 |
| `dynamic_layer_clear_confirm_sec` | 5.0 s | block 위치가 다시 관찰 가능하고 비어 있음을 확인해야 해제하는 시간 |
| `dynamic_layer_publish_period` | 1.0 s | `/dynamic_obstacle_layer` 발행 최소 간격. 너무 잦은 overlay 변경으로 A\* 경로가 흔들리는 것을 줄인다 |
| `dynamic_layer_position_alpha` / `dynamic_layer_velocity_alpha` | 0.35 / 0.25 | 같은 동적 block의 중심과 예측 속도를 새 관측에 얼마나 빠르게 따라붙일지 정하는 LPF 계수 |
| `dynamic_layer_radius_margin` | 0.25 m | 관찰 반경에 로봇 반경/안전 여유를 더해 점유 영역을 확장. 30초 유지 정책에서 제한이 과해지지 않도록 기본 no-go 반경을 줄임 |
| `dynamic_layer_min_radius` | 0.65 m | cluster가 작게 잡혀도 최소 이 반경만큼 no-go 처리 |
| `dynamic_layer_max_radius` | 1.30 m | 큰 cluster/merge가 과도하게 커지는 것을 막는 상한 |
| `dynamic_layer_prediction_horizon` | 4.0 s | 움직이는 track의 속도 방향으로 추가 점유 capsule을 예측할 시간. 뒤에서 접근하는 track도 이 예측 선분이 path/로봇 CPA를 침범하면 no-go 후보가 된다 |
| `dynamic_layer_prediction_max_distance` | 2.7 m | 예측 capsule이 한 번에 너무 길어지지 않도록 제한 |
| `dynamic_layer_min_track_age` | 2 | 새로 생긴 LiDAR 조각이 바로 no-go layer가 되지 않도록 요구하는 최소 track age |
| `dynamic_layer_trail_ttl_sec` | 60.0 s | 동적 장애물이 지나간 관측 궤적을 no-go corridor로 유지할 시간 |
| `dynamic_layer_trail_min_distance` | 0.25 m | trail point를 새로 남기는 최소 이동 거리 |
| `dynamic_layer_trail_max_points` | 24 | 단일 block trail이 지나치게 길어져 맵 일부를 통째로 막지 않도록 제한하는 최대 point 수 |
| `dynamic_layer_escape_distance` | 1.20 m | layer 재계획 대기 중이어도 접근 장애물이 이 거리 안이면 짧은 escape 허용 |
| `dynamic_layer_inside_margin` | 0.06 m | DWA가 로봇이 dynamic no-go block/trail 안에 있는지 판단할 때 block radius에 더하는 여유 |
| `dynamic_layer_inside_risk_threshold` | 0.32 | dynamic no-go 경계 접촉은 통과/감속으로 두고, 관측된 core/trail 중심 방향으로 충분히 들어갔을 때만 `INSIDE_DYNAMIC_ZONE` escape를 수행하는 침투 위험도 기준. 예측 capsule(`pred`)은 A*/target risk에는 쓰지만 robot-inside 판정에서는 제외한다 |
| `dynamic_layer_inside_escape_speed` | 0.18 m/s | 로봇이 dynamic no-go 내부에 있을 때 Pure Pursuit를 막고 가장 가까운 no-go feature 반대 방향으로 빠져나가는 저속 탈출 속도 |
| `dynamic_layer_inside_turn_speed` | 0.55 rad/s | dynamic no-go 내부에서 탈출 방향을 향해 정렬할 때 쓰는 회전 속도 상한 |
| `dynamic_layer_inside_align_angle` | 0.75 rad | 탈출 방향이 이 각도 이내일 때만 전진 탈출을 허용하고, 그보다 크면 먼저 회전 또는 안전 후진을 선택 |
| `dynamic_layer_reentry_margin` | 0.12 m | dynamic no-go 경계 근처 lookahead를 감속/조정하는 soft margin. 실제 no-go 내부가 아니면 정지하지 않는다 |
| `dynamic_layer_reentry_hard_margin` | -0.10 m | lookahead segment가 no-go를 깊게 관통할 때만 hard reentry로 본다. 얕게 스치는 경우는 감속/회전으로 해결 |
| `dynamic_layer_reentry_soft_risk` / `dynamic_layer_reentry_hard_risk` | 0.16 / 0.45 | no-go 중심 침투율 기준. soft는 감속/target 조정만 하고, hard일 때만 제자리 회전으로 재진입을 차단 |
| `dynamic_layer_reentry_rotate_ticks` | 12 tick | 과거 soft reentry 반복 회전 기준. 현재는 soft에서는 회전하지 않고 hard risk에서만 회전하므로 로그 호환용으로 유지 |
| `dynamic_layer_reentry_turn_speed` | 0.55 rad/s | hard risk reentry 때 사용하는 회전 속도 |
> 2026-06-01: `/dynamic_obstacle_layer`는 임시 overlay이므로 `/map`처럼 latched(`TRANSIENT_LOCAL`)로 남기지 않고 `VOLATILE` QoS로 발행/구독한다. 전체 맵 크기 grid 대신 실제 occupied cell 주변의 cropped grid만 발행해 Foxglove에서 정적 `/map` 전체를 덮어 사라지거나 검게 보이는 현상을 줄인다. Foxglove 가시성을 위해 같은 내용을 `/dynamic_obstacle_layer_markers` MarkerArray로도 발행한다.

| `align_release_angle` | 0.70 rad | ALIGN 중 안전하면 15도까지 기다리지 않고 NORMAL로 조기 복귀 |
| `rejoin_align_release_angle` | 0.95 rad | REJOIN 중 안전하면 더 이른 각도에서 path 추종으로 복귀 |
| `align_drive_angle` | 1.57 rad | ALIGN 중 전방 여유가 있으면 저속 turn-in-motion 허용 각도 |
| `turn_clearance_brake_angle` | 0.45 rad | 큰 회전 수요에서 전방 LiDAR 여유가 짧으면 제동거리 기반으로 속도 제한 |
| `v_brake_a_max` | 5.0 m/s² | 목표 선속도가 낮아질 때 적용하는 감속 전용 상한. 벽 근접 시 잔류 전진을 빠르게 줄임 |
| `w_brake_alpha_max` | 6.0 rad/s² | 목표 회전량이 작아진 뒤 남은 각속도를 빠르게 감쇠 |
| `forward_only_dist` | 0.35 m | SPIN 후 위치만 살짝 바꾸는 강제 전진 거리 |
| `forward_only_settle_w` | 0.20 rad/s | SPIN 직후 회전 관성이 이보다 크면 전진 보류 |
| `forward_only_rejoin_offset` | 0.25 m | FORWARD_ONLY 중 path에 가까워지면 강제 전진 조기 종료 |
| `spin_forward_clearance_margin` | 0.15 m | SPIN 후 FORWARD_ONLY 시작 전 전방 여유에 추가로 요구하는 안전 마진 |
| `max_path_offset` | 1.0 m | 새 `/global_path`가 현재 pose와 너무 멀면 stale path로 무시 |
| `recovery_path_accept_offset` | 1.8 m | stuck/recovery 직후 새 `/global_path` 수신 시 허용하는 완화 offset |
| `recovery_path_accept_duration` | 5.0 s | stuck/recovery 직후 완화 offset을 적용하는 시간 |
| `reached_new_path_rearm_dist` | 0.75 m | `REACHED` 중 새 path endpoint가 현재 위치와 충분히 멀면 다음 목표로 보고 재무장 |
| `path_lost_offset` | 1.8 m | 추종 중 이 이상 path에서 벗어나면 `PATH_LOST` 후 A\* 재계획 유도 |

> T14부터 DWA는 nearest point가 아니라 path 선분 위 투영점을 기준으로 lookahead를 고른다. T15부터 path 이탈 시에는 `REJOIN` 상태로 들어가 가장 가까운 점 대신 현재 조향각, 합류 지점 heading mismatch, 이동 거리를 함께 최소화하는 미래 path 점으로 부드럽게 재합류한다.

### 4.4 DWA — 평가함수 가중치

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `weight_heading` | 0.8 | goal 방향 정렬도 |
| `weight_clearance` | 0.4 | 장애물 여유 거리 |
| `weight_velocity` | 0.2 | 빠른 속도 선호 |

> 좁은 통로(< 1.5 m)에서는 `weight_clearance`를 0.6으로 상향 권장.

### 4.5 DWA — 안전

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `safety_distance` | 0.30 m | 명세 §7 안전거리 |
| `near_wall_creep_speed` | 0.12 m/s | 전방이 열린 측면 벽 근접 상황에서 v=0 고착 방지. **TODO 미확정, RunPod 튜닝 필요** |
| `near_wall_creep_min_clearance` | 0.60 m | creep을 허용할 최소 arc clearance. 이보다 벽에 붙으면 전진 대신 STOPPED/RECOVERY로 넘김 |
| `rejoin_creep_min_clearance` | 0.70 m | `REJOIN` 중 creep을 허용할 더 보수적인 최소 arc clearance |
| `near_wall_escape_clearance` | 0.45 m | 전방이 열려 있고 arc clearance만 낮은 경우 벽 반대 방향 보정을 켜는 기준 |
| `near_wall_escape_speed` | 0.28 m/s | 벽 옆 저속 탈출 시 제동거리 한도 안에서 허용하는 최소 속도 후보 |
| `near_wall_escape_turn` | 0.22 rad/s | 가까운 벽 반대쪽으로 더하는 작은 조향 bias |
| `near_wall_escape_max_curvature` | 0.80 | 이미 큰 곡률로 돌고 있을 때 wall escape bias를 막는 상한 |
| `odom_timeout` | 0.5 s | 이 시간 안에 `/odometry/filtered` 없으면 `/odom` fallback |

### 4.6 A\* — 그리드 / 휴리스틱 (제안, HU 확정 대기)

| 파라미터 | 기본값 | 비고 |
|---|---|---|
| `heuristic` | `"octile"` | `manhattan` / `euclidean` / `octile` (heuristics.py 등록됨) |
| `allow_diagonal` | `true` | 8-connected |
| `inflation_radius` | 0.50 m | 로봇 반경 0.20 m + DWA 정지 여유 0.30 m |
| `preferred_clearance` | 1.25 m | 이 거리 안쪽 free 셀에 비용을 부여해 벽 경계 path를 피함. **TODO 미확정, RunPod 튜닝 필요** |
| `clearance_cost_weight` | 9.0 | clearance 비용 가중치. 0이면 shortest path 우선 |
| `wall_avoid_clearance` | 1.05 m | inflation 경계 바로 바깥 후보를 강하게 밀어내는 barrier 기준 clearance |
| `wall_avoid_cost_weight` | 3.0 | wall-avoid barrier 비용 가중치 |
| `wall_avoid_min_margin` | 0.05 m | barrier 분모 최소 margin. inflation 경계에 붙은 cell 비용 폭주 방지용 clamp |
| `smoothing` | `"catmull_rom"` | `none` / `catmull_rom` / `bezier` |
| `smoothing_min_clearance` | 0.90 m | Catmull-Rom 스무딩 segment가 유지해야 하는 최소 clearance. 미달 시 raw A* path 유지 |
| `replan_period` | 1.0 s | 0이면 goal 입력 시에만 1회, > 0이면 주기적 재계획 |
| `dwa_status_topic` | `"/dwa/status"` | A\*가 이벤트 재계획 판단에 쓰는 DWA 상태 토픽 |
| `status_replan_cooldown` | 2.0 s | 상태 이벤트 재계획 최소 간격 |
| `dynamic_status_replan_cooldown` | 1.0 s | 동적 장애물 상태 전용 재계획 최소 간격. `/dynamic_obstacle_layer` 발행 주기와 맞춰 과도한 이벤트 재계획을 줄인다 |
| `status_replan_states` | `["EMERGENCY", "PATH_LOST", "RECOVERY_DONE", "STOPPED_NEAR_WALL", "DYNAMIC_BLOCKED", "INSIDE_DYNAMIC_ZONE", "APPROACHING_DYNAMIC", ...]` | 수신 즉시 현재 pose 기준 A\* 재계획. 동적 상태는 `/dynamic_obstacle_layer` overlay를 반영한 우회 경로 생성을 유도 |
| `status_replan_after_states` | `["FORWARD_ONLY", "RECOVERY"]` | fallback: 이 상태 뒤 reset 상태가 오면 1회 재계획 |
| `dynamic_layer_enabled` | true | DWA가 발행한 `/dynamic_obstacle_layer`를 static inflated grid 위에 합성 |
| `dynamic_layer_occupied_threshold` | 65 | dynamic layer cell을 점유로 볼 최소 OccupancyGrid 값 |
| `dynamic_layer_timeout_sec` | 35.0 s | 이 시간보다 오래된 dynamic layer는 stale로 보고 overlay 무시. DWA publish jitter로 A* overlay가 순간 해제되지 않도록 `dynamic_layer_min_hold_sec`보다 약간 길게 둔다 |
| `dynamic_layer_start_escape_*` | enabled=true, search=3.0m, corridor=0.45m, min_clear=0.60m | start cell이 정적 맵에서는 free지만 dynamic layer 때문에 막힌 경우, 정적 장애물은 보존한 채 dynamic layer 안에서 가장 안전한 바깥 셀까지 임시 escape corridor를 열어 A\*가 탈출 경로를 만들게 함 |
| `dynamic_path_side_lock_*` | lock=12.0s, lookahead=3.0m, deadband=0.20m | 동적 layer 회피 중 좌/우 우회 후보가 1Hz로 번갈아 선택되는 현상을 줄이기 위해 초기 path 가지를 잠깐 고정한다 |
| `dynamic_path_side_preference_*` | cost=0.50, distance=8.0m | dynamic branch lock이 살아 있을 때 A\*가 로봇 주변 8m 안에서 반대쪽 가지에 soft cost를 더해, 좌/우 후보가 거의 동률일 때 1Hz마다 번갈아 찍는 현상을 줄인다 |
| `dynamic_path_side_switch_min_*` | improvement=1.0m, clearance_gain=0.35m | 반대쪽 가지가 이만큼 짧거나 안전해졌을 때만 기존 가지 lock을 풀고 전환한다 |
| `path_switch_hysteresis` | 0.35 m | 새 주기 재계획 후보가 이만큼 짧지 않으면 기존 path 유지 |
| `path_switch_max_start_offset` | 0.80 m | 현재 pose가 기존 path에서 이 이상 멀면 hysteresis 해제 |
| `path_hysteresis_stable_states` | `["NORMAL", "ALIGN", "AVOIDING_DYNAMIC", "DYNAMIC_BLOCKED", "INSIDE_DYNAMIC_ZONE", "APPROACHING_DYNAMIC", "CROSSING_DYNAMIC", "RECEDING_DYNAMIC", "STOPPED_DYNAMIC"]` | 이 DWA 상태에서만 기존 path 유지 hysteresis 적용. REJOIN/복구/벽 정지 중에는 새 후보 수용성 우선 |
| `new_goal_force_publish_sec` | 5.0 s | 새 goal 직후 이 시간 동안 hysteresis를 건너뛰어 `/global_path` 재수신 기회 확보 |
| `goal_direct_distance` | 2.0 m | 목표 근처에서 안전한 직선 final approach path 허용 거리 |
| `goal_direct_min_clearance` | 0.90 m | 직선 final approach segment의 최소 raw obstacle clearance |
| `path_switch_bad_clearance` | 0.90 m | 기존 path 최소 clearance가 이보다 낮으면 safety switch 후보로 본다 |
| `path_switch_clearance_gain` | 0.18 m | 후보 path 최소 clearance가 이만큼 개선되면 길이 hysteresis보다 안전성을 우선 |
| `path_switch_clearance_max_extra_length` | 3.0 m | clearance 개선으로 바꿀 때 허용하는 후보 path 추가 길이 상한 |
| `path_switch_clearance_skip_distance` | 1.0 m | 현재 위치 바로 주변의 공통 벽 근접 구간을 제외하고 앞쪽 clearance를 비교 |
| `path_switch_safety_clearance_loss` | 0.20 m | 안정 주행 중 새 후보가 이만큼 더 벽에 가까우면 기존 path 유지 후보로 본다 |
| `path_switch_safety_max_length_sacrifice` | 1.20 m | 더 안전한 기존 path를 유지하기 위해 감수할 수 있는 후보 대비 최대 길이 손해 |

---

## 5. 실패 / 안전 정책

| 상황 | A\* 동작 | DWA 동작 |
|---|---|---|
| goal이 점유 셀 | 빈 path 발행 + 로그 | — |
| goal 도달 불가 (장애물로 막힘) | 빈 path 발행 | 정지 |
| DWA가 `EMERGENCY`/`PATH_LOST`/`RECOVERY_DONE` 발행 | 현재 pose 기준 1회 재계획. 실패해도 기존 성공 path가 있으면 빈 path 미발행 | 새 path 수신 시 추종 재개 |
| DWA가 `NORMAL`/`ALIGN`으로 정상 추종 중 | 기본값에서는 재계획 없음. 기존 latched path 유지 | path 초입 재정렬 반복 방지 |
| DWA가 `DYNAMIC_BLOCKED`/동적 motion 상태를 발행 | `/dynamic_obstacle_layer`를 static inflated grid 위에 overlay한 뒤 현재 pose 기준 재계획한다. layer block이 있으면 해당 영역을 임시 no-go로 보고 우회 경로를 찾는다 | 기본값에서는 즉석 side bypass보다 전역 우회 재계획을 우선한다. 새 path가 오기 전까지는 감속/정지 상태를 유지한다 |
| DWA가 `INSIDE_DYNAMIC_ZONE`을 발행하거나 start가 dynamic layer 안에 있음 | start가 정적 맵에서는 free이고 dynamic layer 때문에만 막혔다면, A\*가 작은 start escape corridor를 임시로 열어 no-go 바깥쪽 안전 셀까지 빠지는 경로를 만든다 | 기존 path를 무작정 따라 동적 장애물 궤적으로 들어가는 대신 새 `/global_path`를 기다리거나, corridor가 열리면 저속 추종으로 빠져나온다 |
| 주기 재계획 실패 + 기존 성공 path 있음 | 빈 path 미발행, 기존 path 유지 | 기존 path 계속 추종 |
| `/global_path` empty 수신(새 goal 직후) | — | 즉시 정지, `status="STOPPED"` |
| `/global_path` empty 수신(새 goal 없음 + 기존 path 있음) | — | stale/중복 publisher 가능성으로 보고 기존 path 유지 |
| `/global_path`가 현재 pose와 `max_path_offset` 초과로 멂 | — | stale path 가능성으로 보고 path 무시, 기존 path 유지 |
| DWA가 추종 중 path와 `path_lost_offset` 초과로 멂 | 현재 pose 기준 1회 재계획 | 정지, `status="PATH_LOST"` |
| 전방은 열렸지만 측면 벽/초기 arc clearance가 낮음 | — | `near_wall_creep_min_clearance` 이상일 때만 `near_wall_creep_speed`로 최소 전진하고, `REJOIN` 중에는 `rejoin_creep_min_clearance`까지 더 보수적으로 확인 |
| 모든 trajectory 후보 충돌 | — | 정지, `status="EMERGENCY"` |
| 진행 방향 장애물 거리 < `safety_distance` (0.30 m) | — | 즉시 정지 (명세 §7 안전거리). 측면 벽은 near-wall creep 조건을 별도로 적용 |
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
ros2 topic echo /dwa/status    # WAITING_ODOM → STOPPED → NORMAL 상태 변화

# waypoint 여러 개를 한 번에 실행
ros2 run amr_navigation waypoint_sequence.py \
  --goals "20,10;10,5;35,-10"
# → 각 waypoint마다 /goal_pose를 짧게 발행하고,
#   DWA가 GOAL_REACHED/REACHED를 발행하면 다음 waypoint로 넘어간다.
#   A*는 goal을 받은 뒤 자체적으로 /global_path를 1Hz 재계획하므로
#   같은 goal을 터미널에서 계속 발행할 필요가 없다.

# 통합 검증 (warehouse.launch.py가 같이 떠 있을 때)
ros2 topic pub --once /global_path nav_msgs/msg/Path \
  "{header: {frame_id: 'map'}, poses: [
     {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]}"
# → /dwa/status가 STOPPED → NORMAL 으로 전환되면 통합 OK
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

- [x] A\* 재계획 트리거 정책: 동적 장애물 상태 수신 시 `/dynamic_obstacle_layer` overlay 기반 자동 재계획
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
5. 통합 테스트(mock `/global_path` → DWA `NORMAL` 전이) 통과 후 `dev` 머지

---

## 12. 변경 이력

| 일자 | 변경 | 작성자 | 리뷰 |
|---|---|---|---|
| 2026-06-02 | dynamic no-go 경계에 닿기만 해도 `DYNAMIC_BLOCKED` 회전 루프가 생기는 문제를 줄이기 위해 reentry/inside 판정을 경계 거리 대신 중심 침투 위험도(`risk`) 기준으로 변경했다. soft risk는 감속과 target 조정만 수행하고, hard risk일 때만 회전 차단한다. `REJOIN` 중 path heading 차이가 클수록 추가 회전 명령을 주는 boost도 추가했다 | Codex | py_compile 통과. RunPod 단위/주행 테스트 필요 |
| 2026-06-01 | dynamic no-go에 얕게 걸친 `target_clear=-0.02m` 상황에서 정지 대기만 반복하던 문제를 줄이기 위해 shallow reentry는 통과/감속하고, 반복되거나 깊게 관통하면 전진 대신 회전 탈출로 전환하도록 수정했다. 접근 동적 장애물의 후진 속도도 closing speed에 비례해 최대 0.45m/s까지 올라가도록 보강했다 | Codex | RunPod 주행 검증 필요 |
| 2026-06-01 | dynamic no-go 경계 근처에서 `target_clear`가 양수인데도 `DYNAMIC_BLOCKED`로 영구 대기하던 문제를 수정했다. 경계 근처는 soft reentry로 감속/동적 layer-aware lookahead 조정하고, 실제 no-go 내부를 관통할 때만 hard block한다 | Codex | RunPod 주행 검증 필요 |
| 2026-06-01 | dynamic no-go 반경을 0.65m 중심으로 축소하고, 정지/UNKNOWN track은 path 중심을 직접 막는 경우에만 layer 후보가 되도록 조건을 보수화했다. no-go 탈출 직후 lookahead가 같은 영역을 다시 관통하면 Pure Pursuit 재진입을 차단해 후진-재진입 반복을 줄인다 | Codex | RunPod 주행 검증 필요 |
| 2026-06-01 | dynamic layer 최소 유지 기준을 최초 관측이 아니라 마지막 관측 기준으로 변경하고, A*의 dynamic layer stale timeout을 35초로 늘려 `/dynamic_obstacle_layer`가 순간적으로 사라지며 경로가 좌우로 흔들리는 현상을 줄였다. 접근 동적 장애물은 로봇 기준 앞/뒤 방향을 계산해 앞에서 오면 후진, 뒤에서 오면 전진 회피를 우선한다 | Codex | RunPod 주행 검증 필요 |
| 2026-06-01 | 여러 waypoint를 한 번의 터미널 명령으로 순차 실행하는 `waypoint_sequence.py` 추가. `/dwa/status`의 `GOAL_REACHED`/`REACHED`를 보고 다음 `/goal_pose`를 발행하며, stale `REACHED` 방지를 위해 발행 직후 무시 시간과 motion 확인 조건을 둔다 | Codex | RunPod 주행 검증 필요 |
| 2026-06-01 | DWA dynamic layer가 후방 접근 동적 장애물을 놓치지 않도록 전방-only 필터를 제거하고, track 속도 벡터의 swept segment가 global path/CPA를 침범하면 `/dynamic_obstacle_layer` no-go 후보로 등록하도록 보강했다 | Codex | RunPod 주행 검증 필요 |
| 2026-05-31 | Dynamic layer temporal smoothing 추가. `/dynamic_obstacle_layer` 발행 간격을 1.0s로 완화하고 block 중심/속도 LPF를 추가했으며, A\* dynamic side preference를 cost=0.50, distance=8.0m, lock=6.0s로 보강했다 | Codex | RunPod 주행 검증 필요 |
| 2026-05-31 | A\* dynamic branch side preference cost 추가. `dynamic_path_side_preference_cost=0.35`, `dynamic_path_side_preference_distance=6.0m`로 dynamic branch lock 중 반대쪽 우회 가지에 soft cost를 주어 1Hz 재계획 좌우 flip을 줄인다 | Codex | RunPod 주행 검증 필요 |
| 2026-05-31 | DWA 동적 장애물 layer(`/dynamic_obstacle_layer`)와 A\* overlay 합성 계약 추가. 동적 장애물은 기본적으로 임시 no-go 영역으로 보고 A\* 전역 우회 재계획을 우선한다 | Codex | RunPod 주행 검증 필요 |
| 2026-05-10 | 초안 작성 — Nav2 기반 → 직접 구현으로 정정, 토픽 계약 추가, `v_max=2.0` 명세 반영 | SW(지상원) | — |
| 2026-05-15 | **N-0 작업 (디스코드 `#navigation`) 반영** — A\* Python 확정, stamp 정책 §3.4 신설, QoS 프로파일 명시, 잠정 결정 표시 강화, `/goal_pose` 스펙 §3.0 추가, 통합 검증 명령 §8 추가 | SW(지상원) | HU(양현욱) 리뷰 OK |
