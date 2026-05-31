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
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | **A\*** | Reliable, depth 1, **TRANSIENT_LOCAL** | frame_id = `map`, latched |
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
| `/dwa/status` | `std_msgs/String` | DWA | 1 Hz + edge 이벤트 | Reliable | 상태 문자열 — 상세는 §3.3 (`NORMAL`/`ALIGN`/`RECOVERY`/`RECOVERY_DONE`/`EMERGENCY`/`PATH_LOST`/`REACHED` 등) |

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
| `rejoin_predicted_exit_offset` | 0.42 m | `REJOIN` 유지용 예측 offset hysteresis. 진입은 민감하게, 해제는 현재/예측 offset을 분리해 판단 |
| `rejoin_align_angle_thresh` | 1.75 rad | `REJOIN` 중 ALIGN 진입 완화(약 100도) |
| `short_lookahead_rejoin_min_distance` | 0.35 m | path가 로봇 근처에서 접혀 실제 local lookahead가 너무 짧아지면 `REJOIN`으로 승격 |
| `short_lookahead_rejoin_ratio` | 0.55 | 실제 local lookahead가 effective lookahead 대비 이 비율보다 짧으면 접힌 lookahead로 판단 |
| `short_lookahead_goal_margin` | 1.0 m | goal 근처 final approach에서는 short-lookahead REJOIN을 비활성화 |
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
| `status_replan_states` | `["EMERGENCY", "PATH_LOST", "RECOVERY_DONE", "STOPPED_NEAR_WALL"]` | 수신 즉시 현재 pose 기준 A\* 재계획 |
| `status_replan_after_states` | `["FORWARD_ONLY", "RECOVERY"]` | fallback: 이 상태 뒤 reset 상태가 오면 1회 재계획 |
| `path_switch_hysteresis` | 0.35 m | 새 주기 재계획 후보가 이만큼 짧지 않으면 기존 path 유지 |
| `path_switch_max_start_offset` | 0.80 m | 현재 pose가 기존 path에서 이 이상 멀면 hysteresis 해제 |
| `path_hysteresis_stable_states` | `["NORMAL", "ALIGN"]` | 이 DWA 상태에서만 기존 path 유지 hysteresis 적용. 복구/벽 정지 중에는 새 후보 발행 |
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
5. 통합 테스트(mock `/global_path` → DWA `NORMAL` 전이) 통과 후 `dev` 머지

---

## 12. 변경 이력

| 일자 | 변경 | 작성자 | 리뷰 |
|---|---|---|---|
| 2026-05-10 | 초안 작성 — Nav2 기반 → 직접 구현으로 정정, 토픽 계약 추가, `v_max=2.0` 명세 반영 | SW(지상원) | — |
| 2026-05-15 | **N-0 작업 (디스코드 `#navigation`) 반영** — A\* Python 확정, stamp 정책 §3.4 신설, QoS 프로파일 명시, 잠정 결정 표시 강화, `/goal_pose` 스펙 §3.0 추가, 통합 검증 명령 §8 추가 | SW(지상원) | HU(양현욱) 리뷰 OK |
