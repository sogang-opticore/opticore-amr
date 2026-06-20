# F-2 Fleet — PLAN.md (ultrathink)

**Branch:** `feature/fleet-f2-deadlock` (분기 `feature/fleet-f1-cc`)
**Goal:** 좁은 복도 정면 교착을 강제하고, 우선순위 기반 자동 해소(저우선 HOLD→RETREAT→RESUME)로 2대 모두 goal 도달을 **bag 정량 검증**.
**Scope guard:** pairwise(2대)만 · 현 `warehouse.world` · `goal_pose` 조작 + launch additive만. 순환교착(3대+ cycle)·2존 월드(S-9)·우선순위 결정 마스터(F-3)는 범위 밖. 해소 로직은 **맵 독립**으로 작성(나중에 2존 swap 대비).
**작성 근거:** EXPLORE 6-subsystem 병렬 정독(launch infra / nav goal mechanics / world corridor / package conventions / F-1 harness / odom·TF) + `warehouse.world` 직접 파싱 + `dynamic_obstacle_mover.py` 정독.

> **사람 승인 요청 항목** = §1 복도/좌표, §3 T/R/D 파라미터. 나머지는 이 PLAN대로 무인 진행.

---

## 0. 환경 사실 (EXPLORE 확정 — 설계 근거, 다시 파지 마라)

### 0.1 토폴로지 / 토픽 (launch-infra, odom-tf)
- 4대 `ns=amrN`, `frame_prefix=amrN/`, spawn world `(3, 13/17/21/25)`, **yaw=0(spawn yaw 미지정, 전부 +x)**. `multi_robot.launch.py`는 `robot.launch.py`를 include 안 하고 노드를 직접 빌드.
- 전역 싱글톤: `fleet_localization_init`(+40s, AMCL active self-heal + 자동 initialpose, 수동 pub 0), `fused_tracker`(+50s). → **deadlock_manager는 세 번째 전역 싱글톤으로 additive 추가**.
- 토픽(전부 절대 `/amrN/...`로 해소):
  - goal `/amrN/goal_pose` (`geometry_msgs/PoseStamped`, header.frame_id=`map`)
  - cmd_vel `/amrN/cmd_vel` (`geometry_msgs/Twist`) — **매니저 절대 발행 금지(DWA 소유)**
  - odom `/amrN/odometry/filtered` (`nav_msgs/Odometry`, 50Hz nominal / 관측 21~24Hz, frame_id=`amrN/odom_filtered`, child=`amrN/base_footprint`)
  - status `/amrN/dwa/status` (`std_msgs/String`), path `/amrN/global_path` (`nav_msgs/Path`, map), truth `/amrN/ground_truth`
- **map↔world**: `map = (world_x−3, world_y−15)`, `MAP_ORIGIN_WORLD=(3,15)`.
- **위치 소스 = `/amrN/odometry/filtered`(결정론적)**. `fused_tracker`(`/perception/tracked_objects`)는 확률적(YOLO+LiDAR 클러스터+EKF assoc, track 생멸/ID run-to-run 변동) → **교착 탐지에 절대 사용 금지**.

### 0.2 액추에이터 가능성 (nav-goal-mechanics — ★설계 키스톤)
파이프라인: `goal_pose → A*(goal 소유, /global_path 발행) → DWA(path pure-pursuit) → cmd_vel`. **DWA는 path만 추종**; goal_pose는 new-goal edge 감지용으로만 구독.
- **HOLD(goal=현 pose) 성립**: A*가 현 위치 종단 path 발행 → DWA의 도착판정 거리 = **path 종점까지**(goal_pose 메시지 아님) ≤ `goal_tol`(멀티 override **0.4m**) → `REACHED` → **empty Twist 클린 정지**(진동 없음). 현 위치가 inflation 안이면 A* empty path → `STOPPED`로도 정지(둘 다 정지=OK).
- **RETREAT은 후진(reverse)이 아니다**: `v_min=−1.0`이나 `allow_backward=false` → `cmd_vel.linear.x≥0` 클램프 → **DWA 역주행 불가**. RETREAT = 뒤쪽 점을 goal로 주면 A*가 **전방 path 재계획** → DWA가 `ALIGN`(제자리 회전, thresh 1.10rad/63°)으로 ~180° 돌아 전진. 회전반경: circumscribed radius 0.36(직경 0.72m) < 복도 유효폭 1.28m → **복도 안 제자리 회전 가능**.
- A* goal **dedup**: 직전 goal과 `0.10m` & `0.10rad` 내면 무시 → HOLD/RETREAT/RESUME goal은 서로 `>0.10m` 차이 필수. (HOLD=현pose는 mission goal과 멀어 OK; RESUME=mission goal은 HOLD/RETREAT과 `>0.75m`라 OK.)
- DWA `REACHED` latch: REACHED 중 같은-goal `0.75m` 내 path 무시 → RESUME goal은 `>0.75m`라 클린 재무장(`_reset_path_state`: REACHED→NORMAL).
- 신규 goal 즉시 재계획 + `5s force-publish`(hysteresis bypass). `max_path_offset 1.0m`(복구창 1.8) — retreat/hold path는 robot에서 시작 → offset~0 → 수용.
- 폐루프 피드백: `/amrN/dwa/status` (REACHED/NORMAL/ALIGN/STOPPED/PATH_LOST/EMERGENCY). control_rate 20Hz, v_max 1.5, w_max 1.5.

### 0.3 패키지 (package-conventions)
- `amr_fleet` = 기존 **ament_cmake 스켈레톤**(package.xml/CMakeLists/README + `src/`·`config/`·`launch/` `.gitkeep`). C++ 스캐폴드(rclcpp dep)지만 소스 0.
- 파이썬 노드 설치 = **amr_slam 패턴**: `install(PROGRAMS src/*.py DESTINATION lib/${PROJECT_NAME})`. `.py`는 `src/`에 `#!/usr/bin/env python3` + **chmod +x**. ament_cmake=복사설치 → **매 .py 수정 후 `colcon build` 필수**(symlink-install 무효).
- deps 이미 충분: `std_msgs`/`geometry_msgs`/`nav_msgs`. **rclpy만 추가**(rclcpp는 미사용, 정리 가능). `amr_msgs`(TrackedObject*만)는 안 건드림 → `/fleet/priorities`는 `std_msgs/Int32MultiArray`.

### 0.4 검증 하네스 (f1-validation, 재사용)
- `f1_common.py` import: `rect_corners`/`poly_overlap`/`rect_gap`/`seg_dist`(SAT), `read_topic`(db3 직접 sqlite3 `SELECT timestamp,data FROM messages` + `deserialize_message`), `read_odom_xy_yaw`/`read_pose_xy_yaw`, `ROBOTS`/`MAP_ORIGIN_WORLD`/`w2m`/`spawn_map`/`yaw_q`. `RL,RW=0.60,0.40`. `ARRIVAL_TOL=0.6`.
- `clean_restart.sh`(스크립트파일 pkill + DDS daemon + `/dev/shm/fastrtps_*` purge) / `bringup.sh`(T1 warehouse→T2 multi_robot, 자동 localize) / `drive_and_record.sh` / `run_full.sh` 골격 재사용. **clean_restart PAT에 `deadlock_manager` 추가**.
- `fleet_goal_pub.py`: PoseStamped `/amrN/goal_pose` frame=map, **arrival wait 없음** → F-2는 **arrival-wait + 2-phase** 버전 필요.

---

## 1. 시나리오 (복도·좌표 — ★사람 승인)

### 1.1 복도 선정 (★축 정정)
미션 문구는 "amr1 동→서 / amr2 서→동"(E-W)이나, **현 월드의 좁은 복도는 N-S 십자통로(2.0m)뿐**이다. E-W 주아일(rack Row 간)은 **5.0m**로 교착 불가. 따라서 **N-S 통과 정면 조우**로 잡는다(해소 로직은 맵 독립이라 무영향).
- 통로: `rack_B4(32,15)[X 29.5~34.5]` ↔ `rack_B5(39,15)[X 36.5~41.5]` 사이 → **X=35.5 게이트, 물리폭 2.0m(X 34.5~36.5), 유효폭 ≈1.28m**(robot_radius 0.36 양측 차감). **1대 통과 OK, 2대 병렬 불가** → 정면 교착 성립.
- 게이트가 **B-C 아일(Y≈12, 폭5m)** ↔ **A-B 아일(Y≈18, 폭5m)** 연결. 클러터 무: `pallet_2(42,12)`·`box_stack_1(28,18)`은 시나리오 X창(32~40) 밖. (백업 통로: X=49.5 동일 구조.)

### 1.2 좌표 (map frame; world = map + (3,15))
| robot | role(default) | start world→map | mission goal world→map | 경로 |
|---|---|---|---|---|
| **amr1** | 고우선 | (32,12)→**(29,−3)** | (40,18)→**(37,3)** | B-C(S) → X=35.5 게이트 N → A-B(E측) |
| **amr2** | 저우선 | (32,18)→**(29,3)** | (40,12)→**(37,−3)** | A-B(N) → X=35.5 게이트 S → B-C(E측) |
| amr3 | idle | (3,21)→(0,6) | (없음) | spawn 정지 |
| amr4 | idle | (3,25)→(0,10) | (없음) | spawn 정지 |

**★비대칭 설계 (미러 교착 함정 회피)**: 둘 다 게이트 **西측서 출발**, 통과 후 둘 다 **東측 목표** — 단 **서로 다른 아일**(amr1→A-B, amr2→B-C). 그래서 패자 retreat 구역(자기 출발측, 西)이 승자 goal(東)과 **안 겹침**. (순수 미러(start↔goal 대칭)는 "승자 goal = 패자 정지점"이라 승자가 영구 막힘 → GATE-RESOLVE FAIL. 비대칭이 이를 구조적으로 제거.) A* 최단경로상 양쪽 다 X=35.5가 최단(타 게이트 +5~6m) → 게이트 강제.

### 1.3 진입 방식 (2-phase, **spawn override 0**)
spawn 좌표(F-1 Y=13/17/21/25) **불변**(F-1 무회귀). 매니저/시나리오 스크립트가 `goal_pose`로만 위치:
- **Phase A (pre-position)**: amr1 goal=world(32,12), amr2 goal=world(32,18) 발행 → 둘 다 `REACHED` 대기. (B-C·A-B 평행 아일, Y 6m 분리 → 상호 R 밖 → **조기 트리거 0**.)
- **Phase B (contest)**: amr1 goal=world(40,18), amr2 goal=world(40,12) **동시** 발행 → X=35.5 게이트 정면 조우.
- (대안: spawn에서 직접 contest goal — 더 단순하나 조우지점 비결정적. baseline로 유효성 판정 후 택1.)
- amr3/amr4: goal 미발행(또는 spawn 좌표 goal)으로 idle. amr1 경로에서 `>R`(>8m).

### 1.4 베이스라인 (실험 통제)
매니저 OFF(`enable_deadlock_manager:=false`)로 Phase A+B → **양쪽 영구정지(둘 다 goal 미도달 + 잔여 런 동안 변위<ε)** 확인 = 시나리오 유효성 증명. (F-1 "재현→수정" 방식; 미도달이면 좌표 보정.)

### 1.5 음성 테스트 (탐지 오발동 방지 — ★핵심)
amr1 단독, `person_1`(동적, world(14,18)↔(17,18)=map(11,3)↔(14,3), 0.6m/s) 가로지르는 **dwa_dyn `route_person_cross` 경로 재사용**. amr2/3/4 idle(X=3, `>R`). → **매니저 트리거 0**(다른 AMR이 R 내 전방에 없음; `person_1`은 odometry 없는 비-AMR이라 매니저 시야 밖) + amr1 DWA 회피로 goal 도달.

---

## 2. 매니저 설계 (`amr_fleet/src/deadlock_manager.py`)

### 2.1 인터페이스
- **구독**: `/amrN/odometry/filtered`(pose·속도), `/amrN/goal_pose`(**NORMAL일 때만 mission goal 캐싱** — 자기 override와 구분), `/amrN/dwa/status`(폐루프 보조), `/fleet/priorities`(`Int32MultiArray`, **transient_local**).
- **발행**: `/amrN/goal_pose`(**HOLD/RETREAT/RESUMING일 때만**), `/fleet/deadlock_status`(`std_msgs/String`, per-robot FSM + 탐지쌍, 2Hz, **관측·게이트용**).
- **우선순위**: `priorities[robotN−1]` = priority(클수록 우선). 미수신 시 default = **낮은 id 높은우선**. 동률 = 낮은 id tiebreak.
- `priority_publisher.py`: `Int32MultiArray` transient_local 주입 테스트 노드(주입 인터페이스 시연; **우선순위 결정 마스터는 F-3, 범위 밖**).

### 2.2 per-robot 상태
pose **ring buffer**(t,x,y,yaw), `mission_goal`(NORMAL시 캐싱), `reached`, **FSM∈{NORMAL, HOLD, RETREAT, RESUMING}**, stuck timer, livelock cnt.

### 2.3 탐지 (🔴-2 / GATE-DETECT)
- `stuck_i` = (윈도 T초 **변위<ε_move**) AND (mission goal 보유 · 미도달). ★순간 v 아닌 **변위** 기준 → 정지·SPIN 모두 포착.
- `blocked_by(i)` = ∃ robot j: `forward = (p_j−p_i)·heading_i ∈ (0, R)` AND `|lateral| < L_gate`. **j는 AMR만**(odometry 보유) → pedestrian/forklift 무동작.
- 양보 후보 = `stuck_i` AND `blocked_by(i)=j` AND `prio(i)<prio(j)` → **i가 양보(HOLD 진입)**. `prio(i)>prio(j)`면 **i 무동작** — 승자는 mission goal 유지 → j가 비키면 DWA가 **자동 재진행**(=최소 개입, 매니저가 승자를 안 건드림).

### 2.4 해소 FSM (🔴-3)
- **HOLD**: goal=현 pose(2Hz). hold_timer 시작. j가 전방 R서 사라짐 → **RESUMING**. hold_timer>`X_hold` & 여전 blocked → **RETREAT**.
- **RETREAT**: goal=pose history 역방향 arc-length `D` 뒤쪽 점. 도달 OR j 통과(전방 clear) OR `retreat_timeout` → **RESUMING**. HOLD↔RETREAT 진동 cnt++.
- **RESUMING**: goal=`mission_goal`(재무장될 때까지 2Hz; status NORMAL/ALIGN or goal 방향 변위 시작 감지) → **NORMAL**.
- **livelock 방지**: HOLD↔RETREAT cnt>`N` → status=`LIVELOCK` + `BLOCKED.md` 기록, HOLD 동결(스래싱 금지·강행 금지).

### 2.5 후진점 선정 (★방법)
매니저는 이미 구독 중인 odometry pose를 **ring buffer**에 적재. RETREAT 시 현 위치에서 buffer를 **역방향으로 Euclid arc-length 누적**, **`D` 도달점**을 retreat goal로 삼는다(이미 통과한 경로 → clear 보장). orientation = 그 시점 진행 접선(또는 현 yaw; A*가 ALIGN으로 보정). buffer가 `D`보다 짧으면 **leg 시작점**(가장 오래된 기록)으로.

---

## 3. 파라미터 제안 (T/R/D — ★사람 승인, 런타임이 최종 심판 · RTF≈0.33)
| 파라미터 | 제안값 | 근거 |
|---|---|---|
| `T_stuck` (정지 지속) | **4.0s** (sim) | 미션 기본. DWA `stuck_recovery 1.5s`/`spin 2.0s` 초과 → 정상 일시정지 오인 회피. |
| `eps_move` (윈도 변위) | 0.10m | 정지/SPIN 판정. odom 변위 기준. |
| `R_proximity` (전방거리) | **2.0m** | 정면 정지 시 중심간 ≈1.0~1.4m(0.6 body + inflation), 여유 포함. |
| `L_gate` (측방 허용) | **0.7m** | "복도서 앞" 한정 → 인접아일(Y 6m 분리) 오탐 차단. |
| `goal_tol`(참고) | 0.4m | DWA 멀티 override(공유 0.20). HOLD 도착판정 거리. |
| `X_hold` (HOLD→RETREAT) | **6.0s** (sim) | 승자 통과 대기. 너무 짧으면 불필요 후진. |
| `D_retreat` | **4.0m** (튜닝 4~6) | 게이트(meet~mouth ~3m) 완전 이탈 + 5m 광폭아일 진입. 부족 시 광폭아일서 DWA 우회 가능(5m). |
| `retreat_timeout` | 20.0s (sim) | RETREAT 무한 회피 상한. |
| `resume_clear_dist` | >0.75m | DWA goal-match 재무장 보장(0.2 참조). |
| `livelock_max_N` | 3 | HOLD↔RETREAT 진동 상한. |
| `republish_hz` | 2Hz | goal 비-latch → 재전달. A* dedup이 idempotent 처리. |

---

## 4. 작업 순서 & 게이트 (한 번에 한 🔴, PASS=정량만)
- **🔴-1 패키지+골격+주입** → DONE: `/fleet/priorities` test pub → 매니저 반영(로그/status). pub 없어도 default 동작. (`colcon build --packages-select amr_fleet`, `ros2 run amr_fleet deadlock_manager.py` 확인.)
- **🔴-2 탐지** → **GATE-DETECT**: 강제교착에서 T초 내 양쪽 탐지(`/fleet/deadlock_status`) AND 음성테스트(person-cross 단독) **오발동 0**.
- **🔴-3 해소** → **GATE-RESOLVE**(둘 다 goal `ARRIVAL_TOL 0.6` 도달, 영구정지 0) + **GATE-COLLISION**(0.60×0.40 실yaw SAT overlap **0 샘플**) + **GATE-BOUNDED**(해소시간 bounded, `LIVELOCK 0`, 진동 없음).
- **🔴-4 통합+무회귀** → F-2 전 게이트 PASS + **F-1 4게이트 재PASS**(`enable_deadlock_manager:=false`로 F-1 그래프 불변·4대 독립주행·단일로봇 불변).

**launch 추가(additive, `multi_robot.launch.py`)**: `DeclareLaunchArgument('enable_deadlock_manager', default='true')` + `TimerAction(period≈55s, [Node(amr_fleet, deadlock_manager, IfCondition, params=robots+T/R/D)])` → `return LaunchDescription(all_actions + [loc_init, fused_tracker, deadlock_manager])`. **per-robot 블록 무수정** (loc_init/fused_tracker 싱글톤 패턴 그대로). ※ F-1 회귀 런은 `:=false`로 동일 그래프.

---

## 5. 검증 하네스 (`/workspace/validation/fleet_f2/`)
- `f2_common.py`: `f1_common` import + 추가(윈도 변위 기반 stuck, 탐지쌍 재현, `/fleet/deadlock_status` 파싱).
- `gate_detect.py` / `gate_resolve.py` / `gate_collision.py`(=f1 SAT 재사용) / `gate_bounded.py`.
- `scenario_run.py`: 2-phase goal(arrival-wait) + priorities 주입 + bag record(`/tf /amrN/{ground_truth,odometry/filtered,goal_pose,cmd_vel} /fleet/{priorities,deadlock_status}`).
- `run_full_f2.sh`(매니저 ON) / `baseline_run.sh`(`:=false`) / `negative_run.sh`(person-cross 단독). bag은 `fleet_f2/<run>/`. (RTF≈0.33 → leg ≥240s.)

## 6. 검증 프로토콜 (매 🔴 후)
클린 재시작(§2) → T1 `warehouse.launch.py` → T2 `multi_robot.launch.py`(enable_deadlock_manager) → scenario → 해당 게이트 → **PASS면 게이트당 명시 커밋**(amr_fleet `src`/`launch`/`config` + validation 사본) + PROGRESS 갱신. **3회 FAIL → BLOCKED.md 적고 정지**(강행 금지, 24h 헛돌지 마라).

## 7. 가드레일 (HARD STOP 재확인)
Pod terminate 금지 · `git add -A` 금지(명시 경로만, `install/`·build 제외) · PR 금지 · `feature/fleet-f2-deadlock`에서만 · 파괴셸은 `validation/scratch`에서만 · 3h 체크포인트 커밋. **공유 DWA/A*/yaml·노드 코드 변경 0**(goal_pose 조작 + launch additive만) = **F-1 무회귀 by construction**.

## 8. 산출물
- 코드: `feature/fleet-f2-deadlock` (amr_fleet 신규 + `multi_robot.launch.py` additive).
- 검증 캐노니컬: `/workspace/validation/fleet_f2/`. repo `docs/fleet_f2/` PLAN/PROGRESS/report 사본 커밋.

## 9. 리스크 / 미지수 (런타임이 심판)
- **A* 게이트 선택**: 좌표가 X=35.5를 강제하는지 baseline로 검증; 다른 게이트 택하면 좌표 보정.
- **RETREAT 회전공간**: 게이트 내 제자리회전(직경0.72<1.28 OK)이나 타이트 시 `STOPPED`/`SPIN` 가능 → `D`를 광폭아일까지 충분히.
- **HOLD가 inflation서 `STOPPED`로 떨어질 수 있음**(둘 다 정지=OK, odom 변위로 확인).
- **승자 자동진행 가정**(매니저 무개입) — j 비킨 후 DWA가 재진행하는지 GATE-RESOLVE로 확인.
- **`eps_move`/`T_stuck`가 정상 감속을 오인 안 하는지** — 음성테스트가 1차 방어선.
