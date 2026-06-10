# F-1 Fleet — PLAN.md (ultrathink)

**Branch:** `feature/fleet-f1-cc` (분기점 `eb6995f`, from `feature/fleet-multi-robot-spawn`)
**Goal:** AMR 4대(amr1~4)가 각자 goal까지 충돌 없이 독립 주행 — bag 정량 검증으로 증명.
**Scope guard:** F-1 인프라 + 4대 독립 goal 주행까지. S-9(2존 월드)·F-2(교착 해소)는 범위 밖. 현 월드 + temp 좌표(Y=13/17/21/25)로 인프라만 증명.
**작성 근거:** EXPLORE 6-subsystem 병렬 맵 (launch/spawn, EKF/SLAM, odom injector, navigation, fused_tracker, fleet+validation) + 키스톤 파일 직접 정독(multi_robot.launch.py, ekf.yaml, amcl_params.yaml, odom/imu injector, fused_tracker.py).

---

## 0. 현재 상태 요약 (EXPLORE 결론)

### 🟢 이미 되어 있는 것 (JW 스캐폴딩, 확인됨)
- 4대 spawn (`ROBOTS` dict, multi_robot.launch.py:15-20), 로봇별 10s stagger.
- bridge per-robot remap (이중 prefix 해결): `/odom→/amrN/odom`, `/lidar→/amrN/lidar` 등 (lines 206-216).
- EKF 동적 yaml(OpaqueFunction 패턴, lines 36-52): `odom_frame/base_link_frame/world_frame = amrN/odom_filtered|base_footprint`, `odom0=/amrN/odom_with_cov`, `imu0=/amrN/imu`. 노드명 `amrN_ekf_filter_node` = yaml 루트키 일치.
- EKF 노드 **namespace 없음** → 글로벌 `/tf` 발행 (tf2 broadcaster는 절대 `/tf` 사용) = §4 "단일 글로벌 /tf + prefixed frames" 설계 그대로. **이게 정답 방향.**
- AMCL per-robot frame override (lines 108-120): `odom_frame_id=amrN/odom_filtered`, `base_frame_id=amrN/base_footprint`, `scan_topic=/amrN/lidar`.
- RSP `frame_prefix=amrN/` (lines 57-68) → `amrN/base_link`, `amrN/lidar_link` 등.
- LiDAR/camera static TF fix (lines 70-85): `amrN/lidar_link → amrN/base_footprint/lidar_sensor` (ign scoped sensor frame을 트리에 연결).
- odom injector frame_id (eb6995f): `header.frame_id=amrN/odom`, `child_frame_id=amrN/base_footprint`.

### 🔴 풀어야 할 것 — 진짜 원인 (EXPLORE로 규명)

**🔴-1 EKF /tf multi-robot (키스톤).** EKF TF 인프라는 대부분 맞다. GATE-T가 깨지는 **진짜 원인은 AMCL 초기위치**:
- `amcl_params.yaml:43-47` → `set_initial_pose:true, initial_pose:(0,0,0)` 가 **4대 공유**. 그런데 로봇은 world Y=13/17/21/25에 스폰. map(0,0)≈world(3,15)이므로 4대 다 엉뚱한 곳(world 3,15)에서 출발한다고 믿음 → 오정합 → `map→amrN/odom_filtered` 안 나옴 → GATE-T FAIL.
- 부차 원인(클린업): injector `frame_id=amrN/odom` ≠ EKF `world_frame=amrN/odom_filtered`. robot_localization은 odom0 pose(yaw)를 world_frame으로 변환하려는데 `amrN/odom→amrN/odom_filtered` TF가 없어 **odom yaw 측정 드롭**. 단, twist(vx,vyaw)는 child_frame_id=`amrN/base_footprint`=base_link_frame이라 정상 융합 → EKF는 속도+IMU로 계속 돌고 TF는 나온다 → **GATE-T 치명적 아님**. 그래도 깨끗한 융합 위해 `frame_id=amrN/odom_filtered`로 맞춘다.
- 부차(확인 필요): IMU injector가 `frame_id` 미설정 → ign 원본 프레임 통과. 트리에 없으면 IMU 드롭(역시 비치명적). 런타임 확인.

**🔴-2 initialpose 자동화.** 현재 자동 발행 없음. 단일소스(spawn 좌표) → map 초기위치 = **(x−3, y−15, 0)**:
| robot | spawn world (x,y) | map initialpose (x,y,yaw) |
|---|---|---|
| amr1 | (3, 13) | (0, −2, 0) |
| amr2 | (3, 17) | (0, +2, 0) |
| amr3 | (3, 21) | (0, +6, 0) |
| amr4 | (3, 25) | (0, +10, 0) |
- `MAP_ORIGIN_WORLD=(3.0,15.0)` 상수 1개로 ROBOTS에서 계산(단일 소스). **런타임 GATE-T로 검증**(looked-up 값이 위 map 좌표 근방인지) 후 필요시 보정.

**🔴-3 fused_tracker multi-robot.** 단일 공유 인스턴스 유지. `tracking_frame` 기본 `'odom_filtered'`(멀티로봇서 없는 프레임). `OUTPUT_FRAME='map'` 상수. `tracking_frame==OUTPUT_FRAME`이면 `_output_transform()`=항등(line 410-411). 모든 토픽/프레임이 **이미 파라미터**다 → **노드 코드 변경 0, 런치 파라미터 오버라이드만**.

**🔴-4 4대 독립 goal 주행.** A* 노드가 토픽을 **절대경로**(`/map`,`/goal_pose`,`/global_path`)로 하드코딩 → `namespace=rn`로 안 갈림(현재 상대 remap 무효). DWA도 출력(`/dwa/status`,`/dwa/trajectories`,`/dwa/best_trajectory`) 절대 하드코딩 + `map_topic` 미설정(기본 `/map`). 4대 크로스토크 발생. → 절대경로 remap(또는 파라미터화) 필요.

---

## 1. 작업 순서 & DONE 기준 (한 번에 한 🔴, 게이트로만 PASS 판정)

### 🔴-1 → GATE-T 4 PASS (수동 initialpose 허용)
1. `colcon build --symlink-install` (변경 없어도 1회). 클린 재시작(§2).
2. T1 `warehouse.launch.py` → T2 `multi_robot.launch.py`.
3. **수동** per-robot initialpose pub(위 표 map 좌표)로 AMCL 락.
4. `gate_t.py` 실행: 4대 `lookup_transform(map, amrN/base_footprint)` 성공 + 값이 위 map 좌표 근방(±0.5m, yaw sane). NaN/원점고정/실패 0건.
5. 깨끗한 융합 위해 **injector `frame_id`→`amrN/odom_filtered`** 수정(EKF world_frame 일치). IMU 프레임 런타임 확인 후 필요시 imu_tf_fix static TF 또는 injector frame_id 세팅.
6. PASS → 명시 커밋(injector .py + 필요한 launch만) + PROGRESS 갱신.

### 🔴-2 → 원샷 락 (수동 pub 0회) + GATE-T 유지
- **Primary:** multi_robot.launch.py AMCL Node params에 per-robot `set_initial_pose:true` + `initial_pose.{x,y,yaw}` = `(x−3, y−15, 0)` 오버라이드(MAP_ORIGIN_WORLD 단일소스). 공유 yaml은 **안 건드림**(단일로봇 무회귀).
- **Fallback (Primary가 flaky하면):** `amr_slam/initialpose_publisher.py` — AMCL lifecycle active 감지(`/amrN/amcl/get_state` 또는 첫 `/amrN/amcl_pose`) 후 `/amrN/initialpose` 1~3회 발행. ROBOTS 좌표 param 주입.
- DONE: 원샷 launch로 4대 GATE-T PASS, 수동 pub 0.

### 🔴-3 → no crash/lookup-fail + 발행 + 단일로봇 무회귀
- multi_robot.launch.py에서 fused_tracker를 **직접 Node**로 교체(현 IncludeLaunchDescription 대신; 단일로봇 `fused_tracker.launch.py`는 안 건드림). params: `tracking_frame='map'`, `lidar_topic='/amr1/lidar'`, `map_topic='/amr1/map'`, `camera_info_topic='/amr1/camera_info'`, `robot_detections_topic='/amr1/perception/detections'`, `lidar_frame='amr1/lidar_link'`, `camera_frame='amr1/camera_optical_link'`.
- 근거: `tracking_frame='map'`이면 `_output_transform`=항등, `_process_lidar`는 `amr1/lidar_link→map` lookup(amr1 AMCL 락 후 성립). LiDAR-only로도 발행.
- 트레이드오프(명시): 단일 인스턴스라 **amr1 입력만** 추적(4대 동시 융합은 노드의 multi-lidar 구독 확장 필요 → 본 런 인프라 범위 밖). 계약(단일 인스턴스, map 출력, 무회귀) 충족.
- 단일로봇 회귀 체크: `robot.launch.py`(또는 fused_tracker.launch.py 단독)로 `/perception/tracked_objects` 정상 발행 확인.
- DONE: 4대 환경서 crash/lookup-fail 0 + `/perception/tracked_objects`(map) 발행 + 단일로봇 거동 동일.

### 🔴-4 → GATE-GOAL 4 + GATE-COLLISION(0) + GATE-TOPIC
- A* per-robot: 절대 remap `('/map','/amrN/map'), ('/goal_pose','/amrN/goal_pose'), ('/global_path','/amrN/global_path')` + `dwa_status_topic` 정합. (remap만으로 부족하면 astar_node 토픽 파라미터화 = DWA 패턴.)
- DWA per-robot: 출력 절대 remap `('/dwa/status','/amrN/dwa/status'), ('/dwa/trajectories','/amrN/dwa/trajectories'), ('/dwa/best_trajectory','/amrN/dwa/best_trajectory')` + `map_topic='/amrN/map'` 추가.
- (선택) DWA `LOCAL_FRAME` 하드코딩 경고 정리: `local_frame` 파라미터화(기본 odom_filtered) — frame_id 체크 경고만이라 비치명, 시간 남으면.
- 목표 발행: `fleet_goal_pub.py`(validation) — 4대에 서로 다른 goal 동시 발행. goal은 빈 공간 좌표로 선정(맵/스폰 고려).
- bag 기록: `/tf /ground_truth /amrN/odometry/filtered /amrN/goal_pose /amrN/cmd_vel`. RTF≈0.33 → 충분히 길게(leg ≈240s 스케일).
- 게이트: GATE-GOAL(최종 pose가 goal ARRIVAL_TOL 0.6m 내), GATE-COLLISION(0.60×0.40 실제 yaw 사각형 pairwise overlap 0 샘플), GATE-TOPIC(`amrN/amrN` 공집합 + `/amrN/odometry/filtered`×4 Hz 정상).

---

## 2. 검증 하네스 (`/workspace/validation/fleet_f1/`)
- `gate_t.py` — **live** tf2 lookup 4대(map→amrN/base_footprint) 성공/값 sane. (bag 아닌 라이브가 DONE 기준에 직결.)
- `gate_topic.py` — `ros2 topic list` 파싱: `amrN/amrN` 공집합, `/amrN/odometry/filtered`×4 Hz.
- `gate_goal.py` / `gate_collision.py` — **db3 직접 파싱**(sqlite3 → topics 테이블 → `deserialize_message`; metadata 없을 수 있음) + **SAT 사각형 overlap**(`rect_corners`/`poly_overlap`/`rect_gap`/`seg_dist`를 `dwa_dyn/analyze_refix.py`에서 재사용).
- `fleet_goal_pub.py` — per-robot goal 발행.
- per-gate 리포트 `report_<gate>.md`, bag은 `fleet_f1/<run>/`.

## 3. 검증 프로토콜 (매 🔴 후 동일)
클린 재시작(§2) → T1 warehouse → T2 multi_robot → (initialpose) → 해당 게이트 → PASS면 게이트당 명시 커밋 + PROGRESS 갱신. **3회 FAIL → BLOCKED.md 적고 정지**(강행 금지).

## 4. 가드레일 준수 (HARD STOP 재확인)
- Pod terminate 금지 · `git add -A` 금지(명시 경로만, install/ 제외) · PR 금지 · `feature/fleet-f1-cc`에서만 · 파괴적 셸은 `validation/scratch`에서만 · 3시간 체크포인트 커밋.
- 커밋 대상: 노드/런치/yaml 소스만. bag/log/install/build 제외.

## 5. 산출물 위치
- **코드:** `feature/fleet-f1-cc` 커밋(명시 경로).
- **검증 캐노니컬:** `/workspace/validation/fleet_f1/` (analyze 스크립트 + bag + per-gate 리포트 + PLAN/PROGRESS/BLOCKED — 아침 대시보드).
- **git 리뷰용 사본:** repo `docs/fleet_f1/` (PLAN/PROGRESS 커밋 — §6 "PLAN 커밋" 충족. validation/은 repo 밖이라 커밋 불가하므로 사본 커밋).

## 6. 리스크 / 미지수 (런타임이 심판)
- map↔world 관계가 translation-only(yaw 0)이라는 가정 — GATE-T 값으로 검증, 어긋나면 보정.
- AMCL `set_initial_pose` param이 활성화 타이밍에 안 먹으면 → publisher fallback.
- DWA `map_topic`/`/dwa/status` 정합이 주행에 필수인지 — GATE-GOAL로 판정, 필요시 remap 보강.
- fused_tracker amr1-only가 계약 위반 아닌지 — "단일 인스턴스·map 출력·무회귀"만 요구하므로 OK, 4대 융합은 follow-on으로 명시.
