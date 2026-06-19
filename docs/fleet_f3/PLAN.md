# F-3 Fleet — PLAN.md (ultrathink)

**Branch:** `feature/fleet-f3-priority` (분기 `feature/fleet-f2-deadlock`)
**Goal:** path+track 으로 로봇-로봇/로봇-장애물 충돌을 **시간기반 예측** → F-2 우선순위 인터페이스(`/fleet/priorities`)에
**일찍 주입** → F-2 의 반응형 2m HOLD/RETREAT/RESUME 가 발동하기 전에 우선순위를 미리 정한다.
**Scope guard (additive):** F-2 enforcement(HOLD/RETREAT/RESUME) **재구현 금지**. F-3 는 우선순위를 **예측·발행**만.
`/amrN/goal_pose`(F-2 액추에이터) **발행 금지**. 새 메시지 타입 추가 금지. handoff(`<2m`)는 F-2 영역 → 손 안 댐.
**키스톤 = GATE-F3-LOGIC**(순수함수 T1~T7 단위테스트, sim 불필요·결정론). 스모크는 보너스.
**작성 근거:** EXPLORE — F-2 노드(`deadlock_manager.py`/`priority_publisher.py`) 정독, `multi_robot.launch.py`
원격탐색(per-robot remap 확인), `amr_msgs/TrackedObject*` 필드, perception 토폴로지(병렬 Explore 2개) +
코어 수치 적대적 검증(Plan agent — T1~T7 샘플링 그리드 손계산).

> **사람 승인 요청 항목 없음** — 무인 런 위임. 단 §1 의 *판단 결정*(파일 배치·obstacle 비강제·T7 재구성)은 아침 검토 대상.

---

## 0. 환경 사실 (EXPLORE 확정 — 코드로 검증, 다시 파지 마라)

### 0.1 F-2 우선순위 인터페이스 (★ 키스톤 — `grep`+정독으로 확정)
- 토픽 **`/fleet/priorities`**, 타입 **`std_msgs/Int32MultiArray`**, QoS **latched**(RELIABLE+TRANSIENT_LOCAL+
  KEEP_LAST depth 1). `data[i]`(`i=robotN-1`, 0-based), **값 클수록 우선**.
  근거: `amr_fleet/src/deadlock_manager.py:17,29,127,155,192-211`, `priority_publisher.py:24-32`.
- F-2 미주입 시 기본 `default_prio = N-index`(낮은 id 높은 우선). `higher(a,b)` = `prio(a)>prio(b)`, 동률 → 낮은 rid.
  패자가 deadlock 에서 양보. (`deadlock_manager.py:124-126,206-211`)
- **F-2 는 *다른 AMR* 하고만 교착 발동**(`_blocker` 가 `self.robots` 만 스캔; 보행자/지게차는 구조적으로 시야 밖 →
  DWA 로컬회피 담당). → **F-3 의 robot-obstacle 양보는 `/fleet/priorities`로 강제 불가** = 예측/결정 기록 + 마커로만 노출.
- F-2 는 수신 배열로 `injected_prio` dict 를 **통째 덮어씀** → F-3 는 **전체 배열을 매번**, **변할 때만** 발행(flapping/로그 폭주 방지).

### 0.2 입력 토픽/프레임 (multi_robot.launch.py 원격 Explore 확정)
- 4대 `amr1..amr4`, world `(3.0, 13/17/21/25)`. `robots_param = ['amr1:3.0:13.0',...,'amr4:3.0:25.0']`.
- `astar_node.py:610` 은 절대 `/global_path` 발행이나, `multi_robot.launch.py:144-198` 의 per-node
  **remap** 으로 런타임은 `/amrN/global_path`. 동일 패턴: `/amrN/odometry/filtered`·`/amrN/goal_pose`·`/amrN/dwa/status`.
- 위치 = TF `lookup_transform('map', 'amrN/base_footprint')`(frame_prefix `amrN/`; F-2 와 동일 소스
  `deadlock_manager.py:221-239`). 속도 = `/amrN/odometry/filtered` twist 크기(F-2 `_on_odom`).
- **장애물** = `amr_msgs/TrackedObjectArray` on **`/perception/tracked_objects`**(단일 전역 토픽, 미네임스페이스
  `fused_tracker`, frame=map; `multi_robot.launch.py:321-339`). 필드 전부 채워짐(`fused_tracker.py:485-497`):
  `tracks[].position`(Point), `.velocity`(Vector3 vx/vy), `.class_name`('person'|'forklift'|'unknown'),
  `.source`(bitmask LIDAR=1/ROBOT_CAM=2/CCTV=4).
- **⚠ self-tracking**: fused_tracker 에 ego-filter 없음 → AMR 이 `'unknown'` 트랙으로 잡힐 수 있음.
  F-3 노드는 **알려진 로봇 pose `self_filter_radius_m` 내 트랙을 드롭**(로봇을 장애물로 이중계산 방지).

### 0.3 패키지/빌드 (F-2 와 동일)
- `<fleet_pkg> = amr_fleet`(전용 패키지 **존재** — CLAUDE.md §7 "fleet 패키지 없음"은 stale). **ament_cmake**.
- 파이썬 노드 = src/ + `#!/usr/bin/env python3` + **chmod +x**, `install(PROGRAMS src/*.py DESTINATION lib/${PROJECT_NAME})`.
  ament_cmake=복사설치 → **수정 후 colcon build 필수**. → **F-3 파일은 CLAUDE.md §8.1 의 `<pkg>/<pkg>/`가 아니라 `src/`** 에.
- deps 충분(`std_msgs`/`geometry_msgs`/`nav_msgs`/`rclpy`/`tf2_ros`). 마커용 `visualization_msgs` + 테스트용 `ament_cmake_pytest` 추가.

---

## 1. 판단 결정 (무인 위임 범위 — 아침 검토)
1. **파일 배치**: `amr_fleet/src/`(F-2 ament_cmake 관례) — CLAUDE.md §8.1 의 generic `<pkg>/<pkg>/` 와 다름. 의도는 동일(코어+노드+config+launch+test).
2. **launch 별도**: `amr_fleet/launch/f3_priority.launch.py`(독립) — `amr_bringup/multi_robot.launch.py` **미편집**(브랜치 격리/whitelist 안전). §8.1 "얹거나 별도" 중 별도 선택.
3. **robot-obstacle 비강제**: F-2 가 비-AMR 에 enforce 불가 → F-3 는 결정 기록 + `/fleet/conflict_predictions` 마커만, **`/fleet/priorities` 배열 불변**. 정직·additive.
4. **T7 재구성**: CLAUDE.md T7 의도(amr1 먼저 도착, amr2 고우선 → priority override) 유지하되 same-index 샘플링서 arrival gap ≥ epsilon 되도록 좌표 조정(geometric crossing arrival).

---

## 2. 산출물 (화이트리스트 — additive only)
| 파일 | 목적 |
|---|---|
| `amr_fleet/src/fleet_priority.py` | **순수** 코어(stdlib only, no rclpy). GATE-F3-LOGIC 대상. |
| `amr_fleet/src/f3_priority_node.py` | ROS2 노드 wrapper(구독/발행, 코어 호출). |
| `amr_fleet/config/f3_params.yaml` | 정책 숫자 전부(§8.4). |
| `amr_fleet/launch/f3_priority.launch.py` | F-3 독립 launch(F-2 Node 블록 미러). |
| `amr_fleet/test/test_fleet_priority.py` | T1~T7 + H1~H4 pytest. |
| `amr_fleet/CMakeLists.txt` | **+추가** install(PROGRAMS) 신규 노드 + `ament_add_pytest_test`. F-2 기존 라인 불변. |
| `amr_fleet/package.xml` | **+추가** `ament_cmake_pytest`(test_depend), `visualization_msgs`. |
| `docs/fleet_f3/{PLAN,PROGRESS,BLOCKED}.md` | §6 커밋. |

**금지(STOP+BLOCKED.md)**: F-2/F-1 노드, `amr_msgs`, perception, `maps/`, `*.world`, `env.sh`, `setup.sh`, 새 메시지 타입.

---

## 3. 코어 설계 — `fleet_priority.py` (순수·결정론)
진입점 `compute_priorities(robots, obstacles, params) -> Result`.
- 입력 robots `{id(1-based), pose(x,y,yaw), path[(x,y)..], speed|None, priority|None}`; obstacles `{x,y,vx,vy,class_name}`.
  `priority` = 외부할당/F-2 기존 우선순위(priority-dominant 트리거).
- Result: `conflicts[Conflict]`, `decisions[Decision{yielding_id, winner_id|None, reason}]`,
  `priorities{id->int}`(발행 배열). reason ∈ {priority_dominant, right_of_way, id_tiebreak, obstacle}.

**알고리즘**
1. `sample_trajectory`: `eff_speed = speed if speed>stopped_speed_eps else cruise_speed`(멈춤→cruise). pose 를 path 폴리라인에 스냅 → 호장 `eff_speed*t`(t=0..horizon step dt) 전진, 끝점 clamp. 모든 로봇 동일 t-grid.
2. robot-robot(쌍 id_a<id_b 정렬): **handoff 게이트 먼저** — 현재 중심거리 `<handoff_distance_m` → 쌍 **스킵**(F-2 영역). 아니면 same-index `dist²<radius²` 첫 i → 충돌, `t_conflict=i·dt`.
   **arrival(geometric)**: 두 path 가 점 C 에서 교차 → `arrival_X=arclen(pose_X→C)/eff_speed_X`, point=C. 비교차 → `arrival_a=arrival_b=t_conflict`(동률), point=midpoint. *(midpoint-최근접샘플 arrival 은 대칭교차서 gap<epsilon 으로 붕괴 → right_of_way 도달불가 → 기각, Plan-agent.)*
   **승자**: (a) priority_dominant ∧ 양쪽 priority ∧ 상이 → 고우선 승 → (b) `|Δarrival|≥epsilon` → 먼저도착 승(right_of_way) → (c) else 낮은 id(id_tiebreak). 패자 양보.
3. robot-obstacle: 장애물 등속투영 `(x+vx*t,y+vy*t)`, `dist<obstacle_conflict_radius_m` 첫 i → **로봇 무조건 양보**(reason obstacle, winner None). 멀어지는 장애물 → 충돌 0.
4. **priorities 배열(위상 layering)**: baseline `prio[id]=priority or (N-(id-1))`(F-2 기본과 동일 규약). decision 으로 `winner→loser` 방향그래프 → **cycle 검출**(Kahn, id-순 큐; 잔여 cycle 시 사전식 최대 edge 드롭+log). **위상순**(승자 먼저)으로 `prio[loser]=min(prio[loser], prio[winner]-1)` → 한 로봇이 승자이자 패자여도 모든 edge 에 `winner>loser` 보장. **obstacle 결정은 배열 불변.**

**결정론 핀(T6)**: 난수/wallclock 무. robots/pairs/obstacles **id 정렬** 순회(set 금지). `dist²<radius²` 비교(보고용만 sqrt). 스냅·최근접 동률 → 최소 호장. Kahn 큐 id 순. → 동일입력 동일출력.

### T1~T7 매핑 (Plan-agent 수치검증)
- **T1** amr1(0,0)→(10,0)·amr2(5,-5)→(5,5), 1m/s. 첫 충돌 **t=4.0**(d=1.414<1.5); (5,0) 교차 arrival 5.0 동률 → **id_tiebreak** → amr1 승. assert: robot-robot 충돌 1(amr1,amr2), reason id_tiebreak, prio[amr1]>prio[amr2].
- **T2** amr1(0,0)→(10,0)·amr2(0,3)→(10,3) → same-index sep≡3>1.5 → **충돌 0, 결정 0**, priorities==baseline.
- **T3** amr1(0,0)→(10,0)·amr2(2,-5)→(2,5) → same-index 최소 sep 2.121@t=3.5 > 1.5 → **충돌 0**(FP 방지).
- **T4** amr1(0,0)→(10,0)@1; 장애물A(5,5)vy=-1 → 충돌 t=4.0(d=1.414<2.0) → amr1 양보(reason obstacle, winner None). 장애물B(5,5)vy=+1(멀어짐) → 충돌 0/결정 0.
- **T5 far** amr1(0,0)→(10,0)·amr2(8,0)→(0,0)@1, 현 8m>handoff → 충돌 t=3.5, arrival 동률@4.0 → id_tiebreak → **결정 존재(proactive)**. **T5 close** amr2(1.5,0) 현 1.5<2.0 → 스킵 → **결정 없음**(handoff).
- **T6** 대칭교차(T1) → arrival 동률 → 낮은 id; 2회 호출 → Result **동일**(flapping 0).
- **T7** amr1(0,0)→(10,0)(교차 arrival 5.0, **먼저**)·amr2(5,-6.5)→(5,3.5)(arrival 6.5); 충돌 t=5.5, gap 1.5>epsilon. amr1.priority=1, amr2.priority=5.
  - pd=true → **amr2 승**(reason priority_dominant) — amr1 right-of-way override(CLAUDE.md T7).
  - pd=false → **amr1 승**(reason right_of_way, 1.5s 먼저).

### Hardening (T1~T7 외 추가 — 게이트 강화)
- **H1** 고-id 가 먼저도착(>epsilon)·무priority·pd=false → **고-id 승**(reason right_of_way) — right_of_way ≠ id_tiebreak 입증.
- **H2** 4대 무충돌 → `priorities==[4,3,2,1]`(F-2 default_prio `N-(id-1)` 규약 일치) → baseline 발행=F-2 무영향 입증.
- **H3** 한 로봇이 한쪽에 지고 다른쪽 이김 → 두 edge 모두 winner>loser → 위상 layering 검증.
- **H4** robot-obstacle 충돌 → `priorities==baseline`(결정 기록·배열 불변).

---

## 4. 노드 설계 — `f3_priority_node.py`
- 파라미터: §8.4 전체 + `robots`(F-2 포맷) + `map_frame`/`base_frame_suffix`/`self_filter_radius_m`/`use_sim_time`.
- 구독(robot N=1..4): `/{name}/global_path`(Path→[(x,y)]), `/{name}/odometry/filtered`(Odometry→speed), TF `map→{name}/base_footprint`(pose). + 단일 `/perception/tracked_objects`(TrackedObjectArray→obstacles, **self-filter**).
- **P-10 CCTV 미구현** → tracked_objects 하나만, CCTV 전용경로 없음(§8.3).
- 발행: `/fleet/priorities`(Int32MultiArray, **latched**, **변할 때만**); `/fleet/conflict_predictions`(MarkerArray, 디버그).
- 타이머 @`publish_rate_hz`(5): pose/speed/path/obstacle 갱신 → robot dict → `compute_priorities` → (변하면)배열+마커 발행.
- 코어 import: 노드와 `fleet_priority.py` 가 `lib/amr_fleet/` 동거(스크립트 dir = sys.path[0]) → `import fleet_priority` 동작. ModuleNotFoundError 시 §3 PYTHONPATH 패턴(**env.sh 미편집** → STOP+BLOCKED.md).

---

## 5. 빌드/테스트 배선
- `CMakeLists.txt` **추가**: `install(PROGRAMS src/f3_priority_node.py src/fleet_priority.py DESTINATION lib/${PROJECT_NAME})`,
  BUILD_TESTING 내 `find_package(ament_cmake_pytest REQUIRED)` + `ament_add_pytest_test(test_fleet_priority test/test_fleet_priority.py TIMEOUT 120)`.
- `package.xml` **추가**: `<test_depend>ament_cmake_pytest</test_depend>`, `<depend>visualization_msgs</depend>`.
- `test/test_fleet_priority.py`: `sys.path.insert(0, <pkg>/src)` → `import fleet_priority`; 순수함수 assert.
- 신규 `src/*.py` **chmod +x**.

---

## 6. 검증 (§5)
1. **GATE-F3-LOGIC(키스톤)**: `colcon build --packages-select amr_fleet --symlink-install` → `colcon test --packages-select amr_fleet && colcon test-result --verbose`. 원출력 PROGRESS.md 에 그대로(위조 금지). PASS=T1~T7(+H1~H4) green.
2. **스모크(보너스, §5b)**: 클린재시작(§2) → 4대 + F-3 → ≤180s sim, `timeout 900`. GATE-T(TF)·GATE-TOPIC(이중prefix 0)·GATE-F3-PUB(`/fleet/priorities`+`/fleet/conflict_predictions` 살아있음·F-2 무파손)·GATE-COLLISION(bag rect overlap 0, `f1_common.poly_overlap` 재사용). 멈춤/AMCL 드리프트 → kill + "inconclusive" + 넘어감(재시도 루프 금지).

## 7. 커밋 규율 (§6-5, §0)
- 명시 경로만(**`git add -A` 금지**), `install/`/빌드산출물 **미커밋**.
- 순서: (1) PLAN.md → (2) 코어+테스트(GATE PASS 후) → (3) 노드+config+launch+CMake/pkg → (4) PROGRESS.md Morning Summary.
- **`feature/fleet-f3-priority` 만**. dev merge/rebase/PR 금지.
- 같은 게이트 3회 FAIL or 인터페이스 변경 필요 → `BLOCKED.md` + 멈춤.
