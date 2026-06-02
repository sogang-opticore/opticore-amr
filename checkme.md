# checkme.md

이 문서는 `feature/dwa-ys`를 `dev` 브랜치로 PR 올릴 때 리뷰어와 각 팀원의 LLM에게 먼저 읽히기 위한 작업 요약 문서입니다.

대상 리뷰어: @us788, @pcycoding

---

## PR 메시지 초안

### 제목

```text
feat(nav): add custom A* + DWA navigation with dynamic obstacle avoidance
```

### 본문

```markdown
## Summary

직접 구현한 A* global planner와 DWA local controller를 `dev`에 병합하기 위한 PR입니다.

이번 작업은 Nav2 planner를 가져다 쓰지 않고, 프로젝트 요구사항에 맞춰 `/goal_pose → /global_path → /cmd_vel` 흐름을 직접 구현한 navigation stack입니다. 새 warehouse map, 동적 장애물 시나리오, A* 재계획, DWA 경로 추종/복구/동적 장애물 회피 로직을 포함합니다.

Reviewers: @us788 @pcycoding

## Main Changes

- Add custom A* global planner
  - `/goal_pose`와 현재 pose를 기준으로 `/global_path` 발행
  - static map inflation과 clearance cost 기반으로 벽에 너무 붙는 경로를 완화
  - dynamic obstacle overlay(`/dynamic_obstacle_layer`)를 static map 위에 합성해 우회 경로 재계획
  - stale/empty path 방어 및 goal dedup 보강

- Add custom DWA local controller
  - `/global_path`를 구독하고 DWA가 `/cmd_vel`을 단독 발행
  - Pure Pursuit 기반 path tracking
  - adaptive velocity, trajectory clearance, turn clearance brake, near-wall creep 적용
  - `NORMAL`, `REJOIN`, `ALIGN`, `RECOVERY`, `EMERGENCY`, `REACHED` 등 상태기계 적용
  - `GOAL_REACHED` edge와 `REACHED` hold로 목표 도착 상태 안정화

- Improve path rejoin and goal approach
  - 경로 이탈 시 가장 가까운 path point가 아니라 미래 합류 후보를 비용 기반으로 선택
  - heading, distance, curvature, clearance 비용으로 REJOIN target 평가
  - goal 방향이 LiDAR/dynamic layer 기준으로 안전하면 global path 복귀보다 goal shortcut을 선호

- Add dynamic obstacle handling
  - LiDAR 점에서 static wall을 필터링하고 dynamic cluster/track 생성
  - CPA(`t_cpa`, `d_cpa`)와 swept segment prediction으로 접근/횡단/이탈 장애물 판단
  - 동적 장애물을 `/dynamic_obstacle_layer` no-go 영역으로 발행해 A*가 우회 경로를 만들도록 함
  - 한 번 등록된 dynamic no-go 영역은 최소 30초 유지
  - 로봇이 dynamic no-go 내부에 들어가면 Pure Pursuit를 중단하고 local escape 수행
  - 후방 접근 동적 장애물도 예측 경로가 로봇/path를 침범하면 no-go 후보로 등록

- Add launch/config/test support
  - `nav_full.launch.py`, `astar_dwa.launch.py`, `dwa_only.launch.py`, `astar_only.launch.py`
  - `dwa_params.yaml`, `astar_params.yaml`
  - A*, DWA, frontier explorer, heuristics 테스트 추가/보강
  - 새 warehouse world/map 및 dynamic obstacle mover 시나리오 반영

## How to Test

RunPod 기준:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select amr_navigation amr_bringup amr_slam
source install/setup.bash
ros2 launch amr_bringup warehouse.launch.py
```

새 터미널:

```bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 launch amr_navigation nav_full.launch.py
```

이후 Foxglove/RViz 또는 topic pub으로 `/goal_pose`를 넣고 다음을 확인합니다.

- `/global_path`가 목표까지 생성되는지
- `/cmd_vel` 발행자가 DWA 하나인지
- `/dwa/status`가 `NORMAL → REJOIN/AVOIDING_DYNAMIC/... → GOAL_REACHED/REACHED`로 정상 전이되는지
- 벽 근접 시 `STOPPED_NEAR_WALL`/`EMERGENCY`가 반복되지 않는지
- 동적 장애물 감지 시 `/dynamic_obstacle_layer`가 발행되고 A*가 우회 경로를 만드는지
- 로봇이 dynamic no-go 내부에 들어간 경우 `INSIDE_DYNAMIC_ZONE`에서 escape하는지

## Known Risks / Review Focus

- Dynamic obstacle layer는 안전성과 속도의 trade-off가 큽니다. 영역이 너무 작으면 충돌하고, 너무 크면 우회가 과하거나 로봇이 멈출 수 있습니다.
- 최신 no-go 내부 탈출 로직은 실제 동적 장애물 시나리오에서 재검증이 필요합니다.
- `warehouse.launch.py`, `warehouse.world`는 공동 소유 파일이므로 map/world 변경 의도를 함께 확인해주세요.
- PR diff가 크므로 리뷰는 `amr_navigation` → dynamic layer/A* 연동 → bringup/world 순서로 보는 것을 추천합니다.
```

---

## 팀원 LLM에게 먼저 줄 요약

이 PR은 단순 튜닝 PR이 아니라 `amr_navigation`의 핵심 navigation 흐름을 직접 구현한 작업입니다.

큰 구조는 다음과 같습니다.

```text
/goal_pose
  ↓
A* global planner
  ↓ /global_path
DWA local controller
  ↓ /cmd_vel
Gazebo / robot
```

중요한 원칙은 다음과 같습니다.

- Nav2의 `dwb_local_planner`, `nav2_navfn`, `nav2_smac_planner`를 사용하지 않습니다.
- A*는 경로만 만들고 `/cmd_vel`을 발행하지 않습니다.
- DWA만 `/cmd_vel`을 발행합니다.
- `/global_path`는 map frame 기준입니다.
- DWA는 path를 local frame으로 변환해서 Pure Pursuit + 안전 제약 + 상태기계로 제어합니다.
- 동적 장애물은 DWA가 LiDAR로 감지하고 `/dynamic_obstacle_layer`로 A*에 알려줍니다.

---

## 먼저 봐야 할 파일

리뷰 또는 LLM 분석은 아래 순서로 보는 것을 추천합니다.

1. `ros2_ws/src/amr_navigation/README.md`
   - A* ↔ DWA 토픽 계약, 상태값, 파라미터 의도 정리.

2. `ros2_ws/src/amr_navigation/amr_navigation/dwa_node.py`
   - DWA local controller 본체.
   - Pure Pursuit, REJOIN, dynamic obstacle tracking, no-go escape가 모두 여기에 있음.

3. `ros2_ws/src/amr_navigation/amr_navigation/astar_node.py`
   - A* global planner 본체.
   - static map, dynamic obstacle overlay, path 재계획 정책 확인.

4. `ros2_ws/src/amr_navigation/config/dwa_params.yaml`
   - DWA 파라미터 기본값.

5. `ros2_ws/src/amr_navigation/config/astar_params.yaml`
   - A* clearance, inflation, replan 파라미터.

6. `ros2_ws/src/amr_navigation/test/test_dwa.py`
   - DWA 순수 함수, goal/rejoin/dynamic obstacle 관련 회귀 테스트.

7. `ros2_ws/src/amr_navigation/test/test_astar_clearance.py`
   - A* clearance/path switching 관련 테스트.

8. `ros2_ws/src/amr_bringup/launch/warehouse.launch.py`
   - Gazebo world 실행.

9. `ros2_ws/src/amr_bringup/src/dynamic_obstacle_mover.py`
   - 동적 장애물 테스트 시나리오.

---

## 핵심 로직 설명

### 1. A* Global Planner

역할:

- 현재 로봇 위치와 `/goal_pose`를 기준으로 목적지까지 가는 전역 경로를 계산합니다.
- 결과는 `/global_path`로 발행합니다.
- DWA는 이 path를 따라 실제 속도 명령을 만듭니다.

주요 특징:

- static map에서 obstacle inflation을 적용합니다.
- 단순 최단거리만 보지 않고 clearance cost를 반영해 벽에 너무 붙는 경로를 완화합니다.
- DWA가 발행한 `/dynamic_obstacle_layer`를 static map 위에 overlay해 동적 장애물 영역을 지나갈 수 없는 영역으로 간주합니다.
- 주기 재계획 중 실패했다고 바로 빈 path를 발행하지 않습니다. 기존 성공 path가 있는 상황에서 일시 실패가 발생하면 DWA가 불필요하게 `STOPPED`로 흔들리지 않도록 기존 path를 유지합니다.

리뷰 포인트:

- dynamic obstacle overlay가 static map 좌표계와 resolution/origin을 정확히 맞춰 합성되는지.
- 빈 path 발행 정책이 새 goal 최초 실패와 기존 path 유지 상황을 올바르게 구분하는지.
- path switching이 너무 자주 발생하지 않는지.

---

### 2. DWA Local Controller

현재 DWA는 고전적인 "속도 샘플링 점수 최적화"만으로 움직이지 않습니다.

기본 제어는 다음 조합입니다.

```text
Pure Pursuit
+ adaptive velocity
+ trajectory clearance check
+ state machine
+ dynamic obstacle handling
```

역할:

- `/global_path`를 구독합니다.
- 현재 odom/TF와 path를 비교합니다.
- LiDAR로 장애물 여유를 계산합니다.
- 최종 `/cmd_vel`을 발행합니다.

중요 함수/개념:

- `project_to_path()`
  - 현재 로봇이 path 어디에 투영되는지 계산합니다.

- `pick_lookahead_from_projection()`
  - 현재 위치보다 앞쪽의 lookahead point를 고릅니다.

- Pure Pursuit
  - lookahead point를 향하는 곡률을 계산합니다.

- `forward_simulate()`
  - 현재 명령으로 앞으로 짧은 시간 이동했을 때의 궤적을 예측합니다.

- `min_clearance_distance()`
  - 예측 궤적과 장애물 점 사이 최소 거리를 계산합니다.

리뷰 포인트:

- `/cmd_vel` 발행자가 DWA 하나인지.
- path를 local frame으로 변환하는 과정에서 frame mismatch가 없는지.
- final collision check가 실제 `v_cmd`, `w_cmd` 기준으로 수행되는지.

---

### 3. REJOIN

문제:

로봇은 global path 선 위에 강제로 붙어서 움직이지 않습니다. Pure Pursuit는 path 위의 앞쪽 lookahead point를 보고 움직이므로 코너, 벽 회피, 속도 제한, 회전 한계 때문에 path에서 살짝 벗어날 수 있습니다.

기존 단순 방식:

```text
path에서 벗어남
→ 가장 가까운 path point로 복귀
```

이 방식은 큰 회전과 지그재그를 유발했습니다.

현재 방식:

```text
path에서 벗어남
→ 미래 path 후보 여러 개 평가
→ heading / distance / curvature / clearance 비용이 낮은 합류점 선택
```

관련 파라미터:

- `rejoin_entry_offset`
- `rejoin_exit_offset`
- `rejoin_exit_heading`
- `rejoin_min_lookahead`
- `rejoin_max_lookahead`
- `rejoin_clearance_min`
- `rejoin_clearance_weight`

리뷰 포인트:

- REJOIN이 너무 자주 들어가면 path tracking이 약한 것입니다.
- REJOIN이 너무 오래 지속되면 합류 target 선택이 보수적이거나 goal shortcut 조건이 빡빡한 것입니다.
- REJOIN 중 cross-track gain은 일부러 줄였습니다. 가장 가까운 선으로 강하게 끌어당기면 오히려 큰 회전이 생깁니다.

---

### 4. Goal Shortcut

문제:

로봇이 path에서 벗어난 뒤 눈으로 보기에는 goal로 바로 가면 빠른 상황에서도, 기존 REJOIN은 일단 path 위로 돌아가려는 경향이 있었습니다.

현재 방식:

- goal 방향이 충분히 열려 있는지 확인합니다.
- LiDAR arc clearance를 확인합니다.
- dynamic no-go layer와 교차하지 않는지 확인합니다.
- REJOIN 후보보다 비용이 낮으면 path 복귀보다 goal shortcut을 사용합니다.

관련 로그:

- `gcut=1`
- `gdist`
- `gclr`
- `garc`
- `gdyn`
- `gscore`

리뷰 포인트:

- 장애물이 가까운 상황에서 `gcut=1`이 켜지면 위험합니다.
- 동적 장애물 no-go 영역을 shortcut이 관통하지 않는지 확인해야 합니다.

---

### 5. Wall / Clearance Safety

벽 근처에서 멈추거나, 빠르게 회전하다가 벽으로 말려 들어가는 문제를 줄이기 위해 clearance 기반 제한을 추가했습니다.

주요 개념:

- forward clearance
  - 로봇 정면 장애물 여유.

- trajectory clearance
  - 현재 `v_cmd`, `w_cmd`로 실제 지나갈 arc 위의 장애물 여유.

- rotation clearance
  - 제자리 회전 가능 공간.

- turn clearance brake
  - 큰 회전 중 전방/arc 여유가 작으면 속도를 줄임.

- near-wall creep
  - 벽 근처에서 완전히 멈추기보다, 안전 조건을 만족하면 아주 낮은 속도로 빠져나오게 함.

리뷰 포인트:

- `STOPPED_NEAR_WALL`이 반복되면 clearance 관련 파라미터를 봐야 합니다.
- `EMERGENCY`가 반복되면 recovery 또는 A* replan 연동을 봐야 합니다.

---

### 6. Dynamic Obstacle Layer

문제:

A*는 static map만 보면 동적 장애물을 모릅니다. 따라서 global path가 동적 장애물을 관통할 수 있습니다.

현재 방식:

```text
LiDAR scan
  → static wall filter
  → dynamic cluster
  → track association
  → velocity / CPA / swept prediction
  → /dynamic_obstacle_layer 발행
  → A*가 우회 경로 재계획
```

중요 개념:

- Static map filter
  - 정적 벽/기둥을 동적 장애물로 오인하지 않도록 제거합니다.

- Dynamic track
  - LiDAR cluster를 시간에 따라 연결해 위치와 속도를 추정합니다.

- CPA
  - `t_cpa`: 가장 가까워질 때까지 남은 시간.
  - `d_cpa`: 그 순간의 예상 최소 거리.

- Swept prediction
  - 장애물이 현재 위치에만 있는 것이 아니라 앞으로 움직일 선분까지 고려합니다.

- No-go layer
  - 동적 장애물 현재 위치, trail, prediction 영역을 임시 점유 영역으로 등록합니다.

관련 파라미터:

- `dynamic_layer_min_hold_sec = 30.0`
- `dynamic_layer_clear_confirm_sec = 2.0`
- `dynamic_layer_radius_margin`
- `dynamic_layer_prediction_horizon`
- `dynamic_layer_prediction_max_distance`
- `dynamic_layer_trail_ttl_sec`
- `dynamic_layer_max_blocks`

리뷰 포인트:

- no-go 영역이 너무 작으면 충돌합니다.
- no-go 영역이 너무 크면 로봇이 느려지거나 갇힙니다.
- false positive로 정적 벽이 dynamic layer에 들어가지 않는지 확인해야 합니다.
- `/dynamic_obstacle_layer`가 Foxglove에서 `/map`을 검게 덮지 않는지 확인해야 합니다.

---

### 7. INSIDE_DYNAMIC_ZONE Escape

문제:

로봇이 dynamic no-go 영역에 들어간 상태에서도 기존 Pure Pursuit가 계속 실행되어 충돌 위험이 있었습니다.

현재 방식:

- `inside_dynamic_layer=True`이면 정상 Pure Pursuit를 즉시 중단합니다.
- `INSIDE_DYNAMIC_ZONE` 상태로 전환합니다.
- 가장 가까운 no-go feature의 반대 방향으로 local escape vector를 계산합니다.
- 전방 탈출이 안전하면 저속 전진합니다.
- 전방이 막혔으면 회전합니다.
- 후방이 안전한 경우 짧은 후진을 허용합니다.

관련 로그:

- `inside dynamic no-go; suppressing Pure Pursuit and escaping`
- `mode=forward`
- `mode=rotate`
- `mode=reverse`
- `mode=front_blocked_rotate`
- `dyninside=1`
- `dyn_in_margin`

리뷰 포인트:

- `dyninside=1`인데 `PP[INSIDE_DYNAMIC_ZONE]`가 계속되면 버그입니다.
- inside escape는 A* 재계획이 오기 전까지 로봇을 no-go 바깥으로 밀어내는 local safety behavior입니다.

---

### 8. Goal Reached / Rearm

문제:

첫 goal에 도착한 뒤 다음 goal이 들어와도 움직이지 않거나, goal 주변에서 배회하는 문제가 있었습니다.

현재 방식:

- goal 근처에서는 `goal_approach_speed_limit()`로 속도를 낮춥니다.
- goal tolerance 안에 들어오면 `GOAL_REACHED` edge를 1회 발행합니다.
- 이후 `REACHED` 상태를 유지합니다.
- 새 goal/path가 들어오면 reached latch를 해제하고 다시 주행합니다.

리뷰 포인트:

- `/dwa/status`에서 `GOAL_REACHED`가 도착 순간 한 번만 나오는지.
- `REACHED` 이후 새 goal에서 정상적으로 재출발하는지.

---

## 상태 로그 해석

주요 `/dwa/status` 값:

| 상태 | 의미 |
|---|---|
| `NORMAL` | 정상 path 추종 |
| `REJOIN` | path 이탈 후 미래 path 지점으로 재합류 중 |
| `ALIGN` | heading 오차가 커서 제자리 회전 정렬 |
| `AVOIDING_DYNAMIC` | 동적 장애물 side-offset 회피 중 |
| `DYNAMIC_BLOCKED` | 동적 장애물이 global path corridor를 막음 |
| `APPROACHING_DYNAMIC` | 동적 장애물이 로봇 쪽으로 접근 중 |
| `INSIDE_DYNAMIC_ZONE` | 로봇이 dynamic no-go 내부에 있음 |
| `STOPPED_NEAR_WALL` | 벽 근처 저속/정지 보호 |
| `RECOVERY` | SPIN/FORWARD_ONLY 복구 중 |
| `EMERGENCY` | 충돌 위험으로 정지 |
| `GOAL_REACHED` | 목표 도착 순간 edge |
| `REACHED` | 도착 후 정지 유지 |

주요 로그 키워드:

| 키워드 | 의미 |
|---|---|
| `cte` | 현재 path 횡오차 |
| `pcte` | 예측 path 횡오차 |
| `clr` | 장애물 clearance |
| `fwd` | 전방 clearance |
| `rjc` | REJOIN 후보 clearance |
| `gcut=1` | goal shortcut 활성 |
| `dynblk=1` | 동적 장애물이 path corridor를 막음 |
| `dyninside=1` | 로봇이 dynamic no-go 내부에 있음 |
| `dyn_tcpa` | 동적 장애물과 가장 가까워질 예상 시간 |
| `dyn_dcpa` | 그 순간 예상 최소 거리 |
| `dyn_layer` | active dynamic layer block 수 |

---

## 테스트 체크리스트

### 기본 주행

- [ ] Gazebo warehouse world가 정상 로딩된다.
- [ ] map server / AMCL / TF가 정상이다.
- [ ] `/goal_pose` 입력 시 A*가 `/global_path`를 발행한다.
- [ ] DWA가 `/cmd_vel`을 단독 발행한다.
- [ ] 로봇이 goal까지 도착한다.
- [ ] 도착 시 `GOAL_REACHED` 후 `REACHED`가 나온다.
- [ ] 다음 goal 입력 시 다시 움직인다.

### 벽 근접 안정성

- [ ] 벽 근처에서 `EMERGENCY`가 반복되지 않는다.
- [ ] `STOPPED_NEAR_WALL`이 짧게 발생하더라도 복귀 가능하다.
- [ ] `REJOIN`이 너무 길게 지속되지 않는다.
- [ ] `clr < 0.5m`가 과도하게 반복되지 않는다.

### 동적 장애물

- [ ] 실제 동적 장애물이 `/dynamic_obstacle_layer`에 등록된다.
- [ ] 정적 벽/기둥이 dynamic layer로 오인되지 않는다.
- [ ] dynamic layer 등록 후 A*가 우회 경로를 만든다.
- [ ] no-go 영역이 너무 빨리 사라지지 않는다.
- [ ] 로봇이 no-go 내부에 들어간 경우 `INSIDE_DYNAMIC_ZONE` escape가 실행된다.
- [ ] 후방 접근 동적 장애물도 위험하면 감지된다.

---

## 리뷰할 때 주의할 점

1. 이 PR은 diff가 크므로 파일 전체를 처음부터 끝까지 보기보다 navigation 흐름 기준으로 보세요.

2. DWA는 path 선 위에 로봇을 강제로 붙이는 코드가 아닙니다.
   - path는 reference입니다.
   - 실제 제어는 Pure Pursuit, clearance, velocity limit, state machine을 통과한 결과입니다.

3. REJOIN이 있다는 것은 버그가 아니라 정상 설계입니다.
   - 다만 너무 자주/오래 발생하면 튜닝 대상입니다.

4. dynamic layer는 안전을 위한 임시 costmap 성격입니다.
   - static map 자체를 바꾸는 것이 아닙니다.
   - 일정 시간 유지 후 관찰이 사라지면 해제됩니다.

5. 최신 dynamic no-go 내부 탈출 로직은 꼭 실제 시나리오로 검증해야 합니다.
   - 특히 `PP[INSIDE_DYNAMIC_ZONE]`가 다시 나오면 안 됩니다.

---

## LLM에게 요청하면 좋은 리뷰 질문

팀원들이 각자 사용하는 LLM에게 아래 질문을 그대로 던지면 리뷰가 수월합니다.

```text
이 레포의 checkme.md를 먼저 읽고, ros2_ws/src/amr_navigation/README.md와
dwa_node.py, astar_node.py를 중심으로 navigation PR을 리뷰해줘.

특히 다음을 봐줘.
1. /cmd_vel 단독 발행 원칙이 깨지는 곳이 있는가?
2. /global_path, /goal_pose, /dynamic_obstacle_layer의 frame/QoS 계약이 깨지는 곳이 있는가?
3. DWA가 dynamic no-go 내부에서 Pure Pursuit를 계속 실행할 가능성이 있는가?
4. A*가 dynamic overlay를 static map에 합성할 때 좌표 변환 문제가 있는가?
5. REJOIN/goal shortcut이 장애물이나 dynamic no-go를 관통할 위험이 있는가?
6. 새 goal 수신 후 REACHED latch가 풀리지 않는 경우가 있는가?
7. 테스트가 부족한 고위험 분기가 있는가?
```

---

## 현재 브랜치 기준 참고 커밋

최근 navigation 관련 커밋:

```text
0f7e9c7 fix(nav): escape dynamic no-go zones locally
7859aed tune(nav): prefer safe goal shortcuts
262a4b6 fix(nav): predict rear dynamic obstacles
9dfce8b fix(nav): hold dynamic layer longer
f9cf580 fix(nav): constrain dynamic layer growth
60e2c7f fix(nav): crop dynamic obstacle layer
b3758cd tune(nav): trim dynamic obstacle layer
282116c tune(nav): smooth dynamic obstacle layer updates
7c6616e tune(nav): prefer locked dynamic path branch
f1c44ad fix(nav): lock dynamic astar path branch
7313b56 fix(nav): add dynamic start escape corridor
fc42ffd fix(nav): harden dynamic obstacle swept layer
```

---

## 최종 한 줄 요약

이 PR의 핵심은 A*가 만든 global path를 DWA가 단순히 따라가는 수준을 넘어서, 벽 근접, 경로 이탈, 목표 도착, 동적 장애물까지 고려해 실제 주행 가능한 navigation stack으로 만든 것입니다.
