# F-3 Fleet 무인 런 — PROGRESS (아침 대시보드)

브랜치 `feature/fleet-f3-priority`. dev/f2-deadlock/f1-cc/spawn 미접촉, PR 미생성, install/ 미커밋.
F-2 노드(deadlock_manager/priority_publisher)·amr_msgs·perception·maps·world·env.sh **불가침 유지**.

## TL;DR (아침 30초 요약)
- **★ GATE-F3-LOGIC PASS — 키스톤 달성.** `fleet_priority` 순수코어 단위테스트 **pytest 13/13**
  (T1~T7 + 보강 H1~H6), 결정론, sim 불필요. **이게 야간 런의 진짜 성공 신호(§5a).**
- F-3 = additive proactive priority. 시간기반 충돌예측 → `/fleet/priorities`(Int32MultiArray latched)에
  **일찍 주입**. F-2 enforcement(HOLD/RETREAT/RESUME) 무재구현, `/amrN/goal_pose` 무발행.
- 노드/launch/config 구현·빌드·boot·런타임 경로(마커·self-filter) 검증 완료. **lint: F-3 파일 0건.**
- **★ 스모크(§5b, 보너스) = PASS (controlled integration)**: 라이브 DDS 에서 F-3 가 합성 TF+path+odom
  를 받아 `/fleet/priorities = [2,3,2,1]` (amr2 right_of_way 승) 발행 → **F-2 가 수신·반영** 확인.
  미국지화 amr3/amr4 는 baseline(2,1) 유지. (Gazebo 풀 bringup 대신 통제교차 — 아래 "스모크" 절 근거.)
- **사람 결정 필요 없음**(무인 위임 범위). 단 §"판단 결정" 4건은 아침 검토 권장.

## 현재 위치
- [완료] EXPLORE(F-2 인터페이스 grep + 병렬 Explore 2 + Plan-agent 수치검증) · PLAN(`8d09264`).
- [완료] 🔴 순수코어 `fleet_priority.py` + 테스트 + GATE-F3-LOGIC(`4d81a81`).
- [완료] 🔴 ROS2 노드 + config + 독립 launch(`27a5fb3`, boot/런타임 검증).
- [완료] 부분 국지화 robustness fix(`e925570`, **pytest 13/13** — H6 추가).
- [완료] 스모크(§5b, 보너스) = **PASS** (controlled F-2↔F-3 integration).

## 게이트 결과 (정량)
| 게이트 | 방식 | 결과 |
|---|---|---|
| **GATE-F3-LOGIC** (키스톤) | **unit (colcon test pytest)** | **PASS 13/13** (T1~T7 + H1~H6, 결정론) |
| 코어 lint (F-3 파일) | ament_flake8/pep257 | **PASS 0건** (fleet_priority.py · test · node · launch) |
| 노드 boot | ros2 run(wall clock) | **PASS** (`__init__`·스핀·empty-data 틱 무오류) |
| 노드 런타임 경로 | 직접 호출 | **PASS** (마커 Duration·robot/obstacle 마커·self-filter 드롭) |
| **GATE-F3-PUB** (스모크) | **controlled DDS** | **PASS** (F-3 발행 [2,3,2,1] · F-2 수신·반영 · F-2 무파손) |

### GATE-F3-LOGIC 원출력 (위조 금지 — colcon 실제)
```
$ colcon test --packages-select amr_fleet && colcon test-result --verbose
# pytest xunit (build/amr_fleet/test_results/amr_fleet/test_fleet_priority.xunit.xml):
<testsuite name="pytest" errors="0" failures="0" skipped="0" tests="13" time="0.266" ...>
  test_T1_robot_robot_crossing                  PASS
  test_T2_parallel_no_conflict                  PASS
  test_T3_temporal_separation_no_false_positive PASS
  test_T4_robot_obstacle_yield_and_recede       PASS
  test_T5_handoff_far_decides_close_skips       PASS
  test_T6_determinism_and_tie_lower_id          PASS
  test_T7_priority_dominant_overrides_right_of_way PASS
  test_H1_right_of_way_distinct_from_id         PASS
  test_H2_baseline_equals_f2_default            PASS
  test_H3_layering_winner_and_loser             PASS
  test_H4_obstacle_does_not_change_array        PASS
  test_H5_cycle_break_deterministic             PASS
  test_H6_unlocalized_robot_kept_in_baseline    PASS
# colcon test 레벨: 5 tests, 2 failures (flake8, pep257) — **전부 PRE-EXISTING F-2**:
#   flake8.xunit:  1 failure → ./src/deadlock_manager.py:251 (E702 semicolon)
#   pep257.xunit:  8 failures → ./src/deadlock_manager.py:207/255/262/333 (D40x/D205/D209/D213)
#   → F-3 파일(fleet_priority.py·test_fleet_priority.py·f3_priority_node.py)은 lint 0건.
#   → §8.1 상 F-2 노드 불가침 → 잔여 lint 미수정(F-3 기여 lint 부채 = 0).
```

## 스모크 (§5b, 보너스) — controlled F-2↔F-3 integration = PASS
**근거(왜 풀 Gazebo bringup 대신 통제교차):** F-2 PROGRESS 가 규명한 대로 현 warehouse 는 개방그리드
→ 4대 bringup 해도 A* 가 우회해 **경로교차가 안 생김** → F-3 는 baseline 만 발행(약한 신호). 그래서
F-3 의 진짜 약속(=충돌예측→우선순위 주입→F-2 반영)을 **통제된 교차**(H1형: amr2 가 교차점 먼저 도착)
로 라이브 검증. 합성 입력(TF map→amrN/base_footprint · /amrN/global_path · /amrN/odometry/filtered)을
`validation/fleet_f3/smoke_inputs.py`(비커밋)로 발행하고 실 설치 노드를 띄움.
```
# F-3 발행측 로그:
[f3_priority_node]: f3_priority_node up — robots=['amr1','amr2','amr3','amr4'] ...
[f3_priority_node]: /fleet/priorities 갱신: [2, 3, 2, 1] (decisions=[(2, 1, 'right_of_way')])
# F-2 수신측 로그(deadlock_manager):
[deadlock_manager]: deadlock_manager up — default_prio={'amr1':4,'amr2':3,'amr3':2,'amr4':1} ...
[deadlock_manager]: /fleet/priorities 수신·반영: {'amr1':2,'amr2':3,'amr3':2,'amr4':1}
```
- **GATE-F3-PUB PASS**: F-3 crash 없이 뜸 → `/fleet/priorities` 발행 → **F-2 수신·반영** → F-2 무파손.
- **proactive 정확**: amr2 가 교차점 먼저 도착 → `right_of_way` 로 amr2 승, amr1 을 4→2 로 강등(amr2=3 아래).
- **부분 국지화 라이브 검증**: TF 없는 amr3/amr4 는 baseline(2,1) 유지(absent→0 버그 없음, H6 의 sim 판).
- 주의: `ros2 topic echo` CLI 는 이 환경 ros2 데몬 이슈(`!rclpy.ok()` xmlrpc)로 실패 — **노드간 DDS 전달
  자체는 위 F-2 수신 로그로 직접 입증**(CLI echo 불필요). GATE-T/TOPIC/COLLISION(풀 bringup TF·bag)은
  F-1/F-2 가 이미 검증한 인프라 → 본 통제 스모크 범위 밖(필요 시 §"HU 검증" 3번으로 실 sim 가능).

## 핵심 결론 (설계 검증)
1. **F-2 인터페이스 = `/fleet/priorities`(std_msgs/Int32MultiArray, latched, 값 클수록 우선, index=robotN-1).**
   코드로 확정(deadlock_manager.py:17/155/192-211). F-3 baseline=`N-(id-1)`=F-2 default 규약 일치(H2 입증)
   → 무충돌 시 F-3 발행이 F-2 동작 무변경.
2. **충돌예측 핵심**: same-time 샘플 거리 < radius → 충돌(T1 t=4.0). arrival = **geometric path-crossing**
   호장/속도(midpoint 정의는 대칭교차서 right_of_way 붕괴 → Plan-agent 적대검증으로 기각).
   승자 = priority_dominant(T7) → right_of_way(T7/H1) → id_tiebreak(T1/T5/T6).
3. **handoff < 2m 쌍은 F-3 미개입 = F-2 영역**(T5 close) → 명령충돌 방지(layering 안전 핵심).
4. **robot-obstacle = 로봇 무조건 양보**(T4), 단 F-2 는 비-AMR enforce 불가 → **결정/마커로만 노출,
   `/fleet/priorities` 배열 불변**(H4). 정직·additive(과대주장 회피).
5. **priorities 배열 = 위상정렬 layering**(단일패스 감소의 비건전성 회피 — 한 로봇이 승자이자 패자여도
   모든 edge strict, H3). 사이클 입력도 결정론적으로 깸(H5, 방어).

## 판단 결정 (무인 위임 — 아침 검토 대상)
1. **파일 배치 `amr_fleet/src/`** (F-2 ament_cmake 관례; CLAUDE.md §8.1 의 generic `<pkg>/<pkg>/` 아님).
2. **launch 별도**(`amr_fleet/launch/f3_priority.launch.py`) — `amr_bringup/multi_robot.launch.py` 미편집(격리).
3. **robot-obstacle 비강제**(위 결론 4).
4. **T7 좌표 조정** — CLAUDE.md 의도(amr1 먼저·amr2 고우선·override) 유지, same-index 샘플서 arrival gap ≥ ε 되게.

## 환경 함정 / 주의 (재현·오해 방지)
- `amr_fleet` = ament_cmake → **.py 수정 후 `colcon build` 필수**(복사설치). 신규 .py `chmod +x` 완료.
- `rclpy.duration`·`rclpy.time` 은 `import rclpy` 만으론 **자동 노출 안 됨** → 노드서 `from rclpy.duration
  import Duration`/`from rclpy.time import Time` 명시(런타임 잠복버그 차단; empty-data boot 으론 안 잡힘).
- F-2 deadlock_manager 에 PRE-EXISTING flake8/pep257 부채(위 원출력). F-3 무관·불가침.
- ROS 노드 실행 = DDS(Bash 샌드박스 해제 필요). 검증은 모두 bounded `timeout` + PID 추적 kill
  (광역 pkill 금지 — 잔존 ign·self-kill 회피). 스모크 후 잔존 python 노드 명시 kill 완료.
- **exec-bit**: repo `core.fileMode=false` + 모든 amr_fleet `src/*.py` git mode `100644`(F-2 포함).
  `--symlink-install` 은 src 의 +x 에 의존 → F-2 노드는 `ros2 run` 으로 "No executable found" 날 수
  있음(스모크는 `python3 <install_path>` 직접구동으로 우회). **F-3 노드는 `git update-index --chmod=+x`
  로 mode 100755 기록**(§3 준수, fresh clone 재현성). F-2 파일은 불가침이라 100644 유지.
- `ros2` CLI(topic echo/node list)가 이 환경 데몬 이슈로 `!rclpy.ok()` xmlrpc 실패 가능 →
  노드 로그/직접 DDS 로 검증(CLI 의존 회피).

## git diff --stat (vs f2 tip e68581d)
```
 docs/fleet_f3/PLAN.md                              | 137 +++++++
 docs/fleet_f3/PROGRESS.md                          | 153 ++++++++
 ros2_ws/src/amr_fleet/CMakeLists.txt               |   8 +
 ros2_ws/src/amr_fleet/config/f3_params.yaml        |  23 ++
 ros2_ws/src/amr_fleet/launch/f3_priority.launch.py |  47 +++
 ros2_ws/src/amr_fleet/package.xml                  |   3 +
 ros2_ws/src/amr_fleet/src/f3_priority_node.py      | 268 ++++++++++++  (mode 100755)
 ros2_ws/src/amr_fleet/src/fleet_priority.py        | 428 ++++++++++++  (mode 100755)
 ros2_ws/src/amr_fleet/test/test_fleet_priority.py  | 237 +++++++++++
 9 files changed, 1304 insertions(+)   # install/·build/ 미포함, validation/ 비커밋
```

## HU 가 칠 검증 명령어
```bash
# 1) 키스톤 재현 (GATE-F3-LOGIC)
source /workspace/env.sh && cd /workspace/ros2_ws \
  && colcon build --packages-select amr_fleet --symlink-install \
  && colcon test --packages-select amr_fleet && colcon test-result --verbose
#  → pytest 12/12 PASS. (flake8/pep257 잔여 실패는 PRE-EXISTING F-2 deadlock_manager — F-3 무관)

# 1b) 코어만 빠르게 (sim·colcon 불필요)
source /workspace/env.sh \
  && python3 -m pytest /workspace/ros2_ws/src/amr_fleet/test/test_fleet_priority.py -v

# 2) F-3 노드 단독 boot 확인
source /workspace/env.sh && timeout 8 ros2 run amr_fleet f3_priority_node.py
#  → "f3_priority_node up — robots=[amr1..amr4] ..." 로그 (ExternalShutdownException=timeout 정상종료)

# 3) 스모크(보너스): 4대 bringup 후 별도 터미널서
source /workspace/env.sh && ros2 launch amr_bringup multi_robot.launch.py     # 터미널 A
source /workspace/env.sh && ros2 launch amr_fleet f3_priority.launch.py        # 터미널 B
ros2 topic echo /fleet/priorities --once   # 라이브 주입 확인
ros2 topic hz /fleet/conflict_predictions  # 마커 발행 확인
```

## 다음 추천 스텝
1. (사람) f2-deadlock 가 dev 머지되면 이 브랜치 dev rebase 후 PR(§0).
2. (검증 확장) 단일병목 맵(S-9 2존, F-2 PROGRESS 권장)서 **실 교차 교착** 발생 → F-3 proactive 주입이
   F-2 의 2m HOLD 전에 우선순위를 푸는지 end-to-end bag 정량. 현 warehouse 개방그리드는 교차 강제 난망(F-2 발견).
3. (P-10) CCTV 트랙 enrich 시 `/perception/tracked_objects` 그대로 → F-3 무변경(설계대로).
4. (선택) F-3 외부 우선순위 소스(미션 긴급도) 주입구 — 현재 `priority=None`(baseline). 필요 시 토픽/파라미터 추가.
