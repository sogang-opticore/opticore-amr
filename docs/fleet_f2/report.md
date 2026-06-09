# F-2 Fleet 무인 런 — 리포트 (아침 요약)

**결론: 매니저(우선순위 교착 해소) = 구현·검증 완료. 단, 현 맵은 *창발적* 영구 교착을
강제할 수 없음(개방 그리드 = A* 우회). → end-to-end 창발 demo 는 S-9 2존맵 대기.
매니저 핵심(탐지+우선순위 양보)은 유닛테스트 + 실 sim controlled demo + 음성테스트로 입증.**

브랜치 `feature/fleet-f2-deadlock`. dev/f1-cc/spawn 미접촉, PR 미생성, install/ 미커밋.

## 게이트 결과 (정량)
| 게이트 | 방식 | 결과 | 근거 |
|---|---|---|---|
| 🔴-1 우선순위 주입 | sim | **PASS** | default amr1=4..amr4=1 / 주입 [1,4,2,3]→status injected amr1=1 |
| 🔴-2+3 매니저 로직 | **unit** | **PASS 23/23** | 우선순위(default/injected/tie)·탐지 geometry·_is_stuck·FSM 전이 전체·livelock·retreat_point |
| GATE-DETECT(negative) | sim | **PASS** | amr1 단독+person_3, status 122샘플 오발동 **0** |
| GATE-SHARED (controlled) | sim | **PASS** | 두 로봇 같은 점 경합 → 탐지+저우선(amr2) 양보 HOLD→RETREAT→RESUMING, 고우선(amr1) 자원획득 |
| GATE-BASELINE (창발 교착) | sim | **N/A (맵 한계)** | 3회 모두 reroute, closest approach 2.66~2.76m — 영구 교착 불성립([[BLOCKED.md]]) |
| F-1 회귀 4게이트 | sim | (별도 런) | enable_deadlock_manager 기본 그래프 = 추가만, 무회귀 by construction |

## 핵심 기술 결론
1. **순수 goal_pose 액추에이터로 해소** — HOLD(현pose)/RETREAT(지나온경로 D뒤)/RESUME(mission goal).
   DWA/A*/yaml 코드 변경 0 = F-1 무회귀 by construction. 실 sim 에서 amr2 양보 동작 확인.
2. **최소개입**: 매니저가 **저우선(패자)만** 조작. 승자는 mission goal 유지(NORMAL) → 자동 진행.
   실 sim 로그: `amr2: NORMAL→HOLD (양보, amr1 우선)`, amr1 내내 NORMAL.
3. **결정론 탐지**: TF(map,base_footprint)+odometry(AMR만). pedestrian/forklift 는 odometry 무 →
   매니저 시야 밖 = 오발동 구조적 0 (음성테스트로 입증).
4. **DWA 후진 불가**(allow_backward=false) → RETREAT 은 전방 path 재계획(ALIGN 회전). 정상.
5. **★맵 한계 발견**: warehouse.world = 완전연결 개방 그리드(N-S 게이트 6 + 5m 아일) →
   A* 가 상대 로봇을 미리 우회 → 두 로봇이 R 내로 접근조차 안 함(≥2.66m). **창발 영구교착 불가.**
   로봇 0.4m 라 5m 아일 못막고, idle 2대로 대체게이트 5개 못막음. = S-9 단일병목 맵이 정 venue.

## controlled demo(GATE-SHARED) 의 의미
창발 교착이 불가하므로, 두 로봇을 **같은 점(경합 자원)**으로 보내 강제 co-location →
매니저의 핵심(탐지+우선순위 양보)을 실 sim 으로 입증. 패자 amr2 는 같은 점을 영원히 못
가지므로 HOLD↔RETREAT 반복 → **livelock 가드 동작(정상)**. "서로 다른 goal 둘 다 도달"은
경로교차 교착이 필요하나 그건 우회되는 맵이라 불가 → 유닛테스트로 FSM 완결성 검증.

## 환경/시나리오 함정 (재현 방지)
- 개방 그리드는 정면 교착 불가 → spawn override(env)로 핀치 직결 시도해도 우회. (BLOCKED.md 상세)
- prepos 긴 드라이브는 보행자/정적 클러터(pallet_1/box_stack_1)에 amr2 가 길잃음 →
  spawn override + dynamic_obstacles:=false 로 청정화.
- ROS 노드 실행은 Bash 샌드박스 해제 필요(DDS). 인라인 node-path pkill=셸 self-kill →
  스크립트파일(clean_restart_f2.sh). ([[ros2-restart-rebuild-gotchas]])

## 재현
```
# 매니저 로직 유닛검증(빠름, sim 불요)
source /workspace/env.sh && python3 /workspace/validation/fleet_f2/f2_unit_test.py
# controlled demo (탐지+양보 실 sim)
bash /workspace/validation/fleet_f2/run_shared.sh
# 음성 (오발동 0)
bash /workspace/validation/fleet_f2/run_negative.sh
# baseline (맵 한계 재현: reroute, 교착 불성립)
bash /workspace/validation/fleet_f2/run_baseline.sh
```

## 산출물
- 코드: `feature/fleet-f2-deadlock` — amr_fleet(deadlock_manager + priority_publisher),
  multi_robot.launch.py(매니저 additive + spawn env override), warehouse.launch.py(obstacle 토글).
- 검증: `/workspace/validation/fleet_f2/` — f2_unit_test.py, gate_*.py, scenario_run.py,
  run_*.sh, f2_common.py, bag, PLAN/PROGRESS/report/BLOCKED. repo `docs/fleet_f2/` 사본.

## 사람 결정 필요 (교착 end-to-end demo 방법) — BLOCKED.md §결정
1. **(권장) S-9 2존맵**: 단일 병목 → 창발 교착 자연발생 → GATE-DETECT/RESOLVE 그대로 PASS.
2. controlled demo 확장(현 맵): idle 로봇 주차로 강제 co-location(엔지니어드).
3. reframe: "영구정지" 대신 "head-on contention 우선순위 중재".
