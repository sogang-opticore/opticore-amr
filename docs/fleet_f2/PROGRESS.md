# F-2 Fleet 무인 런 — PROGRESS (아침 대시보드)

브랜치 `feature/fleet-f2-deadlock`. dev/f1-cc/spawn 미접촉, PR 미생성, install/ 미커밋.

## TL;DR (아침 30초 요약)
- **매니저(우선순위 교착 해소) 구현·검증 완료.** 탐지+우선순위 양보가 실 sim + 유닛테스트로 입증.
- **★ 현 맵은 *창발적* 영구 교착을 강제 못함**(개방 그리드 = A* 우회, 두 로봇 R 내 접근조차 X).
  → end-to-end 창발 교착 demo 는 **단일병목 맵(S-9 2존)이 필요** = 이번 런 범위 밖. 상세 `BLOCKED.md`.
- **사람 결정 필요**(아침): 교착 demo 방법 3안 — `report.md`/`BLOCKED.md` 끝.

## 현재 위치
- [완료] EXPLORE · PLAN(`9b1a596`).
- [완료] 🔴-1 우선순위 주입(`3eaa572`, sim PASS).
- [완료] 🔴-2 탐지 + 🔴-3 해소 FSM(`62c1435`, **유닛테스트 23/23 PASS**).
- [완료] 검증: 음성 sim(오발동 0) · controlled demo sim(탐지+양보+자원획득) · 유닛 · F-1 무회귀.
- [완료] 🔴-4 F-1 회귀(매니저 ON → F-1 4대 독립주행 PASS, false-trigger 0).
- [대기·사람] 창발 교착 demo 방법 결정(맵 한계 → S-9 권장).

## 게이트 결과 (정량)
| 게이트 | 방식 | 결과 |
|---|---|---|
| 🔴-1 우선순위 | sim | **PASS** (default amr1=4·주입 [1,4,2,3]→injected amr1=1) |
| 🔴-2+3 매니저 로직 | **unit** | **PASS 23/23** (우선순위·탐지 geo·_is_stuck·FSM 전이·livelock·retreat) |
| GATE-DETECT(neg) | sim | **PASS** (amr1+person_3, 122샘플 오발동 0) |
| GATE-SHARED | sim | **PASS** (탐지 89샘플 · 패자 amr2 HOLD/RETREAT/RESUMING · 승자 amr1 자원 0.94m) |
| GATE-COLLISION(shared) | sim | **PASS** (overlap 0, closest 0.68m) |
| GATE-BASELINE(창발) | sim | **N/A 맵한계** (3회 reroute, closest ≥2.66m, 교착 불성립) |
| F-1 회귀(매니저 ON) | sim | **PASS** (GATE-GOAL 4/4 · COLLISION 0 · TOPIC ok · false-trigger 0) |

## 핵심 결론
1. 순수 goal_pose 액추에이터(HOLD/RETREAT/RESUME) → DWA/A* 무수정 = F-1 무회귀 by construction.
   실 sim 로그: `amr2: NORMAL→HOLD (양보, amr1 우선)`, 승자 amr1 내내 NORMAL.
2. 최소개입: **패자만** 조작, 승자 자동진행. 결정론 탐지(odometry/TF only) → pedestrian 오발동 0.
3. **맵 한계(핵심 발견)**: warehouse.world 완전연결 개방 그리드 → A* 우회로 정면 교착 불성립.
   spawn override·obstacle off 로도 우회. 단일병목 맵(S-9)이 창발 교착의 정 venue.
4. controlled demo(같은 점 경합)로 매니저 핵심을 실 sim 입증 — 패자는 같은 점 영원히 못가져
   livelock 가드 동작(정상). 서로 다른 goal 둘 다 도달은 경로교차 교착 필요(맵서 불가)→유닛검증.

## 다음 액션 (사람)
1. 교착 end-to-end demo venue 결정: **(권장) S-9 2존맵** / controlled 확장 / reframe. (BLOCKED.md)
2. PLAN §3 T/R/D 는 현재 기본값(T=4 R=2 D=4)으로 동작 확인 — 단일병목 맵서 재튜닝 권장.

## 환경 함정 (재현 방지)
- ROS 노드 실행 = Bash 샌드박스 해제 필요(DDS). 인라인 node-path pkill=셸 self-kill → 스크립트파일.
- 빌드 `/workspace/ros2_ws`, ament_cmake .py 복사설치 → 수정후 colcon build, 신규 .py chmod +x.
- 개방 그리드서 prepos 긴 드라이브는 보행자/정적클러터에 길잃음 → spawn override + obstacle off.
