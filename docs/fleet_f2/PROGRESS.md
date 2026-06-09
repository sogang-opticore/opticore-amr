# F-2 Fleet 무인 런 — PROGRESS (아침 대시보드)

브랜치 `feature/fleet-f2-deadlock`. dev/f1-cc/spawn 미접촉, PR 미생성, install/ 미커밋.

## 현재 위치
- **[완료] EXPLORE** — 6-subsystem 병렬 정독(launch/nav-goal/world/package/f1-harness/odom-tf) + `warehouse.world` 직접 파싱 + `dynamic_obstacle_mover.py` 정독.
- **[완료] PLAN** — `docs/fleet_f2/PLAN.md` 커밋(`9b1a596`). 복도좌표·T/R/D·후진점 방법 포함. **사람 승인: 복도 = N-S X=35.5 게이트 ✓**.
- **[완료] 🔴-1** — `amr_fleet` deadlock_manager 골격 + priority_publisher 주입 커밋(`3eaa572`). 빌드 OK, 인터페이스 게이트 PASS(아래). FSM 전부 NORMAL(탐지/해소 미활성).
- **[대기·사람] §3 T/R/D 승인** → **[다음] 🔴-2 탐지**(시나리오 검증 필요 → §1 좌표 승인됨, T/R/D 승인 대기). 무인 진행 약속대로 시나리오 검증 직전에서 정지.

## 🔴-1 인터페이스 게이트 (PASS)
- 기본(미수신): `/fleet/deadlock_status` `prio_source=default`, amr1=4/amr2=3/amr3=2/amr4=1(낮은 id 높은우선), FSM NORMAL.
- 주입(`priority_publisher.py -p priorities:="[1,4,2,3]"`): 매니저 로그 `수신·반영: {amr1:1,amr2:4,amr3:2,amr4:3}`, status `prio_source=injected` amr1=1.
- 빌드: `colcon build --packages-select amr_fleet` (active ws `/workspace/ros2_ws`). 노드 standalone 동작(시뮬 불요).
- ※ 환경: ROS 노드 실행은 Bash 샌드박스 해제 필요(DDS). 인라인 node-path pkill = 셸 self-kill 주의.

## 게이트 현황 (정량 — PASS만 진짜 완료)
| 게이트 | 🔴 | 상태 | 근거 |
|---|---|---|---|
| (인터페이스) | 🔴-1 | ⬜ 미착수 | /fleet/priorities 반영 + default 동작 |
| GATE-DETECT | 🔴-2 | ⬜ | 강제교착 T초 탐지 + 음성 오발동 0 |
| GATE-RESOLVE | 🔴-3 | ⬜ | 둘 다 goal 0.6m 도달, 영구정지 0 |
| GATE-COLLISION | 🔴-3 | ⬜ | SAT overlap 0 샘플 |
| GATE-BOUNDED | 🔴-3 | ⬜ | 해소 bounded, LIVELOCK 0 |
| F-1 회귀 4게이트 | 🔴-4 | ⬜ | enable_deadlock_manager:=false로 F-1 재PASS |

## 핵심 설계 결정 (EXPLORE 결론)
1. **순수 goal_pose 액추에이터** — HOLD=현pose / RETREAT=경로 뒤 D점 / RESUME=mission goal. DWA·A*·yaml 코드 변경 0 → F-1 무회귀 by construction.
2. **DWA 후진 불가**(allow_backward=false) → RETREAT은 ALIGN 제자리회전 후 전진(역주행 X). 게이트 유효폭 1.28m > 회전직경 0.72m라 복도 내 회전 OK.
3. **최소개입**: 매니저는 **저우선(패자)만** 조작. 승자는 mission goal 유지 → 패자 비키면 DWA 자동 재진행.
4. **비대칭 시나리오**(둘 다 西출발→東목표, 다른 아일) — 미러 교착의 "승자 goal=패자 정지점" 함정 제거.
5. **결정론적 탐지**: `/amrN/odometry/filtered`(AMR만) → pedestrian/forklift는 odometry 없어 매니저 시야 밖 = 오발동 구조적 0.
6. **복도 = X=35.5 N-S 게이트**(rack_B4↔B5, 물리 2.0m/유효 1.28m). 미션 E-W 문구는 본 월드에 맞춰 N-S로 정정(맵 독립 로직이라 무영향).

## 환경 함정 (F-1 계승 — 재현 방지)
- 빌드는 `/workspace/ros2_ws`에서. ament_cmake는 .py 복사설치 → 수정 후 `colcon build` 필수, 신규 .py는 src에서 `chmod +x`.
- `clean_restart.sh`는 스크립트파일로(pkill self-kill 회피) + `/dev/shm/fastrtps_*` purge. **PAT에 `deadlock_manager` 추가**.
- RTF≈0.33 정상(ign CPU 바운드) → 모든 타임아웃 RTF 스케일. AMCL 장시간/텔레포트 후 락 상실 가능 → garbage 위 검증 금지, 클린 재시작.

## 다음 액션
1. (사람) §1 좌표 / §3 T/R/D 승인 또는 보정.
2. (CC) 🔴-1: amr_fleet에 deadlock_manager.py + priority_publisher.py 골격 + CMakeLists install(PROGRAMS) + rclpy dep. build·run 확인.
3. (CC) baseline_run.sh로 시나리오 유효성(교착 성립) 먼저 증명 → 🔴-2 탐지.
