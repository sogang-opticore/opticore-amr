# F-1 Fleet — PROGRESS (아침 대시보드)

> 한 줄 요약을 위에서부터 시간순. 캐노니컬 PLAN/PROGRESS는 여기 + repo `docs/fleet_f1/`(커밋).
> 게이트 PASS는 **bag/TF 정량으로만** 판정. "노드 떴다"는 PASS 아님.

## 상태 보드
| 🔴 | 항목 | 상태 | DONE 기준 | 비고 |
|---|---|---|---|---|
| — | EXPLORE + PLAN | ✅ 완료 | PLAN.md 작성·커밋 | 6-subsystem 병렬 맵 + 키스톤 정독 |
| 1 | EKF /tf multi-robot | ⏳ 착수 | GATE-T 4 PASS | 진짜 원인=AMCL initialpose(0,0) 공유 |
| 2 | initialpose 자동화 | ⬜ 대기 | 원샷 0 수동pub + GATE-T | map=(x−3,y−15,0) 단일소스 |
| 3 | fused_tracker multi | ⬜ 대기 | no crash/lookup-fail + 발행 + 무회귀 | launch param만(코드 0) |
| 4 | 4대 독립 goal | ⬜ 대기 | GATE-GOAL4 + COLLISION0 + TOPIC | A*/DWA 절대토픽 remap |

## 로그
- **[2026-06-09]** EXPLORE 완료(workflow 6 agents, 369k tok). 키스톤 규명:
  - 🔴-1 진짜 원인 = `amcl_params.yaml` `initial_pose:(0,0,0)` 4대 공유 → 오정합. EKF /tf 인프라는 대체로 맞음(글로벌 /tf + prefixed). 부차: injector `frame_id=amrN/odom`≠EKF world_frame.
  - 🔴-3 = launch 파라미터 오버라이드만으로 해결 가능(tracking_frame=map). 노드 코드 변경 0.
  - 🔴-4 = A* 절대토픽 하드코딩 → namespace 무효. 절대 remap 필요.
  - 검증 재사용: `dwa_dyn/analyze_refix.py`의 SAT overlap + db3 직접 파싱.
- **[2026-06-09]** PLAN.md 작성. 다음: 🔴-1 빌드→런치→수동 initialpose→GATE-T.

## BLOCKED
- 없음.
