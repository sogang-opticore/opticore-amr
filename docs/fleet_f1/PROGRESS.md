# F-1 Fleet — PROGRESS (아침 대시보드)

> 한 줄 요약을 위에서부터 시간순. 캐노니컬 PLAN/PROGRESS는 여기 + repo `docs/fleet_f1/`(커밋).
> 게이트 PASS는 **bag/TF 정량으로만** 판정. "노드 떴다"는 PASS 아님.

## 상태 보드
| 🔴 | 항목 | 상태 | DONE 기준 | 비고 |
|---|---|---|---|---|
| — | EXPLORE + PLAN | ✅ 완료 | PLAN.md 작성·커밋 | 6-subsystem 병렬 맵 + 키스톤 정독 |
| 1 | EKF /tf multi-robot | ✅ **PASS (4/4)** | GATE-T 4 PASS | 진짜원인=ign 센서/odom 토픽 공유→URDF prefix로 분리 |
| 2 | initialpose 자동화 | ✅ **PASS** | 원샷 0 수동pub + GATE-T | fleet_localization_init: AMCL active 보장(self-heal)+initialpose |
| 3 | fused_tracker multi | ✅ **PASS** | no crash/lookup-fail + 발행 + 무회귀 | tracking_frame=map + amr1 입력(launch만) |
| 4 | 4대 독립 goal | ⏳ 착수 | GATE-GOAL4 + COLLISION0 + TOPIC | A*/DWA 절대토픽 remap |

## 로그
- **[2026-06-09]** EXPLORE 완료(workflow 6 agents, 369k tok). 키스톤 규명:
  - 🔴-1 진짜 원인 = `amcl_params.yaml` `initial_pose:(0,0,0)` 4대 공유 → 오정합. EKF /tf 인프라는 대체로 맞음(글로벌 /tf + prefixed). 부차: injector `frame_id=amrN/odom`≠EKF world_frame.
  - 🔴-3 = launch 파라미터 오버라이드만으로 해결 가능(tracking_frame=map). 노드 코드 변경 0.
  - 🔴-4 = A* 절대토픽 하드코딩 → namespace 무효. 절대 remap 필요.
  - 검증 재사용: `dwa_dyn/analyze_refix.py`의 SAT overlap + db3 직접 파싱.
- **[2026-06-09]** PLAN.md 작성. 다음: 🔴-1 빌드→런치→수동 initialpose→GATE-T.
- **[2026-06-09]** 🔴-1 진행 중 환경 함정 3개 규명·해결:
  1. 활성 워크스페이스는 `/workspace/ros2_ws`(install 별도 트리). repo의 ros2_ws에
     빌드하면 `ros2 launch`가 못 찾음 → `/workspace/ros2_ws`에서 빌드해야.
  2. `pkill -f "...robot_state_publisher..."`를 bash -c 인라인으로 돌리면 그 패턴이
     호출 셸 argv에 들어가 self-kill → 스크립트 파일(`clean_restart.sh`)로 실행.
  3. `clean_restart`가 `ros2 launch` 부모를 안 죽여 두 월드 겹침("jump back in time")
     → clean_restart에 `ros2 launch` 패턴 추가(강화판).
- **[2026-06-09]** 🔴-1 키스톤 진짜 원인 = **ign 센서/odom 토픽 공유**(URDF 고정 topic).
  URDF `prefix` 인자 + per-robot xacro + per-robot 브리지로 데이터 크로스토크 차단.
  → **GATE-T PASS (4/4)** (수동 initialpose + amr4 수동 lifecycle activate). report_gate_t.md.
- **[2026-06-09]** 다음: 🔴-2 initialpose 자동화 + AMCL active 보장(원샷 신뢰성).
- **[2026-06-09]** 🔴-2 PASS: `amr_slam/fleet_localization_init.py` 신규 노드 —
  per-robot map_server+amcl을 직접 lifecycle 전이(configure→activate, 재시도)로 active
  보장(amr4 lifecycle 타임아웃 self-heal) + initialpose 자동 발행(map=(x−3,y−15,0),
  단일소스). multi_robot.launch.py에 +40s TimerAction으로 1개 추가.
  **원샷 검증**: warehouse 8s → loc_init DONE 36s(all_ready=True) → GATE-T **4/4**
  (수동 pub 0, 수동 activate 0) + GATE-TOPIC PASS(이중prefix 0, odom 19~24Hz).
  ⚠ 빌드 함정: 신규 .py 노드는 src에 `chmod +x` 필요(symlink-install이 +x로 노출).
- **[2026-06-09]** 다음: 🔴-3 fused_tracker(launch param: tracking_frame=map + amr1 입력).
- **[2026-06-09]** 🔴-3 PASS: multi_robot.launch.py에서 fused_tracker를 직접 Node로
  (tracking_frame=map → _output_transform 항등, amr1 입력 결선). 노드/단일로봇 launch
  무수정(무회귀 by construction). 검증: /perception/tracked_objects 38msgs/8s frame=map
  트랙2, lookup-fail은 기동 1회 transient뿐, no crash. report_gate_3.md.
- **[2026-06-09]** 다음: 🔴-4 4대 독립 goal 주행(A*/DWA 절대토픽 remap → bag → GATE).

## BLOCKED
- 없음.
