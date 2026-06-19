# F-1 Fleet 무인 런 — 최종 리포트 (아침 요약)

**결론: F-1 인프라 + 4대 독립 goal 주행 = 완료. 4 🔴 전부 게이트 PASS.**
브랜치 `feature/fleet-f1-cc`. dev/spawn 브랜치 미접촉, PR 미생성, install/ 미커밋.

## 게이트 결과 (정량)
| 게이트 | 결과 | 근거 |
|---|---|---|
| GATE-T | **4/4 PASS** | lookup(map, amrN/base_footprint) err ≤0.03m, NaN/원점고정 0 (run3) |
| GATE-TOPIC | **PASS** | amrN/amrN 이중prefix 0, /amrN/odometry/filtered 4개 21~24Hz (run9) |
| GATE-GOAL | **4/4 PASS** | 최종 pose err 0.18/0.30/0.34/0.49m ≤ ARRIVAL_TOL 0.6 (run9) |
| GATE-COLLISION | **PASS** | 0.60×0.40 실제 yaw, 1216 샘플 overlap 0, closest 1.51m (run9) |

## 커밋 (명시 경로만)
```
6cf3002 docs(fleet): PLAN/PROGRESS/CLAUDE.md
58a9feb 🔴-1 ign 센서/odom 토픽 per-robot 분리 (URDF prefix) → GATE-T 4/4
4ea2190 🔴-2 fleet_localization_init (AMCL active self-heal + initialpose) → 원샷 0 수동pub
8193377 🔴-3 fused_tracker tracking_frame=map (단일 인스턴스·map 출력 유지)
f13ed37 🔴-4 A*/DWA per-robot 토픽·프레임 + goal_tol + amr3 reroute → GATE 전부 PASS
```

## 핵심 기술 결론 (EXPLORE 가설을 런타임이 갱신)
1. **🔴-1 진짜 원인은 initialpose 아니라 ign 토픽 공유.** URDF가 lidar/odom/cmd_vel 을
   고정 ign 토픽으로 발행 → 4대 spawn 시 충돌 → per-robot 브리지가 섞인 데이터 재발행.
   해결: URDF `prefix` 인자로 ign 토픽/프레임 per-robot 분리. (TF prefix 자체는 JW가 이미 정상.)
2. **🔴-4 진짜 블로커는 stale 실행파일.** amr_navigation(ament_cmake)이 astar_planner 를
   복사 설치 + 미재빌드 → base_frame 파라미터 옛버전 → A* 가 `base_footprint`(미prefix)
   lookup 실패. 재빌드 + DWA 프레임 파라미터화 + 절대토픽 remap 으로 해결.
3. 단일로봇 무회귀 원칙: 공유 config/노드는 안 건드리고 **multi_robot.launch.py override**
   + URDF `prefix` 기본 ''(빈값) 으로만 멀티로봇 동작 분기.

## 환경 함정 (재현 방지 — clean_restart.sh 강화판에 반영)
- 활성 워크스페이스 = `/workspace/ros2_ws`(install 별도 트리). repo의 ros2_ws에 빌드하면
  ros2 launch가 못 찾음. → `/workspace/ros2_ws`에서 빌드. ([[active-ros2-workspace-build-path]])
- `pkill -f "...robot_state_publisher..."` 를 bash -c 인라인으로 돌리면 self-kill →
  스크립트 파일(clean_restart.sh)로 실행.
- clean_restart 가 `ros2 launch` 부모 미종료 → 두 월드 겹침("jump back in time").
- 반복 pkill -9 로 `/dev/shm` FastRTPS stale 누적 → DDS 통신 저하. → shm 정리 추가.
- 신규 .py 노드는 src에 `chmod +x` 필요(symlink-install이 +x로 노출).

## 재현 방법
```
bash /workspace/validation/fleet_f1/run_full.sh /workspace/validation/fleet_f1/runX 240
# = clean_restart → warehouse → multi_robot(원샷 localize) → 240s 주행+bag → 4 게이트
# 또는 수동: T1 warehouse.launch.py → T2 multi_robot.launch.py (수동 pub 불필요)
#           → fleet_goal_pub.py → gate_t/gate_topic/gate_goal/gate_collision.py
```

## 산출물
- 코드: feature/fleet-f1-cc (위 5 커밋). repo `docs/fleet_f1/`에 PLAN/PROGRESS 사본.
- 검증: `/workspace/validation/fleet_f1/` — gate_*.py, f1_common.py, fleet_*.py,
  clean_restart/bringup/drive_and_record/run_full.sh, run9/drive_bag (49MB),
  report_gate_t/2/3/4(.md/.txt), PLAN.md, PROGRESS.md.

## 스코프 경계 (의도적 미수행)
- 4대 동시 fused_tracker 융합(현재 amr1 입력만; 단일 인스턴스·map 출력 계약은 충족).
- S-9(2존 월드)·F-2(교착 해소): 범위 밖.
- DWA goal-latch 정밀 튜닝(공유 0.20m): 멀티로봇은 launch override 0.4로 우회, 공유 yaml 불변.
