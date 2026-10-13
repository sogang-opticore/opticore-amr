# amr_fleet

멀티로봇(amr1~amr4) 플릿 조율 패키지 (ament_cmake, Python 노드).

> 2026-10-13 작성. 설계·검증 기록은 [`docs/fleet_f1`](../../../docs/fleet_f1), [`docs/fleet_f2`](../../../docs/fleet_f2), [`docs/fleet_f3`](../../../docs/fleet_f3).
> 서버 구동 시 알려진 문제·처방은 [`docs/server-run-prescription.md`](../../../docs/server-run-prescription.md).

## 구성

| 파일 | 역할 | 기동 방법 |
|---|---|---|
| `src/deadlock_manager.py` (F-2) | 좁은 통로 정면 교착 탐지 → **저우선 로봇만** HOLD → RETREAT → RESUMING. 액추에이터는 `/amrN/goal_pose` 재발행뿐(A*/DWA 무수정). `/fleet/deadlock_status`(JSON) 발행 | `multi_robot.launch.py`가 +55 s에 자동 기동. `enable_deadlock_manager:=false`로 끔 |
| `src/priority_publisher.py` | `/fleet/priorities`(latched) 수동 주입 테스트 도구 | `ros2 run amr_fleet priority_publisher.py --ros-args -p priorities:="[1,4,2,3]"` |
| `src/fleet_priority.py` (F-3 코어) | ROS 의존 없는 순수 함수: 경로·트랙 기반 충돌 예측 + 우선순위 결정 | `f3_priority_node.py`가 import |
| `src/f3_priority_node.py` (F-3) | 예측 기반 선제 우선순위 → `/fleet/priorities`, `/fleet/conflict_predictions`(MarkerArray) | 별도: `ros2 launch amr_fleet f3_priority.launch.py` (`enable_f3_priority:=false`로 끔) |
| `config/f3_params.yaml` | F-3 정책 파라미터 (horizon, 충돌 반경, handoff 거리 등) | |
| `launch/f3_priority.launch.py` | F-3 단독 launch (multi_robot과 별개로 추가 기동) | |
| `test/test_fleet_priority.py` | F-3 코어 단위테스트 (sim 불필요) | `python3 -m pytest ros2_ws/src/amr_fleet/test/test_fleet_priority.py -v` |

## 토픽

| 토픽 | 방향 | 비고 |
|---|---|---|
| `/amrN/goal_pose` | 구독 + 발행 | 매니저는 NORMAL 상태에서 들어온 goal만 mission goal로 캐싱하고, HOLD/RETREAT/RESUMING 중에는 0.5 s 간격으로 덮어쓴다 |
| `/amrN/odometry/filtered`, `/amrN/dwa/status` | 구독 | |
| TF `map → amrN/base_footprint` | 조회 | 로봇 pose 소스 (AMCL + EKF) |
| `/fleet/priorities` | F-3 발행 → F-2 구독 | `std_msgs/Int32MultiArray`, index = 로봇 번호 − 1, 값이 클수록 우선 |
| `/fleet/deadlock_status` | F-2 발행 | 상태 관측용 JSON (`fsm`, `stuck`, `blocked_by`, `livelock`, `flip` …) |
| `/perception/tracked_objects` | F-3 구독 | 현재 `fused_tracker`가 amr1 LiDAR만 입력으로 사용 |

## 알려진 문제 (2026-10-13 코드 리뷰, 미수정)

- **livelock 동결이 해제되지 않음**: `flip_count`가 런 전체에서 누적되고 리셋되지 않는다. HOLD→RETREAT 전이가 `livelock_max_n`(3)회를 넘으면 그 로봇은 런이 끝날 때까지 hold 위치에 고정된다 (`deadlock_manager.py` `_enter_retreat`, `_resolve`).
- **정지 판정이 의도된 정지까지 잡음**: `t_stuck`(4 s) 동안 변위 < `eps_move`(0.10 m)이면 stuck. 제자리 회전, 보행자 대기, goal 경계(0.4 m) 부근 정지도 포함된다. 구독 중인 `lin_speed`, `dwa_status`는 판정에 쓰이지 않는다.
- **관리 중 새 goal 유실**: HOLD/RETREAT/RESUMING 중에 들어온 사용자 goal은 mission goal로 채택되지 않고 덮어쓰인다.
- **F-3 경로 입력 QoS 불일치**: `f3_priority_node`는 `/amrN/global_path`를 `TRANSIENT_LOCAL`로 구독하지만 A*는 `VOLATILE`로 발행한다. 호환되지 않아 경로가 수신되지 않고, 로봇이 정지 상태로 투영된다.

자세한 영향과 처방은 [`docs/server-run-prescription.md`](../../../docs/server-run-prescription.md) 참고.
