# amr_bringup

시뮬레이션 월드, 로봇 모델, 맵, 실행 launch를 묶은 **실행 진입점** 패키지 (ament_cmake).

> 2026-10-13 갱신: 실제 파일 구성 기준으로 재작성. 실행 절차는 [`docs/multi-robot-runbook.md`](../../../docs/multi-robot-runbook.md), 서버 성능·주행 문제는 [`docs/server-run-prescription.md`](../../../docs/server-run-prescription.md) 참고.

## 구성

| 경로 | 내용 |
|---|---|
| `launch/warehouse.launch.py` | Ignition Gazebo 서버(`ign gazebo -v4 -s -r`, headless)와 동적 장애물 mover. **로봇은 스폰하지 않는다** |
| `launch/multi_robot.launch.py` | **현재 주 진입점.** amr1~amr4 스폰. 로봇마다 ros_gz_bridge, RSP, 정적 TF 2개, IMU·odom 공분산 주입기, EKF, map_server, AMCL, lifecycle manager, A\*, DWA를 띄우고, 전역으로 fleet_localization_init(+40 s), fused_tracker(+50 s), deadlock_manager(+55 s)를 띄운다 |
| `launch/robot.launch.py` | 1대분 launch. 단독 실행 시 정상 동작하지 않는다는 기록이 있다([`docs/cctv/PLAN.md`](../../../docs/cctv/PLAN.md) §1.5). 비권장 |
| `worlds/warehouse.world` | 60 × 40 m 창고 v3.1: 랙 21개(Row A/B/C), 2.0 m 핀치 복도 2개(x≈14.5, x≈49.5), CCTV 카메라 4대, 보행자 6명(동적 3), 지게차 1대(동적) |
| `worlds/warehouse_1~9*.world` | 이전 버전 월드(현재 launch에서 사용하지 않음) |
| `urdf/amr_robot.urdf.xacro` | 차동구동 AMR(섀시 0.60 × 0.40 m), gpu_lidar 720샘플 10 Hz, RGB 카메라 640×480 15 Hz, IMU 100 Hz. `prefix` 인자로 로봇별 Ignition 토픽/프레임 분리 |
| `maps/warehouse_map.{yaml,pgm}` | AMCL·A\*용 정적 맵. 1207 × 840 셀, 0.05 m/셀, origin (−3.18, −16.9) |
| `config/ekf.yaml` | robot_localization EKF 기본값. 멀티로봇에서는 launch가 로봇별 frame/topic을 덮어쓴 임시 yaml을 만든다 |
| `models/standing_person`, `models/pickup` | 보행자·지게차 모델. **install 대상이 아니다**(아래 주의 참고) |
| `src/dynamic_obstacle_mover.py` | person_1~3, forklift_1을 waypoint 사이로 이동. 0.2 s(sim)마다 `ign service .../set_pose_vector` 프로세스를 새로 띄운다 |

## launch 인자

| launch | 인자 (기본값) |
|---|---|
| `warehouse.launch.py` | `use_sim_time`(true), `dynamic_obstacles`(true) |
| `multi_robot.launch.py` | `enable_deadlock_manager`(true). 스폰 좌표는 환경변수 `AMR{1..4}_SPAWN_X/Y`로 override (기본 world (3, 13) / (3, 17) / (3, 21) / (3, 25)) |

## 주의

- `warehouse.launch.py`는 `IGN_GAZEBO_RESOURCE_PATH`를 `/workspace/ros2_ws/src/amr_bringup/models`로 **덮어쓴다**. 소스가 다른 경로에 있으면 보행자·지게차 모델이 로드되지 않는다. `/workspace/ros2_ws`가 실제 소스 트리를 가리키게 하라([`docs/environment-setup.md`](../../../docs/environment-setup.md)).
- 좌표계: goal은 `map` 프레임으로 보낸다. map ≈ world − (3.0, 15.0) (`fleet_localization_init` 기준. CCTV 작업 실측은 world − (2.96, 15.12)).
- 로봇마다 브리지가 `/clock`을 중복 브리지하고, 아무도 구독하지 않는 로봇 카메라도 브리지한다. 성능 영향은 처방전 참고.
- `dynamic_obstacle_mover`의 person_1 경로 (14, 18) → (17, 18)은 핀치 동쪽 벽(`wall_pinch_L2`, x 15.5~15.75)을 관통한다(텔레포트 방식이라 충돌 없이 통과).
- `package.xml`의 `gazebo_ros`, `gazebo_ros_pkgs` 의존은 Gazebo Classic 시절 잔재다(현재 Ignition Fortress + ros_gz 사용).
