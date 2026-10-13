# Opticore AMR

지능형 물류 AMR(Autonomous Mobile Robot) 시스템 — 2026 캡스톤 디자인.

ROS 2 Humble과 Ignition Gazebo Fortress 위의 60 × 40 m 가상 물류창고에서 AMR 4대가 동시에 움직인다. 주요 구성은 다음과 같다.

- **주행**: 자체 구현한 A\* 전역 계획 + DWA/Pure Pursuit 지역 주행
- **위치추정**: AMCL + EKF
- **인지**: LiDAR·CCTV 기반 인지
- **플릿 조율**: 교착 해소(F-2), 선제 우선순위(F-3)

> ⚠ **2026-10 서버 구동 결과: 크래시는 없었지만 매우 느렸고 자율주행이 제대로 되지 않았다.**
> 원인 진단과 처방은 [`docs/server-run-prescription.md`](docs/server-run-prescription.md), 구조 개선안은 [`docs/multi-robot-architecture.md`](docs/multi-robot-architecture.md)에 있다.

## 기술 스택 (`dev` 기준 실제 사용)

| 영역 | 사용 기술 | 비고 |
|---|---|---|
| 시뮬레이션 | Ignition Gazebo Fortress (headless 서버, ogre2 렌더링 센서) | `amr_bringup/worlds/warehouse.world` |
| 위치추정 | Nav2 `amcl` + `map_server` + `lifecycle_manager`, `robot_localization` EKF | Nav2는 위치추정 컴포넌트만 사용한다 |
| 전역 경로 | 자체 A\* (`amr_navigation/astar_node.py`, Python) | clearance 비용, 동적 no-go 레이어 합성 |
| 지역 주행 | 자체 DWA 노드 (`dwa_node.py`, Python). 14차 튜닝 이후 Pure Pursuit + 적응 속도 + 상태기계 | Nav2 controller 미사용 |
| 인지 | DWA 내장 LiDAR 트래킹, `fused_tracker`, CCTV 배경차분 no-go 오버레이, YOLOv8(선택) | |
| 플릿 | `deadlock_manager`(F-2), `f3_priority_node`(F-3) | |
| 인프라 | RunPod GPU 파드 (RTX A5000 / A40), Docker | [`docs/environment-setup.md`](docs/environment-setup.md) |

## 저장소 구조

```text
ros2_ws/src/
  amr_bringup/      월드·URDF·맵·EKF 설정·실행 launch(warehouse, multi_robot), 동적 장애물 mover
  amr_navigation/   A* 전역 계획, DWA/Pure Pursuit 지역 주행, frontier 탐색, 진단 도구(lag_diag)
  amr_slam/         AMCL·slam_toolbox 설정, fleet_localization_init, IMU/odom 공분산 주입기
  amr_perception/   fused_tracker, LiDAR 클러스터링, CCTV 검출·no-go 오버레이, YOLO 검출기
  amr_fleet/        deadlock_manager(F-2), f3_priority_node(F-3)
  amr_msgs/         TrackedObject(Array)
  amr_description/, amr_gazebo/   자리표시자(패키지 아님). 실제 URDF·월드는 amr_bringup
docs/               설계·검증 기록, 실행/환경 가이드, 서버 진단·처방 (목차: docs/README.md)
scripts/            clean_build.sh (재빌드 도우미)
```

## 빠른 시작 (4대 멀티로봇)

```bash
# 0) 반드시 dev 브랜치 (GitHub 기본 브랜치 main은 오래된 스텁)
git clone -b dev git@github.com:sogang-opticore/opticore-amr.git

# 1) 워크스페이스 경로: warehouse.launch.py가 모델 경로를 /workspace/ros2_ws/... 로 고정하므로
#    /workspace/ros2_ws 가 이 저장소의 ros2_ws 를 가리키게 한다 (자세한 내용: docs/environment-setup.md)

# 2) 빌드 (symlink-install 없이. 코드·설정을 바꾸면 다시 빌드)
cd /workspace/ros2_ws && colcon build && source install/setup.bash

# 3) 실행 (터미널 2개)
ros2 launch amr_bringup warehouse.launch.py            # 월드 + 동적 장애물 (dynamic_obstacles:=false 로 끔)
ros2 launch amr_bringup multi_robot.launch.py          # 로봇 4대 + 로봇별 위치추정/A*/DWA + 플릿

# 4) goal (map 프레임, map ≈ world − (3, 15)). 예: world (20, 18) = Row A·B 사이 통로
ros2 topic pub --once /amr1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 17.0, y: 3.0}, orientation: {w: 1.0}}}"
```

기동 순서, 준비 완료 신호, 실행 전 정리, 측정 방법은 [`docs/multi-robot-runbook.md`](docs/multi-robot-runbook.md)를 따른다.

## 문서

전체 목차는 [`docs/README.md`](docs/README.md)에 있다. 자주 보는 문서는 다음과 같다.

- [서버 구동 진단·처방전](docs/server-run-prescription.md): 느림과 자율주행 불량의 원인, 단계별 처방
- [멀티로봇 구조 처방](docs/multi-robot-architecture.md): 로봇별 풀스택을 공유 서비스로 재구성하는 안
- [멀티로봇 실행 런북](docs/multi-robot-runbook.md)
- [환경 설정](docs/environment-setup.md)
- [Git 워크플로](docs/git-workflow.md)
- 패키지별 README (`ros2_ws/src/*/README.md`). 특히 [`amr_navigation`](ros2_ws/src/amr_navigation/README.md)은 A\*↔DWA 토픽 계약 문서다

## 팀

| 이름 | 역할 |
|---|---|
| CY | Team Lead / Fleet Management |
| JW | Simulation |
| YS | SLAM / Localization |
| SW | AI Navigation / Obstacle Avoidance |
| HU | Infrastructure / Perception |

## License

MIT License
