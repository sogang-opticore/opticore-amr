# 멀티로봇 실행 런북 (`dev` 기준)

> 작성: 2026-10-13. 대상: 서버(RunPod 등)에서 `warehouse.launch.py`와 `multi_robot.launch.py`로 AMR 4대를 띄우고 측정하려는 사람.
> 이 문서는 **현재 코드를 그대로** 띄우고 상태를 확인하는 절차를 다룬다. 느림·주행 불량의 원인과 처방은 [`server-run-prescription.md`](server-run-prescription.md)에 있다.

---

## 0. 세션 시작 시 1회 확인

| 확인 항목 | 명령 | 기대값 / 판단 |
|---|---|---|
| 브랜치 | `git -C /workspace/opticore-amr rev-parse --abbrev-ref HEAD` | `dev`. GitHub 기본 브랜치 `main`은 오래된 스텁이다 |
| 워크스페이스가 저장소를 가리키는지 | `readlink -f /workspace/ros2_ws` | 저장소의 `ros2_ws`. `warehouse.launch.py`가 모델 경로를 `/workspace/ros2_ws/src/amr_bringup/models`로 고정한다 |
| 모델 존재 | `ls /workspace/ros2_ws/src/amr_bringup/models` | `standing_person`, `pickup` |
| 재빌드 | `cd /workspace/ros2_ws && colcon build && source install/setup.bash` | `git pull`, 코드·설정 수정 후에는 항상 다시 빌드한다(ament_cmake 패키지는 복사 설치라 빌드 전까지 옛 파일이 실행된다) |
| 설치본과 소스 일치 | `diff -r src/amr_navigation/config install/amr_navigation/share/amr_navigation/config` | 출력 없음 |
| GPU 렌더링 | `nvidia-smi`, `echo $NVIDIA_DRIVER_CAPABILITIES` | 런 중 `ign gazebo` 프로세스가 GPU를 사용. capabilities에 `graphics` 또는 `all` 포함 |
| CPU 쿼터 | `nproc; cat /sys/fs/cgroup/cpu.max 2>/dev/null` | 파드에 실제로 배정된 vCPU 수를 기록해 둔다 |

환경 구성 자체는 [`environment-setup.md`](environment-setup.md)를 따른다.

## 1. 실행 전 정리 (매 런)

이전 런의 프로세스가 남아 있으면 월드가 두 개 겹치거나(`jump back in time`), DDS 공유메모리 잔재로 통신이 느려진다. F-1 런에서 실제로 겪은 문제다.
`pkill -f` 패턴을 `bash -c`로 인라인 실행하면 호출한 셸까지 죽을 수 있으니 **스크립트 파일로** 실행한다.

```bash
cat > ~/clean_restart.sh <<'EOF'
#!/usr/bin/env bash
pkill -INT -f "ros2 launch" ; sleep 3
pkill -f "ign gazebo" ; pkill -f "ign service"
pkill -f parameter_bridge
pkill -f "robot_state_publisher|static_transform_publisher|ekf_node|amcl|map_server|lifecycle_manager"
pkill -f "astar_planner|dwa_node.py|fleet_localization_init|fused_tracker|deadlock_manager|covariance_injector|dynamic_obstacle_mover|yolo_detector|cctv_"
sleep 2
ros2 daemon stop
# 모든 ROS 프로세스가 끝난 뒤에만 FastDDS 공유메모리 잔재를 지운다
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
ps -ef | grep -E "ign gazebo|ros2|_node|planner|bridge" | grep -v grep || echo "clean"
df -h /dev/shm
EOF
bash ~/clean_restart.sh
```

## 2. 실행

로그를 파일로 남겨야 나중에 원인을 찾을 수 있다.

```bash
RUN=~/runs/$(date +%m%d_%H%M); mkdir -p $RUN

# T1 — 월드 (+ 동적 장애물 mover). 기준선 측정은 장애물 없이 시작
ros2 launch amr_bringup warehouse.launch.py dynamic_obstacles:=false 2>&1 | tee $RUN/world.log

# T2 — 로봇 4대 + 로봇별 위치추정/A*/DWA + 전역 노드. 기준선은 deadlock_manager 없이 시작
ros2 launch amr_bringup multi_robot.launch.py enable_deadlock_manager:=false 2>&1 | tee $RUN/robots.log

# (선택) T3 — F-3 선제 우선순위
ros2 launch amr_fleet f3_priority.launch.py
# (선택) T4 — CCTV no-go 오버레이 (amr1에만 연결됨)
ros2 launch amr_perception cctv.launch.py map_topic:=/amr1/map layer_topic:=/amr1/dynamic_obstacle_layer
```

- 스폰 좌표 변경: `AMR1_SPAWN_X=… AMR1_SPAWN_Y=… ros2 launch amr_bringup multi_robot.launch.py`. 기본은 world (3, 13) / (3, 17) / (3, 21) / (3, 25)다. `fleet_localization_init`도 같은 값을 쓴다. `f3_priority.launch.py`는 이 override를 읽지 않는다.
- YOLO는 `multi_robot.launch.py`에 포함되어 있지 않다. 주행은 YOLO 출력을 쓰지 않으므로 주행 검증 런에서는 띄우지 않는다.

## 3. 기동 타임라인과 준비 완료 신호

launch의 `TimerAction`은 **벽시계(wall time)** 기준이다. 시뮬레이션이 느리면(RTF < 1) 아래 시각에 각 노드가 실제로 준비되어 있다는 보장이 없다.

| 시각 (wall) | 이벤트 |
|---|---|
| 0 s | amr1 RSP·map_server·AMCL·lifecycle manager |
| +3 / +5 / +6 / +7 s | amr1 spawn / bridge / odom 주입기 / EKF |
| +10 / +20 / +30 s | amr2 / amr3 / amr4 가 같은 순서로 기동 (10 s 간격) |
| +20 / +30 / +40 / +50 s | amr1 / amr2 / amr3 / amr4 의 A\*, DWA |
| +40 s | `fleet_localization_init`: AMCL 활성화 보장 + initialpose 발행 |
| +50 s | `fused_tracker` (amr1 LiDAR만 입력) |
| +55 s | `deadlock_manager` (`enable_deadlock_manager:=true`일 때) |

goal은 다음을 모두 확인한 뒤에 보낸다.

1. 로그에 `fleet_localization_init DONE — all_ready=True`가 찍혔다. `False`면 어떤 로봇의 AMCL이 active가 아니다.
2. `ros2 topic hz /amr4/amcl_pose`처럼 4대 모두 AMCL pose가 나온다. AMCL은 움직여야 갱신되므로, 정지 상태에서는 initialpose 직후 1회만 나와도 정상이다.
3. `ros2 node list | grep -c planner`가 8이다 (A\* 4 + DWA 4).

## 4. goal 보내기

- 프레임은 `map`이다. **map ≈ world − (3.0, 15.0)** (`fleet_localization_init` 기준. CCTV 작업 실측은 world − (2.96, 15.12)).
- goal은 랙·벽에서 0.6 m 이상 떨어뜨린다. A\*가 막힌 goal을 주변 빈 셀로 옮기는데(goal snap), 그 이동이 크면 DWA가 경로를 버린다(처방전 §4.5).

| 위치 (world) | map 좌표 | 설명 |
|---|---|---|
| (20, 18) | (17.0, 3.0) | Row A와 Row B 사이 통로 |
| (20, 12) | (17.0, −3.0) | Row B와 Row C 사이 통로 |

```bash
# 단일 goal
ros2 topic pub --once /amr1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 17.0, y: 3.0}, orientation: {w: 1.0}}}"

# 여러 waypoint 연속 (DWA가 GOAL_REACHED를 보내면 다음 goal). --timeout-sec 는 waypoint당 "벽시계" 초:
# 기본 240 s 는 RTF 0.33 에서 시뮬 80 s 에 불과하므로 늘려 준다
ros2 run amr_navigation waypoint_sequence.py --goal-topic /amr1/goal_pose \
  --status-topic /amr1/dwa/status --goals "17,3;17,-3" --timeout-sec 900
```

`deadlock_manager`가 켜져 있으면 HOLD/RETREAT/RESUMING 상태인 로봇에 보낸 goal은 무시되고 덮어쓰인다(처방전 §4.4).

## 5. 측정 (런마다 기록)

| 항목 | 명령 | 해석 |
|---|---|---|
| 시뮬레이션 속도(RTF) | `ign topic -e -t /world/warehouse/stats \| grep real_time_factor` (몇 줄 보고 Ctrl-C) | 4대 기준 과거 실측 ≈ 0.33. 처방 목표는 0.8 이상 |
| `/clock` 발행자 수 | `ros2 topic info /clock` | Publisher count가 4면 중복 브리지 상태다(처방전 §3.4) |
| 튜닝 YAML 적용 여부 | `ros2 param get /amr1/dwa_planner robot_radius`<br>`ros2 param get /amr1/astar_planner preferred_clearance`<br>`ros2 param get /amr1/amcl update_min_d` | yaml 값이면 0.36 / 0.5 / 0.05. **0.2 / 1.25 / 0.25가 나오면 YAML이 적용되지 않은 것**이다(처방전 §4.1) |
| 제어 주기 | `ros2 topic hz /amr1/cmd_vel` | 정상은 약 20 × RTF Hz. 그보다 낮으면 DWA 틱이 주기를 넘기고 있다 |
| 경로 발행 | `ros2 topic hz /amr1/global_path` | 약 1 × RTF Hz. 훨씬 낮으면 A\* 계획이 오래 걸린다 |
| 프로세스 CPU | `ps -eo pid,pcpu,rss,args --sort=-pcpu \| head -25` | `astar_planner`·`dwa_node.py`가 각각 100% 근처면 Python 연산 포화 |
| TF 지연 | `ros2 run tf2_ros tf2_monitor map amr1/base_footprint` | 평균 지연과 발행 주기 |
| 센서 파이프라인 지연 | `ros2 run amr_navigation lag_diag.py --ros-args -p use_sim_time:=true -r /lidar:=/amr1/lidar -r /odom:=/amr1/odom -r /imu:=/amr1/imu -r /odometry/filtered:=/amr1/odometry/filtered -r /amcl_pose:=/amr1/amcl_pose` | 1 s 이상 lag면 해당 단계가 병목 |
| 주행·플릿 상태 | `ros2 topic echo /amr1/dwa/status`, `ros2 topic echo /fleet/deadlock_status` | `DYNAMIC_BLOCKED` 장기 지속, `livelock: true` 확인 |

로그에서 바로 찾아볼 문자열:

```bash
grep -E "jump back in time|Message Filter dropping|extrapolation|TF lookup|경로 없음|무시|stale /global_path|LIVELOCK|blocks global path corridor|center-risk reentry" $RUN/*.log | head -50
```

## 6. 종료

T2(로봇) → T1(월드) 순서로 Ctrl-C를 누른 뒤 `bash ~/clean_restart.sh`를 실행한다.

## 7. 토픽·프레임 요약

| 구분 | 이름 |
|---|---|
| 로봇별 입력 | `/amrN/goal_pose` |
| 로봇별 주행 | `/amrN/global_path`, `/amrN/cmd_vel`, `/amrN/dwa/status`, `/amrN/dynamic_obstacle_layer` |
| 로봇별 상태 | `/amrN/odometry/filtered`, `/amrN/amcl_pose`, `/amrN/odom`, `/amrN/ground_truth` |
| 로봇별 센서 | `/amrN/lidar`, `/amrN/imu`, `/amrN/camera`(구독자 없음) |
| 로봇별 맵 | `/amrN/map` (로봇마다 map_server 1개, latched), `/amrN/initialpose` |
| 전역 | `/clock`(4중 브리지), `/tf`, `/tf_static`, `/fleet/priorities`, `/fleet/deadlock_status`, `/fleet/conflict_predictions`(F-3), `/perception/tracked_objects`(amr1 기준), `/cctv/*`(T4 실행 시) |
| TF 체인 | `map → amrN/odom_filtered → amrN/base_footprint → …` (AMCL이 첫 단, EKF가 둘째 단 발행) |

## 8. 이 런북으로 확인되지 않는 것

- 단일로봇 launch(`robot.launch.py`, `nav_full.launch.py`, `astar_dwa.launch.py`)는 현재 `warehouse.launch.py`가 로봇을 스폰하지 않으므로 문서상의 사용법대로는 동작하지 않는다.
- 게이트 판정 스크립트(GATE-GOAL/COLLISION 등)는 서버의 `/workspace/validation/`에만 있고 저장소에는 없다.
