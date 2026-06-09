# CLAUDE.md — Opticore AMR / F-1 Fleet 무인 작업 가드

이 워크스페이스는 ROS2 Humble + Gazebo Ignition Fortress 기반 물류 AMR 시뮬이다.
지금 너(Claude Code)는 **F-1 멀티로봇 fleet** 작업을 **격리 브랜치에서 무인으로** 진행한다.
구현·검증·커밋은 위임됐지만, 아래 가드레일은 **절대** 어기지 않는다.

> 기존에 CLAUDE.md가 있으면 이 내용을 머지해서 쓴다. 없으면 **레포 루트(`/workspace/github/opticore-amr/`)**에 둔다 — CC를 거기서 띄우고, 빌드로 `ros2_ws`에 들어가도 상위 탐색으로 잡힌다.

## 0. 절대 금지 (HARD STOP)
- **Pod를 Terminate/Stop 하지 마라.** 비용/세션 종료는 사람(HU)만 결정한다. 너에겐 그 권한이 없다.
- **`git add -A` 금지.** 항상 명시적 파일 경로로만 add. `install/`·빌드 산출물은 **절대** 커밋 금지.
  - `amr_navigation`이 `install/.../local/lib/python3.10/dist-packages`에 깔려 `ModuleNotFoundError`가 나도, 그건 `env.sh`의 PYTHONPATH로 해결된 **기존 이슈**다. install/을 커밋해서 "고치지" 마라.
- **PR 생성 금지.** dev로의 머지/PR은 HU가 직접 한다.
- **브랜치 격리**: `feature/fleet-f1-cc`(= `feature/fleet-multi-robot-spawn`에서 분기) **에서만** 작업·커밋. `dev`와 `feature/fleet-multi-robot-spawn`은 **절대 직접 건드리지 마라**(JW 작업 보호).
- **파괴적 셸 금지**: `rm -rf`는 `/workspace/validation/scratch/` 같은 **명시 스크래치 경로에서만**. `/workspace/ros2_ws`·`$HOME`·상위 경로에 `rm -rf`·`git clean -fdx`·`git reset --hard` **절대 금지**. `/workspace`는 영구 Network Volume이다 — 날리면 복구 불가.
- **세션 체크포인트 = 3시간.** 3시간마다(또는 게이트 통과 시) WIP를 명시 커밋 + `PROGRESS.md`에 한 줄 기록 후 계속. 24시간 런 = "검증된 3시간 세그먼트의 연속"이지, 하나의 거대 블롭이 아니다.

## 1. 셸 부트스트랩 (모든 새 터미널 · 매 세션 첫 줄)
```bash
source /workspace/env.sh
```
`env.sh` 내용(참고, **수정 금지**):
```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash   # = /workspace/github/opticore-amr/ros2_ws (symlink 동일)
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
# (+ amr_navigation PYTHONPATH 우회 포함)
```

## 2. 클린 재시작 의식 (매 launch 전 필수)
```bash
pkill -9 -f "ign gazebo|ruby.*gz|parameter_bridge|dynamic_obstacle_mover|robot_state_publisher|ekf_node|foxglove_bridge|amcl|map_server|yolo_detector|fused_tracker"
ros2 daemon stop && ros2 daemon start
```
잔존 ign 프로세스가 두 월드를 겹치게 만든다. launch가 꼬이면 묻지 말고 클린 재시작.

## 3. 시간 낭비 금지 — 이미 규명된 환경 사실 (다시 파지 마라)
- **RTF ≈ 0.33은 정상**이다. ign 단일프로세스 CPU 바운드. GPU 모델·vCPU 수로 안 빨라진다. RTF 낮다고 soft-render 의심하며 시간 쓰지 마라.
- **soft-render 판정은 단 하나**: `GPU 점유 0% + libEGL "Permission denied"`. 이게 아니면 GPU 렌더 정상(EGL surfaceless, `/dev/nvidia*` 직통). **이 경우에만** EGL을 건드려라.
- **`use_sim_time: true`가 전부에 걸려 있다** → 모든 노드 `now()`가 `/clock` 공유 → 시스템 전체가 RTF 속도로 흐른다. **모든 타임아웃을 RTF로 스케일**하라(leg timeout 기준 ≈ 240s).
- **`dynamic_obstacle_mover`는 ROS 토픽을 발행하지 않는다**(ign `set_pose_vector`로 Gazebo 엔티티만 옮김). `ros2 topic echo`로 장애물 위치 보려다 hang 나는 건 정상. 그라운드 트루스는 `/ground_truth`(또는 Gazebo)에서 가져와라.
- **AMCL은 장시간 주행/텔레포트 후 락을 잃을 수 있다.** 위치 추정이 깨지면 push 하지 말고 **클린 재시작(§2)**이 안전. garbage 위에서 검증 계속하지 마라.

## 4. F-1의 핵심 = TF 분리 3종 세트
`robot_name`만으로는 core TF(`base_footprint`/`odom_filtered`/`base_link`)가 안 갈린다. 반드시 셋 다:
1. `frame_prefix`(robot_state_publisher) → `amrN/base_link` 등
2. EKF frame override(`odom_frame=amrN/odom_filtered`, `base_link_frame=amrN/base_footprint`) — 런타임 yaml(JW가 OpaqueFunction으로 구현)
3. **단일 글로벌 `/tf` + prefixed frames** — 로봇별 `/tf`로 쪼개지 마라(JW가 막다른 길로 확인). 4대가 같은 `/tf`에 **서로 안 겹치는** prefixed frame을 쏴서 공존시키는 게 정답.

목표 TF 체인(로봇별):
```
map → amrN/odom_filtered (AMCL) → amrN/base_footprint (EKF) → amrN/base_link (RSP) → {lidar, camera, imu}
```

## 5. 검증 게이트 (무인 런의 안티-스핀 핵심)
**"노드가 다 떴다"는 PASS가 아니다.** PASS는 **TF·bag 정량**으로만 판정한다.
- **GATE-T (TF 정확성, 키스톤)**: 4대 모두 `lookup_transform(map, amrN/base_footprint)` **성공** + 값이 spawn 근방으로 sane. 하나라도 실패/NaN/원점고정이면 **FAIL**.
- **GATE-TOPIC**: `ros2 topic list | grep "amrN/amrN"` **공집합**(이중 prefix 없음). `/amrN/odometry/filtered` 4개 다 정상 Hz.
- **GATE-GOAL**: 로봇별 `/amrN/goal_pose` 발행 → 최종 pose가 goal의 ARRIVAL_TOL(0.6m) 내. bag(`/tf` 또는 `/amrN/odometry/filtered`)으로 도달 판정.
- **GATE-COLLISION**: 주행 bag에서 로봇 간(0.60×0.40 사각형, 실제 yaw) **오버랩 0 샘플**. 기존 `analyze_*.py`의 사각형 overlap 로직 재사용.

게이트 스크립트는 네가 작성/실행한다(기존 `/workspace/validation/dwa_dyn/analyze_*.py` 패턴 참고; **db3 metadata 없을 수 있으니 db3 직접 파싱**).

## 6. 작업 규율
1. **EXPLORE**: 관련 파일만 읽어라(코드 쓰지 마). `multi_robot.launch.py` / `robot.launch.py` / EKF yaml 생성부 / astar·dwa node / odom injector / fused_tracker.
2. **PLAN**: `ultrathink`로 계획 세우고 **`PLAN.md`에 적고 커밋**(사람이 비동기 리뷰). 아직 코드 쓰지 마.
3. **CODE**: 계획대로 구현. 한 번에 한 🔴씩.
4. **VERIFY**: §5 게이트로 bag 검증. **PASS만 진짜 완료**.
5. **COMMIT**: 게이트당 명시적 파일 커밋(§0 add 규칙 준수).
6. **막히면(같은 게이트 3회 FAIL)**: 강행하지 말고 `BLOCKED.md`에 증상·시도·로그 적고 **그 태스크를 멈춰라**. 24시간 헛돌지 마라. 다음 🔴로 넘어가거나 대기.

## 7. 레포 사실
- 워크스페이스: `/workspace/ros2_ws` (= `/workspace/github/opticore-amr/ros2_ws`, symlink 동일)
- 패키지: `amr_bringup`(launch/spawn) · `amr_navigation`(A*/DWA) · `amr_slam`(EKF/SLAM/AMCL) · `amr_perception`(yolo/fused_tracker) · `amr_msgs`
- launch(**확정 이름**):
  - `amr_bringup`: `warehouse.launch.py` / `robot.launch.py` / `multi_robot.launch.py`
  - `amr_slam`: `slam.launch.py` / `localization.launch.py`  ← `amcl.launch.py` **아님(stale)**
  - `amr_perception`: `perception.launch.py` / `fused_tracker.launch.py`
- 빌드: `cd /workspace/ros2_ws && colcon build --symlink-install` (symlink이라 `.py` 수정은 대개 재빌드 불요, 노드만 재시작)
- 검증 산출물 위치: `/workspace/validation/` (비커밋)
- 단일로봇 spawn 기준: world (3, 15) / 멀티 temp 좌표: amr1~4 Y=13/17/21/25 (S-9 확정 시 조정 — 이번 런 범위 밖)
