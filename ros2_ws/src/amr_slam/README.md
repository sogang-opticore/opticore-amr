# amr_slam

위치추정(Nav2 AMCL)·매핑(slam_toolbox) 설정과 멀티로봇 위치추정 보조 노드 (ament_cmake).

> 2026-10-13 갱신: 실제 파일 구성 기준으로 재작성. 정적 맵 파일은 `amr_bringup/maps/`에 있다.

## 구성

| 경로 | 내용 |
|---|---|
| `config/amcl_params.yaml` | Nav2 AMCL 파라미터. 멀티로봇에서는 `multi_robot.launch.py`가 로봇별 `odom_frame_id`/`base_frame_id`/`scan_topic`을 덮어쓴다 |
| `config/slam_params.yaml` | slam_toolbox(async) 매핑 파라미터 |
| `launch/localization.launch.py` | 단일로봇용 map_server + AMCL + lifecycle (토픽 네임스페이스 없음) |
| `launch/slam.launch.py` | `async_slam_toolbox_node` 매핑 |
| `src/fleet_localization_init.py` | 멀티로봇 전용. 로봇별 map_server/AMCL을 직접 lifecycle 전이(재시도)로 active 보장하고 `/amrN/initialpose` 자동 발행. 초기 pose = (world_x − 3.0, world_y − 15.0) |
| `src/imu_covariance_injector.py` | `/imu_raw` → 공분산 주입 → `/imu` (Ignition IMU가 공분산 0을 발행하는 문제 보정) |
| `src/odom_covariance_injector.py` | `/odom` → 공분산·frame_id 보정 → `/odom_with_cov` (EKF 입력) |
| `src/mapping_tour_ver9.py`, `src/square_moving.py` | 매핑용 자율 주행/테스트 스크립트 |

## 주의

- `localization.launch.py`의 사용법 주석(“warehouse.launch.py가 로봇+EKF를 띄운다”)은 멀티로봇 분리 이후 맞지 않는다. 현재 `warehouse.launch.py`는 월드만 띄운다. 실행은 [`docs/multi-robot-runbook.md`](../../../docs/multi-robot-runbook.md) 참고.
- `fleet_localization_init.py`, `imu_covariance_injector.py`는 git 실행 비트가 없다(100644). `--symlink-install` 빌드에서는 `chmod +x`가 필요하다(`scripts/clean_build.sh`가 처리).
- 월드↔맵 오프셋: `fleet_localization_init`은 (3.0, 15.0)을 쓰고, CCTV 작업 실측은 map = world − (2.96, 15.12)이다. AMCL이 수렴하는 범위의 차이다.
- `amcl_params.yaml`의 맵 관련 주석(구 origin, 800×600)은 현재 맵(1207×840, origin (−3.18, −16.9))과 다르다.
