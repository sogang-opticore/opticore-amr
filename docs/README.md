# 문서 목차

> 2026-10-13 작성. 문서를 추가하거나 옮기면 이 목차도 함께 고친다.

## 운영 — 지금 먼저 볼 문서

| 문서 | 내용 |
|---|---|
| [server-run-prescription.md](server-run-prescription.md) | **서버 구동 진단·처방전 (2026-10).** 4대 구동이 느리고 자율주행이 안 된 원인과 단계별 처방, 서버 검진표 |
| [multi-robot-architecture.md](multi-robot-architecture.md) | **구조 처방.** 로봇별 풀스택을 공유 서비스 + 얇은 로봇별 루프로 재편하는 안과 이행 단계 |
| [multi-robot-runbook.md](multi-robot-runbook.md) | 현재 `dev` 그대로 4대를 띄우고 측정하는 절차(정리, 실행, 준비 신호, goal, 측정) |
| [environment-setup.md](environment-setup.md) | RunPod/GPU 서버 환경: 의존성, 워크스페이스 배치, 빌드, GPU 렌더링 확인 |
| [git-workflow.md](git-workflow.md) | 브랜치·커밋 규칙. **기본 브랜치 `main`은 스텁이니 `dev`로 클론** |

## 기능별 설계·검증 기록

당시 기준으로 쓴 기록이다. 각 문서 상단의 "상태 갱신" 노트에 현재 상태가 적혀 있다.

| 폴더 / 문서 | 내용 | 상태 |
|---|---|---|
| [fleet_f1/](fleet_f1/) | F-1 멀티로봇 인프라 (4대 spawn, 토픽·TF 분리, 위치추정 자동화) | `dev` 머지 |
| [fleet_f2/](fleet_f2/) | F-2 교착 해소 `deadlock_manager` | `dev` 머지. 알려진 결함은 처방전 §4.4 |
| [fleet_f3/](fleet_f3/) | F-3 선제 우선순위 `f3_priority_node` | `dev` 머지. 경로 QoS 불일치는 처방전 §4.4 |
| [cctv/](cctv/) | CCTV no-go 오버레이 (P-9 검출, P-10 오버레이), C4 BLOCKED 기록 | `dev` 머지. C4 재검증 미완료 |
| [branch_overview__astar_planner.md](branch_overview__astar_planner.md) | 2026-05 A\* 브랜치 개요 | 보관 (현재 코드와 다름) |

## 패키지 문서 (`ros2_ws/src/`)

| 패키지 | 문서 |
|---|---|
| amr_bringup | [README](../ros2_ws/src/amr_bringup/README.md): 월드·URDF·맵·launch |
| amr_navigation | [README](../ros2_ws/src/amr_navigation/README.md): A\*↔DWA 토픽 계약, 파라미터 |
| amr_slam | [README](../ros2_ws/src/amr_slam/README.md): AMCL·SLAM 설정, 멀티로봇 위치추정 보조 |
| amr_perception | [README](../ros2_ws/src/amr_perception/README.md): fused_tracker, CCTV, YOLO |
| amr_fleet | [README](../ros2_ws/src/amr_fleet/README.md): F-2·F-3 노드 |
| amr_msgs | [README](../ros2_ws/src/amr_msgs/README.md): TrackedObject(Array) |

## 저장소 밖에 있는 것 (서버 네트워크 볼륨)

다음 파일은 저장소에 없다. 새 서버나 새 볼륨에서는 재현 절차가 사라지므로 필요한 것은 저장소로 옮기는 것을 권장한다.

- `/workspace/validation/...`: 게이트 판정 스크립트, `clean_restart.sh`, `run_*.sh`, bag
- `/workspace/env.sh`, `/workspace/setup.sh`, `/workspace/.bashrc_ros`
- `/workspace/models/yolov8n.pt`: YOLO 가중치
