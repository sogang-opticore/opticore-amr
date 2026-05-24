# amr_navigation (Python 패키지)
> 위치: `ros2_ws/src/amr_navigation/amr_navigation/`
> 대상: 코드 작업자, 보충 인력
> 최종 수정: 2026-05-16 / HU

---

## 이 디렉터리의 역할

ROS2 패키지 `amr_navigation`의 실제 Python 구현이 들어있는 디렉터리다.
런치 파일·파라미터·빌드 설정은 상위 디렉터리(`amr_navigation/`)에 있고,
**알고리즘 구현 코드는 전부 여기에 있다.**

```
amr_navigation/amr_navigation/
├── __init__.py
├── astar_node.py     ← A* Global Planner 노드 (이 브랜치 핵심)
├── heuristics.py     ← 휴리스틱·비용함수 순수 모듈
└── README.md         ← 이 문서
```

---

## astar_node.py

### 한 줄 요약
`/map` + `/goal_pose` 를 받아 A* 경로를 계산하고 `/global_path` 로 발행하는 노드.

### 노드 정보

| 항목 | 값 |
|------|----|
| 노드 이름 | `astar_planner` |
| 실행 | `ros2 run amr_navigation astar_planner` |
| 언어 | Python 3 (ROS2 Humble) |
| 담당 | HU (메인) |

### 구독 / 발행

| 방향 | 토픽 | 타입 | QoS |
|------|------|------|-----|
| 구독 | `/map` | `nav_msgs/OccupancyGrid` | Reliable, TRANSIENT_LOCAL |
| 구독 | `/goal_pose` | `geometry_msgs/PoseStamped` | Reliable |
| 발행 | `/global_path` | `nav_msgs/Path` | Reliable, TRANSIENT_LOCAL |

### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `heuristic` | str | `octile` | `manhattan` / `euclidean` / `octile` |
| `allow_diagonal` | bool | `true` | 8-connectivity 여부 |
| `inflation_radius` | float | `0.30` | 장애물 팽창 반경 [m] |
| `smoothing` | str | `catmull_rom` | `none` / `catmull_rom` |

파라미터 파일: `../config/astar_params.yaml`

### 함수 목록 및 설명

```
__init__()
  노드 초기화. 파라미터 선언, 구독/발행 설정, TF 버퍼 초기화.

_map_callback(msg)
  /map 수신 시 호출. 내부 그리드 캐시 갱신 + _build_inflated_grid() 실행.
  맵이 바뀔 때마다 재빌드됨 → [보강-5] 최적화 여지 있음.

_goal_callback(msg)
  /goal_pose 수신 시 호출. A* 실행 트리거.
  맵 미수신 상태면 경고 로그 후 리턴.

_build_inflated_grid(map_msg)
  OccupancyGrid → binary ndarray 변환 후 scipy binary_dilation으로 장애물 팽창.
  inflation_radius / resolution = 팽창 셀 수.
  → [보강-2] 거리 기반 costmap으로 개선 가능.

_is_free_cell(cell)
  셀 (row, col)이 주행 가능한지 판단. inflation 그리드 기준.
  범위 밖 또는 inflation 셀이면 False.

_world_to_cell(x, y)
  월드 좌표 (x, y) → 그리드 셀 (row, col) 변환.
  origin과 resolution 기준.

_cell_to_world(cell)
  그리드 셀 (row, col) → 월드 좌표 (x, y) 변환. 셀 중심값 반환.

_get_robot_position()
  TF lookup으로 현재 로봇 위치 반환 (map frame 기준).
  lookup: odom_filtered → base_footprint.
  실패 시 None. timeout=0.1s.
  → [보강-4] 재시도 로직 없음, 개선 필요.

_astar(start_cell, goal_cell)
  A* 탐색 메인 루프.
  heapq 기반 open set, set 기반 closed set.
  성공: 셀 리스트 반환 / 실패: None 반환.

_get_neighbors(cell)
  allow_diagonal 파라미터에 따라 4 또는 8방향 이웃 셀 반환.

movement_cost(current, neighbor)
  이웃 이동 비용. 직선=1.0, 대각=1.414 (√2).

_reconstruct_path(came_from, current)
  parent dict → 셀 리스트 역추적. start~goal 순서로 반환.

_smooth_catmull_rom(cells)
  Catmull-Rom 스플라인으로 경로 부드럽게 처리.
  → [보강-3] 스무딩 후 장애물 통과 재검증 로직 없음.

_cells_to_path_msg(cells)
  셀 리스트 → nav_msgs/Path 변환.
  각 점의 heading(yaw)은 다음 점 방향으로 계산 → quaternion 변환.
  frame_id = 'map'.
```

### A* 실행 흐름 (코드 추적용)

```
_goal_callback()
  │
  ├─ _get_robot_position()   → start (x, y)
  ├─ _world_to_cell(start)   → start_cell
  ├─ _world_to_cell(goal)    → goal_cell
  │
  ├─ _astar(start_cell, goal_cell)
  │     │
  │     ├─ heapq.heappush(open_set, (0, start_cell))
  │     └─ loop:
  │           ├─ heapq.heappop → current
  │           ├─ current == goal? → _reconstruct_path() → return cells
  │           └─ _get_neighbors(current)
  │                 └─ f = g + heuristic(neighbor, goal)
  │                       └─ heapq.heappush(open_set, (f, neighbor))
  │
  ├─ _smooth_catmull_rom(cells)
  ├─ _cells_to_path_msg(smoothed)
  └─ publisher.publish(path_msg)
```

---

## heuristics.py

### 한 줄 요약
A* 휴리스틱·이동 비용 함수 모음. ROS 의존성 없는 순수 Python 모듈.

### 단위 테스트
```bash
cd /workspace/ros2_ws
colcon test --packages-select amr_navigation --event-handlers console_direct+
# → test/test_heuristics.py 17/17 통과 확인
```

### 함수

```python
manhattan(a: tuple, b: tuple) -> float
  # |dx| + |dy|. 4-connectivity 최적.

euclidean(a: tuple, b: tuple) -> float
  # √(dx²+dy²). 실제 거리.

octile(a: tuple, b: tuple) -> float
  # max(dx,dy) + (√2-1)·min(dx,dy). 8-connectivity 최적. 현재 default.

get_heuristic(name: str) -> callable
  # 이름으로 함수 반환. astar_node.py에서 파라미터로 선택.
```

### 새 휴리스틱 추가 시
1. `heuristics.py`에 함수 추가
2. `get_heuristic()` 딕셔너리에 등록
3. `test/test_heuristics.py`에 테스트 케이스 추가

---

## 보강 작업 항목 (보충 인력 대상)

> 각 항목은 독립적으로 작업 가능. 작업 전 `feature/astar-planner` 기준으로 브랜치 분기.
> 작업 완료 후 `checkme.md` 작성해서 JW에게 전달.

| 번호 | 파일 | 함수 | 내용 | 우선순위 |
|------|------|------|------|----------|
| 보강-1 | `astar_node.py` | `_is_free_cell`, `_build_inflated_grid` | Unloading zone `(37.0, 15.0)` 경로 생성 실패 원인 분석 및 수정 | **높음** |
| 보강-2 | `astar_node.py` | `_build_inflated_grid` | binary dilation → 거리 기반 costmap으로 교체 | 중간 |
| 보강-3 | `astar_node.py` | `_smooth_catmull_rom` | 스무딩 후 장애물 통과 여부 재검증 로직 추가 | 중간 |
| 보강-4 | `astar_node.py` | `_get_robot_position` | TF lookup 실패 시 재시도 로직 추가 | 낮음 |
| 보강-5 | `astar_node.py` | `_map_callback` | 맵 변경 없을 때 inflation 재빌드 스킵 | 낮음 |

각 항목의 상세 내용(현상, 원인, 코드 예시)은 패키지 루트의 `BRANCH_OVERVIEW.md` → "보충 인력 대상 보강 작업 항목" 섹션 참고.

---

## 빌드 & 실행

```bash
# 빌드
cd /workspace/ros2_ws
colcon build --packages-select amr_navigation --symlink-install
source /workspace/ros2_ws/install/setup.bash

# 단독 실행
source /workspace/ros2_ws/install/setup.bash
ros2 run amr_navigation astar_planner

# 런치로 실행 (파라미터 포함)
source /workspace/ros2_ws/install/setup.bash
ros2 launch amr_navigation astar_only.launch.py

# 단위 테스트
source /workspace/ros2_ws/install/setup.bash
colcon test --packages-select amr_navigation --event-handlers console_direct+
```

---

## 주의사항

- **Nav2 기본 플래너 사용 금지** — NavFn, SMAC, DWB 전부 해당. 직접 구현만 사용.
- **`/cmd_vel` 발행 금지** — 이 노드는 경로만 만든다. cmd_vel은 DWA 단독 발행.
- **`ekf.yaml` `world_frame: odom_filtered` 변경 금지** — AMCL 데드락 발생.
- **TF lookup 프레임**: `odom_filtered` 사용. `odom`은 메인 TF 트리에 없음.
