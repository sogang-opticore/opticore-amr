# feature/astar-planner 브랜치 개요
> 작성: HU | 날짜: 2026-05-16 | 대상: Control Tower, 팀 내부, 보충 인력

---

## 이 브랜치가 하는 일

`feature/astar-planner`는 Opticore AMR의 **전역 경로계획(Global Path Planning)** 기능을 구현한 브랜치다.

로봇이 창고 내에서 "A 지점에서 B 지점까지 어떤 경로로 갈 것인가"를 결정하는 역할이다. 구체적으로는 SLAM이 만들어준 점유 격자 지도(OccupancyGrid)를 받아 A* 알고리즘으로 최단 경로를 계산하고, 그 결과를 `/global_path` 토픽으로 발행한다. DWA Local Planner(N-2)는 이 경로를 따라가면서 실시간 장애물을 회피한다.

**이 브랜치는 "어디로 갈지"만 결정한다. "어떻게 움직일지"는 DWA 담당이다.**

---

## 프로젝트 내 위치

```
전체 파이프라인:
  Gazebo (시뮬레이터)
    └── SLAM (SL-1~SL-4) ← 완료
         ├── /map          → [이 브랜치] A* Global Planner
         ├── /odometry/filtered
         └── TF: map → odom_filtered → base_footprint
                              ↓
                    /global_path (nav_msgs/Path)
                              ↓
                    DWA Local Planner (N-2, 미구현)
                              ↓
                    /cmd_vel → 로봇 실제 이동
```

Navigation 서브시스템의 **첫 번째 레이어**다. SLAM이 완료된 시점부터 착수 가능했고, 이 브랜치가 병목이었다.

---

## 베이스 브랜치

- **베이스**: `dev` (SL-1 ~ SL-4 전부 포함)
- **merge 시 목적지**: `dev`
- 이 브랜치 완료 후 DWA 브랜치(`feature/dwa-planner`, N-2)가 `/global_path`를 소비한다.

---

## 구현된 내용

### 1. A* 노드 — `astar_node.py`

핵심 노드. ROS2 노드로 동작하며 다음을 담당한다.

**구독**
- `/map` (`nav_msgs/OccupancyGrid`, TRANSIENT_LOCAL QoS): SLAM이 만든 창고 지도. 수신 즉시 내부 그리드 캐시 + inflation 그리드 빌드.
- `/goal_pose` (`geometry_msgs/PoseStamped`): 목적지 좌표. 수신 시 즉시 경로 계획 실행.

**발행**
- `/global_path` (`nav_msgs/Path`, TRANSIENT_LOCAL QoS): 계산된 경로. frame_id=`map`. DWA가 소비.

**주요 내부 함수**

| 함수 | 역할 |
|------|------|
| `_map_callback()` | 맵 수신 시 inflation 그리드 자동 빌드 |
| `_goal_callback()` | goal 수신 시 A* 실행 트리거 |
| `_build_inflated_grid()` | binary dilation으로 장애물 팽창 (로봇 크기 여유 확보) |
| `_is_free_cell()` | 셀이 주행 가능한지 판단 (inflation 포함) |
| `_get_robot_position()` | TF lookup으로 현재 로봇 위치 추출 (`odom_filtered → base_footprint`) |
| `_astar()` | A* 탐색 메인 루프 (heapq 기반 open set, closed set) |
| `_get_neighbors()` | 이웃 셀 반환 (4 또는 8-connectivity) |
| `_reconstruct_path()` | parent dict → 셀 리스트 역추적 |
| `_smooth_catmull_rom()` | Catmull-Rom 스플라인으로 경로 부드럽게 처리 |
| `_cells_to_path_msg()` | 셀 리스트 → `nav_msgs/Path` 메시지 변환 (quaternion heading 포함) |

**A* 알고리즘 동작 흐름**

```
1. /goal_pose 수신
2. TF lookup → 현재 로봇 위치 (map frame 기준 x, y)
3. 월드 좌표 → 그리드 셀 변환 (origin + resolution 기준)
4. open set에 start 셀 삽입 (f=0)
5. 루프:
   a. f값 최소 셀 pop
   b. goal 도달 시 → _reconstruct_path() → 종료
   c. 이웃 셀 탐색 → g(n) 계산 (직선=1.0, 대각=1.414)
   d. f(n) = g(n) + h(n) 계산
   e. open set에 push
6. open set 소진 시 → 빈 Path 발행 (도달 불가)
7. 경로 → Catmull-Rom 스무딩
8. /global_path 발행
```

### 2. 휴리스틱 모듈 — `heuristics.py`

ROS 의존성 없는 순수 Python 모듈. A* 노드와 분리돼 있어 단위 테스트가 독립적으로 가능하다.

| 함수 | 수식 | 특징 |
|------|------|------|
| `manhattan(a, b)` | `|dx| + |dy|` | 4-connectivity에 최적 |
| `euclidean(a, b)` | `√(dx²+dy²)` | 실제 직선거리, 과소평가 없음 |
| `octile(a, b)` | `max(dx,dy) + (√2-1)·min(dx,dy)` | 8-connectivity에 최적, 현재 default |

단위 테스트 17/17 통과 (`test/test_heuristics.py`).

### 3. 파라미터 설정 — `config/astar_params.yaml`

```yaml
astar_planner:
  ros__parameters:
    heuristic: octile           # manhattan / euclidean / octile
    allow_diagonal: true        # 8-connectivity (false면 4-connectivity)
    inflation_radius: 0.30      # 장애물 팽창 반경 [m] (로봇 폭 0.40m + 여유 0.10m)
    smoothing: catmull_rom      # none / catmull_rom
```

### 4. 런치 파일 — `launch/astar_only.launch.py`

A* 노드 단독 실행용. DWA 없이 경로 계획만 검증할 때 사용.

```bash
ros2 launch amr_navigation astar_only.launch.py
```

### 5. 패키지 빌드 설정

C++ 골격에서 Python 전면 교체 완료.
- `package.xml`: `ament_cmake_python` 추가, C++ 의존성 제거
- `CMakeLists.txt`: `astar_planner` Python executable 등록
- `include/amr_navigation/astar_planner.hpp`: placeholder 유지 (C++ 미채택 명시)

---

## TF 트리 및 프레임 구조

이 브랜치 작업 중 가장 중요한 인프라 이슈였다. 정확히 이해하고 있어야 보강 작업 시 문제가 없다.

```
map
 └── odom_filtered          (AMCL 발행, ~8.7Hz)
      └── base_footprint    (EKF 발행, ~77Hz)
           └── base_link    (RSP/URDF)
                ├── lidar_link
                ├── imu_link
                └── wheel_links ...

/tf_gazebo (격리 — /tf에 합류 안 함)
  odom → base_footprint    (Gazebo DiffDrive, 충돌 방지용 격리)
```

**핵심**: `odom` 프레임은 메인 TF 트리에 없다. `odom_filtered`가 실제 연결 프레임이다.

`ekf.yaml`의 `world_frame: odom_filtered` 설정은 **의도적 설계**다. `map`으로 바꾸면 AMCL과 TF 데드락이 발생한다. 절대 변경하지 말 것.

A* 노드에서 TF lookup 시:
```python
# 로봇 현재 위치 (전역 경로 계획용)
self.tf_buffer.lookup_transform('odom_filtered', 'base_footprint', ...)
# map → odom_filtered → base_footprint 체인으로 자동 연결됨
```

---

## 토픽 인터페이스 (이 브랜치 관련)

| 방향 | 토픽 | 타입 | QoS | 비고 |
|------|------|------|-----|------|
| 입력 | `/map` | `nav_msgs/OccupancyGrid` | TRANSIENT_LOCAL | SLAM 발행 |
| 입력 | `/goal_pose` | `geometry_msgs/PoseStamped` | Reliable | frame_id=`map` |
| 출력 | `/global_path` | `nav_msgs/Path` | TRANSIENT_LOCAL | frame_id=`map`, DWA 소비 |

**이 브랜치는 `/cmd_vel`을 발행하지 않는다.** `/cmd_vel`은 DWA 단독 발행 원칙.

---

## 실행 순서

```bash
# 터미널 1 — Gazebo + 로봇 (약 8초 기동 대기)
source /workspace/ros2_ws/install/setup.bash
ros2 launch amr_bringup warehouse.launch.py

# 터미널 2 — SLAM + AMCL (warehouse 완전 기동 후)
source /workspace/ros2_ws/install/setup.bash
ros2 launch amr_slam localization.launch.py

# 터미널 3 — A* 노드
source /workspace/ros2_ws/install/setup.bash
ros2 run amr_navigation astar_planner

# 터미널 4 — Foxglove 브릿지
source /workspace/ros2_ws/install/setup.bash
ros2 run foxglove_bridge foxglove_bridge

# 터미널 5 — Goal 발행 (소스 불필요)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 20.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Foxglove (`ws://localhost:8765`) 접속 후:
- Fixed frame: `map`
- Topics: `/map` + `/global_path` 활성화 → 맵 위 경로 오버레이 확인

---

## 완료 기준 달성 현황

| 항목 | 상태 |
|------|------|
| A* 알고리즘 전체 구현 | ✅ |
| 단위 테스트 17/17 통과 | ✅ |
| warehouse 맵에서 경로 생성 성공 | ✅ (goal 20.0, 5.0 기준 / 2001 웨이포인트) |
| Foxglove `/global_path` 시각화 | ✅ (맵 오버레이 확인) |
| DWA 연동 end-to-end | ❌ (N-2 브랜치에서 진행) |

---

## 보충 인력 대상 — 보강 작업 항목

> 아래 항목들은 MVP 완료 후 품질 향상을 위한 후속 작업이다.
> 각 항목은 독립적으로 작업 가능하다. 작업 전 반드시 `feature/astar-planner` 브랜치 기준으로 분기할 것.

---

### [보강-1] Unloading Zone goal 경로 생성 실패 해결 ← 우선순위 높음

**파일**: `astar_node.py` → `_is_free_cell()`, `_build_inflated_grid()`

**현상**: goal `(35.0, 15.0)` (Unloading zone) 경로 생성 실패. goal `(20.0, 5.0)`은 성공.

**원인 추정**: `inflation_radius=0.30m`로 인해 goal 셀 또는 인근 경로가 막힌 것으로 보임. 창고 맵에서 해당 좌표가 선반 인근이라 inflation에 걸릴 가능성 있음.

**작업 내용**:
1. goal 셀 자체가 inflation에 걸렸는지 확인:
   ```python
   # astar_node.py에 임시 로그 추가
   goal_cell = self._world_to_cell(goal_x, goal_y)
   self.get_logger().info(f'goal cell: {goal_cell}, free: {self._is_free_cell(goal_cell)}')
   ```
2. `inflation_radius`를 0.20m로 줄여서 재시도
3. goal 셀이 막힌 경우 nearest free cell로 자동 이동하는 로직 추가 검토

**웨이포인트 좌표 참고**:
- Loading zone: `(3.0, 15.0)`
- Unloading zone: `(37.0, 15.0)` ← 이게 막힘
- Charging station: `(37.0, 3.0)`

---

### [보강-2] binary dilation → 거리 기반 costmap으로 개선 ← 우선순위 중간

**파일**: `astar_node.py` → `_build_inflated_grid()`

**현상**: 현재 inflation은 binary dilation 방식. 장애물로부터 일정 반경 내 셀을 전부 막아버린다. 이 때문에 좁은 통로에서 경로가 없다고 판단하는 경우가 생길 수 있다.

**작업 내용**: 거리 기반 costmap으로 교체. 장애물에서 멀수록 낮은 비용을 부여해 A*가 경로를 찾으면서도 안전한 쪽을 선호하도록 만든다.

```python
# 현재 방식 (binary)
inflated[r, c] = 1  # 막거나 아니거나

# 개선 방식 (거리 기반 costmap)
cost = max(0, 1.0 - dist / inflation_radius)
inflated[r, c] = cost  # 0.0~1.0 사이 비용
# A*의 movement_cost()에서 이 cost를 g(n)에 더함
```

**주의**: `_is_free_cell()`과 `movement_cost()`도 함께 수정 필요. 이진 판단에서 비용 기반으로 바뀌므로 A* 전체 비용 계산 흐름 확인할 것.

---

### [보강-3] 스무딩 후 장애물 통과 여부 재검증 ← 우선순위 중간

**파일**: `astar_node.py` → `_smooth_catmull_rom()`

**현상**: Catmull-Rom 스무딩 적용 후 원래 A* 경로가 장애물을 피해갔더라도 스무딩된 곡선이 장애물을 통과할 수 있다.

**작업 내용**: 스무딩 후 각 점이 `_is_free_cell()` 통과하는지 검증 추가. 실패한 구간은 스무딩을 적용하지 않고 원래 A* 경로로 fallback.

```python
def _smooth_catmull_rom(self, cells):
    smoothed = _apply_catmull_rom(cells)
    # 검증 추가
    for pt in smoothed:
        if not self._is_free_cell(self._world_to_cell(pt.x, pt.y)):
            return cells  # fallback to original
    return smoothed
```

---

### [보강-4] TF lookup 초기 실패 시 재시도 로직 ← 우선순위 낮음

**파일**: `astar_node.py` → `_get_robot_position()`

**현상**: 노드 기동 직후 TF가 아직 올라오지 않은 경우 timeout=0.1s로 즉시 실패하고 None을 반환한다. goal이 이 타이밍에 들어오면 경로 계획이 스킵된다.

**작업 내용**: 재시도 로직 추가.

```python
def _get_robot_position(self, retries=3, delay=0.5):
    for attempt in range(retries):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom_filtered', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),  # timeout 늘림
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as e:
            self.get_logger().warn(f'TF lookup 실패 (시도 {attempt+1}/{retries}): {e}')
            time.sleep(delay)
    return None
```

---

### [보강-5] 맵 업데이트마다 inflation 재빌드 최적화 ← 우선순위 낮음

**파일**: `astar_node.py` → `_map_callback()`

**현상**: `/map` 토픽이 업데이트될 때마다 `_build_inflated_grid()`를 전체 재빌드한다. 창고 맵은 정적이라 실제로는 거의 변하지 않지만 불필요한 연산이 반복된다.

**작업 내용**: 맵 해시 비교 후 변경이 있을 때만 재빌드.

```python
def _map_callback(self, msg):
    new_hash = hash(bytes(msg.data))
    if new_hash == self._map_hash:
        return  # 변경 없음, 재빌드 스킵
    self._map_hash = new_hash
    self._build_inflated_grid(msg)
```

---

## 미확정 사항 (임의 채우기 금지)

| 항목 | 현재값 | 확정 방법 |
|------|--------|----------|
| `alpha_max` (최대 각가속도) | 임시 1.5 rad/s² | 시뮬레이션 step response 측정 후 갱신 |
| Unloading zone `(37.0, 15.0)` 경로 생성 가능 여부 | 미확인 (막힘) | 보강-1 작업 후 확인 |
| 스무딩 후 장애물 재검증 실패율 | 미측정 | 보강-3 작업 후 통계 수집 |

---

## 파일 목록 최종

```
ros2_ws/src/amr_navigation/
├── package.xml                         ← Python 기준 전면 교체
├── CMakeLists.txt                      ← ament_cmake_python, executable 등록
├── README.md                           ← A*↔DWA 인터페이스 계약 문서 (SW 작성, HU 리뷰)
├── include/amr_navigation/
│   └── astar_planner.hpp               ← placeholder (C++ 미채택)
├── amr_navigation/
│   ├── __init__.py
│   ├── astar_node.py                   ← 핵심 구현 (이 브랜치 메인)
│   └── heuristics.py                   ← 휴리스틱·비용함수 (단위 테스트 분리)
├── config/
│   └── astar_params.yaml               ← 파라미터 외부화
├── launch/
│   └── astar_only.launch.py            ← A* 단독 실행용
└── test/
    └── test_heuristics.py              ← 단위 테스트 (17/17 통과)
```
