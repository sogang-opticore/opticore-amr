# CCTV SDF 모델 설계

> 브랜치 분기 전 사전 작업 #1
> 작성: HU
> 목적: Cooperative perception 시연용 천장 고정 카메라 도입
> 의존성 없음 — perception/DWA 머지와 독립적으로 진행 가능

---

## 1. 목적

천장에 고정된 카메라(=CCTV)를 시뮬레이션 환경에 도입해, AMR이 직접 보지 못하는 영역(occlusion, 모퉁이 너머, 사각지대)의 객체를 cooperative master에 입력으로 제공.

**의도된 effect**: 단일 AMR perception 대비 occlusion/모퉁이 시나리오에서의 detection recall 향상.

---

## 2. 카메라 스펙 (AMR 카메라와 호환)

AMR 카메라와 동일 스펙으로 맞춰 yolo_detector 노드 재사용 가능하게 함.

| 항목 | 값 | 근거 |
|------|-----|------|
| 해상도 | 640×480 | AMR 카메라와 동일 (perception_p3_context §5) |
| horizontal_fov | 1.3962634 rad (80°) | 동일 |
| Rate | 15Hz | 동일 |
| Noise | Gaussian σ=0.007 | 동일 |
| Clip near | 0.05m | 동일 |
| Clip far | 25.0m | AMR 20m보다 크게 — 천장 5m에서 바닥까지 모두 커버 |
| Format | R8G8B8 | 동일 |
| Optical frame 변환 | rpy = -π/2, 0, -π/2 (AMR과 동일) | TF 일관성 |

**의도된 K matrix**: fx = fy ≈ 381.4, cx=320, cy=240 (AMR과 동일)

---

## 3. 배치 전략

### 3.1 좌표계 결정 — World frame 고정

CCTV는 천장에 영구 고정 → world frame에 정적으로 배치.

**TF 처리 방식**: SDF에 명시한 좌표를 master 노드 파라미터(YAML)로 직접 받음. `static_transform_publisher` 사용 안 함.

**근거**:
- TF 늘리지 않음 (HU 선호)
- 좌표가 한 파일에 명시돼 디버깅 쉬움
- 시뮬레이션 한정 정당화: "CCTV는 인프라 위치 알려짐" — 실제 산업 환경에서도 캘리브레이션 후 정적 좌표 사용

### 3.2 배치 수량 — 결정 보류, 2~3대 가정

**최소 case**: 1대 (경계 영역 커버)
**권장 case**: 2~3대 (전체 창고 커버, ablation 실험 풍부함)

배치 후보 위치 (warehouse.world v3.1 기준, 60×40m):

| ID | 위치 (x, y, z) | 향함 | 커버 영역 | 의도 |
|----|---------------|------|----------|------|
| cctv_1 | (15.0, 20.0, 5.0) | 아래 (pitch=π/2) | Row A-B 통로 | person 동적 영역 occlusion 시연 |
| cctv_2 | (45.0, 20.0, 5.0) | 아래 | Row B-C 통로 + 동쪽 도크 | forklift 영역 + 모퉁이 시나리오 |
| cctv_3 | (12.0, 32.0, 5.0) | 아래 + 약간 남향 | 북쪽 도크 + forklift 통로 | 도크 진입부 |

z=5.0m 근거: 우리 창고 천장 높이 미확정이나, 천장에서 바닥 평면을 직각 시야로 볼 수 있는 충분한 높이.

**참고**: 도크/통로 좌표는 perception_p3_context §환경 정보 + warehouse.world v3.1 실제 좌표 재확인 필요. 위 값은 후보 1차안.

### 3.3 시각적/물리적 표현

**결정: 천장에 visual 모델만 두고 PGM 맵에는 잡히지 않게.**

근거:
- 기둥형 배치는 통로를 좁혀 AMR 통행 방해
- 2D LiDAR는 천장(z=5m)을 못 봄 → PGM에 자동으로 안 잡힘
- visual로만 두면 Foxglove 시각화에서 CCTV 위치 인식 가능

**구성**:
- `<visual>`: 작은 박스 (예: 0.2×0.2×0.1m) + 카메라 모델 mesh
- `<collision>`: 없음 (어차피 AMR이 천장 안 닿음)
- `<static>true</static>` 필수

---

## 4. SDF 구조 초안

### 4.1 모델 파일 위치
```
ros2_ws/src/amr_bringup/models/cctv/
├── model.config
└── model.sdf
```

### 4.2 model.sdf 골격

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="cctv">
    <static>true</static>
    <link name="link">

      <!-- Visual: 작은 박스 (시각적 인식용) -->
      <visual name="body">
        <geometry>
          <box>
            <size>0.2 0.2 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
        </material>
      </visual>

      <!-- 카메라 센서 (AMR 카메라 스펙 복제) -->
      <sensor name="camera" type="camera">
        <update_rate>15</update_rate>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>25.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <visualize>0</visualize>
        <topic>cctv_camera</topic>  <!-- world.sdf의 include name과 결합되어 namespace 형성 -->
      </sensor>

    </link>
  </model>
</sdf>
```

### 4.3 warehouse.world에 include 추가

```xml
<!-- CCTV 천장 배치 (cooperative perception용) -->
<include>
  <uri>model://cctv</uri>
  <name>cctv_1</name>
  <pose>15.0 20.0 5.0 0 1.5708 0</pose>  <!-- pitch=π/2로 아래 향함 -->
</include>

<include>
  <uri>model://cctv</uri>
  <name>cctv_2</name>
  <pose>45.0 20.0 5.0 0 1.5708 0</pose>
</include>
```

---

## 5. 토픽 네이밍

기존 AMR 카메라: `/camera`, `/camera/camera_info`
**CCTV 토픽 (Ignition → ROS2 bridge 거친 후)**:
- `/cctv_1/camera`, `/cctv_1/camera/camera_info`
- `/cctv_2/camera`, `/cctv_2/camera/camera_info`

ros_gz_bridge에 추가 필요 (`warehouse.launch.py` 수정):
```python
bridge_topics = [
    # 기존 AMR 토픽들 (생략)
    # CCTV 토픽 추가
    '/cctv_1/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
    '/cctv_1/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    '/cctv_2/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
    '/cctv_2/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
]
```

---

## 6. yolo_detector 재사용 — Namespace 분리

기존 `yolo_detector.py`를 그대로 재사용. ROS2 namespace로 인스턴스 분리:

```python
# perception.launch.py에 추가
Node(
    package='amr_perception',
    executable='yolo_detector',
    name='yolo_detector',
    namespace='cctv_1',  # → /cctv_1/perception/detections
    parameters=[{'image_topic': '/cctv_1/camera',
                 'camera_info_topic': '/cctv_1/camera/camera_info'}],
),
Node(
    package='amr_perception',
    executable='yolo_detector',
    name='yolo_detector',
    namespace='cctv_2',
    parameters=[{'image_topic': '/cctv_2/camera',
                 'camera_info_topic': '/cctv_2/camera/camera_info'}],
),
```

**전제**: `yolo_detector.py`가 image/camera_info 토픽을 파라미터로 받게 되어 있어야 함. 현재 코드 확인 필요 (S-ADD 또는 P-1/P-2 산출물). 안 되어 있으면 파라미터화 PR 별도 필요.

---

## 7. 검증 절차 (구현 후)

1. Gazebo 실행 → CCTV 모델 천장에 보임 확인 (Foxglove 또는 Ignition GUI)
2. `ros2 topic list | grep cctv` → `/cctv_1/camera`, `/cctv_1/camera/camera_info` 확인
3. `ros2 topic hz /cctv_1/camera` → 15Hz 확인
4. `ros2 topic echo /cctv_1/camera/camera_info` → K matrix 확인 (fx≈fy≈381.4)
5. yolo_detector namespace 분리 실행 → `/cctv_1/perception/detections` publish 확인
6. Foxglove에서 천장 시점 이미지로 person/forklift bbox 표시 확인

---

## 8. 결정 필요 사항

- [ ] CCTV 수량 (1 / 2 / 3)
- [ ] 각 CCTV 정확한 좌표 (warehouse.world v3.1 좌표계 재확인 후 확정)
- [ ] 천장 높이 z=5m 적정성 (창고 높이 미확정)
- [ ] yolo_detector 파라미터화 여부 (현재 코드 확인 필요)
- [ ] visual 모델을 mesh로 할지 단순 box로 할지 (mesh는 cosmetic, box는 빠름)

---

## 9. 작업 순서 (브랜치 분기 후 첫 작업)

1. `models/cctv/` 디렉토리 생성, model.sdf 작성
2. `warehouse.world`에 `<include>` 추가 (1대로 시작)
3. `warehouse.launch.py`의 ros_gz_bridge에 CCTV 토픽 추가
4. 빌드 후 검증 절차 1~4 수행
5. `perception.launch.py`에 yolo_detector namespace 인스턴스 추가
6. 검증 절차 5~6 수행
7. PR: `feat(sim): CCTV camera infrastructure for cooperative perception`
