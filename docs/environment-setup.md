# 환경 설정 가이드 (RunPod / GPU 서버)

> 2026-10-13 갱신. 워크스페이스 경로 규약, 빌드 방식, 누락 의존성, GPU 렌더링 확인, 런 사이 정리, CPU 스레드 설정을 추가했다.
> 실행 절차는 [`multi-robot-runbook.md`](multi-robot-runbook.md), 성능·주행 문제 진단은 [`server-run-prescription.md`](server-run-prescription.md)에 있다.

## 1. RunPod 정보

| 항목 | 내용 |
|---|---|
| GPU | RTX A5000 / A40 |
| Network Volume | `opticore-amr-vol` (50GB, `/workspace` 마운트) |
| 비용 | $0.27~0.40/hr (GPU), $3.50/월 (Volume) |

이 프로젝트의 병목은 GPU보다 **CPU**다. 로봇 4대분의 Python 계획기·제어기가 동시에 돈다. 파드를 고를 때 vCPU 수를 확인하고, 런 기록에 `nproc`와 `cat /sys/fs/cgroup/cpu.max` 결과를 남긴다.

## 2. 새 Pod 시작 순서

```bash
# 1. bashrc 복구
cp /workspace/.bashrc_ros ~/.bashrc && source ~/.bashrc

# 2. 환경 설치 (최초 1회 또는 업데이트 시)
bash /workspace/setup.sh 2>&1 | tee /workspace/setup.log

# 3. sympy 충돌 시
python3.10 -m pip install ultralytics opencv-python transforms3d foxglove-websocket --ignore-installed sympy
```

> ⚠ `/workspace/.bashrc_ros`, `/workspace/setup.sh`, `/workspace/env.sh`는 **네트워크 볼륨에만 있고 저장소에는 없다.** 새 서버나 새 볼륨에서는 아래 §3 목록으로 직접 설치한다. 이 스크립트들을 저장소(`scripts/`)로 옮기는 것을 권장한다.

## 3. 의존성 목록

| 구분 | 패키지 | 비고 |
|---|---|---|
| ROS (apt) | `ros-humble-desktop`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-robot-localization`, `ros-humble-slam-toolbox`, `ros-humble-ros-gz`, `ros-humble-foxglove-bridge`, `ros-humble-vision-msgs`, `ros-humble-xacro` | Nav2는 AMCL·map_server·lifecycle_manager만 사용 |
| 시뮬레이터 (apt) | `ignition-fortress` | |
| Python | `numpy<2` | `cv_bridge`와 NumPy 2.x 충돌 기록이 있다 |
| Python | `scipy` | **A\* 필수**(`distance_transform_edt`). `package.xml`에 선언되어 있지 않아 다른 패키지가 함께 설치해 줄 때만 존재한다 |
| Python | `transforms3d`, `opencv-python` | |
| Python (선택) | `ultralytics` (torch 포함), 가중치 `/workspace/models/yolov8n.pt` | YOLO를 쓸 때만. 주행에는 쓰이지 않는다 |
| 빌드 도구 | `setuptools==65.5.0`(개인 Dockerfile 기준) 또는 `<80` | 최신 setuptools와 `colcon build --symlink-install` 조합에서 `amr_perception`이 `--editable` 오류를 낸 기록이 있다 |

`amr_bringup/package.xml`에 있는 `gazebo_ros`, `gazebo_ros_pkgs`는 Gazebo Classic 시절의 잔재라 설치할 필요가 없다.

## 4. 워크스페이스 배치 — 소스 트리는 하나만

- 팀 규약상 활성 워크스페이스는 `/workspace/ros2_ws`다(F-1 기록). `warehouse.launch.py`도 모델 경로를 `/workspace/ros2_ws/src/amr_bringup/models`로 **하드코딩**한다.
- 저장소를 다른 곳에 클론해 따로 빌드하면 두 가지 문제가 생긴다.
  - `ros2 launch`가 예전 설치본을 실행한다.
  - 보행자·지게차 모델이 로드되지 않는다.
- 권장 배치는 저장소를 `dev`로 클론한 뒤 `/workspace/ros2_ws`가 저장소의 `ros2_ws`를 가리키게 하는 것이다.

```bash
cd /workspace
git clone -b dev git@github.com:sogang-opticore/opticore-amr.git
# /workspace/ros2_ws 가 이미 별도 디렉터리로 있으면 먼저 내용을 확인하고 백업한 뒤 진행
ln -s /workspace/opticore-amr/ros2_ws /workspace/ros2_ws
```

## 5. 빌드

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build                     # 권장: symlink-install 없이
source install/setup.bash
```

- **코드나 설정을 바꾼 뒤에는 반드시 다시 빌드한다.**
  - `amr_navigation`, `amr_slam`, `amr_fleet`, `amr_bringup`은 ament_cmake 패키지라 Python 스크립트와 yaml을 **복사 설치**한다.
  - 다시 빌드하지 않으면 예전 파일이 실행된다. F-1에서 실제로 "stale astar_planner" 블로커를 겪었다.
- `--symlink-install`을 쓸 때 주의할 점:
  1. git 실행 비트가 없는 스크립트는 실행되지 않는다(`amr_slam/src/fleet_localization_init.py`, `imu_covariance_injector.py`). `chmod +x`가 필요하다(`scripts/clean_build.sh`가 처리).
  2. `amr_perception`은 setuptools 버전에 따라 실패할 수 있다. 실패하면 이 패키지만 symlink 없이 빌드한다: `colcon build --packages-select amr_perception`.
  3. `scripts/clean_build.sh full`은 `amr_perception`을 기본으로 건너뛴다. 그런데 `multi_robot.launch.py`는 `amr_perception` 설치본을 요구하므로, 따로 빌드하지 않으면 launch가 실패한다.
- 설치본과 소스가 같은지 확인: `diff -r src/amr_navigation/config install/amr_navigation/share/amr_navigation/config`

## 6. GPU 렌더링 확인 (중요)

시뮬레이터는 headless(`ign gazebo -s`)로 뜬다. 그래도 센서는 ogre2로 렌더링하므로 OpenGL/EGL이 필요하다. 렌더링하는 센서는 gpu_lidar 4개, 로봇 카메라 4대, CCTV 4대다. 컨테이너에 NVIDIA 그래픽 라이브러리가 노출되지 않으면 Mesa 소프트웨어 렌더러(llvmpipe)로 떨어져 **CPU로 렌더링**하게 되고, 시뮬레이션이 극단적으로 느려진다.

| 확인 | 명령 | 정상 |
|---|---|---|
| 드라이버 capability | `echo $NVIDIA_DRIVER_CAPABILITIES` | `graphics` 또는 `all` 포함 (`compute,utility`만 있으면 그래픽 불가) |
| 시뮬 중 GPU 사용 | `nvidia-smi` | `ign`/`ruby` 프로세스가 GPU 메모리 사용, util > 0 |
| 실제 렌더러 | `grep -E "GL_VENDOR\|GL_RENDERER" ~/.ignition/rendering/ogre2.log` | NVIDIA. `llvmpipe`면 소프트웨어 렌더링 |
| Xvfb 사용 시 | `DISPLAY=:1 glxinfo -B \| grep "OpenGL renderer"` | NVIDIA. Xvfb는 보통 llvmpipe로 동작한다 |
| EGL headless 옵션 | `ign gazebo --help \| grep -i headless` | 지원되면 `--headless-rendering`으로 X 서버 없이 EGL 렌더링 가능(현재 launch는 미사용) |

- ign-rendering은 **`DISPLAY`가 열리면 GLX를 쓰고, 디스플레이를 열 수 없을 때만 EGL headless로 전환**한다. Xvfb를 띄워 `DISPLAY`를 잡아 두면 오히려 llvmpipe로 렌더링될 수 있다.
- GPU 서버에서는 Gazebo를 실행하는 셸에서 `unset DISPLAY`를 하고, 지원되는 경우 `--headless-rendering`을 붙여 EGL로 돌리는 것을 우선 시도한다.
- 2026-06 CCTV 런에서는 "GPU 렌더 정상(util>0, libEGL 정상)"으로 기록되어 있다. 새 파드나 새 서버에서는 매번 `GL_RENDERER`로 다시 확인한다.
- 첫 로드 시 `standing_person` 메시를 Fuel(인터넷)에서 받아온다(`~/.ignition/fuel` 캐시). 인터넷이 막힌 서버에서는 미리 캐시를 채워 둔다.

## 7. ROS 2 / DDS 환경

- 같은 네트워크를 여러 사람이 쓰면 `export ROS_DOMAIN_ID=<사람별 번호>`로 분리한다.
- 런 사이에는 프로세스와 FastDDS 공유메모리 잔재(`/dev/shm/fastrtps_*`)를 정리한다. 절차는 [`multi-robot-runbook.md`](multi-robot-runbook.md) §1에 있다. `pkill -9`를 반복하면 잔재가 쌓여 통신이 느려진다(F-1 기록).
- Docker 기본 `/dev/shm`은 64 MB다. `df -h /dev/shm`으로 확인하고, 작으면 컨테이너 shm 크기를 늘린다.
- 개인 Dockerfile의 `killros` alias(`pkill -f ign && pkill -f foxglove && pkill -f ros2 && pkill Xvfb`)는 `&&`로 이어져 있어, 중간에 대상이 없는 명령이 있으면 뒤의 kill이 실행되지 않는다. `;`로 잇거나 runbook §1 스크립트를 쓴다.

## 8. CPU 스레드 과다 방지

컨테이너가 CPU 쿼터(`cpu.max`)로 제한된 경우 `nproc`·`os.cpu_count()`가 호스트 전체 코어 수를 보고할 수 있다. 그러면 torch·OpenBLAS·OpenCV가 프로세스마다 그 수만큼 스레드를 만들어 과다 구독이 일어난다. ROS 노드를 띄우는 셸에서 다음을 설정하고, YOLO처럼 스레드가 필요한 노드만 따로 조정한다.

```bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
```

## 9. SSH 키 설정 (최초 1회)

```bash
# SSH 키 생성 (Network Volume에 저장)
mkdir -p /workspace/.ssh
ssh-keygen -t ed25519 -C "opticore-runpod" -f /workspace/.ssh/id_ed25519

# 권한 설정
chmod 700 /workspace/.ssh
chmod 600 /workspace/.ssh/id_ed25519

# 공개키 출력 -> GitHub에 등록
cat /workspace/.ssh/id_ed25519.pub
```

GitHub Settings > SSH Keys > New SSH Key에 등록한다.

## 10. 주의사항

- Pod 작업 후에는 반드시 **Terminate**한다 (Stop 아님).
- `/workspace` 밖의 파일은 Pod 종료 시 삭제된다.
- `colcon build`는 `/workspace/ros2_ws`(§4의 심볼릭 링크)에서 실행한다.
- 서버 검증 하네스(`/workspace/validation/...`: clean_restart, gate 스크립트, bag)는 저장소에 없다. 새 볼륨에서는 재현 절차가 사라지므로 필요한 스크립트는 저장소로 옮긴다.
