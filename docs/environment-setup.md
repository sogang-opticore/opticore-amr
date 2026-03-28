# RunPod 환경 설정 가이드

## 개요
RunPod 클라우드 GPU에서 ROS2 Humble 개발 환경을 설정하는 가이드입니다.

## RunPod 정보
| 항목 | 내용 |
|---|---|
| GPU | RTX A5000 / A40 |
| Network Volume | `opticore-amr-vol` (50GB, `/workspace` 마운트) |
| 비용 | $0.27~0.40/hr (GPU), $3.50/월 (Volume) |

## 새 Pod 시작 시 순서

```bash
# 1. bashrc 복구
cp /workspace/.bashrc_ros ~/.bashrc && source ~/.bashrc

# 2. 환경 설치 (최초 1회 또는 업데이트 시)
bash /workspace/setup.sh 2>&1 | tee /workspace/setup.log

# 3. sympy 충돌 시
python3.10 -m pip install ultralytics opencv-python transforms3d foxglove-websocket --ignore-installed sympy

# 4. ROS2 워크스페이스 빌드
cd /workspace/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

## SSH 키 설정 (최초 1회)

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

GitHub Settings > SSH Keys > New SSH Key에 등록

## 주의사항
- Pod 작업 후 반드시 **Terminate** (Stop 아님)
- `/workspace` 외의 파일은 Pod 종료 시 삭제됨
- `colcon build`는 반드시 `/workspace/ros2_ws`에서 실행
