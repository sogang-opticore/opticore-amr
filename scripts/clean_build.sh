#!/usr/bin/env bash
# ============================================================================
#  clean_build.sh — Opticore AMR 안전 재빌드 스크립트 (2026-05-24 SW)
# ============================================================================
#
#  목적:
#    "ros2 launch 가 안 되네 → build/ install/ log/ 지우고 재빌드" 같은 반복
#    작업을 한 줄로 끝낸다. 권한·실행 비트·symlink-install 까지 한꺼번에 정리.
#
#  사용:
#    bash scripts/clean_build.sh              # 전체 패키지 clean 재빌드
#    bash scripts/clean_build.sh nav          # amr_navigation 만 (단축)
#    bash scripts/clean_build.sh full         # build/install/log 통째로 제거 후 재빌드
#    bash scripts/clean_build.sh test         # 단위 테스트까지 실행
#
#  비유:
#    매번 손빨래 하던 걸 세탁기 한 사이클로 돌리는 셈. 권한·실행 비트 같은
#    'symlink-install + .py executable' 함정도 함께 처리한다.
# ============================================================================

set -e   # 한 단계라도 실패하면 즉시 중단

# ── 색상 ──
G='\033[0;32m'  # 초록
Y='\033[1;33m'  # 노랑
R='\033[0;31m'  # 빨강
N='\033[0m'

# ── 경로 ──
# 스크립트 위치에서 ros2_ws 추정
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "${SCRIPT_DIR}/../ros2_ws" 2>/dev/null && pwd )"
if [ -z "$WS_DIR" ] || [ ! -d "$WS_DIR/src" ]; then
    # 기본 RunPod 경로
    WS_DIR="/workspace/opticore-amr/ros2_ws"
fi

if [ ! -d "$WS_DIR/src" ]; then
    echo -e "${R}[ERROR]${N} ros2_ws/src 를 찾지 못함: $WS_DIR"
    echo "        직접 경로 지정: WS_DIR=/your/path bash $0"
    exit 1
fi

cd "$WS_DIR"
echo -e "${G}[INFO]${N} ros2_ws = $WS_DIR"

MODE="${1:-default}"

# ── 0. ROS2 환경 ──
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        echo -e "${G}[OK]${N} ROS2 humble 환경 활성"
    else
        echo -e "${Y}[WARN]${N} ROS_DISTRO 미설정 & /opt/ros/humble 없음. 환경 직접 source 필요."
    fi
fi

# ── 1. 실행 비트 보강 (symlink-install + .py executable 함정) ──
echo -e "${G}[STEP]${N} src/*/*.py 실행 비트 부여"
find src/amr_navigation/amr_navigation -name "*.py" -type f -exec chmod +x {} \;
find src/amr_slam/src -name "*.py" -type f -exec chmod +x {} \; 2>/dev/null || true
find src/amr_bringup/src -name "*.py" -type f -exec chmod +x {} \; 2>/dev/null || true

# ── 2. 빌드 캐시 정리 ──
case "$MODE" in
    full)
        echo -e "${Y}[STEP]${N} full clean: build/ install/ log/ 통째로 삭제"
        rm -rf build install log
        PKGS=""   # 모든 패키지
        ;;
    nav)
        echo -e "${G}[STEP]${N} amr_navigation 캐시만 삭제"
        rm -rf build/amr_navigation install/amr_navigation log/latest_build
        PKGS="--packages-select amr_navigation"
        ;;
    test)
        echo -e "${G}[STEP]${N} test 모드 — 캐시 정리 안 함"
        PKGS="--packages-select amr_navigation"
        ;;
    *)
        echo -e "${G}[STEP]${N} default: amr_navigation 캐시 삭제 + 빌드"
        rm -rf build/amr_navigation install/amr_navigation log/latest_build
        PKGS="--packages-select amr_navigation"
        ;;
esac

# ── 3. 빌드 ──
if [ "$MODE" != "test" ]; then
    echo -e "${G}[STEP]${N} colcon build --symlink-install $PKGS"
    # shellcheck disable=SC2086
    colcon build --symlink-install $PKGS
fi

# ── 4. 환경 source ──
echo -e "${G}[STEP]${N} source install/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash

# ── 5. install 결과 확인 ──
echo -e "${G}[CHECK]${N} install/amr_navigation/lib/amr_navigation/"
ls -la install/amr_navigation/lib/amr_navigation/ 2>/dev/null || \
    echo -e "${Y}[WARN]${N} install 디렉터리 없음"
echo ""
echo -e "${G}[CHECK]${N} install/amr_navigation/share/amr_navigation/launch/"
ls -la install/amr_navigation/share/amr_navigation/launch/ 2>/dev/null || true

# ── 6. (옵션) 단위 테스트 ──
if [ "$MODE" = "test" ]; then
    echo -e "${G}[STEP]${N} colcon test --packages-select amr_navigation"
    colcon test --packages-select amr_navigation \
                --event-handlers console_direct+
    colcon test-result --verbose
fi

echo ""
echo -e "${G}[DONE]${N} clean_build 완료. 이제 ros2 launch ... 실행 가능."
