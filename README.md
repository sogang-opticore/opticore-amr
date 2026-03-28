# Opticore AMR

Intelligent Logistics AMR System — 2026-1 Capstone Design

## Overview

ROS2 Humble 기반 지능형 물류 AMR(Autonomous Mobile Robot) 시스템입니다.
물류센터 환경에서 자율 주행, 장애물 회피, 객체 인식을 수행합니다.

## Tech Stack

| Category | Technology |
|---|---|
| Robot Framework | ROS2 Humble |
| Navigation | Nav2 |
| SLAM | SLAM Toolbox |
| Simulation | Gazebo |
| Perception | YOLOv8 |
| GPU Cloud | RunPod (A40/A5000) |

## Project Structure

```
ros2_ws/src/
  amr_description/   # URDF/Xacro robot model
  amr_gazebo/         # Gazebo simulation worlds
  amr_bringup/        # Launch files
  amr_slam/           # SLAM configuration
  amr_navigation/     # Nav2 navigation
  amr_perception/     # YOLOv8 detection
  amr_msgs/           # Custom messages
```

## Development Workflow

```
Local (MacBook) --> git push --> GitHub --> RunPod (git pull) --> Build & Test
                                  |
                              PR --> Code Review --> Merge to dev
```

### Branch Strategy

- main - Production (direct push prohibited)
- dev - Integration branch (PR only)
- feat/* - Feature branches

### Commit Convention

```
feat: new feature
fix: bug fix
docs: documentation
refactor: code refactoring
test: test code
chore: build/config changes
```

## Quick Start

```bash
# Clone
git clone git@github.com:sogang-opticore/opticore-amr.git
cd opticore-amr

# Build (on RunPod)
cd ros2_ws
colcon build
source install/setup.bash
```

## Team

| Name | Role |
|---|---|
| CY | Team Lead / Fleet Management |
| JW | Simulation |
| YS | SLAM / Localization |
| SW | AI Navigation / Obstacle Avoidance |
| HU | Infrastructure / Perception |

## License

MIT License
