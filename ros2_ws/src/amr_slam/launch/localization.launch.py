"""
Opticore AMR — Localization Launch
map_server + AMCL + lifecycle_manager

사용법:
  # 터미널 1: Gazebo + 로봇 + EKF
  ros2 launch amr_bringup warehouse.launch.py

  # 터미널 2: Localization (맵 기반 위치 추정)
  ros2 launch amr_slam localization.launch.py

  # (선택) 초기 위치가 스폰 위치와 다를 경우 RViz에서 2D Pose Estimate 사용
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg = get_package_share_directory('amr_slam')
    bringup_pkg = get_package_share_directory('amr_bringup')

    amcl_config  = os.path.join(slam_pkg,    'config', 'amcl_params.yaml')
    map_yaml     = os.path.join(bringup_pkg, 'maps',   'warehouse_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── 1. Map Server ──
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    # ── 2. AMCL ──
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config, {'use_sim_time': use_sim_time}],
    )

    # ── 3. Lifecycle Manager (map_server + amcl 활성화) ──
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        map_server,
        amcl,
        lifecycle_manager,
    ])