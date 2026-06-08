"""
Opticore AMR — Gazebo 월드 전용 launch (멀티로봇 분리 버전)
로봇 spawn / bridge / EKF / nav 는 robot.launch.py 에서 담당.
단일 로봇 테스트: ros2 launch amr_bringup robot.launch.py 별도 실행.
"""
import os
os.environ['IGN_GAZEBO_RESOURCE_PATH'] = \
    '/workspace/ros2_ws/src/amr_bringup/models'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir    = get_package_share_directory('amr_bringup')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. Ignition Gazebo (headless) ──
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v4', '-s', '-r', world_file],
        output='screen',
    )

    # ── 2. Dynamic obstacle mover (Gazebo 안정화 후) ──
    dynamic_obstacle_mover = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='amr_bringup',
                executable='dynamic_obstacle_mover.py',
                name='dynamic_obstacle_mover',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ign_gazebo,
        dynamic_obstacle_mover,
    ])
