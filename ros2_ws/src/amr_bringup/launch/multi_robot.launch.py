"""
Opticore AMR — 멀티로봇 launch (F-1)
존당 2대, 총 4대 동시 운용.

  [존 A] amr1 (3.0, 13.0) / amr2 (3.0, 17.0)
  [존 B] amr3 (3.0, 21.0) / amr4 (3.0, 25.0)
  ⚠ 좌표는 S-9 (2존 분할) 확정 후 조정 필요

사용법:
  터미널 1: ros2 launch amr_bringup warehouse.launch.py
  터미널 2: ros2 launch amr_bringup multi_robot.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ROBOTS = [
    {'name': 'amr1', 'x': '3.0', 'y': '13.0'},
    {'name': 'amr2', 'x': '3.0', 'y': '17.0'},
    {'name': 'amr3', 'x': '3.0', 'y': '21.0'},
    {'name': 'amr4', 'x': '3.0', 'y': '25.0'},
]


def generate_launch_description():
    bringup_share    = FindPackageShare('amr_bringup')
    perception_share = FindPackageShare('amr_perception')

    robot_launches = []
    for i, robot in enumerate(ROBOTS):
        delay = float(i) * 2.0  # 로봇별 2초 stagger (동시 spawn 충돌 방지)
        robot_launches.append(
            TimerAction(
                period=delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([bringup_share, 'launch', 'robot.launch.py'])
                        ),
                        launch_arguments={
                            'robot_name':   robot['name'],
                            'spawn_x':      robot['x'],
                            'spawn_y':      robot['y'],
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                        }.items(),
                    ),
                ],
            )
        )

    # fused_tracker — 전 로봇 공유 단일 인스턴스
    fused_tracker = TimerAction(
        period=12.0,  # 4대 전부 뜬 뒤 (2s × 4 + 여유)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([perception_share, 'launch', 'fused_tracker.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *robot_launches,
        fused_tracker,
    ])
