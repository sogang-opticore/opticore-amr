"""
astar_only.launch.py — A* 노드 단독 검증용 launch
warehouse.launch.py 없이 astar_planner만 띄움.
SLAM이 이미 실행 중인 상태에서 사용.

실행 방법:
    ros2 launch amr_navigation astar_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 런치 인자 ──────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo 시뮬레이션 시간 사용 여부',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='config/astar_params.yaml',
        description='A* 파라미터 yaml 경로',
    )

    # ── A* 노드 ────────────────────────────────────────────────────
    astar_node = Node(
        package='amr_navigation',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        astar_node,
    ])
