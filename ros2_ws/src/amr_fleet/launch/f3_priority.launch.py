"""
F-3 proactive priority — 독립 launch (additive; multi_robot 미편집).

사용:
  source /workspace/env.sh
  # 1) 4대 bringup (F-2 매니저 포함)
  ros2 launch amr_bringup multi_robot.launch.py
  # 2) 별도 터미널에서 F-3 얹기
  ros2 launch amr_fleet f3_priority.launch.py

enable_f3_priority:=false 로 비활성(베이스라인 비교용).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# multi_robot.launch.py 와 동일한 로봇/스폰 규약 (id = 등장 순서)
ROBOTS_PARAM = ['amr1:3.0:13.0', 'amr2:3.0:17.0',
                'amr3:3.0:21.0', 'amr4:3.0:25.0']


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('amr_fleet'), 'config', 'f3_params.yaml')

    f3_node = Node(
        package='amr_fleet',
        executable='f3_priority_node.py',
        name='f3_priority_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_f3_priority')),
        parameters=[params_file, {
            'use_sim_time': True,
            'robots': ROBOTS_PARAM,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_f3_priority', default_value='true'),
        f3_node,
    ])
