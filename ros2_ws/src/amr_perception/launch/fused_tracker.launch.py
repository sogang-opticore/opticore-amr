#!/usr/bin/env python3
"""fused_tracker 단독 launch (P-6). warehouse + slam 띄운 뒤 별도 실행."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('amr_perception'),
        'config', 'fused_tracker_params.yaml')
    return LaunchDescription([
        Node(
            package='amr_perception',
            executable='fused_tracker',
            name='fused_tracker',
            output='screen',
            emulate_tty=True,
            parameters=[config],
        ),
    ])