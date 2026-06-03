#!/usr/bin/env python3
"""fused_tracker 단독 launch (P-5 스캐폴드). warehouse + perception 띄운 뒤 별도 실행."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fused_tracker_node = Node(
        package='amr_perception',
        executable='fused_tracker',
        name='fused_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_topic': '/perception/tracked_objects',
            'lidar_topic': '/lidar',
            'robot_detections_topic': '/perception/detections',
            'cctv_detections_topic': '/cctv/detections',  # [미확정] P-9 확정 시 갱신
            'camera_info_topic': '/camera/camera_info',
        }],
    )

    return LaunchDescription([fused_tracker_node])
