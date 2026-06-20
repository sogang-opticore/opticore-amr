"""cctv.launch.py — CCTV no-go 오버레이 (path B): /cctv 브리지(옵션) + P-9 + P-10.

단일로봇 bringup 은 별도(검증=validation/cctv/single_robot.launch.py). 본 launch 는 perception 만.
토픽 네임스페이스(데모 amr1)는 인자로 override:
  ros2 launch amr_perception cctv.launch.py \
    map_topic:=/amr1/map layer_topic:=/amr1/dynamic_obstacle_layer use_bridge:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_perception')
    params = os.path.join(pkg, 'config', 'cctv_nogo_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_topic = LaunchConfiguration('map_topic')
    layer_topic = LaunchConfiguration('layer_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    use_bridge = LaunchConfiguration('use_bridge')

    cctv_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='cctv_bridge', output='screen',
        condition=IfCondition(use_bridge),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/cctv/corridor_1s/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cctv/corridor_1n/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cctv/corridor_2s/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cctv/corridor_2n/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cctv/corridor_1s/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cctv/corridor_1n/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cctv/corridor_2s/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cctv/corridor_2n/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
    )

    detector = Node(
        package='amr_perception', executable='cctv_detector',
        name='cctv_detector', output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'detections_topic': detections_topic,
        }],
    )

    overlay = Node(
        package='amr_perception', executable='cctv_nogo_overlay',
        name='cctv_nogo_overlay', output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'detections_topic': detections_topic,
            'map_topic': map_topic,
            'layer_topic': layer_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_bridge', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('layer_topic', default_value='/dynamic_obstacle_layer'),
        DeclareLaunchArgument('detections_topic', default_value='/cctv/detections'),
        cctv_bridge,
        detector,
        overlay,
    ])
