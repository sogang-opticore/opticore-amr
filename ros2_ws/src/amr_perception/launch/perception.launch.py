import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('amr_perception'),
        'config',
        'yolo_params.yaml'
    )

    yolo_detector_node = Node(
        package='amr_perception',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        yolo_detector_node,
    ])