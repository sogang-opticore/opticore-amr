from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([
        # LiDAR frame_id 매핑 (Gazebo Ignition 네임스페이스 보정)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'lidar_link',
                '--child-frame-id', 'opticore_amr/base_footprint/lidar_sensor'
            ],
            parameters=[{'use_sim_time': True}],
        ),

        # slam_toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': True}],
            output='screen',
        ),
    ])