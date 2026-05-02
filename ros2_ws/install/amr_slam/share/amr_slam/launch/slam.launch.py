from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_slam')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([
        # slam_toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': True}],
            output='screen',
        ),
    ])