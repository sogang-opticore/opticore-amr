from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')
    map_file = os.path.join(pkg, 'maps', 'warehouse_map.yaml')
    amcl_params = os.path.join(pkg, 'config', 'amcl_params.yaml')

    return LaunchDescription([
        # 저장된 맵 로드
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
            }]
        ),

        # AMCL — map→odom TF 발행 담당 (slam_toolbox 대체)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params],
            remappings=[('scan', '/lidar')],
        ),

        # lifecycle 관리 — map_server, amcl 자동 활성화
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }]
        ),
    ])