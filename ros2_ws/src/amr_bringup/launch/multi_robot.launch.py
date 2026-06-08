"""
Opticore AMR — 멀티로봇 launch (F-1)
robot.launch.py를 IncludeLaunchDescription 대신 직접 함수로 호출.
ROS2 launch LaunchConfiguration 공유 문제 우회.
"""
import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
import xacro

ROBOTS = [
    {'name': 'amr1', 'x': '3.0', 'y': '13.0'},
    {'name': 'amr2', 'x': '3.0', 'y': '17.0'},
    {'name': 'amr3', 'x': '3.0', 'y': '21.0'},
    {'name': 'amr4', 'x': '3.0', 'y': '25.0'},
]


def make_robot_nodes(robot_name, spawn_x, spawn_y):
    """로봇 1대분 노드 리스트를 직접 생성."""
    pkg_dir     = get_package_share_directory('amr_bringup')
    slam_pkg    = get_package_share_directory('amr_slam')
    nav_pkg     = get_package_share_directory('amr_navigation')

    urdf_file        = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    map_yaml         = os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml')
    amcl_config      = os.path.join(slam_pkg, 'config', 'amcl_params.yaml')
    astar_params     = os.path.join(nav_pkg, 'config', 'astar_params.yaml')
    dwa_params       = os.path.join(nav_pkg, 'config', 'dwa_params.yaml')

    # EKF yaml 동적 생성
    ekf_base = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    with open(ekf_base, 'r') as f:
        cfg = yaml.safe_load(f)
    params = cfg.get('ekf_filter_node', {}).get('ros__parameters', {})
    params['odom_frame']      = f'{robot_name}/odom_filtered'
    params['base_link_frame'] = f'{robot_name}/base_footprint'
    params['world_frame']     = f'{robot_name}/odom_filtered'
    params['odom0']           = f'/{robot_name}/odom_with_cov'
    params['imu0']            = f'/{robot_name}/imu'
    params['use_sim_time']    = True
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False,
        prefix=f'ekf_{robot_name}_')
    yaml.dump({f'{robot_name}_ekf_filter_node': {'ros__parameters': params}}, tmp)
    tmp.flush()
    ekf_yaml = tmp.name

    rn = robot_name
    nodes = [
        # RSP
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=rn,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'frame_prefix': f'{rn}/',
            }],
        ),
        # TF fix
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=rn,
            name='lidar_tf_fix',
            arguments=['0','0','0','0','0','0',
                       f'{rn}/lidar_link', f'{rn}/base_footprint/lidar_sensor'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=rn,
            name='camera_tf_fix',
            arguments=['0','0','0','0','0','0',
                       f'{rn}/camera_link', f'{rn}/base_footprint/rgb_camera'],
        ),
        # IMU injector
        Node(
            package='amr_slam',
            executable='imu_covariance_injector.py',
            namespace=rn,
            name='imu_covariance_injector',
            output='screen',
            remappings=[
                ('/imu_raw', f'/{rn}/imu_raw'),
                ('/imu',     f'/{rn}/imu'),
            ],
        ),
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            namespace=rn,
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
        ),
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            namespace=rn,
            name='amcl',
            output='screen',
            parameters=[amcl_config, {
                'use_sim_time':   True,
                'odom_frame_id':  f'{rn}/odom_filtered',
                'base_frame_id':  f'{rn}/base_footprint',
                'scan_topic':     f'/{rn}/lidar',
            }],
        ),
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=rn,
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['map_server', 'amcl']}],
        ),
        # A*
        Node(
            package='amr_navigation',
            executable='astar_planner',
            namespace=rn,
            name='astar_planner',
            output='screen',
            emulate_tty=True,
            parameters=[astar_params, {
                'use_sim_time': True,
                'base_frame':   f'{rn}/base_footprint',
                'dwa_status_topic':    'dwa/status',
                'dynamic_layer_topic': 'dynamic_obstacle_layer',
            }],
            remappings=[
                ('goal_pose',   'goal_pose'),
                ('global_path', 'global_path'),
            ],
        ),
        # DWA
        Node(
            package='amr_navigation',
            executable='dwa_node.py',
            namespace=rn,
            name='dwa_planner',
            output='screen',
            emulate_tty=True,
            parameters=[dwa_params, {
                'use_sim_time':        True,
                'odom_topic':          'odometry/filtered',
                'odom_fallback_topic': 'odom',
                'global_path_topic':   'global_path',
                'goal_pose_topic':     'goal_pose',
                'scan_topic':          'lidar',
                'cmd_vel_topic':       'cmd_vel',
                'dynamic_layer_topic': 'dynamic_obstacle_layer',
            }],
        ),
    ]

    timed = [
        # Spawn +3s
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', f'{rn}/robot_description',
                    '-name', rn,
                    '-x', spawn_x, '-y', spawn_y, '-z', '0.05',
                ],
            ),
        ]),
        # Bridge +5s
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    '/tf_gazebo@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                    '/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                ],
                remappings=[
                    ('/cmd_vel',      f'/{rn}/cmd_vel'),
                    ('/odom',         f'/{rn}/odom'),
                    ('/tf_gazebo',    f'/{rn}/tf_gazebo'),
                    ('/joint_states', f'/{rn}/joint_states'),
                    ('/lidar',        f'/{rn}/lidar'),
                    ('/imu_raw',      f'/{rn}/imu_raw'),
                    ('/camera',       f'/{rn}/camera'),
                    ('/camera_info',  f'/{rn}/camera_info'),
                    ('/ground_truth', f'/{rn}/ground_truth'),
                ],
            ),
        ]),
        # Odom injector +6s
        TimerAction(period=6.0, actions=[
            Node(
                package='amr_slam',
                executable='odom_covariance_injector.py',
                namespace=rn,
                name='odom_covariance_injector',
                output='screen',
                parameters=[{'robot_name': rn}],
                remappings=[
                    ('/odom',          'odom'),
                    ('/odom_with_cov', 'odom_with_cov'),
                ],
            ),
        ]),
        # EKF +7s
        TimerAction(period=7.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name=f'{rn}_ekf_filter_node',
                output='screen',
                parameters=[ekf_yaml],
                remappings=[
                    ('/odometry/filtered', f'/{rn}/odometry/filtered'),
                    ('/set_pose',          f'/{rn}/set_pose'),
                ],
            ),
        ]),
    ]

    return nodes + timed


def generate_launch_description():
    perception_share = get_package_share_directory('amr_perception')

    all_actions = []
    for i, robot in enumerate(ROBOTS):
        delay = float(i) * 10.0  # 로봇별 10초 stagger
        robot_nodes = make_robot_nodes(
            robot['name'], robot['x'], robot['y'])
        for action in robot_nodes:
            if isinstance(action, TimerAction):
                # 기존 타이머에 delay 추가
                all_actions.append(
                    TimerAction(
                        period=action.period + delay,
                        actions=action.actions,
                    )
                )
            else:
                all_actions.append(
                    TimerAction(period=delay, actions=[action])
                )

    # fused_tracker — 공유 단일 인스턴스
    fused_tracker_launch = os.path.join(
        perception_share, 'launch', 'fused_tracker.launch.py')
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    fused_tracker = TimerAction(
        period=50.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fused_tracker_launch),
        )],
    )

    return LaunchDescription(all_actions + [fused_tracker])
