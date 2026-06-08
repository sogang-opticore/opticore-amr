"""
Opticore AMR — 로봇 1대분 launch (멀티로봇용)
robot_name, spawn_x, spawn_y 를 인자로 받아 독립 인스턴스 구성.

사용 예:
  ros2 launch amr_bringup robot.launch.py robot_name:=amr1 spawn_x:=3.0 spawn_y:=13.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_dir     = get_package_share_directory('amr_bringup')
    nav_share   = FindPackageShare('amr_navigation')
    slam_pkg    = get_package_share_directory('amr_slam')
    bringup_pkg = get_package_share_directory('amr_bringup')

    urdf_file        = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    ekf_config       = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    map_yaml         = os.path.join(bringup_pkg, 'maps', 'warehouse_map.yaml')
    amcl_config      = os.path.join(slam_pkg, 'config', 'amcl_params.yaml')
    astar_params     = PathJoinSubstitution([nav_share, 'config', 'astar_params.yaml'])
    dwa_params       = PathJoinSubstitution([nav_share, 'config', 'dwa_params.yaml'])

    # ── 인자 ──
    robot_name   = LaunchConfiguration('robot_name')
    spawn_x      = LaunchConfiguration('spawn_x')
    spawn_y      = LaunchConfiguration('spawn_y')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. Robot State Publisher ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': [robot_name, '/'],
        }],
    )

    # ── 2. Spawn ──
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', [robot_name, '/robot_description'],
                    '-name',  robot_name,
                    '-x', spawn_x, '-y', spawn_y, '-z', '0.05',
                ],
            ),
        ],
    )

    # ── 3. ros_gz_bridge ──
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=robot_name,
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    [robot_name, '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
                    [robot_name, '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                    [robot_name, '/tf_gazebo@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
                    [robot_name, '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'],
                    [robot_name, '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                    [robot_name, '/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
                    [robot_name, '/camera@sensor_msgs/msg/Image[ignition.msgs.Image'],
                    [robot_name, '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
                    [robot_name, '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                ],
            ),
        ],
    )

    # ── 4. TF fix ──
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                   [robot_name, '/lidar_link'],
                   [robot_name, '/base_footprint/lidar_sensor']],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        name='camera_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                   [robot_name, '/camera_link'],
                   [robot_name, '/base_footprint/rgb_camera']],
    )

    # ── 5. IMU covariance injector ──
    imu_injector = Node(
        package='amr_slam',
        executable='imu_covariance_injector.py',
        namespace=robot_name,
        name='imu_covariance_injector',
        output='screen',
        remappings=[
            ('/imu_raw', [robot_name, '/imu_raw']),
            ('/imu',     [robot_name, '/imu']),
        ],
    )

    # ── 6. Odom covariance injector ──
    odom_cov_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='amr_slam',
                executable='odom_covariance_injector.py',
                namespace=robot_name,
                name='odom_covariance_injector',
                output='screen',
                remappings=[
                    ('/odom',          [robot_name, '/odom']),
                    ('/odom_with_cov', [robot_name, '/odom_with_cov']),
                ],
            ),
        ],
    )

    # ── 7. EKF ──
    ekf_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                namespace=robot_name,
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config,
                    {
                        'use_sim_time':    use_sim_time,
                        'odom_frame':      [robot_name, '/odom_filtered'],
                        'base_link_frame': [robot_name, '/base_footprint'],
                        'world_frame':     [robot_name, '/odom_filtered'],
                        'odom0':           [robot_name, '/odom_with_cov'],
                        'imu0':            [robot_name, '/imu'],
                    },
                ],
            ),
        ],
    )

    # ── 8. Map Server ──
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        namespace=robot_name,
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    # ── 9. AMCL ──
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace=robot_name,
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time':   use_sim_time,
                'odom_frame_id':  [robot_name, '/odom_filtered'],
                'base_frame_id':  [robot_name, '/base_footprint'],
                'scan_topic':     [robot_name, '/lidar'],
            },
        ],
    )

    # ── 10. Lifecycle Manager ──
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=robot_name,
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ── 11. A* ──
    astar_node = Node(
        package='amr_navigation',
        executable='astar_planner',
        namespace=robot_name,
        name='astar_planner',
        output='screen',
        emulate_tty=True,
        parameters=[
            astar_params,
            {
                'use_sim_time':  use_sim_time,
                'base_frame':    [robot_name, '/base_footprint'],
                'dwa_status_topic':       [robot_name, '/dwa/status'],
                'dynamic_layer_topic':    [robot_name, '/dynamic_obstacle_layer'],
            },
        ],
        remappings=[
            ('/goal_pose',    [robot_name, '/goal_pose']),
            ('/global_path',  [robot_name, '/global_path']),
        ],
    )

    # ── 12. DWA ──
    dwa_node = Node(
        package='amr_navigation',
        executable='dwa_node.py',
        namespace=robot_name,
        name='dwa_planner',
        output='screen',
        emulate_tty=True,
        parameters=[
            dwa_params,
            {
                'use_sim_time':        use_sim_time,
                'odom_topic':          [robot_name, '/odometry/filtered'],
                'odom_fallback_topic': [robot_name, '/odom'],
                'global_path_topic':   [robot_name, '/global_path'],
                'goal_pose_topic':     [robot_name, '/goal_pose'],
                'scan_topic':          [robot_name, '/lidar'],
                'cmd_vel_topic':       [robot_name, '/cmd_vel'],
                'dynamic_layer_topic': [robot_name, '/dynamic_obstacle_layer'],
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_name',   default_value='amr1'),
        DeclareLaunchArgument('spawn_x',      default_value='3.0'),
        DeclareLaunchArgument('spawn_y',      default_value='15.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        spawn_robot,
        bridge,
        lidar_tf,
        camera_tf,
        imu_injector,
        odom_cov_node,
        ekf_node,
        map_server,
        amcl,
        lifecycle_manager,
        astar_node,
        dwa_node,
    ])
