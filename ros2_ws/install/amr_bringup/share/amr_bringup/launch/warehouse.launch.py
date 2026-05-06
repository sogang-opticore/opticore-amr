"""
Opticore AMR — Gazebo Ignition Fortress + ros_gz_bridge
Bridge stabilized via TimerAction (Gazebo 초기화 대기 후 실행)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.world')

    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── 1. Ignition Gazebo (headless server-only) ──
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v4', '-s', '-r', world_file],
        output='screen',
    )

    # ── 2. Robot State Publisher ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 3. Spawn robot (Gazebo 초기화 3초 대기 후) ──
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_amr',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'opticore_amr',
                    '-x', '3.0', '-y', '15.0', '-z', '0.05',
                ],
            ),
        ],
    )

    # ── 4. ros_gz_bridge (Gazebo + 로봇 스폰 안정화 5초 대기 후) ──
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    # Clock
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    # Cmd vel (ROS→Ign)
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    # Odometry (Ign→ROS)
                    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    # TF (Ign→ROS)
                    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    # Joint states (Ign→ROS)
                    '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    # LiDAR (Ign→ROS)
                    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                    # IMU (Ign→ROS)
                    '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    # Camera (Ign→ROS)
                    '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                ],
            ),
        ],
    )

    # ── 5. EKF (bridge 안정화 7초 대기 후) ──
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    ekf_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # ── 6. Dynamic obstacle mover (EKF 안정화 후) 
    # dynamic_obstacle_mover = TimerAction(
    #     period=8.0,
    #     actions=[
    #         Node(
    #             package='amr_bringup',
    #             executable='dynamic_obstacle_mover.py',
    #             name='dynamic_obstacle_mover',
    #             output='screen',
    #             parameters=[{'use_sim_time': use_sim_time}],
    #         ),
    #     ],
    # )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                'lidar_link',
                'opticore_amr/base_footprint/lidar_sensor'
                ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ign_gazebo,
        robot_state_publisher,
        spawn_robot,     # +3s
        bridge,          # +5s
        ekf_node,        # +7s
        #dynamic_obstacle_mover,  # +8s
        lidar_tf,
    ])
