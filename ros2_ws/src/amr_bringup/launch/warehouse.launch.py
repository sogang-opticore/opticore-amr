"""
Opticore AMR — Gazebo Ignition Fortress + ros_gz_bridge
Bridge stabilized via TimerAction (Gazebo 초기화 대기 후 실행)
"""
import os

# os.environ['IGN_GAZEBO_RESOURCE_PATH'] = \
#     '/workspace/ros2_ws/src/amr_bringup/models'
os.environ['IGN_GAZEBO_RESOURCE_PATH'] = \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models')

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
    robot_name = LaunchConfiguration('robot_name')   # ← 추가 (fleet 대비, 기본 opticore_amr)

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
                    '-name', robot_name,
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
                    # Gazebo TF(odom→base_footprint)는 의도적으로 ROS /tf에 비브릿지.
                    # EKF(odom_filtered→base_footprint)가 단독 권한 — 이중 publish 방지. 사실상 죽은 브리지.
                    '/tf_gazebo@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    # Joint states (Ign→ROS)
                    '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    # LiDAR (Ign→ROS)
                    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                    # IMU (Ign→ROS)
                    '/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    # Camera (Ign→ROS)
                    '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    # CCTV 복도 감시 (Ign→ROS)
                    '/cctv/corridor_1s/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/cctv/corridor_1s/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    '/cctv/corridor_1n/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/cctv/corridor_1n/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    '/cctv/corridor_2s/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/cctv/corridor_2s/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    '/cctv/corridor_2n/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/cctv/corridor_2n/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    # ground_truth
                    '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                ],
            ),
        ],
    )

    # ── 5. EKF (bridge 안정화 7초 대기 후) ──
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    
    
    odom_cov_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='amr_slam',
                executable='odom_covariance_injector.py',
                name='odom_covariance_injector',
                output='screen',
            ),
        ],
    )
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
    imu_injector = Node(
        package='amr_slam',
        executable='imu_covariance_injector.py',
        name='imu_covariance_injector',
        output='screen',
    )

    # ── 6. Dynamic obstacle mover (EKF 안정화 후) 
    # dynamic_obstacle_mover = TimerAction(
    #      period=8.0,
    #      actions=[
    #          Node(
    #              package='amr_bringup',
    #              executable='dynamic_obstacle_mover.py',
    #              name='dynamic_obstacle_mover',
    #              output='screen',
    #              parameters=[{'use_sim_time': use_sim_time}],
    #          ),
    #      ],
    #  )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                'lidar_link',
                [robot_name, '/base_footprint/lidar_sensor']
                ],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_link',
                   [robot_name, '/base_footprint/rgb_camera']],
    )

    

    # ─────────────────────────────────────────────────────────
    # [B] static TF 4개 — camera_tf 아래에 추가
    #     수직 하향: pitch=+1.5708 (90°), yaw=0
    #     tf2_ros 인수 순서: x y z yaw pitch roll parent child
    # ─────────────────────────────────────────────────────────
    
    cctv_corridor_1s_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cctv_corridor_1s_tf',
        ros_arguments=['--ros-args', '-r', '__node:=cctv_corridor_1s_tf'],
        arguments=['14.5', '9.5', '4.49', '0', '1.5708', '0', 'map', 'cctv_corridor_1s_link'],
    )
    
    cctv_corridor_1n_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cctv_corridor_1n_tf',
        ros_arguments=['--ros-args', '-r', '__node:=cctv_corridor_1n_tf'],
        arguments=['14.5', '14.5', '4.49', '0', '1.5708', '0', 'map', 'cctv_corridor_1n_link'],
    )
    
    cctv_corridor_2s_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cctv_corridor_2s_tf',
        ros_arguments=['--ros-args', '-r', '__node:=cctv_corridor_2s_tf'],
        arguments=['49.5', '15.5', '4.49', '0', '1.5708', '0', 'map', 'cctv_corridor_2s_link'],
    )
    
    cctv_corridor_2n_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cctv_corridor_2n_tf',
        ros_arguments=['--ros-args', '-r', '__node:=cctv_corridor_2n_tf'],
        arguments=['49.5', '20.5', '4.49', '0', '1.5708', '0', 'map', 'cctv_corridor_2n_link'],
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='opticore_amr'),
        ign_gazebo,
        robot_state_publisher,
        spawn_robot,     # +3s
        bridge,          # +5s
        odom_cov_node,
        ekf_node,        # +7s
        imu_injector,
        # dynamic_obstacle_mover,  # +8s
        lidar_tf,
        camera_tf,
        cctv_corridor_1s_tf, 
        cctv_corridor_1n_tf,  
        cctv_corridor_2s_tf,   
        cctv_corridor_2n_tf,    
    ])
