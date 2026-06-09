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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def _robot(name, dx, dy):
    # F-2: 시나리오 spawn override(env var) — 미설정 시 F-1 기본좌표 = 무회귀.
    # 같은 ROBOTS 가 spawn create 와 loc_init initialpose 둘 다 먹이므로 단일 소스 유지.
    return {'name': name,
            'x': os.environ.get(f'{name.upper()}_SPAWN_X', dx),
            'y': os.environ.get(f'{name.upper()}_SPAWN_Y', dy)}


ROBOTS = [
    _robot('amr1', '3.0', '13.0'),
    _robot('amr2', '3.0', '17.0'),
    _robot('amr3', '3.0', '21.0'),
    _robot('amr4', '3.0', '25.0'),
]


def make_robot_nodes(robot_name, spawn_x, spawn_y):
    """로봇 1대분 노드 리스트를 직접 생성."""
    pkg_dir     = get_package_share_directory('amr_bringup')
    slam_pkg    = get_package_share_directory('amr_slam')
    nav_pkg     = get_package_share_directory('amr_navigation')

    urdf_file        = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf.xacro')
    # F-1: per-robot prefix → ign 센서/odom 토픽이 로봇별로 갈림(데이터 크로스토크 차단).
    robot_description = xacro.process_file(
        urdf_file, mappings={'prefix': f'{robot_name}/'}).toxml()
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
            # A* 노드가 /map /goal_pose /global_path 를 절대경로로 하드코딩 →
            # namespace 로 안 갈림. 절대→per-robot 명시 remap. (map 'frame'은 공유 유지.)
            remappings=[
                ('/map',         f'/{rn}/map'),
                ('/goal_pose',   f'/{rn}/goal_pose'),
                ('/global_path', f'/{rn}/global_path'),
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
                'map_topic':           'map',
                'dynamic_layer_topic': 'dynamic_obstacle_layer',
                'local_frame':         f'{rn}/odom_filtered',
                'global_frame':        'map',
                'robot_frame':         f'{rn}/base_footprint',
                # F-1 데모: goal latch 를 0.4m 로(공유 yaml 0.20 은 단일로봇 도킹 정밀용
                # → 무회귀 위해 launch override 만). ARRIVAL_TOL(0.6)보다 작아 안정 도착.
                'goal_tolerance':      0.4,
            }],
            # DWA 출력 /dwa/status·/dwa/trajectories·/dwa/best_trajectory 가 절대경로
            # 하드코딩 → per-robot remap(4대 크로스토크/이름충돌 방지). /dwa/status 는
            # A* 가 /amrN/dwa/status 로 구독하므로 정합 필수.
            remappings=[
                ('/dwa/status',          f'/{rn}/dwa/status'),
                ('/dwa/trajectories',    f'/{rn}/dwa/trajectories'),
                ('/dwa/best_trajectory', f'/{rn}/dwa/best_trajectory'),
            ],
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
                namespace=rn,
                output='screen',
                parameters=[{'use_sim_time': True}],
                # ign 토픽이 URDF prefix 로 이미 per-robot(/amrN/...). 동일이름 ROS
                # 토픽으로 직통 브리지 → remap 불필요. (/clock 만 글로벌 공유.)
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    f'/{rn}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    f'/{rn}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    f'/{rn}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    f'/{rn}/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                    f'/{rn}/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    f'/{rn}/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                    f'/{rn}/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    f'/{rn}/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
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

    # 🔴-2: fleet localization 초기화 — map_server+amcl active 보장(self-heal) +
    # per-robot initialpose 자동 발행. 마지막 스폰(amr4 +30s) 이후 기동.
    robots_param = [f"{r['name']}:{r['x']}:{r['y']}" for r in ROBOTS]
    loc_init = TimerAction(period=40.0, actions=[
        Node(
            package='amr_slam',
            executable='fleet_localization_init.py',
            name='fleet_localization_init',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robots': robots_param,
                'map_origin_x': 3.0,
                'map_origin_y': 15.0,
            }],
        ),
    ])

    # 🔴-3: fused_tracker — 단일 공유 인스턴스(쪼개지 않음). 멀티로봇서 내부
    # tracking_frame 을 map 으로(→ _output_transform 항등, odom_filtered TF 의존 제거),
    # 입력은 amr1 네임스페이스로 결선. 출력 /perception/tracked_objects(map) 계약 유지.
    # 단일로봇 fused_tracker.launch.py(기본 tracking_frame=odom_filtered)는 불변(무회귀).
    # (4대 동시 융합은 노드의 multi-lidar 구독 확장 필요 → 본 F-1 인프라 범위 밖.)
    fused_cfg = os.path.join(
        perception_share, 'config', 'fused_tracker_params.yaml')
    fused_tracker = TimerAction(
        period=50.0,
        actions=[Node(
            package='amr_perception',
            executable='fused_tracker',
            name='fused_tracker',
            output='screen',
            parameters=[fused_cfg, {
                'use_sim_time':           True,
                'tracking_frame':         'map',
                'lidar_topic':            '/amr1/lidar',
                'map_topic':              '/amr1/map',
                'camera_info_topic':      '/amr1/camera_info',
                'robot_detections_topic': '/amr1/perception/detections',
                'lidar_frame':            'amr1/lidar_link',
                'camera_frame':           'amr1/camera_optical_link',
            }],
        )],
    )

    # F-2: fleet deadlock manager — 좁은 복도 정면 교착 우선순위 해소(저우선 HOLD/RETREAT/RESUME).
    # 전역 싱글톤(loc_init/fused_tracker 패턴). 액추에이터 = goal_pose 조작만 → DWA/A* 무수정 = F-1 무회귀.
    # enable_deadlock_manager:=false 로 비활성(베이스라인 / F-1 회귀 = 동일 그래프).
    # period 55s: amr4 스택(~+40s)·loc_init(+40s)·fused_tracker(+50s) 이후 기동.
    deadlock_manager = TimerAction(period=55.0, actions=[
        Node(
            package='amr_fleet',
            executable='deadlock_manager.py',
            name='deadlock_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_deadlock_manager')),
            parameters=[{
                'use_sim_time':      True,
                'robots':            robots_param,
                't_stuck':           4.0,
                'eps_move':          0.10,
                'r_proximity':       2.0,
                'l_gate':            0.7,
                'goal_tol':          0.4,
                'x_hold':            6.0,
                'd_retreat':         4.0,
                'retreat_timeout':   20.0,
                'resume_clear_dist': 0.8,
                'livelock_max_n':    3,
            }],
        ),
    ])

    return LaunchDescription(
        [DeclareLaunchArgument('enable_deadlock_manager', default_value='true')]
        + all_actions + [loc_init, fused_tracker, deadlock_manager])
