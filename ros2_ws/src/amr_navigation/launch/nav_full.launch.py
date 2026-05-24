"""
nav_full.launch.py — 풀 네비게이션 통합 launch (2026-05-24 SW)

저장된 맵 + AMCL + A* + DWA 를 한 번에 띄운다. SLAM mapping 모드 대신
사용. 이미 만들어진 warehouse_map 위에서 곧장 풀 시나리오를 검증하는
가장 일반적인 시나리오를 한 줄로 실행 가능하게 한다.

데이터 흐름:
    Gazebo (warehouse.launch.py 별도) ──► EKF ──► /odometry/filtered
                                          │
    저장된 맵 (warehouse_map.yaml) ──► map_server ──► /map (TRANSIENT_LOCAL)
                                                      │
    /lidar + /map ──► AMCL ──► TF: map → odom_filtered → base_footprint
                                                            │
    RViz "2D Goal Pose" / BT ──► /goal_pose                  │
                                    │                        │
                                    ▼                        │
                              astar_planner ──► /global_path │
                                                  │          │
                                                  ▼          │
                                             dwa_planner ◄───┘
                                                  │
                                                  ▼ /cmd_vel
                                             Gazebo DiffDrive

전제 (별도 터미널에서 실행):
    [터미널 1] ros2 launch amr_bringup warehouse.launch.py
    [터미널 2] ros2 launch amr_navigation nav_full.launch.py    ← 본 launch
    [터미널 3] ros2 topic pub /goal_pose ... (또는 RViz/Foxglove)

선택지:
    use_explorer:=true  → frontier_explorer 도 같이 띄움 (자율 매핑 시나리오).
                          단 mapping 모드용. 저장된 맵에선 즉시 FINISHED.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    nav_share = FindPackageShare("amr_navigation")
    slam_share = FindPackageShare("amr_slam")

    astar_params = PathJoinSubstitution([nav_share, "config", "astar_params.yaml"])
    dwa_params   = PathJoinSubstitution([nav_share, "config", "dwa_params.yaml"])

    # ── 런치 인자 ──
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Gazebo 시뮬레이션 시간 사용 여부",
    )
    use_localization_arg = DeclareLaunchArgument(
        "use_localization", default_value="true",
        description="저장된 맵 + AMCL을 사용할지 (false면 SLAM이 별도로 떠 있어야 함)",
    )
    use_explorer_arg = DeclareLaunchArgument(
        "use_explorer", default_value="false",
        description="frontier_explorer를 함께 띄울지 (mapping 시나리오 전용)",
    )

    # ── 1. Localization (map_server + AMCL) — use_localization=true 때만 ──
    #     amr_slam/launch/localization.launch.py 를 포함
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "localization.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_localization")),
    )

    # ── 2. A* 노드 ──
    astar_node = Node(
        package="amr_navigation",
        executable="astar_planner",
        name="astar_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            astar_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ── 3. DWA 노드 ──
    dwa_node = Node(
        package="amr_navigation",
        executable="dwa_node.py",
        name="dwa_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            dwa_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ── 4. (선택) Frontier Explorer — use_explorer=true ──
    explorer_node = Node(
        package="amr_navigation",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "min_cluster_size": 8,
            "min_goal_dist": 0.5,
            "visit_radius": 0.8,
            "replan_period": 3.0,
            "goal_timeout": 30.0,
            "arrival_settle_time": 2.0,
            "robot_frame": "base_footprint",
            "map_frame": "map",
        }],
        condition=IfCondition(LaunchConfiguration("use_explorer")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_localization_arg,
        use_explorer_arg,
        localization_launch,
        astar_node,
        dwa_node,
        explorer_node,
    ])
