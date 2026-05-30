"""
explore.launch.py — 자율 매핑 launch (2026-05-24 SW 추가)

용도:
    A* + DWA + frontier_explorer 를 한 번에 띄워서 창고 자율 매핑을 수행한다.
    SLAM 은 별도로 mapping 모드로 떠 있어야 한다.

데이터 흐름:
    SLAM → /map (OccupancyGrid)
            │
            ▼
       frontier_explorer  ── /exploration/frontiers (시각화)
            │
            │  /goal_pose
            ▼
       astar_planner
            │
            │  /global_path
            ▼
       dwa_planner
            │
            │  /cmd_vel
            ▼
        로봇 이동 → SLAM 맵 확장 → 다시 frontier_explorer …

실행:
    # 터미널 1: 시뮬레이션
    ros2 launch amr_bringup warehouse.launch.py
    # 터미널 2: SLAM (mapping 모드)
    ros2 launch amr_slam slam.launch.py
    # 터미널 3: 본 launch
    ros2 launch amr_navigation explore.launch.py

종료 조건:
    /exploration/status 가 "FINISHED" 가 되면 매핑 완료.
    이후 ros2 service call /slam_toolbox/save_map 으로 맵 저장.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("amr_navigation")

    astar_params = PathJoinSubstitution(
        [pkg_share, "config", "astar_params.yaml"]
    )
    dwa_params = PathJoinSubstitution(
        [pkg_share, "config", "dwa_params.yaml"]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Gazebo 시뮬레이션 시간 사용 여부",
    )

    # ── A* 노드 ────────────────────────────────────────────────────────
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

    # ── DWA 노드 ───────────────────────────────────────────────────────
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

    # ── Frontier Explorer 노드 ─────────────────────────────────────────
    explorer_node = Node(
        package="amr_navigation",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # 자율 매핑 권장 기본값 — 필요 시 yaml 분리
                "min_cluster_size": 8,
                "min_goal_dist": 0.5,
                "visit_radius": 0.8,
                "replan_period": 3.0,
                "goal_timeout": 30.0,
                "arrival_settle_time": 2.0,
                "robot_frame": "base_footprint",
                "map_frame": "map",
            }
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        astar_node,
        dwa_node,
        explorer_node,
    ])
