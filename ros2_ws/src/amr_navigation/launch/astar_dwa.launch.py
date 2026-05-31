"""
astar_dwa.launch.py — A* + DWA 통합 launch (2026-05-24 SW 추가)

용도:
    SW Week 2 통합 검증 — `astar_planner`와 `dwa_planner` 두 노드를
    한 번에 띄워 /goal_pose → /global_path → /cmd_vel 풀 체인이 흐르는지 본다.

    SLAM(또는 AMCL)이 별도로 떠 있다는 가정. /map / /odometry/filtered / /lidar
    가 이미 발행 중이어야 함.

실행:
    # 터미널 1 — 시뮬레이션
    ros2 launch amr_bringup warehouse.launch.py
    # 터미널 2 — SLAM/Localization
    ros2 launch amr_slam slam.launch.py    # 또는 localization.launch.py
    # 터미널 3 — 본 launch
    ros2 launch amr_navigation astar_dwa.launch.py
    # 터미널 4 — goal 발행
    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\
      "{header: {frame_id: 'map'}, pose: {position: {x: 20.0, y: 5.0},
        orientation: {w: 1.0}}}"
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

    astar_params_arg = DeclareLaunchArgument(
        "astar_params_file", default_value=astar_params,
        description="A* 파라미터 yaml 경로",
    )

    dwa_params_arg = DeclareLaunchArgument(
        "dwa_params_file", default_value=dwa_params,
        description="DWA 파라미터 yaml 경로",
    )

    # ── A* 노드 ────────────────────────────────────────────────────────
    astar_node = Node(
        package="amr_navigation",
        executable="astar_planner",
        name="astar_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("astar_params_file"),
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
            LaunchConfiguration("dwa_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        astar_params_arg,
        dwa_params_arg,
        astar_node,
        dwa_node,
    ])
