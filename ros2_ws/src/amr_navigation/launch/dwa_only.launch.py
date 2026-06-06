"""DWA 단독 실행 launch — Day 4 통합 검증용.

용도:
    전체 warehouse.launch.py를 띄우지 않고 DWA 노드만 띄워서:
    1. 빈 평가함수 상태에서도 cmd_vel=(0,0)이 안전하게 발행되는지
    2. /odometry/filtered, /global_path, /lidar 구독이 정상 등록되는지
    3. /dwa/status가 1Hz로 발행되는지

    검증:
        ros2 launch amr_navigation dwa_only.launch.py
        ros2 topic echo /cmd_vel
        ros2 topic echo /dwa/status
        ros2 node info /dwa_planner

    실제 시뮬과 함께 띄울 때는 amr_bringup의 warehouse.launch.py에서
    이 노드를 IncludeLaunchDescription으로 import.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("amr_navigation")
    default_params = PathJoinSubstitution(
        [pkg_share, "config", "dwa_params.yaml"]
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="DWA 파라미터 YAML 경로",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="시뮬레이션 시간 사용 여부",
    )

    dwa_node = Node(
        package="amr_navigation",
        executable="dwa_node.py",
        name="dwa_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        dwa_node,
    ])
