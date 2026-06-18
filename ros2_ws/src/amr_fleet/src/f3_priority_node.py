#!/usr/bin/env python3
"""
F-3 proactive priority — ROS2 노드 wrapper (additive on F-2).

순수 코어(fleet_priority.compute_priorities)에 런타임 입력을 먹여 /fleet/priorities 를
*일찍* 주입한다. F-2(deadlock_manager)의 HOLD/RETREAT/RESUME enforcement 는 건드리지 않고,
2m 반응존 전에 우선순위를 미리 정할 뿐이다(layering 안전).

입력:
  - TF  map → amrN/base_footprint   (pose; F-2 와 동일 소스, AMCL+EKF)
  - /amrN/odometry/filtered (Odometry)   → speed(보조; 멈춤 시 cruise 투영)
  - /amrN/global_path (nav_msgs/Path)     → 예측용 경로
  - /perception/tracked_objects (amr_msgs/TrackedObjectArray) → 장애물(단일 전역 토픽)
출력:
  - /fleet/priorities (std_msgs/Int32MultiArray, latched) — F-2 인터페이스, **변할 때만** 발행
  - /fleet/conflict_predictions (visualization_msgs/MarkerArray) — 디버그 시각화

P-10(CCTV) 미구현 → tracked_objects 하나만 구독. 나중에 같은 토픽을 enrich → F-3 무변경.
fused_tracker 에 ego-filter 가 없어 AMR 이 트랙으로 잡힐 수 있음 → self_filter_radius_m 로 드롭.
"""
import math
import os
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from amr_msgs.msg import TrackedObjectArray

# 코어는 같은 lib/amr_fleet/ 에 설치되어 스크립트 dir(sys.path[0])로 import 됨.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fleet_priority as fp  # noqa: E402


def _yaw_from_quat(z, w):
    """Planar(base_footprint) 가정: yaw 를 (z, w) 로만 복원."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class F3PriorityNode(Node):
    """사전 우선순위 주입 노드. 타이머마다 코어를 호출해 /fleet/priorities 갱신."""

    def __init__(self):
        super().__init__('f3_priority_node')

        self.declare_parameter('robots',
                               ['amr1:3.0:13.0', 'amr2:3.0:17.0',
                                'amr3:3.0:21.0', 'amr4:3.0:25.0'])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame_suffix', 'base_footprint')
        self.declare_parameter('tracked_objects_topic', '/perception/tracked_objects')
        self.declare_parameter('self_filter_radius_m', 0.8)
        # 코어 정책(§8.4)
        self.declare_parameter('horizon_sec', 6.0)
        self.declare_parameter('horizon_dt', 0.5)
        self.declare_parameter('robot_conflict_radius_m', 1.5)
        self.declare_parameter('obstacle_conflict_radius_m', 2.0)
        self.declare_parameter('cruise_speed_mps', 0.8)
        self.declare_parameter('arrival_tie_epsilon_sec', 1.0)
        self.declare_parameter('handoff_distance_m', 2.0)
        self.declare_parameter('priority_dominant', True)
        self.declare_parameter('stopped_speed_eps', 0.05)
        self.declare_parameter('publish_rate_hz', 5.0)

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.base_suffix = gp('base_frame_suffix').value
        self.self_filter_r2 = float(gp('self_filter_radius_m').value) ** 2
        self.core_params = {
            'horizon_sec': gp('horizon_sec').value,
            'horizon_dt': gp('horizon_dt').value,
            'robot_conflict_radius_m': gp('robot_conflict_radius_m').value,
            'obstacle_conflict_radius_m': gp('obstacle_conflict_radius_m').value,
            'cruise_speed_mps': gp('cruise_speed_mps').value,
            'arrival_tie_epsilon_sec': gp('arrival_tie_epsilon_sec').value,
            'handoff_distance_m': gp('handoff_distance_m').value,
            'priority_dominant': gp('priority_dominant').value,
            'stopped_speed_eps': gp('stopped_speed_eps').value,
        }

        # 로봇 목록 (id = 등장 순서, 1-based) — F-2 default_prio 규약과 동일 인덱싱
        self.order = []
        self.id_of = {}
        for i, spec in enumerate(gp('robots').value):
            name = spec.split(':')[0]
            self.order.append(name)
            self.id_of[name] = i + 1
        self.n = len(self.order)

        self.paths = {name: [] for name in self.order}      # name -> [(x,y)]
        self.speeds = {name: None for name in self.order}    # name -> float|None
        self.tracks = []                                     # latest TrackedObject[]
        self.last_array = None                               # on-change 발행용

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        path_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST, depth=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for name in self.order:
            self.create_subscription(
                Path, f'/{name}/global_path',
                lambda m, nm=name: self._on_path(nm, m), path_qos)
            self.create_subscription(
                Odometry, f'/{name}/odometry/filtered',
                lambda m, nm=name: self._on_odom(nm, m), 10)
        self.create_subscription(
            TrackedObjectArray, gp('tracked_objects_topic').value,
            self._on_tracks, 10)

        self.prio_pub = self.create_publisher(
            Int32MultiArray, '/fleet/priorities', latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/fleet/conflict_predictions', 10)

        rate = float(gp('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'f3_priority_node up — robots={self.order} '
            f'(horizon={self.core_params["horizon_sec"]}s '
            f'rr={self.core_params["robot_conflict_radius_m"]} '
            f'handoff={self.core_params["handoff_distance_m"]} '
            f'pdom={self.core_params["priority_dominant"]}) '
            f'[F-3 proactive priority — /fleet/priorities 주입]')

    # ---- 콜백 ----
    def _on_path(self, name, msg):
        self.paths[name] = [(p.pose.position.x, p.pose.position.y)
                            for p in msg.poses]

    def _on_odom(self, name, msg):
        v = msg.twist.twist.linear
        self.speeds[name] = math.hypot(v.x, v.y)

    def _on_tracks(self, msg):
        self.tracks = list(msg.tracks)

    # ---- 입력 수집 ----
    def _lookup_pose(self, name):
        """TF map→name/base_footprint → (x,y,yaw) 또는 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, f'{name}/{self.base_suffix}',
                Time())
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return (t.x, t.y, _yaw_from_quat(r.z, r.w))

    def _build_obstacles(self, robot_poses):
        """tracked_objects → obstacle dict. 로봇 근방(self_filter) 트랙은 제외."""
        obstacles = []
        for tr in self.tracks:
            ox, oy = tr.position.x, tr.position.y
            near_robot = False
            for pose in robot_poses.values():
                dx, dy = ox - pose[0], oy - pose[1]
                if dx * dx + dy * dy < self.self_filter_r2:
                    near_robot = True
                    break
            if near_robot:
                continue
            obstacles.append({
                'x': ox, 'y': oy,
                'vx': tr.velocity.x, 'vy': tr.velocity.y,
                'class_name': tr.class_name,
            })
        return obstacles

    # ---- 메인 틱 ----
    def _tick(self):
        robot_poses = {}
        for name in self.order:
            pose = self._lookup_pose(name)
            if pose is not None:
                robot_poses[name] = pose

        # 전 로봇을 코어에 전달(미국지화는 pose=None) → baseline 은 항상 full N=[N..1],
        # 예측은 국지화된 로봇끼리만. (부분 국지화 시 absent 로봇에 0 발행하는 버그 방지.)
        robots = []
        for name in self.order:
            robots.append({
                'id': self.id_of[name],
                'pose': robot_poses.get(name),   # None if TF 아직 없음
                'path': self.paths.get(name, []),
                'speed': self.speeds.get(name),
                'priority': None,            # 외부 우선순위 소스 미사용 → 기본 baseline
            })

        obstacles = self._build_obstacles(robot_poses)
        result = fp.compute_priorities(robots, obstacles, self.core_params)

        # /fleet/priorities — 항상 전체 배열, 변할 때만 발행. 국지화 0대면 발행 안 함.
        array = fp.to_array(result.priorities, self.n) if robot_poses else None
        if array is not None and array != self.last_array:
            self.last_array = array
            msg = Int32MultiArray()
            msg.data = [int(v) for v in array]
            self.prio_pub.publish(msg)
            dec = [(d.winner_id, d.yielding_id, d.reason)
                   for d in result.decisions]
            self.get_logger().info(
                f'/fleet/priorities 갱신: {array} (decisions={dec})')

        self._publish_markers(robot_poses, result)

    # ---- 디버그 마커 ----
    def _publish_markers(self, robot_poses, result):
        ma = MarkerArray()
        mid = 0
        now = self.get_clock().now().to_msg()
        for c in result.conflicts:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = 'f3_conflicts'
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(c.point[0])
            m.pose.position.y = float(c.point[1])
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.a = 0.8
            if c.kind == 'robot-obstacle':
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            m.lifetime = Duration(seconds=1.0).to_msg()
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = F3PriorityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
