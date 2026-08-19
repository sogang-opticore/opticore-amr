#!/usr/bin/env python3
"""
F-2 우선순위 주입 테스트 노드 (/fleet/priorities).

미래 마스터(F-3)가 한 방에 발행할 인터페이스의 '소비측' 검증용.
std_msgs/Int32MultiArray, transient_local(latched). index=robotN-1, 값 클수록 우선.

usage:
  ros2 run amr_fleet priority_publisher.py --ros-args -p priorities:="[1,4,2,3]"
  → amr1=1 amr2=4 amr3=2 amr4=3 (amr2 가 최우선). latched 라 매니저가 늦게 떠도 수신.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, HistoryPolicy,
                       ReliabilityPolicy)
from std_msgs.msg import Int32MultiArray


class PriorityPublisher(Node):
    def __init__(self):
        super().__init__('priority_publisher')
        self.declare_parameter('priorities', [1, 2, 3, 4])
        vals = [int(v) for v in self.get_parameter('priorities').value]
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(
            Int32MultiArray, '/fleet/priorities', latched)
        msg = Int32MultiArray()
        msg.data = vals
        self.pub.publish(msg)
        self.get_logger().info(f'/fleet/priorities 발행(latched): {vals}')


def main():
    rclpy.init()
    node = PriorityPublisher()
    try:
        # latched: 한 번 발행 후 유지. 늦은 구독자 위해 스핀.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
