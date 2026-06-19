#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdomCovarianceInjector(Node):
    def __init__(self):
        super().__init__('odom_covariance_injector')
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            Odometry, '/odom', self.callback, qos)
        self.pub = self.create_publisher(
            Odometry, '/odom_with_cov', qos)

    def callback(self, msg):
        msg.pose.covariance[0]  = 0.05
        msg.pose.covariance[7]  = 0.05
        msg.pose.covariance[35] = 0.1
        msg.twist.covariance[0]  = 0.1
        msg.twist.covariance[35] = 0.2
        if self.robot_name:
            msg.header.frame_id = f'{self.robot_name}/odom'
            msg.child_frame_id  = f'{self.robot_name}/base_footprint'
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(OdomCovarianceInjector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
