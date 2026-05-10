#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovarianceInjector(Node):
    def __init__(self):
        super().__init__('odom_covariance_injector')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/odom_with_cov', 10)

    def callback(self, msg):
        msg.pose.covariance[0]  = 0.05   # x
        msg.pose.covariance[7]  = 0.05   # y
        msg.pose.covariance[35] = 0.1    # yaw
        msg.twist.covariance[0]  = 0.1   # vx
        msg.twist.covariance[35] = 0.2   # vyaw
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(OdomCovarianceInjector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
