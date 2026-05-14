#!/usr/bin/env python3
"""
IMU covariance injector
Ignition Fortress IMU plugin이 orientation_covariance를 0으로 발행하는 문제를 해결.
/imu_raw → covariance 주입 → /imu
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

ORIENTATION_COV    = 0.001 ** 2   # stddev 0.001 rad
ANGULAR_VEL_COV    = 0.005 ** 2   # stddev 0.005 rad/s (URDF 노이즈와 일치)
LINEAR_ACCEL_COV   = 0.01  ** 2   # stddev 0.01  m/s²

class ImuCovarianceInjector(Node):
    def __init__(self):
        super().__init__('imu_covariance_injector')
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.sub = self.create_subscription(Imu, '/imu_raw', self.callback, 10)

    def callback(self, msg: Imu):
        msg.orientation_covariance = [
            ORIENTATION_COV, 0.0, 0.0,
            0.0, ORIENTATION_COV, 0.0,
            0.0, 0.0, ORIENTATION_COV,
        ]
        msg.angular_velocity_covariance = [
            ANGULAR_VEL_COV, 0.0, 0.0,
            0.0, ANGULAR_VEL_COV, 0.0,
            0.0, 0.0, ANGULAR_VEL_COV,
        ]
        msg.linear_acceleration_covariance = [
            LINEAR_ACCEL_COV, 0.0, 0.0,
            0.0, LINEAR_ACCEL_COV, 0.0,
            0.0, 0.0, LINEAR_ACCEL_COV,
        ]
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuCovarianceInjector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()