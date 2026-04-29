import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareDrive(Node):
    def __init__(self):
        super().__init__('square_drive')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move(self, linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        time.sleep(duration)
        # 정지
        self.publisher.publish(Twist())
        time.sleep(1)

    def run(self):
        for _ in range(4):
            print("전진 중...")
            self.move(0.5, 0.0, 10.0) # 0.5m/s * 10s = 5m
            print("회전 중...")
            self.move(0.0, 0.5, 3.14) # 90도(1.57rad) / 0.5rad/s = 3.14s

def main():
    rclpy.init()
    node = SquareDrive()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


