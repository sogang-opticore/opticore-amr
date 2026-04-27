#!/usr/bin/env python3
"""dynamic_obstacle_mover.py — set_pose_vector로 단일 호출"""

import rclpy
from rclpy.node import Node
import subprocess
import math


class DynamicObstacleMover(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_mover')

        self.obstacles = {
            'person_1': {
                'waypoints': [(5.0, 16.0), (35.0, 16.0)],
                'speed': 0.8,
                'z': 0.85,
                'x': 12.0, 'y': 16.0,
                'current_wp': 0,
            },
            'forklift_1': {
                'waypoints': [(5.0, 9.0), (5.0, 24.0), (35.0, 24.0), (35.0, 9.0)],
                'speed': 0.5,
                'z': 0.5,
                'x': 28.0, 'y': 21.0,
                'current_wp': 0,
            },
        }

        self.dt = 2.0  # 0.5Hz — 한 번에 둘 다 처리
        self.timer = self.create_timer(self.dt, self.update_all)
        self.get_logger().info('Dynamic obstacle mover started (set_pose_vector, 0.5Hz)')

    def update_all(self):
        poses = []
        for name, obs in self.obstacles.items():
            wp = obs['waypoints'][obs['current_wp']]
            dx = wp[0] - obs['x']
            dy = wp[1] - obs['y']
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.3:
                obs['current_wp'] = (obs['current_wp'] + 1) % len(obs['waypoints'])
                self.get_logger().info(f'{name}: next waypoint -> {obs["waypoints"][obs["current_wp"]]}')
            else:
                step = obs['speed'] * self.dt
                obs['x'] += (dx / dist) * step
                obs['y'] += (dy / dist) * step

            yaw = math.atan2(dy, dx)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            poses.append(
                f'{{name: "{name}", '
                f'position: {{x: {obs["x"]:.3f}, y: {obs["y"]:.3f}, z: {obs["z"]:.3f}}}, '
                f'orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}'
            )
            self.get_logger().info(f'{name} -> ({obs["x"]:.2f}, {obs["y"]:.2f})')

        req = 'pose: [' + ', '.join(poses) + ']'
        try:
            subprocess.run(
                [
                    'ign', 'service',
                    '-s', '/world/warehouse/set_pose_vector',
                    '--reqtype', 'ignition.msgs.Pose_V',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '1800',
                    '--req', req,
                ],
                capture_output=True,
                timeout=1.9,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn('set_pose_vector timeout', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(f'set_pose_vector failed: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
