#!/usr/bin/env python3
"""dynamic_obstacle_mover.py
- 5Hz update (dt=0.2)
- subprocess.Popen 비동기
- waypoint 도달 시 일시정지 (랜덤)
- ease-in/out 가감속
- yaw: 진행 방향 추적 (기본) 또는 고정 (fixed_yaw)
"""
import math
import random
import subprocess

import rclpy
from rclpy.node import Node


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Obstacle:
    def __init__(self, name, waypoints, speed, z,
                 pause_min, pause_max, yaw_rate, fixed_yaw=None):
        self.name = name
        self.waypoints = waypoints
        self.speed = speed
        self.z = z
        self.pause_min = pause_min
        self.pause_max = pause_max
        self.yaw_rate = yaw_rate
        self.fixed_yaw = fixed_yaw  # None이면 진행방향 추적, 값이 있으면 고정

        self.x, self.y = waypoints[0]
        self.yaw = fixed_yaw if fixed_yaw is not None else 0.0
        self.idx_from = 0
        self.idx_to = 1 % len(waypoints)
        self.state = 'moving'
        self.t_in_segment = 0.0
        self.pause_remaining = 0.0

    def _segment_duration(self):
        x0, y0 = self.waypoints[self.idx_from]
        x1, y1 = self.waypoints[self.idx_to]
        dist = math.hypot(x1 - x0, y1 - y0)
        return dist / max(self.speed, 1e-6), dist

    def update(self, dt):
        if self.state == 'paused':
            self.pause_remaining -= dt
            if self.pause_remaining <= 0:
                self.state = 'moving'
                self.t_in_segment = 0.0
            return

        duration, dist = self._segment_duration()
        if dist < 1e-3:
            self._advance_waypoint()
            return

        self.t_in_segment += dt
        progress = self.t_in_segment / duration

        x0, y0 = self.waypoints[self.idx_from]
        x1, y1 = self.waypoints[self.idx_to]

        if progress >= 1.0:
            self.x, self.y = x1, y1
            self._advance_waypoint()
            self.state = 'paused'
            self.pause_remaining = random.uniform(self.pause_min, self.pause_max)
            return

        s = smoothstep(progress)
        self.x = x0 + (x1 - x0) * s
        self.y = y0 + (y1 - y0) * s

        # yaw 처리
        if self.fixed_yaw is not None:
            self.yaw = self.fixed_yaw  # 고정
        else:
            target_yaw = math.atan2(y1 - y0, x1 - x0)
            dyaw = wrap_angle(target_yaw - self.yaw)
            max_step = self.yaw_rate * dt
            if abs(dyaw) <= max_step:
                self.yaw = target_yaw
            else:
                self.yaw += math.copysign(max_step, dyaw)
                self.yaw = wrap_angle(self.yaw)

    def _advance_waypoint(self):
        self.idx_from = self.idx_to
        self.idx_to = (self.idx_to + 1) % len(self.waypoints)

    def to_pose_msg(self):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        return (
            f'{{name: "{self.name}", '
            f'position: {{x: {self.x:.3f}, y: {self.y:.3f}, z: {self.z:.3f}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}'
        )


class DynamicObstacleMover(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_mover')
        self.dt = 0.2  # 5Hz
        self.world_name = 'warehouse'

        self.obstacles = [
            # 기존 동적 2개
            Obstacle(
                name='person_1',
                waypoints=[(14.0, 18.0), (17.0, 18.0)],
                speed=0.6, z=0.0,
                pause_min=3.0, pause_max=5.0,
                yaw_rate=2.0,
            ),
            Obstacle(
                name='forklift_1',
                waypoints=[(10.0, 30.0), (14.0, 30.0)],
                speed=0.4, z=0.0,
                pause_min=4.0, pause_max=6.0,
                yaw_rate=1.5,
                fixed_yaw=math.pi / 2.0,  # 옵션 A: yaw 고정 (visual +x 방향)
            ),
            # 신규 동적 2개 (좁은 1m 왕복, 선반 옆)
            Obstacle(
                name='person_2',
                waypoints=[(32.0, 20.0), (33.0, 20.0)],   # Row A 남쪽 면 옆
                speed=0.4, z=0.0,
                pause_min=5.0, pause_max=8.0,
                yaw_rate=2.0,
            ),
            Obstacle(
                name='person_3',
                waypoints=[(25.0, 14.0), (26.0, 14.0)],   # Row B 남쪽 면 옆
                speed=0.4, z=0.0,
                pause_min=5.0, pause_max=8.0,
                yaw_rate=2.0,
            ),
        ]

        self.timer = self.create_timer(self.dt, self.tick)
        self.tick_count = 0
        self.get_logger().info(
            f'Dynamic obstacle mover started (dt={self.dt}s, {len(self.obstacles)} obstacles)'
        )

    def tick(self):
        for obs in self.obstacles:
            obs.update(self.dt)

        poses = ', '.join(o.to_pose_msg() for o in self.obstacles)
        req = f'pose: [{poses}]'
        try:
            subprocess.Popen(
                [
                    'ign', 'service',
                    '-s', f'/world/{self.world_name}/set_pose_vector',
                    '--reqtype', 'ignition.msgs.Pose_V',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '500',
                    '--req', req,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().warn(f'spawn failed: {e}', throttle_duration_sec=5.0)

        self.tick_count += 1
        if self.tick_count % 25 == 0:
            for o in self.obstacles:
                self.get_logger().info(
                    f'{o.name}: ({o.x:.2f}, {o.y:.2f}) yaw={math.degrees(o.yaw):.0f}° state={o.state}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()