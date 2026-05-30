#!/usr/bin/env python3
"""
lag_diag.py — TF/sensor stamp lag 동시 측정 진단 노드 (2026-05-25, SW · 페어)

DWA spiral bug 추적 도구. 다음 토픽의 header.stamp 를 sim_time(/clock) 과
실시간 비교해서 1초마다 평균/최대 lag 를 ms 단위로 출력한다.

목적:
    - `ros2 topic echo --once` 가 메시지 한 개씩 받느라 1.3초씩 걸려 stamp 비교가
      부정확. 한 프로세스에서 callback 으로 모든 토픽을 받으면 같은 sim_time
      시점의 lag 를 정확히 잴 수 있다.
    - 어느 sensor / pipeline 노드가 진짜 lag source 인지 좁힌다.

사용:
    ros2 run amr_navigation lag_diag.py
    # 또는 직접
    python3 /workspace/opticore-amr/ros2_ws/src/amr_navigation/amr_navigation/lag_diag.py \
        --ros-args -p use_sim_time:=true

해석 가이드:
    - 모든 sensor lag ~0ms: 라이브 — bridge 정상
    - /lidar 만 1.3s: Gazebo lidar plugin 또는 bridge 의 lidar 매핑 lag
    - /odom 만 1.3s: DiffDrive plugin lag
    - /imu 만 1.3s: IMU plugin lag
    - 다 정상인데 /odometry/filtered 만 1.3s: EKF queue 처리 지연
    - 다 정상인데 /amcl_pose 만 1.3s: AMCL particle filter 처리 시간

비유: 의사가 환자의 어디가 아픈지 모를 때, 한 번에 여러 부위 압통 검사를
      하는 것. 차례차례 누르면 환자가 매번 자세를 바꿔서 비교가 안 됨.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


class LagDiag(Node):
    """모든 핵심 토픽의 header.stamp 와 /clock 의 sim_time 을 동시 비교."""

    def __init__(self) -> None:
        super().__init__('lag_diag')

        # use_sim_time 강제 활성 — sim_time 안에서 lag 측정해야 의미 있음
        if not self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().warn(
                'use_sim_time 이 false — sim_time 기반 lag 측정이 부정확할 수 있음. '
                '--ros-args -p use_sim_time:=true 로 실행 권장.'
            )

        # 현재 sim_time 캐시 (/clock 콜백에서 갱신)
        self._sim_now: float = 0.0

        # 토픽별 lag 샘플 누적
        self._samples: dict[str, list[float]] = {
            '/lidar': [],
            '/odom': [],
            '/imu': [],
            '/odometry/filtered': [],
            '/amcl_pose': [],
            '/tf (latest)': [],
        }

        # QoS — sensor 들은 BEST_EFFORT, 다른 건 RELIABLE 기본
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # /clock — sim_time 캐시용
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

        # sensor 토픽들
        self.create_subscription(
            LaserScan, '/lidar', self._make_cb('/lidar'), sensor_qos)
        self.create_subscription(
            Odometry, '/odom', self._make_cb('/odom'), sensor_qos)
        self.create_subscription(
            Imu, '/imu', self._make_cb('/imu'), sensor_qos)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._make_cb('/odometry/filtered'), 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._make_cb('/amcl_pose'), 10)

        # /tf — TFMessage 안에는 여러 transform. 가장 최근 stamp 만 추출.
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)

        # 1초마다 리포트
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            'lag_diag 시작 — 1초마다 각 토픽의 stamp 와 sim_time 차이를 ms 로 보고.'
        )

    # -- 콜백 --------------------------------------------------------------
    def _on_clock(self, msg: Clock) -> None:
        self._sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _make_cb(self, topic: str):
        def cb(msg) -> None:
            if self._sim_now == 0.0:
                return  # /clock 아직 안 옴
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._samples[topic].append((self._sim_now - stamp) * 1000.0)
        return cb

    def _on_tf(self, msg: TFMessage) -> None:
        """TFMessage 안의 가장 최근 stamp 를 lag 로 잰다."""
        if self._sim_now == 0.0 or not msg.transforms:
            return
        # 가장 큰 (=최신) stamp 찾기
        latest = 0.0
        for t in msg.transforms:
            s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            if s > latest:
                latest = s
        self._samples['/tf (latest)'].append((self._sim_now - latest) * 1000.0)

    # -- 리포트 ------------------------------------------------------------
    def _report(self) -> None:
        if self._sim_now == 0.0:
            self.get_logger().info('clock 아직 — sim_time 대기 중')
            return
        lines = [f'-- sim_time={self._sim_now:.2f}s --']
        for topic, vs in self._samples.items():
            if not vs:
                lines.append(f'  {topic:24s}: no data')
                continue
            avg = sum(vs) / len(vs)
            mn = min(vs)
            mx = max(vs)
            n = len(vs)
            # 1초 단위에서 50ms 이상이면 의심
            tag = ''
            if avg > 500:
                tag = '  ⚠ HIGH'
            elif avg > 100:
                tag = '  · elevated'
            lines.append(
                f'  {topic:24s}: n={n:3d} avg={avg:+6.0f}ms '
                f'min={mn:+6.0f}ms max={mx:+6.0f}ms{tag}'
            )
            vs.clear()
        self.get_logger().info('\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LagDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
