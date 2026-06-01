#!/usr/bin/env python3
"""Publish a sequence of /goal_pose waypoints from one terminal command."""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw_deg: float = 0.0


def _parse_goal_token(token: str) -> Waypoint:
    parts = [part.strip() for part in token.split(",") if part.strip()]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(
            f"goal '{token}' must be x,y or x,y,yaw_deg"
        )

    try:
        x = float(parts[0])
        y = float(parts[1])
        yaw_deg = float(parts[2]) if len(parts) == 3 else 0.0
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"goal '{token}' contains a non-numeric value"
        ) from exc

    return Waypoint(x=x, y=y, yaw_deg=yaw_deg)


def parse_goals(
    goals_text: Optional[str], repeated_goals: Iterable[Sequence[str]]
) -> List[Waypoint]:
    waypoints: List[Waypoint] = []

    if goals_text:
        for token in goals_text.split(";"):
            token = token.strip()
            if token:
                waypoints.append(_parse_goal_token(token))

    for goal_args in repeated_goals:
        if len(goal_args) not in (2, 3):
            raise argparse.ArgumentTypeError("--goal requires x y [yaw_deg]")
        try:
            x = float(goal_args[0])
            y = float(goal_args[1])
            yaw_deg = float(goal_args[2]) if len(goal_args) == 3 else 0.0
        except ValueError as exc:
            raise argparse.ArgumentTypeError("--goal values must be numeric") from exc
        waypoints.append(Waypoint(x=x, y=y, yaw_deg=yaw_deg))

    if not waypoints:
        raise argparse.ArgumentTypeError(
            "at least one waypoint is required via --goals or --goal"
        )

    return waypoints


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Publish /goal_pose waypoints one by one. The next waypoint is sent "
            "after DWA reports GOAL_REACHED or stable REACHED."
        )
    )
    parser.add_argument(
        "--goals",
        help='semicolon-separated waypoints, e.g. "20,10;10,5;35,-10,90"',
    )
    parser.add_argument(
        "--goal",
        action="append",
        nargs="+",
        default=[],
        metavar=("X", "Y"),
        help="append one waypoint as x y [yaw_deg]; can be used multiple times",
    )
    parser.add_argument("--frame", default="map", help="frame_id for PoseStamped goals")
    parser.add_argument("--goal-topic", default="/goal_pose", help="goal topic")
    parser.add_argument("--status-topic", default="/dwa/status", help="DWA status topic")
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=240.0,
        help="maximum wall-clock seconds allowed for each waypoint",
    )
    parser.add_argument(
        "--reached-ignore-sec",
        type=float,
        default=0.8,
        help="ignore REACHED/GOAL_REACHED statuses briefly after publishing a new goal",
    )
    parser.add_argument(
        "--accept-reached-without-motion-sec",
        type=float,
        default=3.0,
        help=(
            "accept REACHED even if no active motion status was observed after this "
            "many seconds; useful when a waypoint is already very close"
        ),
    )
    parser.add_argument(
        "--publish-repeats",
        type=int,
        default=3,
        help="number of short repeated publishes for each new waypoint",
    )
    parser.add_argument(
        "--publish-period",
        type=float,
        default=0.2,
        help="seconds between repeated waypoint publishes",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=0.25,
        help="small delay before sending the next waypoint after reach detection",
    )
    return parser


class WaypointSequenceNode(Node):
    REACHED_STATUSES = {"GOAL_REACHED", "REACHED"}
    ACTIVE_STATUSES = {
        "NORMAL",
        "ALIGN",
        "REJOIN",
        "AVOIDING_DYNAMIC",
        "DYNAMIC_BLOCKED",
        "INSIDE_DYNAMIC_ZONE",
        "APPROACHING_DYNAMIC",
        "CROSSING_DYNAMIC",
        "RECEDING_DYNAMIC",
        "STOPPED_DYNAMIC",
        "RECOVERY",
        "RECOVERY_DONE",
        "STOPPED_NEAR_WALL",
        "EMERGENCY",
        "PATH_LOST",
        "FORWARD_ONLY",
        "SPIN",
    }

    def __init__(self, args: argparse.Namespace, waypoints: List[Waypoint]) -> None:
        super().__init__("waypoint_sequence")
        self.declare_parameter("use_sim_time", True)

        self._waypoints = waypoints
        self._frame = args.frame
        self._timeout_sec = max(0.0, args.timeout_sec)
        self._reached_ignore_sec = max(0.0, args.reached_ignore_sec)
        self._accept_reached_without_motion_sec = max(
            self._reached_ignore_sec, args.accept_reached_without_motion_sec
        )
        self._publish_repeats = max(1, args.publish_repeats)
        self._publish_period = max(0.05, args.publish_period)
        self._settle_sec = max(0.0, args.settle_sec)

        self._goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.create_subscription(String, args.status_topic, self._on_status, 10)
        self._timer = self.create_timer(0.05, self._tick)

        self._index = 0
        self._goal_active = False
        self._goal_started_at = 0.0
        self._next_goal_at = time.monotonic()
        self._last_publish_at = 0.0
        self._remaining_publishes = 0
        self._motion_seen = False
        self._last_status = ""
        self.done = False
        self.failed = False

        summary = " -> ".join(
            f"({wp.x:.2f}, {wp.y:.2f}, yaw={wp.yaw_deg:.1f})" for wp in waypoints
        )
        self.get_logger().info(f"loaded {len(waypoints)} waypoint(s): {summary}")

    def _tick(self) -> None:
        if self.done or self.failed:
            return

        now = time.monotonic()

        if not self._goal_active:
            if self._index >= len(self._waypoints):
                self.done = True
                self.get_logger().info("all waypoints reached")
                self._request_shutdown()
                return
            if now >= self._next_goal_at:
                self._start_current_goal(now)
            return

        if self._remaining_publishes > 0 and (
            self._last_publish_at <= 0.0
            or now - self._last_publish_at >= self._publish_period
        ):
            self._publish_current_goal()
            self._remaining_publishes -= 1
            self._last_publish_at = now

        if self._timeout_sec > 0.0 and now - self._goal_started_at > self._timeout_sec:
            wp = self._waypoints[self._index]
            self.failed = True
            self.get_logger().error(
                "waypoint %d/%d timed out after %.1fs: x=%.3f y=%.3f yaw=%.1f "
                "(last_status=%s)"
                % (
                    self._index + 1,
                    len(self._waypoints),
                    self._timeout_sec,
                    wp.x,
                    wp.y,
                    wp.yaw_deg,
                    self._last_status or "none",
                )
            )
            self._request_shutdown()

    def _start_current_goal(self, now: float) -> None:
        wp = self._waypoints[self._index]
        self._goal_active = True
        self._goal_started_at = now
        self._last_publish_at = 0.0
        self._remaining_publishes = self._publish_repeats
        self._motion_seen = False
        self._last_status = ""
        self.get_logger().info(
            "starting waypoint %d/%d: x=%.3f y=%.3f yaw=%.1f"
            % (self._index + 1, len(self._waypoints), wp.x, wp.y, wp.yaw_deg)
        )

    def _publish_current_goal(self) -> None:
        wp = self._waypoints[self._index]
        msg = PoseStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = wp.x
        msg.pose.position.y = wp.y
        msg.pose.position.z = 0.0
        yaw = math.radians(wp.yaw_deg)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self._goal_pub.publish(msg)

    def _on_status(self, msg: String) -> None:
        status = msg.data.strip()
        if not status:
            return
        self._last_status = status

        if not self._goal_active:
            return

        elapsed = time.monotonic() - self._goal_started_at
        if status in self.ACTIVE_STATUSES:
            self._motion_seen = True

        if status not in self.REACHED_STATUSES:
            return

        if elapsed < self._reached_ignore_sec:
            self.get_logger().debug(
                "ignoring early %s %.2fs after waypoint publish" % (status, elapsed)
            )
            return

        if (
            status == "GOAL_REACHED"
            or self._motion_seen
            or elapsed >= self._accept_reached_without_motion_sec
        ):
            self._finish_current_goal(status, elapsed)
            return

        self.get_logger().debug(
            "waiting for motion/new edge before accepting %s at %.2fs" % (status, elapsed)
        )

    def _finish_current_goal(self, status: str, elapsed: float) -> None:
        wp = self._waypoints[self._index]
        self.get_logger().info(
            "reached waypoint %d/%d via %s after %.1fs: x=%.3f y=%.3f"
            % (self._index + 1, len(self._waypoints), status, elapsed, wp.x, wp.y)
        )
        self._index += 1
        self._goal_active = False
        self._next_goal_at = time.monotonic() + self._settle_sec
        self._remaining_publishes = 0
        self._motion_seen = False

    def _request_shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Optional[List[str]] = None) -> None:
    argv = sys.argv if argv is None else argv
    parser = build_arg_parser()
    parsed_args = parser.parse_args(remove_ros_args(args=argv)[1:])

    try:
        waypoints = parse_goals(parsed_args.goals, parsed_args.goal)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    rclpy.init(args=argv)
    node = WaypointSequenceNode(parsed_args, waypoints)
    exit_code = 0
    try:
        rclpy.spin(node)
        if node.failed:
            exit_code = 2
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.failed:
            exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
