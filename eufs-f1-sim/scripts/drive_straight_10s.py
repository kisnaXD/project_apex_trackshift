#!/usr/bin/env python3
"""Run the bounded ten-second Ackermann smoke test on simulation time.

The dashboard and this helper intentionally share the same command contract:
AckermannDriveStamped on /eufs/cmd is the only control input.  Plant-specific
conversion, if any, belongs to the launch stack.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty

from eufs_racecar.drive_contract import (
    BRAKE_S,
    DURATION_S,
    MAX_ACCEL_MPS2,
    MAX_SPEED_MPS,
    RATE_HZ,
    SimTimedDrive,
    clamp_command,
    forward_feedback_acceleration,
    telemetry_fresh,
)


BRAKE_SETTLE_S = 0.3
BRAKE_TIMEOUT_S = 10.0


class DriveTest(Node):
    def __init__(self, topic: str):
        super().__init__('eufs_drive_straight_10s')
        self.pub = self.create_publisher(AckermannDriveStamped, topic, 10)
        self.unpause = self.create_client(Empty, '/unpause_physics')
        self.mission = self.create_client(SetCanState, '/ros_can/set_mission')
        self.sim_time_s: float | None = None
        self.odom_speed: float | None = None
        self.last_odom_wall: float | None = None
        self.odom_seq = 0
        self.mission_state: CanState | None = None
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)
        self.create_subscription(Odometry, '/eufs/odom', self._on_odom, 10)
        self.create_subscription(CanState, '/ros_can/state', self._on_mission, 10)

    def _on_clock(self, msg: Clock):
        self.sim_time_s = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_odom(self, msg: Odometry):
        self.odom_speed = msg.twist.twist.linear.x
        self.last_odom_wall = time.monotonic()
        self.odom_seq += 1

    def _on_mission(self, msg: CanState):
        self.mission_state = msg

    def odom_fresh(self):
        return telemetry_fresh(self.last_odom_wall, time.monotonic())

    def request_manual_mission(self):
        if not self.mission.wait_for_service(timeout_sec=0.5):
            if self.count_publishers('/ros_can/state'):
                return False
            return True
        request = SetCanState.Request()
        request.ami_state = CanState.AMI_MANUAL
        future = self.mission.call_async(request)
        deadline = time.monotonic() + 4.0
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None or not future.result().success:
            return False
        deadline = time.monotonic() + 4.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.mission_state is not None and (
                self.mission_state.ami_state == CanState.AMI_MANUAL
                or self.mission_state.as_state == CanState.AS_DRIVING
            ):
                return True
        return False

    def command(self, speed: float, acceleration: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = speed
        msg.drive.acceleration = acceleration
        self.pub.publish(msg)

    def brake(self):
        speed = self.odom_speed or 0.0
        self.command(0.0, -MAX_ACCEL_MPS2 if speed >= 0.0 else MAX_ACCEL_MPS2)


def _spin(node: DriveTest, seconds: float):
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/eufs/cmd')
    parser.add_argument('--speed', type=float, default=MAX_SPEED_MPS)
    parser.add_argument('--accel', type=float, default=MAX_ACCEL_MPS2)
    parser.add_argument('--duration', type=float, default=DURATION_S)
    args = parser.parse_args()
    speed, accel = clamp_command(args.speed, args.accel)

    rclpy.init()
    node = DriveTest(args.topic)
    try:
        if not node.unpause.wait_for_service(timeout_sec=3.0):
            node.get_logger().error('/unpause_physics is not available')
            return 2
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and node.pub.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                node.get_logger().error(f'no subscriber on {args.topic}')
                return 2
            rclpy.spin_once(node, timeout_sec=0.05)

        future = node.unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not node.request_manual_mission():
            node.get_logger().error('mission gate is unavailable or did not reach a drive state')
            return 3
        baseline = node.sim_time_s
        if baseline is None:
            clock_deadline = time.monotonic() + 5.0
            while rclpy.ok() and node.sim_time_s is None and time.monotonic() < clock_deadline:
                node.command(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
            baseline = node.sim_time_s
        if baseline is None:
            node.get_logger().error('simulation clock is unavailable')
            return 3
        drive = SimTimedDrive(duration=args.duration)
        drive.arm(baseline)
        odom_baseline = node.odom_seq
        node.get_logger().info(
            f'10s Ackermann test on {args.topic}: speed={speed:g} accel={accel:g}'
        )
        while rclpy.ok():
            state = drive.state(node.sim_time_s)
            if state == 'done':
                break
            if state == 'timeout':
                node.get_logger().error('wall safety timeout; simulation clock stalled or physics too slow')
                return 4
            if not node.odom_fresh() or node.odom_seq <= odom_baseline:
                if drive.started_wall and time.monotonic() - drive.started_wall >= 3.0:
                    node.get_logger().error('odometry is stale before or during the drive')
                    return 5
                node.command(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
                continue
            node.command(speed, forward_feedback_acceleration(node.odom_speed, speed))
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted; zeroing command')
    finally:
        brake_start = time.monotonic()
        settled = None
        while rclpy.ok() and time.monotonic() - brake_start < BRAKE_TIMEOUT_S:
            if node.odom_fresh() and abs(node.odom_speed or 0.0) < 0.05:
                node.command(0.0, 0.0)
                settled = settled or time.monotonic()
                if time.monotonic() - settled >= BRAKE_SETTLE_S:
                    break
            else:
                settled = None
                node.brake()
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
        node.command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
