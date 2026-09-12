#!/usr/bin/env python3
"""Drive one closed COTA lap from odometry using the pure lap controller.

The vehicle accepts :class:`AckermannDriveStamped` on ``/eufs/cmd``.  The
native race-car plugin is run in acceleration mode, therefore ``drive.speed``
is telemetry/intent while ``drive.acceleration`` is the actuator command.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import struct
import time
from collections import deque
from pathlib import Path
from typing import Any

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState

try:
    from .cota_path import CotaRoute, LapController, _wrap
except ImportError:
    from cota_path import CotaRoute, LapController, _wrap


CONTROL_PERIOD = 0.05
READY_TIMEOUT = 8.0
BRAKE_TIMEOUT = 30.0
BRAKE_SETTLE = 0.3
ODOM_WATCHDOG = 1.0
CLOCK_WATCHDOG = 2.0


def _sim_time(msg: Clock) -> float:
    return float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9


def _gid_bytes(gid: Any) -> bytes | None:
    if gid is None:
        return None
    try:
        return bytes(gid)
    except (TypeError, ValueError):
        return None


class Lap(Node):
    """ROS adapter around :class:`LapController`."""

    def __init__(self, route: CotaRoute, args: argparse.Namespace):
        super().__init__("eufs_cota_lap")
        self.route, self.args = route, args
        self.controller = LapController(route)

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/eufs/cmd", 10)
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.plan_pub = self.create_publisher(MarkerArray, "/track_markers", marker_qos)
        self.route_pub = self.create_publisher(Marker, "/eufs/lap_route", marker_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.mission = self.create_client(SetCanState, "/ros_can/set_mission")

        self.map_x: float | None = None
        self.map_y: float | None = None
        self.map_yaw: float | None = None
        self.vx: float | None = None
        self.vy: float | None = None
        self.yaw_rate: float | None = None
        self.last_odom_wall: float | None = None
        self.last_odom_seq = 0
        self.last_odom_map: tuple[float, float] | None = None
        self.reverse_since_wall: float | None = None
        self.tf_ready = False
        self.sim_time: float | None = None
        self.last_clock_wall: float | None = None
        self.last_control_sim: float | None = None
        self.clock_backwards = False
        self.mission_state: CanState | None = None
        self.external_override = False
        self.external_node = ""
        self.own_command_gids: set[bytes] = set()
        self._own_command_fingerprints: deque[tuple[int, int, bytes]] = deque(maxlen=1024)
        self._own_command_fingerprint_set: set[tuple[int, int, bytes]] = set()
        self.last_steer = 0.0
        self.brake_started = False
        self.brake_settled = False
        self.control_started = False
        self.done = False
        self.failed: str | None = None
        self.interrupted = False
        self.trace: list[dict[str, Any]] = []
        self.last_print_sim: float | None = None
        self.run_started_wall = time.monotonic()
        self._csv_stream = None
        self._csv_writer = None

        self.create_subscription(Odometry, "/eufs/odom", self._on_odom, 10)
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)
        self.create_subscription(CanState, "/ros_can/state", self._on_mission, 10)
        self.create_subscription(AckermannDriveStamped, "/eufs/cmd",
                                 self._on_external_command, 10)
        self._open_trace_csv()
        self.publish_route()

    def _on_clock(self, msg: Clock):
        now = _sim_time(msg)
        if self.sim_time is not None and now < self.sim_time - 1e-9:
            self.clock_backwards = True
            self.failed = "clock backwards"
        self.sim_time = now
        self.last_clock_wall = time.monotonic()

    def _on_mission(self, msg: CanState):
        self.mission_state = msg

    def _refresh_own_command_gids(self):
        try:
            infos = self.get_publishers_info_by_topic("/eufs/cmd")
        except Exception:
            return
        for info in infos:
            if info.node_name != self.get_name():
                continue
            gid = _gid_bytes(getattr(info, "endpoint_gid", None))
            if gid is None:
                gid = _gid_bytes(getattr(info, "publisher_gid", None))
            if gid:
                self.own_command_gids.add(gid)

    def _on_external_command(self, _msg: AckermannDriveStamped, info=None):
        if self.external_override:
            return
        self._refresh_own_command_gids()
        gid = _gid_bytes(getattr(info, "publisher_gid", None)) if info is not None else None
        if info is not None and gid is not None:
            # When Humble supplies MessageInfo, publisher GID is authoritative.
            if gid in self.own_command_gids:
                return
        elif self._command_fingerprint(_msg) in self._own_command_fingerprint_set:
            # Humble installations that invoke a one-argument callback do not
            # provide MessageInfo.  Match the exact wire-level command sent by
            # this node, retaining enough entries for late DDS delivery.
            return
        publisher_name = getattr(info, "publisher_name", "") if info is not None else ""
        self.external_override = True
        self.external_node = publisher_name or "unknown"
        self.failed = "external command override"
        self.get_logger().warning(f"yielding /eufs/cmd to {self.external_node}")

    @staticmethod
    def _command_fingerprint(msg: AckermannDriveStamped) -> tuple[int, int, bytes]:
        stamp = msg.header.stamp
        values = struct.pack("<5f", float(msg.drive.steering_angle),
                             float(msg.drive.steering_angle_velocity),
                             float(msg.drive.speed), float(msg.drive.acceleration),
                             float(msg.drive.jerk))
        return int(stamp.sec), int(stamp.nanosec), values

    def _on_odom(self, msg: Odometry):
        self.last_odom_wall = time.monotonic()
        self.last_odom_seq += 1
        self.vx = float(msg.twist.twist.linear.x)
        self.vy = float(msg.twist.twist.linear.y)
        self.yaw_rate = float(msg.twist.twist.angular.z)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (self.vx, self.vy, self.yaw_rate, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            self.failed = "non-finite odometry"
            return
        if self.vx < -0.1:
            if self.reverse_since_wall is None:
                self.reverse_since_wall = self.last_odom_wall
            elif self.last_odom_wall - self.reverse_since_wall > 1.0:
                self.failed = "sustained reverse motion"
        else:
            self.reverse_since_wall = None
        odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        source = msg.header.frame_id or "odom"
        try:
            tf = self.tf_buffer.lookup_transform("map", source, rclpy.time.Time())
            tq = tf.transform.rotation
            tf_yaw = math.atan2(2.0 * (tq.w * tq.z + tq.x * tq.y),
                                1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z))
            c, s = math.cos(tf_yaw), math.sin(tf_yaw)
            self.map_x = tf.transform.translation.x + c * p.x - s * p.y
            self.map_y = tf.transform.translation.y + s * p.x + c * p.y
            self.map_yaw = _wrap(tf_yaw + odom_yaw)
            self.tf_ready = True
            if self.last_odom_map is not None:
                jump = math.hypot(self.map_x - self.last_odom_map[0],
                                  self.map_y - self.last_odom_map[1])
                if jump > 20.0:
                    self.failed = "odometry teleport"
            self.last_odom_map = (self.map_x, self.map_y)
        except Exception:
            self.tf_ready = False
            self.map_x = self.map_y = self.map_yaw = None

    def publish_route(self):
        points = [Point(x=p.x, y=p.y, z=0.05) for p in self.route.points]
        if points:
            points.append(points[0])
        line = Marker()
        line.header.frame_id = "map"
        line.ns, line.id, line.type, line.action = "cota_lap_plan", 410, Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.12
        line.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.9)
        line.points = points
        dots = Marker()
        dots.header.frame_id = "map"
        dots.ns, dots.id, dots.type, dots.action = "cota_lap_plan", 411, Marker.POINTS, Marker.ADD
        dots.scale.x = dots.scale.y = 0.28
        dots.color = ColorRGBA(r=1.0, g=0.65, b=0.05, a=0.9)
        dots.points = points[:-1] if len(points) > 1 else points
        self.plan_pub.publish(MarkerArray(markers=[line, dots]))
        route_marker = Marker()
        route_marker.header.frame_id = "map"
        route_marker.ns, route_marker.id = "cota_lap_plan", 412
        route_marker.type, route_marker.action = Marker.LINE_STRIP, Marker.ADD
        route_marker.scale.x = 0.12
        route_marker.color = line.color
        route_marker.points = points
        self.route_pub.publish(route_marker)

    def _command(self, steering: float, target_speed: float, acceleration: float,
                 *, allow_after_override: bool = False) -> bool:
        if self.external_override and not allow_after_override:
            return False
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = max(0.0, float(target_speed))
        msg.drive.acceleration = float(acceleration)
        fingerprint = self._command_fingerprint(msg)
        if len(self._own_command_fingerprints) == self._own_command_fingerprints.maxlen:
            self._own_command_fingerprint_set.discard(self._own_command_fingerprints[0])
        self._own_command_fingerprints.append(fingerprint)
        self._own_command_fingerprint_set.add(fingerprint)
        self.cmd_pub.publish(msg)
        self.last_steer = float(steering)
        return True

    def _mission_valid(self) -> bool:
        state = self.mission_state
        return state is not None and (state.ami_state == CanState.AMI_MANUAL or
                                      state.as_state == CanState.AS_DRIVING)

    def _request_manual_if_unselected(self, deadline: float) -> bool:
        while rclpy.ok() and time.monotonic() < deadline and self.mission_state is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._mission_valid():
            return True
        if self.mission_state is None:
            self.failed = "mission state unavailable"
            return False
        not_selected = getattr(CanState, "AMI_NOT_SELECTED", None)
        if not_selected is None or self.mission_state.ami_state != not_selected:
            self.failed = "mission already selected"
            return False
        if not self.mission.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
            self.failed = "mission service unavailable"
            return False
        req = SetCanState.Request()
        req.ami_state = CanState.AMI_MANUAL
        future = self.mission.call_async(req)
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None or not getattr(future.result(), "success", False):
            self.failed = "manual mission request rejected"
            return False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._mission_valid():
                return True
        self.failed = "manual mission acknowledgement timeout"
        return False

    def _verify_acceleration_mode(self):
        if not getattr(self.args, "verify_acceleration", False):
            return
        client = self.create_client(Trigger, "/race_car_model/command_mode")
        if not client.wait_for_service(timeout_sec=0.4):
            self.get_logger().warning("race_car_model command mode could not be queried")
            return
        req = Trigger.Request()
        future = client.call_async(req)
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            self.get_logger().warning("race_car_model command mode query timed out")
            return
        response = future.result()
        mode = str(getattr(response, "message", "")).strip().lower()
        if not getattr(response, "success", False) or mode != "acceleration":
            self.failed = "race_car_model is not in acceleration mode"

    def _ready(self) -> bool:
        deadline = time.monotonic() + READY_TIMEOUT
        if not self.unpause.wait_for_service(timeout_sec=READY_TIMEOUT):
            self.failed = "physics service unavailable"
            return False
        future = self.unpause.call_async(Empty.Request())
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            self.failed = "unpause service timeout"
            return False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._refresh_own_command_gids()
            fresh_odom = self.last_odom_wall is not None and time.monotonic() - self.last_odom_wall <= ODOM_WATCHDOG
            fresh_clock = self.last_clock_wall is not None and time.monotonic() - self.last_clock_wall <= CLOCK_WATCHDOG
            if self.clock_backwards:
                return False
            if fresh_odom and fresh_clock and self.tf_ready and self._mission_valid():
                self._verify_acceleration_mode()
                return self.failed is None
            if (self.mission_state is not None and
                    getattr(CanState, "AMI_NOT_SELECTED", None) == self.mission_state.ami_state):
                if not self._request_manual_if_unselected(deadline):
                    return False
        if self.failed is None:
            self.failed = "clock/TF/odometry/mission readiness timeout"
        return False

    def _control_once(self, dt: float):
        if self.external_override:
            return
        if self.map_x is None or self.map_y is None or self.map_yaw is None or self.vx is None:
            self.failed = "odometry unavailable"
            return
        result = self.controller.step(self.map_x, self.map_y, self.map_yaw, self.vx, dt)
        self.control_started = True
        target = min(max(0.0, result.target_speed), self.args.max_speed)
        accel = max(-self.args.brake, min(self.args.max_accel, result.accel))
        # Keep the final projected metre moving slowly enough to reach the
        # controller's full-lap condition.  The helper intentionally reports
        # target zero there; a crawl command prevents an early stall.
        remaining = self.route.length - result.progress
        if not result.done and 0.0 < remaining < 1.0:
            target = max(target, min(0.5, self.args.max_speed))
            accel = max(accel, min(self.args.max_accel, 0.8 * (target - self.vx)))
        if target < result.target_speed and self.vx > target:
            drag = (0.8269 / 788.0) * self.vx * abs(self.vx)
            accel = min(accel, 0.8 * (target - self.vx) + drag)
            accel = max(-self.args.brake, min(self.args.max_accel, accel))
        self._command(result.steer, target, accel)
        self.done = bool(result.done)
        if result.failure:
            self.failed = result.failure
        sample = {
            "sim": self.sim_time, "x": self.map_x, "y": self.map_y,
            "yaw": self.map_yaw, "progress": result.progress,
            "distance": result.progress, "route_length": self.route.length,
            "clearance": result.clearance, "steer": result.steer,
            "target_speed": target, "measured_vx": self.vx,
            "measured_vy": self.vy, "yaw_rate": self.yaw_rate,
            "accel": accel, "done": self.done, "failure": result.failure,
        }
        self.trace.append(sample)
        self._write_csv(sample)
        if self.sim_time is not None and (self.last_print_sim is None or self.sim_time - self.last_print_sim >= 10.0):
            self.last_print_sim = self.sim_time
            print(f"COTA lap: {result.progress:.1f}/{self.route.length:.1f} m "
                  f"({100.0 * result.progress / max(self.route.length, 1e-9):.1f}%), "
                  f"target {target:.2f} m/s, actual {self.vx:.2f} m/s, "
                  f"clearance {result.clearance:.2f} m", flush=True)

    def run(self) -> int:
        self.run_started_wall = time.monotonic()
        exit_code = 2
        try:
            if self._ready():
                end = time.monotonic() + self.args.max_wall
                while rclpy.ok() and not self.done and self.failed is None:
                    now = time.monotonic()
                    if now >= end:
                        self.failed = "wall timeout"
                        break
                    rclpy.spin_once(self, timeout_sec=0.01)
                    now = time.monotonic()
                    if self.external_override or self.clock_backwards:
                        break
                    if self.last_odom_wall is None or now - self.last_odom_wall > ODOM_WATCHDOG:
                        self.failed = "stale odometry"
                        break
                    if self.last_clock_wall is None or now - self.last_clock_wall > CLOCK_WATCHDOG:
                        self.failed = "stale clock"
                        break
                    if self.sim_time is not None and (self.last_control_sim is None or self.sim_time - self.last_control_sim >= CONTROL_PERIOD):
                        dt = CONTROL_PERIOD if self.last_control_sim is None else self.sim_time - self.last_control_sim
                        if dt <= 0.0:
                            self.failed = "clock backwards"
                            break
                        self.last_control_sim = self.sim_time
                        self._control_once(dt)
                exit_code = 0 if self.done else 2
        except KeyboardInterrupt:
            self.interrupted = True
            self.failed = "interrupted"
            exit_code = 130
        except Exception as exc:
            self.failed = f"exception: {exc}"
            self.get_logger().error(f"lap runner failed: {exc}")
            exit_code = 2
        finally:
            self.finalize()
            self.write_summary()
            if exit_code == 0 and self.failed is not None:
                exit_code = 2
        return exit_code

    def finalize(self):
        if not self.control_started or self.last_odom_wall is None or self.vx is None:
            return
        if self.external_override and not self.brake_started:
            return
        self.brake_started = True
        start_wall = time.monotonic()
        settle_start_sim: float | None = None
        while rclpy.ok() and time.monotonic() - start_wall < BRAKE_TIMEOUT:
            if not self.external_override:
                speed = self.vx or 0.0
                signed_brake = -self.args.brake if speed >= 0.0 else self.args.brake
                self._command(self.last_steer, 0.0, signed_brake, allow_after_override=True)
            rclpy.spin_once(self, timeout_sec=0.05)
            if abs(self.vx or 0.0) < 0.05:
                if settle_start_sim is None:
                    settle_start_sim = self.sim_time
                if self.sim_time is not None and settle_start_sim is not None and self.sim_time - settle_start_sim >= BRAKE_SETTLE:
                    self.brake_settled = True
                    break
            else:
                settle_start_sim = None
        if self.vx is not None and abs(self.vx) < 0.05 and settle_start_sim is not None:
            for _ in range(3):
                if not self.external_override:
                    self._command(0.0, 0.0, 0.0, allow_after_override=True)
                rclpy.spin_once(self, timeout_sec=0.05)
        if not self.brake_settled and self.failed is None:
            self.failed = "brake timeout"

    def _open_trace_csv(self):
        trace = Path(self.args.trace)
        csv_path = trace.with_suffix(".csv")
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_stream = csv_path.open("w", newline="", encoding="utf-8")
            fields = ["sim", "x", "y", "yaw", "progress", "distance", "route_length", "clearance", "steer", "target_speed", "measured_vx", "measured_vy", "yaw_rate", "accel", "done", "failure"]
            self._csv_writer = csv.DictWriter(self._csv_stream, fieldnames=fields)
            self._csv_writer.writeheader()
        except OSError as exc:
            self.get_logger().warning(f"could not stream CSV trace: {exc}")

    def _write_csv(self, sample: dict[str, Any]):
        if self._csv_writer is not None:
            self._csv_writer.writerow(sample)
            self._csv_stream.flush()

    def write_summary(self):
        if self._csv_stream is not None:
            self._csv_stream.close()
            self._csv_stream = None
        summary = {
            "completed": bool(self.done and self.failed is None),
            "done": self.done, "failure": self.failed,
            "external_override": self.external_override,
            "sim_time": self.sim_time, "wall_seconds": time.monotonic() - self.run_started_wall,
            "progress_m": self.controller.progress, "route_length_m": self.route.length,
            "final_pose_map": {"x": self.map_x, "y": self.map_y, "yaw": self.map_yaw},
            "final_vx": self.vx, "final_vy": self.vy,
            "final_clearance": self.trace[-1].get("clearance") if self.trace else None,
            "trace_csv": str(Path(self.args.trace).with_suffix(".csv")),
        }
        try:
            path = Path(self.args.trace)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({"summary": summary, "trace": self.trace}, indent=2), encoding="utf-8")
        except OSError as exc:
            self.get_logger().error(f"could not write lap summary: {exc}")


def _build_parser() -> argparse.ArgumentParser:
    base = Path(__file__).resolve().parents[1] / "overlay/eufs_tracks/cota"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--centerline", default=str(base / "centerline.csv"))
    parser.add_argument("--boundaries", default=str(base / "boundaries.csv"))
    parser.add_argument("--max-speed", type=float, default=15.0)
    parser.add_argument("--max-accel", type=float, default=1.0)
    parser.add_argument("--brake", type=float, default=1.2)
    parser.add_argument("--max-wall", type=float, default=1800.0)
    parser.add_argument("--trace", default="cota_lap_trace.json")
    parser.add_argument("--verify-acceleration", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if args.max_speed <= 0.0 or args.max_accel <= 0.0 or args.brake <= 0.0 or args.max_wall <= 0.0:
        raise SystemExit("speed, acceleration, brake and wall limits must be positive")
    if not Path(args.centerline).is_file():
        raise SystemExit(f"centerline does not exist: {args.centerline}")
    if not Path(args.boundaries).is_file():
        raise SystemExit(f"boundaries do not exist: {args.boundaries}")
    rclpy.init()
    node: Lap | None = None
    result = 2
    try:
        node = Lap(CotaRoute(args.centerline, args.boundaries), args)
        def _signal(_signum, _frame):
            if node is not None:
                node.interrupted = True
                node.failed = "interrupted"
        signal.signal(signal.SIGINT, _signal)
        signal.signal(signal.SIGTERM, _signal)
        result = node.run()
    except KeyboardInterrupt:
        result = 130
        if node is not None:
            node.interrupted = True
            node.failed = "interrupted"
            node.finalize()
            node.write_summary()
    except Exception as exc:
        if node is not None:
            node.failed = f"exception: {exc}"
            node.write_summary()
        else:
            print(f"cota lap setup failed: {exc}")
        result = 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
