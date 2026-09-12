#!/usr/bin/env python3
"""Drive the three-layer quick overtake against an already running ROS sim."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from eufs_control_msgs.msg import ActuationFeedback, EnergyState, HybridDriveStamped
from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState

from eufs_race_control.evaluation.quick_pass import QuickPassEvaluator
from eufs_race_control.geometry.track import TrackGeometry
from eufs_race_control.runtime.quick_controller import QuickConfig, QuickController


def _yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp(message) -> float:
    value = message.header.stamp
    return float(value.sec) + 1e-9 * float(value.nanosec)


def _clock_stamp(value) -> float:
    return float(value.sec) + 1e-9 * float(value.nanosec)


class QuickNode(Node):
    def __init__(self, output: Path, config: QuickConfig, *,
                 ego_odom_topic: str, opponent_topic: str,
                 energy_topic: str, feedback_topic: str, command_topic: str):
        super().__init__("quick_overtake_controller")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.controller = QuickController(config, track=self._load_track(config))
        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output.open("w", encoding="utf-8")
        self.sim_time: float | None = None
        self.sim_start: float | None = None
        self.wall_start = time.monotonic()
        self.last_score = None
        self.ego_raw: tuple[float, float, float] | None = None
        self.opponent_raw: tuple[float, float, float] | None = None
        self.pub = self.create_publisher(HybridDriveStamped, command_topic, 16)
        self.create_subscription(Clock, "/clock", self._clock, 4)
        self.create_subscription(Odometry, ego_odom_topic, self._ego, 10)
        self.create_subscription(Odometry, opponent_topic, self._opponent, 10)
        self.create_subscription(EnergyState, energy_topic, self._energy, 10)
        self.create_subscription(ActuationFeedback, feedback_topic, self._feedback, 10)
        self.mission = self.create_client(SetCanState, "/ros_can/set_mission")
        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.create_subscription(CanState, "/ros_can/state", self._mission_state, 4)
        self.mission_requested = False
        self.mission_future = None
        self.mission_ready = False
        self.unpause_requested = False
        self.timer = self.create_timer(1.0 / max(config.control_rate_hz, 1e-6), self._tick)
        # A ROS-time timer cannot fire while Gazebo is paused.  This wall-time
        # setup callback starts the world only after solver preparation above.
        self.startup_timer = self.create_wall_timer(0.2, self._startup)
        self.last_strategy_time = -math.inf
        self.evaluator = QuickPassEvaluator(self.controller.track) if self.controller.track else None
        # Solver code generation is intentionally outside the control timer.
        try:
            self.controller.prepare_solver()
            self.solver_error = ""
        except Exception as exc:
            self.solver_error = str(exc)
            self.get_logger().error(f"MPC solver preparation failed: {exc}")

    @staticmethod
    def _load_track(config: QuickConfig) -> TrackGeometry | None:
        root = Path(__file__).resolve().parents[3] / "overlay" / "eufs_tracks" / "cota"
        centerline = Path(config.track_centerline) if config.track_centerline else root / "centerline.csv"
        boundaries = Path(config.track_boundaries) if config.track_boundaries else root / "boundaries.csv"
        if not centerline.is_file():
            return None
        return TrackGeometry(centerline, boundaries if boundaries.is_file() else None)

    def _clock(self, message: Clock) -> None:
        self.sim_time = _clock_stamp(message.clock)

    def _ego_map_pose(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        c, s = math.cos(self.controller.config.map_origin_yaw_rad), math.sin(self.controller.config.map_origin_yaw_rad)
        return (
            self.controller.config.map_origin_x_m + c * x - s * y,
            self.controller.config.map_origin_y_m + s * x + c * y,
            yaw + self.controller.config.map_origin_yaw_rad,
        )

    def _ego(self, message: Odometry) -> None:
        pose = message.pose.pose
        x, y, yaw = float(pose.position.x), float(pose.position.y), _yaw(pose.orientation)
        self.ego_raw = (x, y, yaw)
        self.controller.observe_ego(_stamp(message), x, y, yaw,
                                    message.twist.twist.linear.x,
                                    map_pose=self._ego_map_pose(x, y, yaw))

    def _opponent(self, message: Odometry) -> None:
        pose = message.pose.pose
        x, y, yaw = float(pose.position.x), float(pose.position.y), _yaw(pose.orientation)
        self.opponent_raw = (x, y, yaw)
        self.controller.observe_opponent(_stamp(message), x, y, yaw)

    def _energy(self, message: EnergyState) -> None:
        stored = message.stored_energy_j if message.stored_energy_valid else None
        self.controller.observe_energy(stored, message=message)

    def _feedback(self, message: ActuationFeedback) -> None:
        self.controller.observe_feedback(message)

    def _mission_state(self, message: CanState) -> None:
        self.mission_ready = (message.ami_state == CanState.AMI_MANUAL or
                              message.as_state == CanState.AS_DRIVING)

    def _request_mission_once(self) -> None:
        if self.mission_requested or self.mission_future is not None:
            return
        if not self.mission.wait_for_service(timeout_sec=0.0):
            return
        request = SetCanState.Request()
        request.ami_state = CanState.AMI_MANUAL
        self.mission_future = self.mission.call_async(request)
        self.mission_requested = True

    def _startup(self) -> None:
        if not self.unpause_requested and self.unpause.wait_for_service(timeout_sec=0.0):
            self.unpause.call_async(Empty.Request())
            self.unpause_requested = True
        self._request_mission_once()
        if self.unpause_requested and self.mission_requested:
            self.startup_timer.cancel()

    def _tick(self) -> None:
        self._request_mission_once()
        if self.mission_future is not None and self.mission_future.done():
            result = self.mission_future.result()
            if result is not None and not result.success:
                self.get_logger().warning("/ros_can/set_mission rejected AMI_MANUAL")
            self.mission_future = None
        stamp = self.sim_time
        if stamp is None:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        if self.sim_start is None:
            self.sim_start = stamp
        if stamp - self.last_strategy_time >= 1.0 / max(self.controller.config.strategy_rate_hz, 1e-6):
            self.controller.update_strategy(stamp)
            self.last_strategy_time = stamp
        result = self.controller.update_control(stamp)
        message = HybridDriveStamped()
        now = self.get_clock().now()
        message.header.schema_version = "1.0"
        message.header.run_id = self.controller.run_id
        message.header.epoch_id = self.controller.epoch_id
        message.header.stamp = now.to_msg()
        message.header.expires_at = (now + rclpy.duration.Duration(seconds=0.5)).to_msg()
        message.header.source = "command"
        message.header.valid = True
        message.header.frame_id = "base_link"
        # MPC tuples are (steering target, pre-drag acceleration, MGU-K force).
        steering, acceleration, mguk = result["command"]
        message.acceleration_mps2 = acceleration
        message.steering_rad = steering
        message.requested_mguk_force_n = mguk
        message.command_sequence = result["sequence"]
        message.owner = result["owner"]
        self.pub.publish(message)
        score = None
        if self.evaluator and self.controller.ego_history and self.controller.opponent_history:
            ego = self.controller.ego_history[-1][1:]
            opponent = self.controller.opponent_history[-1][1:]
            score = self.evaluator.update(stamp, ego, opponent).as_dict()
            self.last_score = score
        record = self.controller.record(stamp, result)
        record["data"]["mission_ready"] = self.mission_ready
        record["data"]["evaluation"] = score
        self.stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.stream.flush()
        if (stamp - self.sim_start) >= self.controller.config.max_runtime_s or time.monotonic() - self.wall_start >= self.controller.config.max_runtime_s + 30.0:
            self.get_logger().info("quick demo timeout; stopping")
            self.close()
            rclpy.shutdown()

    def close(self) -> None:
        if self.stream.closed:
            return
        summary = {
            "run_id": self.controller.run_id,
            "epoch_id": self.controller.epoch_id,
            "sim_time_s": self.sim_time,
            "solver_prepare_error": self.solver_error,
            "mission_ready": self.mission_ready,
            "last_evaluation": self.last_score,
            "command_count": self.controller.command_sequence,
        }
        self.stream.close()
        self.output.with_suffix(".summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/tmp/quick-demo/quick_demo.jsonl")
    parser.add_argument("--max-sim-seconds", "--max-runtime", dest="max_runtime", type=float, default=120.0)
    parser.add_argument("--ego-odom-topic", default="/eufs/odom")
    parser.add_argument("--opponent-topic", default="/eufs2/replay_truth")
    parser.add_argument("--energy-topic", default="/hybrid/energy_state")
    parser.add_argument("--feedback-topic", default="/hybrid/actuation_feedback")
    parser.add_argument("--command-topic", default="/hybrid_cmd")
    parser.add_argument("--map-origin-x", type=float, default=0.0)
    parser.add_argument("--map-origin-y", type=float, default=0.0)
    parser.add_argument("--map-origin-yaw", type=float, default=0.0)
    args = parser.parse_args()
    config = QuickConfig(max_runtime_s=args.max_runtime,
                         map_origin_x_m=args.map_origin_x,
                         map_origin_y_m=args.map_origin_y,
                         map_origin_yaw_rad=args.map_origin_yaw)
    rclpy.init()
    node = QuickNode(Path(args.output), config,
                     ego_odom_topic=args.ego_odom_topic,
                     opponent_topic=args.opponent_topic,
                     energy_topic=args.energy_topic,
                     feedback_topic=args.feedback_topic,
                     command_topic=args.command_topic)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
