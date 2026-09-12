"""Threaded ROS adapter for the GUI.

The bridge owns its ROS executor thread.  ROS callbacks only decode canonical
records and emit Qt signals; they never touch widgets or the GUI data source.
Operator requests are queued back to the ROS thread and published on the
orchestrator request topic.  This module remains importable without ROS.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from typing import Any, Mapping, Optional

from PyQt5.QtCore import QObject, pyqtSignal

from .core import RECORD_SCHEMA_VERSION, Record


class CanonicalEnvelopeDecoder:
    """Single adapter boundary for append-only JSON record envelopes."""

    def decode(self, payload: str | bytes | Mapping[str, Any], *, source: str = "live_ros") -> Record:
        if isinstance(payload, bytes):
            payload = payload.decode("utf-8")
        value = json.loads(payload) if isinstance(payload, str) else dict(payload)
        # Transport provenance is authoritative at this boundary. A JSON
        # payload cannot relabel a truth subscription as operational data.
        value["source"] = source
        return Record.from_mapping(value, source=source)

    @staticmethod
    def encode_operator_request(action: str, parameters: Mapping[str, Any], *, run_id: str = "", epoch_id: int = 0) -> str:
        return json.dumps({
            "schema_version": RECORD_SCHEMA_VERSION, "run_id": run_id, "epoch_id": epoch_id,
            "scenario_id": str(parameters.get("scenario", "")), "sim_time": float(parameters.get("sim_time", 0.0)),
            "wall_time": time.monotonic(), "component": "race_gui", "record_type": "operator_request",
            "ids": {}, "data": {"action": action, "parameters": dict(parameters)},
        }, separators=(",", ":"))


class OdomMapper:
    """Map native odometry into an estimate record; truth uses a separate source."""

    @staticmethod
    def _stamp(message: Any) -> float:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return 0.0
        return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9

    @staticmethod
    def _yaw(orientation: Any) -> float:
        x, y, z, w = (float(getattr(orientation, field, 0.0)) for field in ("x", "y", "z", "w"))
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def to_record(self, message: Any, car: str, *, run_id: str = "", epoch_id: int = 0,
                  source: str = "live_ros") -> Record:
        pose = message.pose.pose; twist = message.twist.twist
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", "")); child_frame = str(getattr(message, "child_frame_id", ""))
        return Record.from_mapping({
            "schema_version": RECORD_SCHEMA_VERSION, "run_id": run_id, "epoch_id": epoch_id,
            "scenario_id": "", "sim_time": self._stamp(message), "wall_time": time.monotonic(),
            "component": "telemetry", "record_type": "raw_odometry", "ids": {}, "source": source,
            "data": {"car": car, "classification": "truth" if source == "truth" else "measurement",
                      "frame_id": frame_id, "child_frame_id": child_frame,
                      "raw_position": {"x_m": float(pose.position.x), "y_m": float(pose.position.y), "z_m": float(pose.position.z)},
                      "raw_yaw_rad": self._yaw(pose.orientation), "raw_speed_mps": float(twist.linear.x),
                      "raw_velocity": {"x_mps": float(twist.linear.x), "y_mps": float(twist.linear.y), "z_mps": float(twist.linear.z)}},
        }, source=source)


class LiveROSBridge(QObject):
    """ROS executor isolated from the Qt GUI thread."""

    recordReceived = pyqtSignal(object)
    healthChanged = pyqtSignal(object)
    errorRaised = pyqtSignal(str)
    connectedChanged = pyqtSignal(bool)

    def __init__(self, *, run_id: str = "live", epoch_id: int = 0, domain_id: Optional[int] = None, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.run_id, self.epoch_id, self.domain_id, self.sim_time = run_id, int(epoch_id), domain_id, 0.0
        self.decoder = CanonicalEnvelopeDecoder(); self.odom_mapper = OdomMapper()
        self._requests: "queue.Queue[Mapping[str, Any]]" = queue.Queue(maxsize=100)
        self._thread: Optional[threading.Thread] = None; self._stop = threading.Event(); self._node = None; self._context = None
        self._last_health_monotonic = 0.0; self.health_timeout_s = 2.0
        self.ready = False

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> bool:
        if self.running:
            return True
        self._stop.clear(); self._thread = threading.Thread(target=self._run, name="eufs-race-gui-ros", daemon=True); self._thread.start(); return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread(): self._thread.join(timeout=2.0)
        self._thread = None; self._node = None; self._context = None; self.ready = False; self.connectedChanged.emit(False)

    def send_operator_request(self, action: str, parameters: Mapping[str, Any]) -> None:
        # Called from Qt; publication occurs in the ROS thread's timer.
        try: self._requests.put_nowait({"action": action, "parameters": dict(parameters)})
        except queue.Full: self.errorRaised.emit("operator request queue full")

    def ingest_json(self, payload: str | bytes | Mapping[str, Any], *, source: str = "live_ros") -> None:
        try:
            record = self.decoder.decode(payload, source=source); self.sim_time = record.sim_time; self.recordReceived.emit(record)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.errorRaised.emit(f"record rejected: {exc}")

    def _run(self) -> None:
        context = None
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
            context = rclpy.context.Context(); self._context = context
            if self.domain_id is not None: rclpy.init(context=context, domain_id=self.domain_id)
            else: rclpy.init(context=context)
            node = rclpy.create_node("eufs_race_gui_bridge", context=context); self._node = node
            executor = SingleThreadedExecutor(context=context); executor.add_node(node)
            node.create_subscription(String, "/race/records", lambda message: self.ingest_json(message.data), 100)
            node.create_subscription(String, "/race/truth", lambda message: self.ingest_json(message.data, source="truth"), 100)
            node.create_subscription(String, "/race/orchestrator/health", self._on_health, 20)
            node.create_subscription(Odometry, "/eufs/odom", lambda message: self._on_odom(message, "eufs", "live_ros"), 20)
            node.create_subscription(Odometry, "/eufs2/odom", lambda message: self._on_odom(message, "eufs2", "truth"), 20)
            publisher = node.create_publisher(String, "/race/operator_request", 20)
            node.create_timer(0.02, lambda: self._drain_requests(publisher, String))
            self.connectedChanged.emit(True)
            while rclpy.ok(context=context) and not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
                if self.ready and time.monotonic() - self._last_health_monotonic > self.health_timeout_s:
                    self.ready = False; self.healthChanged.emit({"ready": False, "reason": "health timeout"})
            executor.remove_node(node); node.destroy_node(); rclpy.shutdown(context=context)
        except Exception as exc:  # ROS is optional for offline review/testing.
            self.errorRaised.emit(f"ROS bridge unavailable: {exc}"); self.connectedChanged.emit(False)
            if context is not None:
                try: rclpy.shutdown(context=context)
                except Exception: pass
        finally:
            self._node = None; self._context = None; self.ready = False

    def _on_odom(self, message: Any, car: str, source: str) -> None:
        try:
            record = self.odom_mapper.to_record(message, car, run_id=self.run_id, epoch_id=self.epoch_id, source=source); self.sim_time = record.sim_time; self.recordReceived.emit(record)
        except (AttributeError, TypeError, ValueError) as exc: self.errorRaised.emit(f"odometry rejected: {exc}")

    def _on_health(self, message: Any) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, Mapping): raise ValueError("health payload must be an object")
            if "epoch_id" in payload:
                epoch = int(payload["epoch_id"])
                if epoch < 0 or isinstance(payload["epoch_id"], float) and epoch != payload["epoch_id"]: raise ValueError("invalid epoch_id")
                self.epoch_id = epoch
            if "run_id" in payload: self.run_id = str(payload["run_id"])
            if "sim_time" in payload: self.sim_time = float(payload["sim_time"])
            self._last_health_monotonic = time.monotonic(); self.ready = bool(payload.get("ready", payload.get("healthy", False))); self.healthChanged.emit(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc: self.errorRaised.emit(f"health rejected: {exc}")

    def _drain_requests(self, publisher: Any, string_type: Any) -> None:
        while True:
            try: request = self._requests.get_nowait()
            except queue.Empty: return
            parameters = dict(request["parameters"]); parameters.setdefault("sim_time", self.sim_time)
            message = string_type(); message.data = CanonicalEnvelopeDecoder.encode_operator_request(request["action"], parameters, run_id=self.run_id, epoch_id=self.epoch_id); publisher.publish(message)
