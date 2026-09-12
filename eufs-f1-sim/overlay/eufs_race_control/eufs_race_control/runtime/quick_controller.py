"""Causal three-layer controller used by the bounded two-car demo."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from dataclasses import replace
import math
import time
import uuid
from typing import Any

from ..contracts import (ContractHeader, EnergyState, EstimatedRaceState,
                         OpponentBelief, Source, TrajectoryPlan,
                         TrajectoryPoint, VehicleState)
from ..estimation.pose import PoseSample, SyntheticPoseSensor
from ..geometry.track import TrackGeometry
from ..logging.schema import make_record
from ..mpc.adapter import build_mpc_input
from ..mpc.model import HybridProfile, MPCConfig, MPCController
from ..strategy.attack_follow import StrategyConfig, StrategyLayer
from ..tactical.frenet_candidates import Candidate, TacticalPlanner


@dataclass(frozen=True)
class QuickConfig:
    run_id: str = ""
    epoch_id: int = 0
    target_speed_mps: float = 20.0
    opponent_speed_mps: float = 15.0
    reserve_energy_j: float = 400000.0
    control_rate_hz: float = 15.0
    strategy_rate_hz: float = 1.0
    tactical_rate_hz: float = 5.0
    max_runtime_s: float = 120.0
    map_origin_x_m: float = 0.0
    map_origin_y_m: float = 0.0
    map_origin_yaw_rad: float = 0.0
    track_centerline: str = ""
    track_boundaries: str = ""


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


class QuickController:
    """Own the causal state and the only ego hybrid command stream."""

    def __init__(self, config: QuickConfig = QuickConfig(), *,
                 mpc: MPCController | None = None,
                 track: TrackGeometry | None = None):
        self.config = config
        self.run_id = config.run_id or uuid.uuid4().hex
        self.epoch_id = int(config.epoch_id)
        self.track = track
        profile = replace(HybridProfile(), ice_max_force_n=3500.0, ice_max_power_w=60000.0)
        self.mpc = mpc or MPCController(MPCConfig(deadline_s=0.1),
                                        hybrid_profile=profile)
        self.strategy = StrategyLayer(StrategyConfig(hard_reserve_j=config.reserve_energy_j))
        self.tactical = TacticalPlanner(track)
        self.ego_history: deque[tuple[float, float, float, float]] = deque(maxlen=32)
        self.opponent_history: deque[tuple[float, float, float, float]] = deque(maxlen=32)
        self.opponent_sensor = SyntheticPoseSensor(delay_s=0.0, noise_std_m=0.0,
                                                   heading_noise_std_rad=0.0)
        self.energy_j = 4_000_000.0
        self.energy_message: Any | None = None
        self.applied_feedback_steering = 0.0
        self.applied_feedback_acceleration = 0.0
        self.applied_feedback_mguk = 0.0
        self.pending_commands: deque[dict[str, float]] = deque(maxlen=32)
        self.ego_speed = 0.0
        self.last_strategy = None
        self.last_candidates: tuple[Candidate, ...] = ()
        self.last_candidate: Candidate | None = None
        self.last_command = (0.0, 0.0, 0.0)
        self.command_sequence = 0
        self.snapshot_sequence = 0
        self.started = time.monotonic()
        self.last_strategy_stamp = -math.inf
        self.last_tactical_stamp = -math.inf

    def prepare_solver(self) -> None:
        self.mpc.ensure_solver()

    def observe_ego(self, stamp: float, x: float, y: float, yaw: float,
                    speed: float, *, map_pose: tuple[float, float, float] | None = None) -> None:
        pose = map_pose or (float(x), float(y), float(yaw))
        self.ego_history.append((float(stamp), *pose))
        self.ego_speed = _finite(speed)

    def observe_energy(self, stored_energy_j: float | None, *, message: Any | None = None) -> None:
        if stored_energy_j is not None and math.isfinite(float(stored_energy_j)):
            self.energy_j = max(0.0, float(stored_energy_j))
        self.energy_message = message
        header = getattr(message, "header", None)
        native_run = str(getattr(header, "run_id", "") or "")
        if native_run:
            self.run_id = native_run
        native_epoch = getattr(header, "epoch_id", None)
        if native_epoch is not None:
            try:
                self.epoch_id = max(0, int(native_epoch))
            except (TypeError, ValueError):
                pass

    def observe_feedback(self, message: Any) -> None:
        self.applied_feedback_steering = _finite(
            getattr(message, "delivered_steering_rad", self.applied_feedback_steering),
            self.applied_feedback_steering)
        self.applied_feedback_acceleration = _finite(
            getattr(message, "delivered_acceleration_mps2", self.applied_feedback_acceleration),
            self.applied_feedback_acceleration)
        self.applied_feedback_mguk = _finite(
            getattr(message, "delivered_mguk_force_n", self.applied_feedback_mguk),
            self.applied_feedback_mguk)

    def observe_opponent(self, stamp: float, x: float, y: float, yaw: float,
                         *, map_pose: tuple[float, float, float] | None = None) -> None:
        pose = map_pose or (float(x), float(y), float(yaw))
        truth = PoseSample(float(stamp), *pose, source_id="replay_truth")
        observation = self.opponent_sensor.observe(truth, receive_stamp=float(stamp))
        if observation is not None:
            value = observation.pose
            self.opponent_history.append((value.stamp, value.x_m, value.y_m, value.yaw_rad))

    def _gap_and_speed(self) -> tuple[float | None, float | None]:
        if not self.ego_history or not self.opponent_history:
            return None, None
        ego, opponent = self.ego_history[-1], self.opponent_history[-1]
        if self.track is not None:
            try:
                e = self.track.project(ego[1], ego[2], previous_heading_rad=ego[3], max_distance_m=None)
                o = self.track.project(opponent[1], opponent[2], previous_heading_rad=opponent[3], max_distance_m=None)
                gap = o.unwrapped_progress_m - e.unwrapped_progress_m
            except ValueError:
                gap = math.hypot(opponent[1] - ego[1], opponent[2] - ego[2])
        else:
            gap = math.hypot(opponent[1] - ego[1], opponent[2] - ego[2])
        opponent_speed = self.config.opponent_speed_mps
        if len(self.opponent_history) >= 2:
            a, b = self.opponent_history[-2], self.opponent_history[-1]
            dt = max(1e-6, b[0] - a[0])
            opponent_speed = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
        return gap, opponent_speed

    def update_strategy(self, stamp: float | None = None) -> None:
        gap, opponent_speed = self._gap_and_speed()
        self.last_strategy = self.strategy.decide(
            gap_m=gap, ego_speed_mps=self.ego_speed,
            opponent_speed_mps=opponent_speed, stored_energy_j=self.energy_j)
        self.last_strategy_stamp = float(stamp if stamp is not None else 0.0)

    def _candidate_points(self, candidate: Candidate, stamp: float) -> tuple[TrajectoryPoint, ...]:
        points = []
        previous_progress = None
        for index, (x, y, yaw) in enumerate(candidate.points):
            progress = None
            if self.track is not None:
                try:
                    projection = self.track.project(x, y, previous_progress_m=previous_progress,
                                                    previous_heading_rad=yaw, max_distance_m=None)
                    progress, previous_progress = projection.unwrapped_progress_m, projection.unwrapped_progress_m
                except ValueError:
                    progress = previous_progress
            points.append(TrajectoryPoint(
                stamp=float(stamp) + index * 0.5, x_m=x, y_m=y, yaw_rad=yaw,
                progress_m=progress, lateral_m=candidate.lateral_m,
                speed_mps=max(0.0, candidate.speed_mps),
                corridor_left_m=-7.0, corridor_right_m=7.0,
                energy_reference_j=self.energy_j))
        return tuple(points)

    def update_tactical(self, stamp: float | None = None) -> None:
        stamp = float(stamp if stamp is not None else 0.0)
        if self.last_strategy is None:
            self.update_strategy(stamp)
        if self.track is None or not self.ego_history or not self.opponent_history:
            self.last_candidates, self.last_candidate = (), None
            self.last_tactical_stamp = stamp
            return
        ego, opponent = self.ego_history[-1], self.opponent_history[-1]
        try:
            projection = self.track.project(ego[1], ego[2], previous_heading_rad=ego[3], max_distance_m=None)
            velocity = self.tactical.constant_velocity(self.opponent_history)
            candidates = self.tactical.candidates(
                progress_m=projection.unwrapped_progress_m,
                speed_mps=max(self.ego_speed, self.config.target_speed_mps),
                opponent_pose=(opponent[1], opponent[2], opponent[3]),
                opponent_velocity_mps=velocity,
                hold_side=None if self.last_candidate is None else self.last_candidate.lateral_m)
        except ValueError:
            candidates = ()
        self.last_candidates = tuple(candidates)
        action = self.last_strategy.action if self.last_strategy is not None else "follow"
        self.last_candidate = self.tactical.choose(candidates, action=action)
        if self.last_candidate is None:
            self.last_candidate = next((item for item in candidates if item.candidate_id == "follow"), None)
        self.last_tactical_stamp = stamp

    def _estimate_and_plan(self, stamp: float) -> tuple[EstimatedRaceState, TrajectoryPlan]:
        if self.track is None:
            raise RuntimeError("quick controller requires TrackGeometry for MPC adapter")
        if not self.ego_history:
            raise RuntimeError("ego pose unavailable")
        ego_pose = self.ego_history[-1]
        projection = self.track.project(ego_pose[1], ego_pose[2], previous_heading_rad=ego_pose[3], max_distance_m=None)
        header = ContractHeader(
            run_id=self.run_id, epoch_id=self.epoch_id, stamp=stamp,
            snapshot_id=f"state-{self.snapshot_sequence:08d}",
            trajectory_id=f"trajectory-{self.snapshot_sequence:08d}",
            expires_at=stamp + 0.5, source=Source.ESTIMATED, frame_id="map")
        temperature = _finite(getattr(self.energy_message, "temperature_k", 298.15), 298.15)
        energy = EnergyState(header=header, stored_energy_j=self.energy_j,
                             reserve_energy_j=self.config.reserve_energy_j,
                             temperature_k=temperature, stored_energy_valid=True,
                             temperature_valid=True)
        opponent = self.opponent_history[-1] if self.opponent_history else None
        opponent_belief = OpponentBelief(
            header=header, opponent_id="russell",
            x_m=None if opponent is None else opponent[1],
            y_m=None if opponent is None else opponent[2],
            yaw_rad=None if opponent is None else opponent[3],
            speed_mps=self._gap_and_speed()[1], pose_valid=opponent is not None,
            speed_valid=opponent is not None, identity_valid=opponent is not None)
        vx = max(0.0, self.ego_speed)
        vehicle = VehicleState(
            header=header, x_m=ego_pose[1], y_m=ego_pose[2], yaw_rad=ego_pose[3],
            vx_mps=vx, steering_rad=self.applied_feedback_steering,
            track_speed_mps=vx, pose_valid=True, velocity_valid=True, steering_valid=True)
        estimate = EstimatedRaceState(
            header=header, ego=vehicle, energy=energy, opponent=opponent_belief,
            progress_m=projection.progress_m, unwrapped_progress_m=projection.unwrapped_progress_m,
            track_length_m=self.track.length_m, ready=True)
        candidate = self.last_candidate
        if candidate is None:
            candidate = Candidate("follow", 0.0, max(vx, self.config.target_speed_mps), True,
                                  "fallback_follow", ((ego_pose[1], ego_pose[2], ego_pose[3]),))
        trajectory = TrajectoryPlan(
            header=header, candidate_id=candidate.candidate_id,
            points=self._candidate_points(candidate, stamp), feasible=candidate.feasible,
            infeasibility_reason=None if candidate.feasible else candidate.reason)
        return estimate, trajectory

    def update_control(self, stamp: float) -> dict[str, Any]:
        stamp = float(stamp)
        self.snapshot_sequence += 1
        if self.last_strategy is None:
            self.update_strategy(stamp)
        if stamp - self.last_tactical_stamp >= 1.0 / max(self.config.tactical_rate_hz, 1e-6):
            self.update_tactical(stamp)
        try:
            estimate, trajectory = self._estimate_and_plan(stamp)
            request = build_mpc_input(
                estimate, trajectory, self.track, now_monotonic_s=time.monotonic(),
                actual_pending_commands=tuple(self.pending_commands), sim_time_s=stamp)
            result = self.mpc.solve(request)
            command = self.mpc.command_from_result(result, self.applied_feedback_steering) if result.status == "solved" else None
        except Exception as exc:
            result, command, failure = None, None, str(exc)
        else:
            failure = ""
        if command is None:
            self.last_command = (self.applied_feedback_steering, -12.0, 0.0)
            owner = "mpc_unavailable_brake"
            status = "runtime_failure" if result is None else result.status
            reason = failure or (result.rejection_reason if result is not None else "no_result")
            solve_time = None if result is None else result.solve_time_s
        else:
            self.last_command = tuple(float(value) for value in command)
            owner = "native_mpc"
            status, reason, solve_time = result.status, result.rejection_reason, result.solve_time_s
        self.command_sequence += 1
        self.pending_commands.append({
            "stamp_s": stamp, "steering_angle_rad": self.last_command[0],
            "predrag_acceleration_mps2": self.last_command[1],
            "signed_mguk_force_n": self.last_command[2]})
        return {
            "sequence": self.command_sequence, "snapshot_id": f"state-{self.snapshot_sequence:08d}",
            "trajectory_id": f"trajectory-{self.snapshot_sequence:08d}", "owner": owner,
            "command": self.last_command, "solver_status": status,
            "solver_reason": reason, "solver_time_s": solve_time,
            "strategy": self.last_strategy.action if self.last_strategy else "follow",
            "candidate": None if self.last_candidate is None else self.last_candidate.candidate_id,
            "candidates": [{"id": item.candidate_id, "lateral_m": item.lateral_m,
                            "feasible": item.feasible, "reason": item.reason,
                            "points": item.points} for item in self.last_candidates],
            "selected_path": None if self.last_candidate is None else self.last_candidate.points}

    def record(self, stamp: float, command_record: dict[str, Any]) -> dict[str, Any]:
        gap, opponent_speed = self._gap_and_speed()
        ego = self.ego_history[-1] if self.ego_history else None
        opponent = self.opponent_history[-1] if self.opponent_history else None
        return make_record(
            component="quick_controller", record_type="mpc_command", sim_time=float(stamp),
            run_id=self.run_id, epoch_id=self.epoch_id,
            ids={"state_snapshot_id": command_record["snapshot_id"],
                 "trajectory_id": command_record["trajectory_id"],
                 "command_sequence": command_record["sequence"]},
            data={"ego_pose": None if ego is None else ego[1:],
                  "opponent_pose": None if opponent is None else opponent[1:],
                  "energy_j": self.energy_j, "gap_m": gap,
                  "opponent_speed_mps": opponent_speed,
                  "intent": self.last_strategy.action if self.last_strategy else "follow",
                  **command_record})


__all__ = ["QuickConfig", "QuickController"]
