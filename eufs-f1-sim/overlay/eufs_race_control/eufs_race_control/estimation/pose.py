"""Causal pose buffering, synthetic observations and time-aligned tracking."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..contracts import ContractHeader, OpponentBelief, Source, Validity, VehicleState
from ..geometry import TrackGeometry


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True, slots=True)
class PoseSample:
    stamp: float
    x_m: float
    y_m: float
    yaw_rad: float = 0.0
    source_id: str = ""
    observation_id: Optional[str] = None
    covariance: Tuple[float, ...] = ()
    received_stamp: Optional[float] = None
    measurement_stamp: Optional[float] = None
    epoch_id: Optional[int] = None

    def __post_init__(self):
        for name in ("stamp", "x_m", "y_m", "yaw_rad"):
            _finite(getattr(self, name), name)
        if self.received_stamp is not None:
            _finite(self.received_stamp, "received_stamp")
        if self.measurement_stamp is not None:
            _finite(self.measurement_stamp, "measurement_stamp")
        if self.received_stamp is not None:
            source_stamp = self.measurement_stamp if self.measurement_stamp is not None else self.stamp
            if self.received_stamp < source_stamp:
                raise ValueError("received_stamp cannot precede measurement_stamp")
        if self.epoch_id is not None and (isinstance(self.epoch_id, bool) or not isinstance(self.epoch_id, int) or self.epoch_id < 0):
            raise ValueError("epoch_id must be a non-negative integer")
        for value in self.covariance:
            _finite(value, "covariance")


@dataclass(frozen=True, slots=True)
class PoseObservation:
    source_stamp: float
    receive_stamp: float
    pose: PoseSample
    dropped: bool = False
    delayed: bool = False

    def __post_init__(self):
        _finite(self.source_stamp, "source_stamp")
        _finite(self.receive_stamp, "receive_stamp")
        if self.receive_stamp < self.source_stamp:
            raise ValueError("receive_stamp cannot precede source_stamp")
        if abs(self.pose.stamp - self.source_stamp) > 1e-9:
            raise ValueError("pose stamp must equal source_stamp")


class PoseBuffer:
    """Monotonic timestamp buffer that never serves future measurements."""

    def __init__(self, max_samples: int = 256):
        if max_samples < 2:
            raise ValueError("max_samples must be at least two")
        self.max_samples = max_samples
        self._samples: List[PoseSample] = []
        self.last_rejection: Optional[str] = None

    @property
    def samples(self) -> Tuple[PoseSample, ...]:
        return tuple(self._samples)

    def clear(self) -> None:
        self._samples.clear()
        self.last_rejection = None

    def add(self, sample: PoseSample) -> bool:
        if self._samples and sample.stamp < self._samples[-1].stamp:
            self.last_rejection = "out_of_order"
            return False
        if self._samples and sample.stamp == self._samples[-1].stamp:
            self.last_rejection = "duplicate"
            return False
        self._samples.append(sample)
        del self._samples[:-self.max_samples]
        self.last_rejection = None
        return True

    def at(self, stamp: float, max_extrapolation_s: float = 0.0, epoch_id: Optional[int] = None) -> Optional[PoseSample]:
        """Interpolate only from samples at or before ``stamp``.

        A bounded extrapolation is allowed after the latest sample using the
        last two historical poses; a future sample is never used to answer a
        query.
        """
        stamp = _finite(stamp, "stamp")
        if not self._samples:
            return None
        prior = [sample for sample in self._samples if sample.stamp <= stamp + 1e-12 and (sample.received_stamp is None or sample.received_stamp <= stamp + 1e-12) and (epoch_id is None or sample.epoch_id == epoch_id)]
        if not prior:
            return None
        latest = prior[-1]
        if abs(latest.stamp - stamp) <= 1e-12:
            return latest
        if stamp - latest.stamp > max_extrapolation_s:
            return None
        if len(prior) < 2:
            return latest
        previous = prior[-2]
        dt = latest.stamp - previous.stamp
        if dt <= 1e-12:
            return latest
        ratio = (stamp - latest.stamp) / dt
        covariance = tuple(value * (1.0 + max(0.0, ratio)) ** 2 for value in latest.covariance)
        return PoseSample(
            stamp=stamp,
            x_m=latest.x_m + ratio * (latest.x_m - previous.x_m),
            y_m=latest.y_m + ratio * (latest.y_m - previous.y_m),
            yaw_rad=latest.yaw_rad + ratio * ((latest.yaw_rad - previous.yaw_rad + math.pi) % (2 * math.pi) - math.pi),
            source_id=latest.source_id,
            observation_id=latest.observation_id,
            covariance=covariance,
            received_stamp=latest.received_stamp,
            measurement_stamp=latest.measurement_stamp if latest.measurement_stamp is not None else latest.stamp,
            epoch_id=latest.epoch_id,
        )


class SyntheticPoseSensor:
    """Deterministic pose-only sensor model for an opponent truth stream."""

    def __init__(self, delay_s: float = 0.0, noise_std_m: float = 0.0, heading_noise_std_rad: float = 0.0, dropout_probability: float = 0.0, seed: int = 0, source_id: str = "synthetic_opponent"):
        for value, name in ((delay_s, "delay_s"), (noise_std_m, "noise_std_m"), (heading_noise_std_rad, "heading_noise_std_rad"), (dropout_probability, "dropout_probability")):
            _finite(value, name)
        if delay_s < 0.0 or noise_std_m < 0.0 or heading_noise_std_rad < 0.0:
            raise ValueError("delay and noise values must be non-negative")
        if not 0.0 <= dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be in [0, 1]")
        self.delay_s = delay_s
        self.noise_std_m = noise_std_m
        self.heading_noise_std_rad = heading_noise_std_rad
        self.dropout_probability = dropout_probability
        self.source_id = source_id
        self._rng = random.Random(seed)

    def observe(self, truth: PoseSample, receive_stamp: Optional[float] = None) -> Optional[PoseObservation]:
        receive_stamp = truth.stamp + self.delay_s if receive_stamp is None else _finite(receive_stamp, "receive_stamp")
        if receive_stamp < truth.stamp + self.delay_s - 1e-12:
            raise ValueError("receive_stamp is earlier than configured delay")
        if self._rng.random() < self.dropout_probability:
            return None
        pose = PoseSample(
            stamp=truth.stamp,
            x_m=truth.x_m + self._rng.gauss(0.0, self.noise_std_m),
            y_m=truth.y_m + self._rng.gauss(0.0, self.noise_std_m),
            yaw_rad=truth.yaw_rad + self._rng.gauss(0.0, self.heading_noise_std_rad),
            source_id=self.source_id,
            observation_id=f"{self.source_id}:{truth.stamp:.9f}",
            covariance=(self.noise_std_m * self.noise_std_m,) * 2,
        )
        return PoseObservation(truth.stamp, receive_stamp, pose, delayed=self.delay_s > 0.0)


@dataclass(frozen=True, slots=True)
class AlignedEstimate:
    stamp: float
    ego: Optional[VehicleState]
    opponent: Optional[OpponentBelief]
    gap_m: Optional[float]
    closing_speed_mps: Optional[float]
    epoch_id: int
    valid: bool
    ego_unwrapped_progress_m: Optional[float] = None
    ego_lateral_m: Optional[float] = None
    ego_heading_error_rad: Optional[float] = None
    reason: Optional[str] = None


class RaceEstimator:
    """Causal own/opponent estimator using only pose observations."""

    def __init__(self, track: TrackGeometry, run_id: str = "", max_extrapolation_s: float = 0.25, stale_after_s: float = 0.5, max_projection_error_m: float = 20.0, opponent_lap_offset: int = 0):
        if max_extrapolation_s < 0.0 or stale_after_s < 0.0 or max_projection_error_m < 0.0:
            raise ValueError("staleness limits must be non-negative")
        self.track = track
        self.run_id = run_id
        self.max_extrapolation_s = max_extrapolation_s
        self.stale_after_s = stale_after_s
        self.max_projection_error_m = max_projection_error_m
        if isinstance(opponent_lap_offset, bool) or not isinstance(opponent_lap_offset, int):
            raise ValueError("opponent_lap_offset must be an integer")
        self.opponent_lap_offset = opponent_lap_offset
        self.ego_buffer = PoseBuffer()
        self.opponent_buffer = PoseBuffer()
        self.epoch_id = 0
        self._ego_progress: Optional[float] = None
        self._opponent_progress: Optional[float] = None
        self._last_gap: Optional[Tuple[float, float]] = None
        self._last_query_stamp: Optional[float] = None
        self._reset_stamp: Optional[float] = None

    def _tag_epoch(self, sample: PoseSample) -> PoseSample:
        if sample.epoch_id is not None and sample.epoch_id != self.epoch_id:
            raise ValueError("sample epoch does not match estimator epoch")
        if self._reset_stamp is not None and sample.stamp < self._reset_stamp - 1e-12:
            raise ValueError("sample precedes reset time")
        if sample.epoch_id == self.epoch_id:
            return sample
        return PoseSample(sample.stamp, sample.x_m, sample.y_m, sample.yaw_rad, sample.source_id, sample.observation_id, sample.covariance, sample.received_stamp, sample.measurement_stamp, self.epoch_id)

    def reset(self, stamp: Optional[float] = None) -> None:
        if stamp is not None:
            stamp = _finite(stamp, "reset stamp")
        self.ego_buffer.clear()
        self.opponent_buffer.clear()
        self._ego_progress = None
        self._opponent_progress = None
        self._last_gap = None
        self._last_query_stamp = None
        self._reset_stamp = stamp
        self.epoch_id += 1

    def add_ego(self, sample: PoseSample) -> bool:
        try:
            return self.ego_buffer.add(self._tag_epoch(sample))
        except ValueError:
            return False

    def add_opponent_observation(self, observation: PoseObservation | PoseSample) -> bool:
        sample = observation.pose if isinstance(observation, PoseObservation) else observation
        if isinstance(observation, PoseObservation):
            sample = PoseSample(observation.source_stamp, sample.x_m, sample.y_m, sample.yaw_rad, sample.source_id, sample.observation_id, sample.covariance, observation.receive_stamp, observation.source_stamp, sample.epoch_id)
        try:
            return self.opponent_buffer.add(self._tag_epoch(sample))
        except ValueError:
            return False

    def _velocity(self, buffer: PoseBuffer, sample: PoseSample, stamp: float, progress_m: float):
        """Small-window least-squares velocity, projected onto track tangent."""
        points = [item for item in buffer.samples if item.stamp <= stamp and (item.received_stamp is None or item.received_stamp <= stamp) and item.epoch_id == self.epoch_id]
        if len(points) < 2:
            return None, None, None
        points = points[-6:]
        times = [item.stamp for item in points]
        mean_t = sum(times) / len(times)
        denom = sum((time - mean_t) ** 2 for time in times)
        if denom <= 1e-12:
            return None, None, None
        mean_x = sum(item.x_m for item in points) / len(points)
        mean_y = sum(item.y_m for item in points) / len(points)
        vx = sum((time - mean_t) * (item.x_m - mean_x) for time, item in zip(times, points)) / denom
        vy = sum((time - mean_t) * (item.y_m - mean_y) for time, item in zip(times, points)) / denom
        tangent = self.track.interpolate(progress_m).yaw_rad
        return vx * math.cos(tangent) + vy * math.sin(tangent), vx, vy

    def _progress_speed(self, buffer: PoseBuffer, stamp: float, current_progress: float) -> Optional[float]:
        """Fit continuous track progress, preserving seam/corner branches."""
        history = [item for item in buffer.samples if item.stamp <= stamp and (item.received_stamp is None or item.received_stamp <= stamp) and item.epoch_id == self.epoch_id][-6:]
        if len(history) < 2:
            return None
        values = []
        branch = current_progress
        for item in reversed(history):
            try:
                projection = self.track.project(item.x_m, item.y_m, previous_progress_m=branch, previous_heading_rad=None, reachable_window_m=self.track.length_m * 0.5, max_heading_error_rad=math.pi, max_distance_m=self.max_projection_error_m)
            except ValueError:
                continue
            branch = projection.unwrapped_progress_m
            values.append((item.stamp, branch))
        values.reverse()
        if len(values) < 2:
            return None
        mean_t = sum(item[0] for item in values) / len(values)
        mean_s = sum(item[1] for item in values) / len(values)
        denominator = sum((item[0] - mean_t) ** 2 for item in values)
        return sum((item[0] - mean_t) * (item[1] - mean_s) for item in values) / denominator if denominator > 1e-12 else None

    def estimate(self, stamp: float) -> AlignedEstimate:
        stamp = _finite(stamp, "stamp")
        if self._last_query_stamp is not None and stamp < self._last_query_stamp:
            raise ValueError("estimation query time moved backwards; reset required")
        self._last_query_stamp = stamp
        ego_pose = self.ego_buffer.at(stamp, self.max_extrapolation_s, self.epoch_id)
        opponent_pose = self.opponent_buffer.at(stamp, self.max_extrapolation_s, self.epoch_id)
        if ego_pose is None:
            return AlignedEstimate(stamp, None, None, None, None, self.epoch_id, False, reason="no_ego_pose")
        ego_measurement_stamp = ego_pose.measurement_stamp if ego_pose.measurement_stamp is not None else ego_pose.stamp
        if stamp - ego_measurement_stamp > self.stale_after_s:
            return AlignedEstimate(stamp, None, None, None, None, self.epoch_id, False, reason="ego_stale")
        try:
            ego_projection = self.track.project(ego_pose.x_m, ego_pose.y_m, self._ego_progress, ego_pose.yaw_rad, reachable_window_m=max(20.0, self.max_extrapolation_s * 100.0), max_heading_error_rad=math.pi * 0.75, max_distance_m=self.max_projection_error_m)
        except ValueError as error:
            return AlignedEstimate(stamp, None, None, None, None, self.epoch_id, False, reason=f"ego_projection:{error}")
        self._ego_progress = ego_projection.unwrapped_progress_m
        ego_tangent = self.track.interpolate(ego_projection.progress_m).yaw_rad
        ego_heading_error = (ego_pose.yaw_rad - ego_tangent + math.pi) % (2.0 * math.pi) - math.pi
        ego_speed, ego_world_vx, ego_world_vy = self._velocity(self.ego_buffer, ego_pose, stamp, ego_projection.progress_m)
        ego_track_speed = self._progress_speed(self.ego_buffer, stamp, self._ego_progress)
        ego_body_vx = None if ego_world_vx is None else math.cos(ego_pose.yaw_rad) * ego_world_vx + math.sin(ego_pose.yaw_rad) * ego_world_vy
        ego_body_vy = None if ego_world_vx is None else -math.sin(ego_pose.yaw_rad) * ego_world_vx + math.cos(ego_pose.yaw_rad) * ego_world_vy
        ego = VehicleState(ContractHeader(run_id=self.run_id, epoch_id=self.epoch_id, stamp=stamp, source=Source.ESTIMATED, validity=Validity.VALID), x_m=ego_pose.x_m, y_m=ego_pose.y_m, yaw_rad=ego_pose.yaw_rad, vx_mps=ego_body_vx, vy_mps=ego_body_vy, world_vx_mps=ego_world_vx, world_vy_mps=ego_world_vy, track_speed_mps=ego_track_speed, pose_valid=True, velocity_valid=ego_track_speed is not None)
        if opponent_pose is None:
            return AlignedEstimate(stamp, ego, None, None, None, self.epoch_id, False, ego_unwrapped_progress_m=self._ego_progress, ego_lateral_m=ego_projection.lateral_m, ego_heading_error_rad=ego_heading_error, reason="no_opponent_pose")
        opponent_measurement_stamp = opponent_pose.measurement_stamp if opponent_pose.measurement_stamp is not None else opponent_pose.stamp
        if stamp - opponent_measurement_stamp > self.stale_after_s:
            return AlignedEstimate(stamp, ego, None, None, None, self.epoch_id, False, ego_unwrapped_progress_m=self._ego_progress, ego_lateral_m=ego_projection.lateral_m, ego_heading_error_rad=ego_heading_error, reason="opponent_stale")
        try:
            opponent_projection = self.track.project(opponent_pose.x_m, opponent_pose.y_m, self._opponent_progress, opponent_pose.yaw_rad, reachable_window_m=max(20.0, self.max_extrapolation_s * 100.0), max_heading_error_rad=math.pi * 0.75, max_distance_m=self.max_projection_error_m)
        except ValueError as error:
            return AlignedEstimate(stamp, ego, None, None, None, self.epoch_id, False, ego_unwrapped_progress_m=self._ego_progress, ego_lateral_m=ego_projection.lateral_m, ego_heading_error_rad=ego_heading_error, reason=f"opponent_projection:{error}")
        if self._opponent_progress is None:
            self._opponent_progress = opponent_projection.progress_m + round((self._ego_progress - opponent_projection.progress_m) / self.track.length_m) * self.track.length_m + self.opponent_lap_offset * self.track.length_m
        else:
            self._opponent_progress = opponent_projection.unwrapped_progress_m
        opponent_speed = self._progress_speed(self.opponent_buffer, stamp, self._opponent_progress)
        gap = self._opponent_progress - self._ego_progress
        closing = None if ego_track_speed is None or opponent_speed is None else ego_track_speed - opponent_speed
        if closing is None and self._last_gap is not None:
            previous_gap, previous_stamp = self._last_gap
            dt = stamp - previous_stamp
            if dt > 1e-9:
                closing = -(gap - previous_gap) / dt
        self._last_gap = (gap, stamp)
        belief = OpponentBelief(ContractHeader(run_id=self.run_id, epoch_id=self.epoch_id, stamp=stamp, source=Source.ESTIMATED, validity=Validity.VALID), opponent_id=opponent_pose.source_id, x_m=opponent_pose.x_m, y_m=opponent_pose.y_m, yaw_rad=opponent_pose.yaw_rad, progress_m=opponent_projection.progress_m, unwrapped_progress_m=self._opponent_progress, speed_mps=opponent_speed, gap_m=gap, lateral_m=opponent_projection.lateral_m, closing_speed_mps=closing, freshness_s=max(0.0, stamp - opponent_measurement_stamp), covariance=opponent_pose.covariance, pose_source=Source.SYNTHETIC, pose_valid=True, speed_valid=opponent_speed is not None, gap_valid=True, identity_valid=bool(opponent_pose.source_id))
        return AlignedEstimate(stamp, ego, belief, gap, closing, self.epoch_id, True, self._ego_progress, ego_projection.lateral_m, ego_heading_error)


__all__ = ["PoseSample", "PoseObservation", "PoseBuffer", "SyntheticPoseSensor", "AlignedEstimate", "RaceEstimator"]
