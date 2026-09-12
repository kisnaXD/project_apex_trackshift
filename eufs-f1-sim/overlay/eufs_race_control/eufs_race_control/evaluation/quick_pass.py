"""Bounded truth scoring for the first two-car pass demonstration.

This evaluator consumes poses from the simulator and the frozen opponent's
truth stream.  It does not expose replay samples, speed, or future positions to
the operational planner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

from ..geometry.collision import footprints_collide, swept_collision
from ..geometry.track import Footprint, TrackGeometry


def _pose(value: Sequence[float] | Mapping[str, float], name: str) -> tuple[float, float, float]:
    if isinstance(value, Mapping):
        try:
            result = (float(value["x_m"]), float(value["y_m"]), float(value["yaw_rad"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{name} must contain x_m, y_m and yaw_rad") from exc
    else:
        if len(value) != 3:
            raise ValueError(f"{name} must be (x_m, y_m, yaw_rad)")
        try:
            result = tuple(float(item) for item in value)  # type: ignore[assignment]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must contain finite numeric values") from exc
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite numeric values")
    return result


@dataclass(frozen=True, slots=True)
class QuickPassResult:
    sim_time_s: float
    ego_unwrapped_progress_m: float | None
    opponent_unwrapped_progress_m: float | None
    gap_m: float | None
    overlap: bool
    ego_track_violation: bool
    opponent_track_violation: bool
    pass_complete: bool
    hold_s: float
    update_count: int
    overlap_count: int
    track_violation_count: int
    reset: bool
    valid: bool
    reason: str

    @property
    def track_violation(self) -> bool:
        return self.ego_track_violation or self.opponent_track_violation

    def as_dict(self) -> dict[str, object]:
        return {
            "sim_time_s": self.sim_time_s,
            "ego_unwrapped_progress_m": self.ego_unwrapped_progress_m,
            "opponent_unwrapped_progress_m": self.opponent_unwrapped_progress_m,
            "gap_m": self.gap_m,
            "overlap": self.overlap,
            "ego_track_violation": self.ego_track_violation,
            "opponent_track_violation": self.opponent_track_violation,
            "pass_complete": self.pass_complete,
            "hold_s": self.hold_s,
            "update_count": self.update_count,
            "overlap_count": self.overlap_count,
            "track_violation_count": self.track_violation_count,
            "reset": self.reset,
            "valid": self.valid,
            "reason": self.reason,
        }


class QuickPassEvaluator:
    """Evaluate one bounded pass attempt from current and previous truth poses."""

    def __init__(self, track: TrackGeometry, footprint: Footprint | None = None, clearance_m: float = 1.0, hold_s: float = 2.0):
        if not isinstance(track, TrackGeometry):
            raise TypeError("track must be a TrackGeometry")
        self.track = track
        self.footprint = footprint or Footprint()
        self.clearance_m = float(clearance_m)
        self.required_hold_s = float(hold_s)
        if not math.isfinite(self.clearance_m) or self.clearance_m < 0.0:
            raise ValueError("clearance_m must be finite and non-negative")
        if not math.isfinite(self.required_hold_s) or self.required_hold_s <= 0.0:
            raise ValueError("hold_s must be finite and positive")
        self.reset()

    def reset(self) -> None:
        self._last_time: float | None = None
        self._last_ego_pose: tuple[float, float, float] | None = None
        self._last_opponent_pose: tuple[float, float, float] | None = None
        self._ego_progress: float | None = None
        self._opponent_progress: float | None = None
        self._hold_s = 0.0
        self._pass_complete = False
        self._updates = 0
        self._overlap_count = 0
        self._track_violation_count = 0

    def update(self, sim_time_s: float, ego_pose: Sequence[float] | Mapping[str, float], opponent_pose: Sequence[float] | Mapping[str, float]) -> QuickPassResult:
        sim_time_s = float(sim_time_s)
        if not math.isfinite(sim_time_s):
            raise ValueError("sim_time_s must be finite")
        ego = _pose(ego_pose, "ego_pose")
        opponent = _pose(opponent_pose, "opponent_pose")
        if self._last_time is not None and sim_time_s < self._last_time:
            self.reset()
            self._last_time = sim_time_s
            self._last_ego_pose, self._last_opponent_pose = ego, opponent
            return self._result(sim_time_s, reset=True, reason="simulation_time_reset")
        dt = 0.0 if self._last_time is None else sim_time_s - self._last_time
        try:
            ego_projection = self.track.project(
                ego[0], ego[1], previous_progress_m=self._ego_progress,
                previous_heading_rad=ego[2], reachable_window_m=max(80.0, self.track.length_m * 0.1),
                max_heading_error_rad=math.pi, max_distance_m=None,
            )
            opponent_projection = self.track.project(
                opponent[0], opponent[1], previous_progress_m=self._opponent_progress,
                previous_heading_rad=opponent[2], reachable_window_m=max(80.0, self.track.length_m * 0.1),
                max_heading_error_rad=math.pi, max_distance_m=None,
            )
        except ValueError as exc:
            self._last_time = sim_time_s
            self._last_ego_pose, self._last_opponent_pose = ego, opponent
            return self._result(sim_time_s, reason=f"projection_invalid:{exc}")
        if self._ego_progress is None and self._opponent_progress is None:
            # Put both cars on the same initial lap branch.  This keeps the
            # first back-straight comparison causal and avoids a ±lap gap.
            opponent_unwrapped = opponent_projection.unwrapped_progress_m + round(
                (ego_projection.unwrapped_progress_m - opponent_projection.unwrapped_progress_m) / self.track.length_m
            ) * self.track.length_m
        else:
            opponent_unwrapped = opponent_projection.unwrapped_progress_m
        self._ego_progress = ego_projection.unwrapped_progress_m
        self._opponent_progress = opponent_unwrapped
        overlap = False
        if self._last_ego_pose is not None and self._last_opponent_pose is not None:
            overlap = swept_collision(
                self._last_ego_pose, ego, self._last_opponent_pose, opponent,
                self.footprint, clearance_m=0.0,
            )
        else:
            overlap = footprints_collide(ego, opponent, self.footprint)
        ego_violation = self._body_outside_track(ego)
        opponent_violation = self._body_outside_track(opponent)
        if overlap:
            self._overlap_count += 1
        if ego_violation or opponent_violation:
            self._track_violation_count += 1
        gap = self._opponent_progress - self._ego_progress
        clear_ahead = (
            self._ego_progress - self.footprint.rear_m
            >= self._opponent_progress + self.footprint.front_m + self.clearance_m
        )
        if clear_ahead and not overlap and not ego_violation and not opponent_violation:
            self._hold_s += max(0.0, dt)
        else:
            self._hold_s = 0.0
        if self._hold_s >= self.required_hold_s:
            self._pass_complete = True
        self._last_time = sim_time_s
        self._last_ego_pose, self._last_opponent_pose = ego, opponent
        self._updates += 1
        reason = "pass_complete" if self._pass_complete else ("overlap" if overlap else ("track_violation" if ego_violation or opponent_violation else "running"))
        return self._result(sim_time_s, overlap=overlap, ego_violation=ego_violation, opponent_violation=opponent_violation, reason=reason)

    def _body_outside_track(self, pose: tuple[float, float, float]) -> bool:
        try:
            for x_m, y_m in self.footprint.corners(*pose):
                projection = self.track.project(x_m, y_m, max_distance_m=None)
                half_width = self.track.width_at(projection.progress_m, default_half_width_m=7.0)
                if abs(projection.lateral_m) > half_width + 1e-9:
                    return True
        except ValueError:
            return True
        return False

    def _result(self, sim_time_s: float, *, overlap: bool = False, ego_violation: bool = False, opponent_violation: bool = False, reset: bool = False, reason: str = "running") -> QuickPassResult:
        return QuickPassResult(
            sim_time_s=sim_time_s,
            ego_unwrapped_progress_m=self._ego_progress,
            opponent_unwrapped_progress_m=self._opponent_progress,
            gap_m=None if self._ego_progress is None or self._opponent_progress is None else self._opponent_progress - self._ego_progress,
            overlap=overlap,
            ego_track_violation=ego_violation,
            opponent_track_violation=opponent_violation,
            pass_complete=self._pass_complete,
            hold_s=self._hold_s,
            update_count=self._updates,
            overlap_count=self._overlap_count,
            track_violation_count=self._track_violation_count,
            reset=reset,
            valid=self._ego_progress is not None and self._opponent_progress is not None,
            reason=reason,
        )


__all__ = ["QuickPassEvaluator", "QuickPassResult"]
