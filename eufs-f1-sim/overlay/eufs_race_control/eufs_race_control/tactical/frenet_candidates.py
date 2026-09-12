"""Causal 5 Hz Frenet follow/left/right candidate generation."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from ..geometry.track import Footprint, TrackGeometry
from ..geometry.collision import swept_footprint_collision


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    lateral_m: float
    speed_mps: float
    feasible: bool
    reason: str
    points: tuple[tuple[float, float, float], ...]


class TacticalPlanner:
    def __init__(self, track: TrackGeometry | None = None, *, lane_width_m: float = 3.0,
                 footprint: Footprint = Footprint()):
        self.track = track
        self.lane_width_m = float(lane_width_m)
        self.footprint = footprint
        self._held_side: float | None = None

    @staticmethod
    def constant_velocity(history: Sequence[tuple[float, float, float, float]]) -> tuple[float, float]:
        """Estimate opponent map velocity from causal pose history only."""
        if len(history) < 2:
            return 0.0, 0.0
        first, last = history[-2], history[-1]
        dt = max(1.0e-6, float(last[0]) - float(first[0]))
        return ((last[1] - first[1]) / dt, (last[2] - first[2]) / dt)

    def candidates(self, *, progress_m: float, speed_mps: float,
                   opponent_pose: tuple[float, float, float] | None = None,
                   opponent_velocity_mps: tuple[float, float] = (0.0, 0.0),
                   horizon_s: float = 3.0, hold_side: float | None = None) -> tuple[Candidate, ...]:
        if self.track is None:
            return (Candidate("follow", 0.0, speed_mps, True, "track_unavailable", ()),)
        sides = (0.0,) if hold_side is None else (float(hold_side),)
        if hold_side is None:
            sides = (0.0, -self.lane_width_m, self.lane_width_m)
        result: list[Candidate] = []
        for lateral in sides:
            identifier = "follow" if lateral == 0.0 else ("left" if lateral < 0.0 else "right")
            points = tuple(self._point(progress_m + speed_mps * 0.5 * i, lateral)
                           for i in range(7))
            feasible = True
            reason = "corridor_and_separation_ok"
            try:
                width = self.track.width_at(progress_m, default_half_width_m=7.0)
                feasible = abs(lateral) + self.footprint.halfwidth_m <= width - 0.1
            except ValueError:
                feasible = False
                reason = "track_width_unavailable"
            if feasible and opponent_pose is not None and points:
                opponent_end = (opponent_pose[0] + opponent_velocity_mps[0] * horizon_s,
                                opponent_pose[1] + opponent_velocity_mps[1] * horizon_s,
                                opponent_pose[2])
                if swept_footprint_collision(points[0], points[-1], opponent_pose,
                                             self.footprint, other_start=opponent_pose,
                                             other_end=opponent_end, clearance_m=0.2):
                    feasible = False
                    reason = "full_footprint_separation_failed"
            result.append(Candidate(identifier, lateral, speed_mps, feasible, reason, points))
        return tuple(result)

    def choose(self, candidates: Sequence[Candidate], *, action: str,
               current_lateral_m: float = 0.0) -> Candidate | None:
        feasible = [candidate for candidate in candidates if candidate.feasible]
        if not feasible:
            return None
        if action == "attack":
            chosen = min(feasible, key=lambda item: (abs(item.lateral_m - current_lateral_m), item.candidate_id == "follow"))
            if chosen.candidate_id != "follow":
                self._held_side = chosen.lateral_m
            return chosen
        # Keep a committed side until the opponent is fully clear; callers
        # can release it by passing a follow action after clearance.
        if self._held_side is not None:
            held = [item for item in feasible if abs(item.lateral_m - self._held_side) < 1e-9]
            if held:
                return held[0]
            self._held_side = None
        return min(feasible, key=lambda item: (abs(item.lateral_m - current_lateral_m), item.candidate_id != "follow"))

    def _point(self, progress_m: float, lateral_m: float) -> tuple[float, float, float]:
        point = self.track.frenet_to_map(progress_m, lateral_m)
        return point.x_m, point.y_m, point.yaw_rad
