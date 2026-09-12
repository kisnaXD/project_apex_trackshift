"""Simulation-clock driven, exact kinematic replay state.

The replay owns only the frozen opponent schedule.  It has no command input,
vehicle model, battery state, or dependency on ego state.  A caller supplies
the authoritative Gazebo simulation timestamp and receives the one pose that
is valid at that timestamp.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from .reference import FrozenReference


class ReplayClockError(RuntimeError):
    """The replay clock needs an explicit reset before it can advance."""


@dataclass(frozen=True)
class ReplayPose:
    sim_time_s: float
    reference_time_s: float
    epoch_id: int
    lap_index: int
    completed: bool
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    curvature_1pm: float
    acceleration_mps2: float

    def observation(self) -> Mapping[str, Any]:
        """Return the timestamped operational pose observation.

        The observation deliberately has no future samples, reference hash or
        direct schedule handle.  Evaluators can retain :class:`ReplayPose`
        while the planner receives this bounded pose-only mapping.
        """

        return {
            "sim_time_s": self.sim_time_s,
            "epoch_id": self.epoch_id,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "yaw_rad": self.yaw_rad,
        }


class ExactKinematicReplay:
    """Advance a frozen reference from simulation time and run epoch only."""

    def __init__(self, reference: FrozenReference, *, run_id: str = "", loop: bool = False) -> None:
        if not isinstance(reference, FrozenReference):
            raise TypeError("reference must be a FrozenReference")
        self.reference = reference
        self.run_id = str(run_id)
        self.loop = bool(loop)
        self._duration = float(reference.artifact["samples"][-1]["time_s"])
        if not math.isfinite(self._duration) or self._duration <= 0.0:
            raise ReplayClockError("reference duration must be positive")
        self._epoch_id = 0
        self._phase_s = 0.0
        self._last_sim_s: float | None = None
        self._paused = False
        self._invalid = False
        self._completed = False
        self._lap_index = 0
        self._last: ReplayPose | None = None

    @property
    def epoch_id(self) -> int:
        return self._epoch_id

    @property
    def duration_s(self) -> float:
        return self._duration

    @property
    def phase_s(self) -> float:
        return self._phase_s

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def valid(self) -> bool:
        return not self._invalid

    @property
    def completed(self) -> bool:
        return self._completed

    def reset(self, sim_time_s: float = 0.0, *, epoch_id: int | None = None, reference_phase_s: float = 0.0) -> ReplayPose:
        sim_time_s = self._finite(sim_time_s, "sim_time_s")
        reference_phase_s = self._finite(reference_phase_s, "reference_phase_s")
        if reference_phase_s < 0.0 or reference_phase_s > self._duration:
            raise ReplayClockError("reference_phase_s is outside the frozen lap")
        if epoch_id is None:
            epoch_id = self._epoch_id + 1
        if isinstance(epoch_id, bool) or not isinstance(epoch_id, int) or epoch_id < 0:
            raise ReplayClockError("epoch_id must be a non-negative integer")
        self._epoch_id = epoch_id
        self._phase_s = reference_phase_s
        self._last_sim_s = sim_time_s
        self._paused = False
        self._invalid = False
        self._completed = False
        self._lap_index = 0
        self._last = self._make_pose(sim_time_s, reference_phase_s, completed=False, lap_index=0)
        return self._last

    def set_paused(self, paused: bool) -> None:
        self._paused = bool(paused)

    def pause(self) -> None:
        self.set_paused(True)

    def resume(self) -> None:
        self.set_paused(False)

    def advance(self, sim_time_s: float, *, epoch_id: int | None = None) -> ReplayPose:
        """Return the exact pose at ``sim_time_s``.

        A backwards jump is recorded as invalid and remains rejected until
        :meth:`reset` is called.  At the endpoint a non-looping run clamps to
        the final knot and reports explicit one-lap completion.
        """

        sim_time_s = self._finite(sim_time_s, "sim_time_s")
        if self._last_sim_s is None:
            return self.reset(sim_time_s, epoch_id=epoch_id)
        if self._invalid:
            raise ReplayClockError("replay clock is invalid; call reset explicitly")
        if epoch_id is not None and (isinstance(epoch_id, bool) or not isinstance(epoch_id, int) or epoch_id != self._epoch_id):
            self._invalid = True
            raise ReplayClockError("simulation epoch changed without explicit reset")
        if sim_time_s < self._last_sim_s:
            self._invalid = True
            raise ReplayClockError("simulation time moved backwards; call reset explicitly")
        elapsed = sim_time_s - self._last_sim_s
        self._last_sim_s = sim_time_s
        if self._paused or self._completed and not self.loop:
            self._last = self._make_pose(sim_time_s, self._phase_s, completed=self._completed, lap_index=self._lap_index)
            return self._last
        schedule_time = self._phase_s + elapsed
        if self.loop:
            lap_increment = int(schedule_time // self._duration)
            phase = schedule_time - lap_increment * self._duration
            # Keep an exact endpoint at integer lap boundaries rather than
            # exposing a tiny post-seam value from floating-point division.
            if phase == 0.0 and schedule_time > 0.0:
                phase = self._duration
                lap_increment -= 1
            self._lap_index += max(0, lap_increment)
            self._phase_s = phase
            self._completed = self._lap_index > 0 or self._phase_s >= self._duration
            self._last = self._make_pose(sim_time_s, phase, completed=self._completed, lap_index=self._lap_index)
            return self._last
        self._phase_s = min(self._duration, schedule_time)
        self._completed = self._phase_s >= self._duration
        self._last = self._make_pose(sim_time_s, self._phase_s, completed=self._completed, lap_index=0)
        return self._last

    # Common names used by adapters and tests.
    update = advance
    step = advance

    def truth_pose(self, sim_time_s: float, *, epoch_id: int | None = None) -> ReplayPose:
        """Return evaluator truth while advancing the same causal clock."""

        return self.advance(sim_time_s, epoch_id=epoch_id)

    def _make_pose(self, sim_time_s: float, reference_time_s: float, *, completed: bool, lap_index: int) -> ReplayPose:
        sample = self.reference.time_sample(reference_time_s)
        return ReplayPose(
            sim_time_s=sim_time_s,
            reference_time_s=reference_time_s,
            epoch_id=self._epoch_id,
            lap_index=lap_index,
            completed=completed,
            x_m=float(sample["x_m"]),
            y_m=float(sample["y_m"]),
            yaw_rad=float(sample["yaw_rad"]),
            speed_mps=float(sample["speed_mps"]),
            curvature_1pm=float(sample["curvature_1pm"]),
            acceleration_mps2=float(sample.get("acceleration_mps2", 0.0)),
        )

    @staticmethod
    def _finite(value: float, name: str) -> float:
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ReplayClockError(f"{name} must be finite") from exc
        if not math.isfinite(value):
            raise ReplayClockError(f"{name} must be finite")
        return value


ReplayClock = ExactKinematicReplay
KinematicReplay = ExactKinematicReplay
RussellReplay = ExactKinematicReplay

__all__ = ["ReplayClockError", "ReplayPose", "ExactKinematicReplay", "ReplayClock", "KinematicReplay", "RussellReplay"]
