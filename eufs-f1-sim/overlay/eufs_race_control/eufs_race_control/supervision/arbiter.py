"""Exclusive, bounded command arbitration for one car."""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Dict, Optional

from ..contracts import ContractHeader, HybridDriveStamped, Source, Validity


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


class CommandOwner(str, enum.Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    BRAKE = "brake"
    STOP = "stop"
    FALLBACK = "fallback"


@dataclass(frozen=True, slots=True)
class CommandLimits:
    min_acceleration_mps2: float = -12.0
    max_acceleration_mps2: float = 8.0
    max_steering_rad: float = 0.6458
    max_steering_rate_radps: float = 1.2916
    max_acceleration_rate_mps3: float = 50.0
    max_mguk_force_n: float = 10000.0
    stale_after_s: float = 0.25

    def __post_init__(self):
        for name in ("min_acceleration_mps2", "max_acceleration_mps2", "max_steering_rad", "max_steering_rate_radps", "max_acceleration_rate_mps3", "max_mguk_force_n", "stale_after_s"):
            value = _finite(getattr(self, name), name)
            if name != "min_acceleration_mps2" and value < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.min_acceleration_mps2 > self.max_acceleration_mps2:
            raise ValueError("acceleration limits are inverted")


@dataclass(frozen=True, slots=True)
class CommandRequest:
    header: ContractHeader
    owner: CommandOwner
    acceleration_mps2: float
    steering_rad: float
    requested_mguk_force_n: float = 0.0
    command_sequence: int = 0

    def __post_init__(self):
        if not isinstance(self.owner, CommandOwner):
            object.__setattr__(self, "owner", CommandOwner(self.owner))
        _finite(self.acceleration_mps2, "acceleration_mps2")
        _finite(self.steering_rad, "steering_rad")
        _finite(self.requested_mguk_force_n, "requested_mguk_force_n")
        if isinstance(self.command_sequence, bool) or not isinstance(self.command_sequence, int) or self.command_sequence < 0:
            raise ValueError("command_sequence must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class Readiness:
    car_id: str
    mission_ready: bool = False
    odometry_ready: bool = False
    estimator_ready: bool = False
    reset: bool = False
    reason: Optional[str] = None

    @property
    def ready(self) -> bool:
        return self.mission_ready and self.odometry_ready and self.estimator_ready and not self.reset


@dataclass(frozen=True, slots=True)
class ArbitrationResult:
    command: Optional[HybridDriveStamped]
    owner: Optional[CommandOwner]
    accepted: bool
    reason: str
    rejected: Dict[str, str] = field(default_factory=dict)


class CommandArbiter:
    """Select one fresh command, with stop/brake taking explicit priority."""

    _priority = {CommandOwner.STOP: 0, CommandOwner.BRAKE: 1, CommandOwner.MANUAL: 2, CommandOwner.AUTONOMOUS: 3, CommandOwner.FALLBACK: 4}

    def __init__(self, run_id: str = "", car_id: str = "eufs", limits: CommandLimits = CommandLimits()):
        self.run_id = run_id
        self.car_id = car_id
        self.limits = limits
        self.epoch_id = 0
        self._pending: Dict[CommandOwner, CommandRequest] = {}
        self._last: Optional[HybridDriveStamped] = None
        self._last_stamp: Optional[float] = None
        self.sequence = 0
        self._submitted_sequences: Dict[CommandOwner, int] = {}
        self._autonomy_latched = False
        self._stop_latched = False
        self._last_arbitration_stamp: Optional[float] = None

    def reset(self) -> None:
        self.epoch_id += 1
        self._pending.clear()
        self._last = None
        self._last_stamp = None
        self._submitted_sequences.clear()
        self._autonomy_latched = False
        self._stop_latched = False
        self._last_arbitration_stamp = None

    def submit(self, request: CommandRequest) -> bool:
        if request.header.epoch_id != self.epoch_id or request.header.run_id != self.run_id:
            return False
        if request.owner is CommandOwner.AUTONOMOUS and self._autonomy_latched:
            return False
        prior_sequence = self._submitted_sequences.get(request.owner, -1)
        if request.command_sequence <= prior_sequence:
            return False
        self._submitted_sequences[request.owner] = request.command_sequence
        if request.owner is CommandOwner.MANUAL:
            self._autonomy_latched = True
            self._pending.pop(CommandOwner.AUTONOMOUS, None)
        if request.owner is CommandOwner.STOP:
            self._stop_latched = True
        self._pending[request.owner] = request
        return True

    def rearm_autonomy(self) -> None:
        """Explicitly allow autonomous requests after manual takeover."""
        self._autonomy_latched = False
        self._pending.pop(CommandOwner.MANUAL, None)

    def release_stop(self) -> None:
        """Release a latched stop only through an explicit supervisory action."""
        self._stop_latched = False
        self._pending.pop(CommandOwner.STOP, None)

    def _fresh(self, request: CommandRequest, now: float) -> tuple[bool, str]:
        if request.header.epoch_id != self.epoch_id:
            return False, "epoch_mismatch"
        if request.header.validity is not Validity.VALID:
            return False, "invalid"
        age = now - request.header.stamp
        if age < 0.0:
            return False, "future_command"
        if age > self.limits.stale_after_s or not request.header.is_fresh(now):
            return False, "stale_or_expired"
        return True, ""

    def _bounded(self, request: CommandRequest, now: float, hard_brake: bool = False) -> HybridDriveStamped:
        acceleration = min(self.limits.max_acceleration_mps2, max(self.limits.min_acceleration_mps2, request.acceleration_mps2))
        if hard_brake:
            acceleration = self.limits.min_acceleration_mps2 if request.owner in (CommandOwner.STOP, CommandOwner.BRAKE) else min(0.0, acceleration)
        steering = min(self.limits.max_steering_rad, max(-self.limits.max_steering_rad, request.steering_rad))
        if self._last is not None and self._last_stamp is not None and not hard_brake:
            dt = max(0.0, now - self._last_stamp)
            maximum_delta = self.limits.max_steering_rate_radps * dt
            steering = min(self._last.steering_rad + maximum_delta, max(self._last.steering_rad - maximum_delta, steering))
            acceleration_delta = self.limits.max_acceleration_rate_mps3 * dt
            acceleration = min(self._last.acceleration_mps2 + acceleration_delta, max(self._last.acceleration_mps2 - acceleration_delta, acceleration))
        force = 0.0 if hard_brake else min(self.limits.max_mguk_force_n, max(-self.limits.max_mguk_force_n, request.requested_mguk_force_n))
        self.sequence += 1
        header = ContractHeader(run_id=self.run_id, epoch_id=self.epoch_id, stamp=now, source=Source.COMMAND, snapshot_id=request.header.snapshot_id, decision_id=request.header.decision_id, trajectory_id=request.header.trajectory_id, expires_at=now + self.limits.stale_after_s)
        return HybridDriveStamped(header, acceleration, steering, force, self.sequence, request.owner.value)

    def _fallback_request(self, now: float, measured_speed_mps: Optional[float]) -> CommandRequest:
        if measured_speed_mps is None:
            acceleration = self.limits.min_acceleration_mps2
        else:
            speed = max(0.0, _finite(measured_speed_mps, "measured_speed_mps"))
            acceleration = -min(abs(self.limits.min_acceleration_mps2), speed / 0.5)
        steering = self._last.steering_rad if self._last else 0.0
        return CommandRequest(ContractHeader(run_id=self.run_id, epoch_id=self.epoch_id, stamp=now, source=Source.COMMAND, expires_at=now + self.limits.stale_after_s), CommandOwner.FALLBACK, acceleration, steering)

    def arbitrate(self, now: float, readiness: Optional[Readiness] = None, measured_speed_mps: Optional[float] = None) -> ArbitrationResult:
        now = _finite(now, "now")
        if self._last_arbitration_stamp is not None and now < self._last_arbitration_stamp:
            self.reset()
            return ArbitrationResult(None, None, False, "time_reversed_reset", {"arbiter": "time_reversed"})
        self._last_arbitration_stamp = now
        rejected: Dict[str, str] = {}
        candidates = []
        if self._stop_latched:
            request = self._pending.get(CommandOwner.STOP)
            if request is None:
                request = CommandRequest(ContractHeader(run_id=self.run_id, epoch_id=self.epoch_id, stamp=now, source=Source.COMMAND, expires_at=now + self.limits.stale_after_s), CommandOwner.STOP, self.limits.min_acceleration_mps2, self._last.steering_rad if self._last else 0.0)
            command = self._bounded(request, now, hard_brake=True)
            self._last, self._last_stamp = command, now
            return ArbitrationResult(command, CommandOwner.STOP, True, "stop_latched", {})
        for owner, request in self._pending.items():
            fresh, reason = self._fresh(request, now)
            if not fresh:
                rejected[owner.value] = reason
            elif owner in (CommandOwner.AUTONOMOUS, CommandOwner.FALLBACK):
                if readiness is None:
                    rejected[owner.value] = "readiness_required"
                elif readiness.car_id != self.car_id:
                    rejected[owner.value] = "readiness_car_mismatch"
                elif not readiness.ready:
                    rejected[owner.value] = readiness.reason or "not_ready"
                else:
                    candidates.append(request)
            else:
                candidates.append(request)
        if not candidates:
            # A missing/stale stream always produces a bounded braking command;
            # leaving a previous positive command active would delegate safety
            # to the native timeout.
            request = self._fallback_request(now, measured_speed_mps)
            command = self._bounded(request, now, hard_brake=True)
            self._last, self._last_stamp = command, now
            return ArbitrationResult(command, CommandOwner.FALLBACK, True, "bounded_braking_fallback", rejected)
        request = min(candidates, key=lambda item: self._priority[item.owner])
        hard_brake = request.owner in (CommandOwner.BRAKE, CommandOwner.STOP) or request.acceleration_mps2 <= 0.0 and request.owner is CommandOwner.FALLBACK
        command = self._bounded(request, now, hard_brake=hard_brake)
        self._last, self._last_stamp = command, now
        return ArbitrationResult(command, request.owner, True, "selected", rejected)

    def stop(self, now: float, readiness: Optional[Readiness] = None, measured_speed_mps: Optional[float] = None) -> ArbitrationResult:
        steering = self._last.steering_rad if self._last is not None else 0.0
        header = ContractHeader(run_id=self.run_id, epoch_id=self.epoch_id, stamp=now, source=Source.COMMAND, expires_at=now + self.limits.stale_after_s)
        request = CommandRequest(header, CommandOwner.STOP, self.limits.min_acceleration_mps2, steering, 0.0, self.sequence + 1)
        self.submit(request)
        return self.arbitrate(now, readiness, measured_speed_mps)


ControlSupervisor = CommandArbiter

__all__ = ["CommandOwner", "CommandLimits", "CommandRequest", "Readiness", "ArbitrationResult", "CommandArbiter", "ControlSupervisor"]
