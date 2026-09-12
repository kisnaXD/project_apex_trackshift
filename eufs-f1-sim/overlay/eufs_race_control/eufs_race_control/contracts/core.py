"""Immutable, ROS-independent control contracts.

All internal quantities use SI units.  ``None`` means unavailable and is kept
as JSON ``null``; it is never silently replaced by a numeric sentinel.
"""
from __future__ import annotations

import enum
import json
import math
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from typing import Any, Mapping, Optional, Sequence, Tuple

SCHEMA_VERSION = "1.0"
ACCELERATION_SEMANTICS = "tyre_force_divided_by_mass_before_drag"


def finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def optional_finite(value: Optional[float], name: str) -> Optional[float]:
    return None if value is None else finite(value, name)


def nonnegative(value: float, name: str) -> float:
    value = finite(value, name)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


def _clean(value: Any) -> Any:
    if is_dataclass(value):
        return {item.name: _clean(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, enum.Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_clean(item) for item in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return value
    return value


class Validity(str, enum.Enum):
    VALID = "valid"
    INVALID = "invalid"
    STALE = "stale"
    RESET = "reset"
    UNAVAILABLE = "unavailable"


class Source(str, enum.Enum):
    MEASURED = "measured"
    ESTIMATED = "estimated"
    SYNTHETIC = "synthetic"
    TRUTH = "truth"
    COMMAND = "command"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True, slots=True)
class ContractHeader:
    run_id: str = ""
    epoch_id: int = 0
    stamp: float = 0.0
    schema_version: str = SCHEMA_VERSION
    snapshot_id: Optional[str] = None
    decision_id: Optional[str] = None
    trajectory_id: Optional[str] = None
    expires_at: Optional[float] = None
    validity: Validity = Validity.VALID
    source: Source = Source.ESTIMATED
    frame_id: Optional[str] = None
    map_id: Optional[str] = None
    validity_reason: Optional[str] = None
    known_fields: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.run_id, str) or not isinstance(self.schema_version, str):
            raise TypeError("run_id and schema_version must be strings")
        if isinstance(self.epoch_id, bool) or not isinstance(self.epoch_id, int) or self.epoch_id < 0:
            raise ValueError("epoch_id must be a non-negative integer")
        if not isinstance(self.validity, Validity):
            object.__setattr__(self, "validity", Validity(self.validity))
        if not isinstance(self.source, Source):
            object.__setattr__(self, "source", Source(self.source))
        finite(self.stamp, "stamp")
        optional_finite(self.expires_at, "expires_at")
        if not isinstance(self.known_fields, tuple):
            object.__setattr__(self, "known_fields", tuple(self.known_fields))
        if self.expires_at is not None and self.expires_at < self.stamp:
            raise ValueError("expires_at cannot precede stamp")

    @property
    def available(self) -> bool:
        return self.validity is Validity.VALID

    def is_fresh(self, now: float) -> bool:
        finite(now, "now")
        return self.available and now >= self.stamp and (self.expires_at is None or now <= self.expires_at)


@dataclass(frozen=True, slots=True)
class VehicleState:
    header: ContractHeader = field(default_factory=ContractHeader)
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    z_m: Optional[float] = None
    yaw_rad: Optional[float] = None
    vx_mps: Optional[float] = None
    vy_mps: Optional[float] = None
    world_vx_mps: Optional[float] = None
    world_vy_mps: Optional[float] = None
    vz_mps: Optional[float] = None
    ax_mps2: Optional[float] = None
    ay_mps2: Optional[float] = None
    yaw_rate_rps: Optional[float] = None
    track_speed_mps: Optional[float] = None
    lateral_speed_mps: Optional[float] = None
    steering_rad: Optional[float] = None
    covariance: Tuple[float, ...] = ()
    pose_valid: bool = False
    velocity_valid: bool = False
    acceleration_valid: bool = False
    steering_valid: bool = False

    def __post_init__(self) -> None:
        for name in fields(self):
            if name.name not in ("header", "covariance", "pose_valid", "velocity_valid", "acceleration_valid", "steering_valid"):
                optional_finite(getattr(self, name.name), name.name)
        if not isinstance(self.covariance, tuple):
            object.__setattr__(self, "covariance", tuple(self.covariance))
        if len(self.covariance) not in (0, 6, 36):
            raise ValueError("covariance must be empty, 6 or 36 elements")
        for value in self.covariance:
            finite(value, "covariance")
        if self.pose_valid and (self.x_m is None or self.y_m is None or self.yaw_rad is None):
            raise ValueError("pose_valid requires x_m, y_m and yaw_rad")
        if self.velocity_valid and self.vx_mps is None and self.track_speed_mps is None:
            raise ValueError("velocity_valid requires a velocity field")


@dataclass(frozen=True, slots=True)
class EnergyState:
    header: ContractHeader = field(default_factory=ContractHeader)
    stored_energy_j: Optional[float] = None
    usable_energy_j: Optional[float] = None
    reserve_energy_j: Optional[float] = None
    signed_delta_energy_j: Optional[float] = None
    electrical_power_w: Optional[float] = None
    battery_current_a: Optional[float] = None
    battery_voltage_v: Optional[float] = None
    temperature_k: Optional[float] = None
    grip_scale: Optional[float] = None
    max_drive_force_n: Optional[float] = None
    max_regen_force_n: Optional[float] = None
    capacity_j: Optional[float] = None
    soc: Optional[float] = None
    available_deploy_power_w: Optional[float] = None
    available_recovery_power_w: Optional[float] = None
    delivered_deploy_power_w: Optional[float] = None
    delivered_recovery_power_w: Optional[float] = None
    signed_power_w: Optional[float] = None
    per_lap_deployed_j: Optional[float] = None
    per_lap_recovered_j: Optional[float] = None
    max_temperature_k: Optional[float] = None
    derate_reason: Optional[str] = None
    stored_energy_valid: bool = False
    usable_energy_valid: bool = False
    signed_delta_valid: bool = False
    electrical_power_valid: bool = False
    battery_current_valid: bool = False
    battery_voltage_valid: bool = False
    temperature_valid: bool = False
    grip_valid: bool = False
    limits_valid: bool = False

    def __post_init__(self) -> None:
        for name in fields(self):
            if name.name not in ("header", "derate_reason") and not name.name.endswith("_valid"):
                optional_finite(getattr(self, name.name), name.name)
        for name in ("stored_energy_j", "usable_energy_j", "reserve_energy_j", "temperature_k", "capacity_j", "max_temperature_k", "per_lap_deployed_j", "per_lap_recovered_j"):
            value = getattr(self, name)
            if value is not None and value < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.grip_scale is not None and self.grip_scale < 0.0:
            raise ValueError("grip_scale must be non-negative")
        if self.soc is not None and not 0.0 <= self.soc <= 1.0:
            raise ValueError("soc must be in [0, 1]")
        validity_pairs = (("stored_energy_j", "stored_energy_valid"), ("usable_energy_j", "usable_energy_valid"), ("signed_delta_energy_j", "signed_delta_valid"), ("electrical_power_w", "electrical_power_valid"), ("battery_current_a", "battery_current_valid"), ("battery_voltage_v", "battery_voltage_valid"), ("temperature_k", "temperature_valid"), ("grip_scale", "grip_valid"))
        for value_name, valid_name in validity_pairs:
            if getattr(self, valid_name) and getattr(self, value_name) is None:
                raise ValueError(f"{valid_name} cannot be true when {value_name} is unavailable")
        if self.limits_valid and self.max_drive_force_n is None and self.max_regen_force_n is None:
            raise ValueError("limits_valid requires at least one force limit")


@dataclass(frozen=True, slots=True)
class OpponentBelief:
    header: ContractHeader = field(default_factory=ContractHeader)
    opponent_id: str = ""
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    yaw_rad: Optional[float] = None
    progress_m: Optional[float] = None
    unwrapped_progress_m: Optional[float] = None
    speed_mps: Optional[float] = None
    gap_m: Optional[float] = None
    lateral_m: Optional[float] = None
    predicted_exit_gap_m: Optional[float] = None
    closing_speed_mps: Optional[float] = None
    identity_confidence: Optional[float] = None
    freshness_s: Optional[float] = None
    covariance: Tuple[float, ...] = ()
    pose_source: Source = Source.SYNTHETIC
    pose_valid: bool = False
    speed_valid: bool = False
    gap_valid: bool = False
    identity_valid: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.covariance, tuple):
            object.__setattr__(self, "covariance", tuple(self.covariance))
        for name in fields(self):
            if name.name not in ("header", "opponent_id", "covariance", "pose_source", "pose_valid", "speed_valid", "gap_valid", "identity_valid"):
                optional_finite(getattr(self, name.name), name.name)
        if not isinstance(self.pose_source, Source):
            object.__setattr__(self, "pose_source", Source(self.pose_source))
        if self.identity_confidence is not None and not 0.0 <= self.identity_confidence <= 1.0:
            raise ValueError("identity_confidence must be in [0, 1]")
        if self.freshness_s is not None and self.freshness_s < 0.0:
            raise ValueError("freshness_s must be non-negative")
        for value in self.covariance:
            finite(value, "covariance")
        if self.pose_valid and (self.x_m is None or self.y_m is None):
            raise ValueError("pose_valid requires x_m and y_m")
        if self.speed_valid and self.speed_mps is None:
            raise ValueError("speed_valid requires speed_mps")
        if self.gap_valid and self.gap_m is None:
            raise ValueError("gap_valid requires gap_m")


@dataclass(frozen=True, slots=True)
class EstimatedRaceState:
    header: ContractHeader = field(default_factory=ContractHeader)
    ego: VehicleState = field(default_factory=VehicleState)
    energy: EnergyState = field(default_factory=EnergyState)
    opponent: OpponentBelief = field(default_factory=OpponentBelief)
    tyres: "TyreState" = field(default_factory=lambda: TyreState())
    progress_m: Optional[float] = None
    unwrapped_progress_m: Optional[float] = None
    lap: int = 0
    track_length_m: Optional[float] = None
    rules_id: Optional[str] = None
    scenario_id: Optional[str] = None
    event_context: Optional[str] = None
    ready: bool = False
    reset: bool = False

    def __post_init__(self) -> None:
        optional_finite(self.progress_m, "progress_m")
        optional_finite(self.unwrapped_progress_m, "unwrapped_progress_m")
        optional_finite(self.track_length_m, "track_length_m")
        if self.lap < 0:
            raise ValueError("lap must be non-negative")
        if self.track_length_m is not None and self.track_length_m <= 0.0:
            raise ValueError("track_length_m must be positive")


@dataclass(frozen=True, slots=True)
class TyreState:
    """Ordered wheels are FL, FR, RL, RR; values remain nullable."""
    surface_temp_k: Tuple[Optional[float], ...] = (None, None, None, None)
    carcass_temp_k: Tuple[Optional[float], ...] = (None, None, None, None)
    wear: Tuple[Optional[float], ...] = (None, None, None, None)
    grip_scale: Tuple[Optional[float], ...] = (None, None, None, None)
    valid: Tuple[bool, ...] = (False, False, False, False)

    def __post_init__(self) -> None:
        for name in ("surface_temp_k", "carcass_temp_k", "wear", "grip_scale", "valid"):
            value = getattr(self, name)
            if not isinstance(value, tuple):
                object.__setattr__(self, name, tuple(value))
            if len(value) != 4:
                raise ValueError(f"{name} must contain FL, FR, RL and RR")
        for name in ("surface_temp_k", "carcass_temp_k", "wear", "grip_scale"):
            for value in getattr(self, name):
                if value is not None:
                    finite(value, name)
        for value in self.wear:
            if value is not None and not 0.0 <= value <= 1.0:
                raise ValueError("tyre wear must be in [0, 1]")
        for value in self.grip_scale:
            if value is not None and value < 0.0:
                raise ValueError("tyre grip_scale must be non-negative")
        for index, is_valid in enumerate(self.valid):
            if is_valid and self.surface_temp_k[index] is None and self.carcass_temp_k[index] is None and self.wear[index] is None and self.grip_scale[index] is None:
                raise ValueError("a valid wheel requires at least one measurement")


@dataclass(frozen=True, slots=True)
class StrategyDirective:
    header: ContractHeader = field(default_factory=ContractHeader)
    action: str = "follow"
    target_id: Optional[str] = None
    opportunity_id: Optional[str] = None
    corridor_left_m: Optional[float] = None
    corridor_right_m: Optional[float] = None
    permitted_after: Optional[float] = None
    permitted_until: Optional[float] = None
    deploy_budget_j: Optional[float] = None
    recovery_budget_j: Optional[float] = None
    minimum_exit_energy_j: Optional[float] = None
    marginal_energy_value: Optional[float] = None
    continuation: Optional[str] = None
    abort_conditions: Tuple[str, ...] = ()
    committed: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.abort_conditions, tuple):
            object.__setattr__(self, "abort_conditions", tuple(self.abort_conditions))
        for name in fields(self):
            if name.name not in ("header", "action", "target_id", "opportunity_id", "continuation", "abort_conditions", "committed"):
                optional_finite(getattr(self, name.name), name.name)
        if self.permitted_after is not None and self.permitted_until is not None and self.permitted_until < self.permitted_after:
            raise ValueError("directive timing window is inverted")
        for name in ("deploy_budget_j", "recovery_budget_j", "minimum_exit_energy_j"):
            value = getattr(self, name)
            if value is not None and value < 0.0:
                raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True, slots=True)
class TacticalEvaluation:
    header: ContractHeader = field(default_factory=ContractHeader)
    candidate_id: str = ""
    state_snapshot_id: Optional[str] = None
    status: str = "unevaluated"
    reason_code: Optional[str] = None
    time_cost_s: Optional[float] = None
    energy_cost_j: Optional[float] = None
    predicted_exit_progress_m: Optional[float] = None
    predicted_exit_energy_j: Optional[float] = None
    predicted_exit_uncertainty: Optional[float] = None
    retention_feasible: Optional[bool] = None
    abort_available: Optional[bool] = None
    coverage: Optional[float] = None
    computation_time_s: Optional[float] = None

    def __post_init__(self) -> None:
        for name in fields(self):
            if name.name not in ("header", "candidate_id", "state_snapshot_id", "status", "reason_code", "retention_feasible", "abort_available"):
                optional_finite(getattr(self, name.name), name.name)
        if self.coverage is not None and not 0.0 <= self.coverage <= 1.0:
            raise ValueError("coverage must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    stamp: float
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    yaw_rad: Optional[float] = None
    progress_m: Optional[float] = None
    lateral_m: Optional[float] = None
    speed_mps: Optional[float] = None
    curvature_1pm: Optional[float] = None
    corridor_left_m: Optional[float] = None
    corridor_right_m: Optional[float] = None
    energy_reference_j: Optional[float] = None
    acceleration_mps2: Optional[float] = None
    steering_rad: Optional[float] = None

    def __post_init__(self) -> None:
        finite(self.stamp, "stamp")
        for name in fields(self):
            if name.name != "stamp":
                optional_finite(getattr(self, name.name), name.name)


@dataclass(frozen=True, slots=True)
class TrajectoryPlan:
    header: ContractHeader = field(default_factory=ContractHeader)
    candidate_id: str = ""
    points: Tuple[TrajectoryPoint, ...] = ()
    max_acceleration_mps2: Optional[float] = None
    min_acceleration_mps2: Optional[float] = None
    max_steering_rad: Optional[float] = None
    max_steering_rate_radps: Optional[float] = None
    feasible: bool = False
    infeasibility_reason: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.points, tuple):
            object.__setattr__(self, "points", tuple(self.points))
        previous = None
        for point in self.points:
            if previous is not None and point.stamp < previous:
                raise ValueError("trajectory point stamps must be monotonic")
            previous = point.stamp
        for name in fields(self):
            if name.name not in ("header", "candidate_id", "points", "feasible", "infeasibility_reason"):
                optional_finite(getattr(self, name.name), name.name)
        if self.max_steering_rate_radps is not None and self.max_steering_rate_radps < 0.0:
            raise ValueError("max_steering_rate_radps must be non-negative")


@dataclass(frozen=True, slots=True)
class HybridDriveStamped:
    """Atomic hybrid request; acceleration is tyre force/mass before drag."""
    header: ContractHeader = field(default_factory=lambda: ContractHeader(source=Source.COMMAND))
    acceleration_mps2: float = 0.0
    steering_rad: float = 0.0
    requested_mguk_force_n: float = 0.0
    command_sequence: int = 0
    owner: str = ""

    def __post_init__(self) -> None:
        finite(self.acceleration_mps2, "acceleration_mps2")
        finite(self.steering_rad, "steering_rad")
        finite(self.requested_mguk_force_n, "requested_mguk_force_n")
        if self.command_sequence < 0:
            raise ValueError("command_sequence must be non-negative")


@dataclass(frozen=True, slots=True)
class ControlStatus:
    header: ContractHeader = field(default_factory=ContractHeader)
    owner: Optional[str] = None
    solver_status: Optional[str] = None
    fallback_reason: Optional[str] = None
    tracking_error_m: Optional[float] = None
    heading_error_rad: Optional[float] = None
    speed_error_mps: Optional[float] = None
    computation_time_s: Optional[float] = None
    constraint_residual: Optional[float] = None
    requested_acceleration_mps2: Optional[float] = None
    requested_steering_rad: Optional[float] = None
    delivered_acceleration_mps2: Optional[float] = None
    delivered_steering_rad: Optional[float] = None
    ready: bool = False
    stale: bool = False
    derated: bool = False

    def __post_init__(self) -> None:
        for name in fields(self):
            if name.name not in ("header", "owner", "solver_status", "fallback_reason", "ready", "stale", "derated"):
                optional_finite(getattr(self, name.name), name.name)


@dataclass(frozen=True, slots=True)
class ActuationFeedback:
    header: ContractHeader = field(default_factory=ContractHeader)
    command_sequence: int = 0
    owner: Optional[str] = None
    requested_acceleration_mps2: Optional[float] = None
    requested_steering_rad: Optional[float] = None
    requested_mguk_force_n: Optional[float] = None
    delivered_acceleration_mps2: Optional[float] = None
    delivered_steering_rad: Optional[float] = None
    delivered_mguk_force_n: Optional[float] = None
    friction_brake_force_n: Optional[float] = None
    available_drive_force_n: Optional[float] = None
    available_regen_force_n: Optional[float] = None
    derate_reason: Optional[str] = None
    valid: bool = False

    def __post_init__(self) -> None:
        if self.command_sequence < 0:
            raise ValueError("command_sequence must be non-negative")
        for name in fields(self):
            if name.name not in ("header", "owner", "derate_reason", "valid", "command_sequence"):
                optional_finite(getattr(self, name.name), name.name)


def to_dict(value: Any) -> Any:
    """Return a JSON-ready tree, preserving unavailable values as ``None``."""
    return _clean(value)


def to_json(value: Any, **kwargs: Any) -> str:
    return json.dumps(to_dict(value), allow_nan=False, **kwargs)


__all__ = [
    "SCHEMA_VERSION", "ACCELERATION_SEMANTICS", "Validity", "Source", "ContractHeader", "VehicleState",
    "EnergyState", "OpponentBelief", "TyreState", "EstimatedRaceState", "StrategyDirective",
    "TacticalEvaluation", "TrajectoryPoint", "TrajectoryPlan",
    "HybridDriveStamped", "ControlStatus", "ActuationFeedback", "to_dict", "to_json",
    "CommonHeader", "StateSnapshot", "Trajectory", "DriveCommand",
]

# Descriptive aliases used by adapters and replay tools.
CommonHeader = ContractHeader
StateSnapshot = EstimatedRaceState
Trajectory = TrajectoryPlan
DriveCommand = HybridDriveStamped
