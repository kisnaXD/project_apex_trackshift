"""Typed dashboard telemetry schema.

EUFS publishes only a subset of a race telemetry feed. Every measurement is
optional: ``None`` means the simulator has no authoritative source for it,
while numeric zero remains a real measurement. ``to_dict`` preserves the
requested public keys and serializes unavailable values as JSON ``null``.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional

_CORNERS = ("FL", "FR", "RL", "RR")


def _empty_corners() -> Dict[str, Optional[float]]:
    return {corner: None for corner in _CORNERS}


@dataclass(frozen=True)
class TireChannel:
    compound: Optional[str] = None
    laps_old: Optional[int] = None
    wear_pct: Optional[float] = None
    deg_rate_pct_per_lap: Optional[float] = None
    lap_time_deg_penalty_s: Optional[float] = None
    mu_friction_coeff: Optional[float] = None
    surface_temp_c: Dict[str, Optional[float]] = field(default_factory=_empty_corners)
    carcass_temp_c: Dict[str, Optional[float]] = field(default_factory=_empty_corners)


@dataclass(frozen=True)
class ERSTelemetry:
    soc_pct: Optional[float] = None
    soh_pct: Optional[float] = None
    pack_temp_c: Optional[float] = None
    max_cell_temp_c: Optional[float] = None
    current_demand_a: Optional[float] = None
    voltage_v: Optional[float] = None
    mguk_power_kw: Optional[float] = None
    lap_deployed_mj: Optional[float] = None


@dataclass(frozen=True)
class KinematicsTelemetry:
    s: Optional[float] = None
    y: Optional[float] = None
    x_world: Optional[float] = None
    y_world: Optional[float] = None
    heading_rad: Optional[float] = None
    speed_kmh: Optional[float] = None
    accel_long_g: Optional[float] = None
    accel_lat_g: Optional[float] = None
    yaw_rate_rads: Optional[float] = None


@dataclass(frozen=True)
class EgoTelemetry:
    session_time: Optional[float] = None
    lap_number: Optional[int] = None
    track_status: Optional[str] = None
    kinematics: KinematicsTelemetry = field(default_factory=KinematicsTelemetry)
    ers: ERSTelemetry = field(default_factory=ERSTelemetry)
    tires: TireChannel = field(default_factory=TireChannel)
    throttle_pct: Optional[float] = None
    brake_pressure_bar: Optional[float] = None
    gear: Optional[int] = None
    drs_active: Optional[bool] = None
    fuel_remaining_kg: Optional[float] = None


@dataclass(frozen=True)
class OpponentTelemetry:
    car_id: str
    driver_code: Optional[str] = None
    kinematics: KinematicsTelemetry = field(default_factory=KinematicsTelemetry)
    gap_to_ego_m: Optional[float] = None
    gap_to_ego_s: Optional[float] = None
    speed_gap_kmh: Optional[float] = None
    compound: Optional[str] = None
    laps_old: Optional[int] = None
    inferred_deg_delta_s: Optional[float] = None
    drs_active: Optional[bool] = None
    is_clipping: Optional[bool] = None


@dataclass(frozen=True)
class TelemetryFrame:
    timestamp: Optional[float]
    ego: EgoTelemetry
    opponents: Dict[str, OpponentTelemetry] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        # ROS messages may carry NaN/inf sentinels.  The public dashboard
        # contract uses JSON null for unavailable measurements, so scrub those
        # values at the serialization boundary as well as in the node.
        def scrub(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {key: scrub(item) for key, item in value.items()}
            if isinstance(value, list):
                return [scrub(item) for item in value]
            return value

        return scrub(asdict(self))


# Short aliases keep the dashboard implementation readable without changing
# the public schema names above.
Tire = TireChannel
ERS = ERSTelemetry
Kinematics = KinematicsTelemetry
Ego = EgoTelemetry
Opponent = OpponentTelemetry
Frame = TelemetryFrame

__all__ = [
    "TireChannel", "ERSTelemetry", "KinematicsTelemetry", "EgoTelemetry",
    "OpponentTelemetry", "TelemetryFrame", "Tire", "ERS", "Kinematics",
    "Ego", "Opponent", "Frame",
]
