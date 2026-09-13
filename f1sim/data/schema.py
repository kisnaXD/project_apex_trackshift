from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional


@dataclass
class TireChannel:
    compound: str
    laps_old: int
    wear_pct: float
    deg_rate_pct_per_lap: float
    lap_time_deg_penalty_s: float
    mu_friction_coeff: float
    surface_temp_c: Dict[str, float]
    carcass_temp_c: Dict[str, float]


@dataclass
class ERSTelemetry:
    soc_pct: float
    soh_pct: float
    pack_temp_c: float
    max_cell_temp_c: float
    current_demand_a: float
    voltage_v: float
    mguk_power_kw: float
    lap_deployed_mj: float


@dataclass
class KinematicsTelemetry:
    s: float
    y: float
    x_world: float
    y_world: float
    heading_rad: float
    speed_kmh: float
    accel_long_g: float
    accel_lat_g: float
    yaw_rate_rads: float


@dataclass
class EgoTelemetry:
    session_time: float
    lap_number: int
    track_status: str
    kinematics: KinematicsTelemetry
    ers: ERSTelemetry
    tires: TireChannel
    throttle_pct: float
    brake_pressure_bar: float
    gear: int
    drs_active: bool
    fuel_remaining_kg: float
    driver_code: str = ""
    driver_number: str = ""
    team_name: str = ""
    team_color: str = "#ffffff"
    position: Optional[int] = None
    rpm: int = 0


@dataclass
class OpponentTelemetry:
    car_id: str
    driver_code: str
    kinematics: KinematicsTelemetry
    gap_to_ego_m: float
    gap_to_ego_s: float
    speed_gap_kmh: float
    compound: str
    laps_old: int
    inferred_deg_delta_s: float
    drs_active: bool
    is_clipping: bool
    driver_number: str = ""
    team_name: str = ""
    team_color: str = "#ffffff"
    position: Optional[int] = None
    rpm: int = 0
    throttle_pct: float = 0.0
    brake_pressure_bar: float = 0.0
    gear: int = 0
    active: bool = True


@dataclass
class TelemetryFrame:
    timestamp: float
    ego: EgoTelemetry
    opponents: Dict[str, OpponentTelemetry] = field(default_factory=dict)
    source: str = "simulation"
    session_label: str = ""
    replay_time_s: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
