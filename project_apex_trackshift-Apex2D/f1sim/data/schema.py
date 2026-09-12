from dataclasses import dataclass, field, asdict
from typing import Dict, Optional

@dataclass
class TireChannel:
    compound: str
    laps_old: int
    wear_pct: float                     # Instantaneous wear (0.0 - 100.0%)
    deg_rate_pct_per_lap: float         # Wear rate derivative (% per lap)
    lap_time_deg_penalty_s: float       # Pace penalty incurred (seconds)
    mu_friction_coeff: float            # Dynamic observer surface grip estimate
    surface_temp_c: Dict[str, float]    # Tread IR optical temps: FL, FR, RL, RR
    carcass_temp_c: Dict[str, float]    # Structural carcass core temps: FL, FR, RL, RR

@dataclass
class ERSTelemetry:
    soc_pct: float                      # State of Charge (0.0 - 100.0%)
    soh_pct: float                      # Battery State of Health index
    pack_temp_c: float                  # Bulk cooling circuit pack temp
    max_cell_temp_c: float              # Hottest monitored cell (safety boundary)
    current_demand_a: float             # DC Bus current (A)
    voltage_v: float                    # Nominal 750V DC Bus
    mguk_power_kw: float                # 2026 MGU-K delivery (-350 kW to +350 kW)
    lap_deployed_mj: float              # Accumulator against 2026 8.5 MJ/lap limit

@dataclass
class KinematicsTelemetry:
    s: float                            # Longitudinal Frenet track position (m)
    y: float                            # Lateral offset from centerline (m)
    x_world: float                      # Global Cartesian X
    y_world: float                      # Global Cartesian Y
    heading_rad: float                  # Tangent heading angle (psi)
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
    drs_active: bool                   # In 2026: True = X-Mode / MOM Override
    fuel_remaining_kg: float

@dataclass
class OpponentTelemetry:
    car_id: str
    driver_code: str
    kinematics: KinematicsTelemetry
    gap_to_ego_m: float
    gap_to_ego_s: float
    speed_gap_kmh: float                # Ego speed - Opponent speed (+ = catching)
    compound: str
    laps_old: int
    inferred_deg_delta_s: float         # Apex pace drop estimator
    drs_active: bool                   # Competitor active aero deployment
    is_clipping: bool                   # High-speed straight-line derate

@dataclass
class TelemetryFrame:
    timestamp: float
    ego: EgoTelemetry
    opponents: Dict[str, OpponentTelemetry] = field(default_factory=dict)

    def to_dict(self):
        return asdict(self)