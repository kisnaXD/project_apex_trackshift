"""Reduced hybrid plant model matched to ``eufs_hybrid_model`` C++.

The model is deliberately a force allocator: Ackermann acceleration means
requested tyre force divided by mass before drag.  ``synthetic_benchmark`` is
an explicit profile selector and is never presented as a calibrated 2026 FIA
powertrain.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional, Tuple
from dataclasses import fields


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True, slots=True)
class HybridProfile:
    profile_id: str = "synthetic_benchmark_2026_energy_model"
    vehicle_source: str = "overlay/eufs_racecar/robots/eufs/f1_dynamic_bicycle.yaml"
    mass_kg: float = 788.0
    drag_coefficient_n_per_mps2: float = 0.8269
    rolling_resistance_n: float = 120.0
    gravity_mps2: float = 9.81
    wheel_radius_m: float = 0.346
    ice_max_force_n: float = 9000.0
    ice_max_power_w: float = 120000.0
    mguk_max_force_n: float = 5000.0
    mguk_max_power_w: float = 120000.0
    mguk_regen_force_n: float = 3500.0
    mguk_regen_power_w: float = 80000.0
    mguk_efficiency: float = 0.95
    regen_efficiency: float = 0.70
    auxiliary_power_w: float = 500.0
    # Synthetic F1-scale store for benchmark behaviour; this is not a claim
    # about the legacy 22.2 V / 5.5 Ah telemetry plugin or FIA 2026 rules.
    battery_capacity_j: float = 4_000_000.0
    reserve_energy_j: float = 400_000.0
    ambient_temperature_k: float = 298.15
    max_temperature_k: float = 373.15
    thermal_capacity_j_per_k: float = 150000.0
    cooling_w_per_k: float = 100.0
    thermal_derate_start_k: float = 360.0
    max_deploy_per_lap_j: float = 4_000_000.0
    max_recovery_per_lap_j: float = 2_000_000.0
    tyre_normal_force_n: float = 788.0 * 9.81
    tyre_mu: float = 1.60
    tyre_temp_optimum_k: float = 363.15
    tyre_temp_band_k: float = 80.0
    tyre_heat_capacity_j_per_k: float = 30000.0
    tyre_cooling_w_per_k: float = 25.0
    wear_rate_per_j: float = 1e-10
    min_force_speed_mps: float = 1.0
    downforce_coefficient_n_per_mps2: float = 1.50

    @classmethod
    def synthetic_benchmark(cls) -> "HybridProfile":
        return cls()

    @classmethod
    def from_vehicle_defaults(cls) -> "HybridProfile":
        """Return the explicit vehicle-linked benchmark base profile.

        Electrical parameters remain synthetic and versioned until a calibrated
        powertrain is supplied; the native vehicle mass/drag/radius are copied
        from the existing DynamicBicycle configuration.
        """
        return cls(profile_id="vehicle_linked_synthetic_hybrid")

    def __post_init__(self) -> None:
        for item in fields(self):
            if isinstance(getattr(self, item.name), str):
                continue
            _finite(getattr(self, item.name), item.name)
        if self.mass_kg <= 0 or self.wheel_radius_m <= 0 or self.gravity_mps2 <= 0 or self.battery_capacity_j <= 0 or self.reserve_energy_j < 0 or self.reserve_energy_j > self.battery_capacity_j or self.ambient_temperature_k < 0 or self.max_temperature_k <= 0:
            raise ValueError("invalid mass or energy capacity")
        if not 0 < self.mguk_efficiency <= 1 or not 0 < self.regen_efficiency <= 1:
            raise ValueError("efficiencies must be in (0, 1]")
        if self.thermal_derate_start_k >= self.max_temperature_k:
            raise ValueError("thermal derate onset must precede maximum temperature")
        nonnegative = ("rolling_resistance_n", "ice_max_force_n", "ice_max_power_w", "mguk_max_force_n", "mguk_max_power_w", "mguk_regen_force_n", "mguk_regen_power_w", "auxiliary_power_w", "thermal_capacity_j_per_k", "cooling_w_per_k", "max_deploy_per_lap_j", "max_recovery_per_lap_j", "tyre_normal_force_n", "tyre_mu", "tyre_temp_band_k", "tyre_heat_capacity_j_per_k", "tyre_cooling_w_per_k", "wear_rate_per_j", "downforce_coefficient_n_per_mps2")
        if any(getattr(self, name) < 0.0 for name in nonnegative) or self.min_force_speed_mps <= 0.0:
            raise ValueError("profile limits must be non-negative and min speed must be positive")


@dataclass(frozen=True, slots=True)
class HybridState:
    stored_energy_j: float
    temperature_k: float
    tyre_temperature_k: Tuple[float, float, float, float] = (363.15, 363.15, 363.15, 363.15)
    tyre_wear: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    lap_deployed_j: float = 0.0
    lap_recovered_j: float = 0.0
    lap_index: int = 0

    @classmethod
    def initial(cls, profile: HybridProfile) -> "HybridState":
        return cls(profile.battery_capacity_j, profile.ambient_temperature_k)

    def __post_init__(self) -> None:
        for name in ("stored_energy_j", "temperature_k", "lap_deployed_j", "lap_recovered_j"):
            _finite(getattr(self, name), name)
        if len(self.tyre_temperature_k) != 4 or len(self.tyre_wear) != 4:
            raise ValueError("tyre state requires FL, FR, RL, RR")
        if self.stored_energy_j < 0 or self.lap_deployed_j < 0 or self.lap_recovered_j < 0:
            raise ValueError("energy counters must be non-negative")
        if isinstance(self.lap_index, bool) or not isinstance(self.lap_index, int) or self.lap_index < 0:
            raise ValueError("lap_index must be a non-negative integer")
        if any(not 0 <= wear <= 1 for wear in self.tyre_wear):
            raise ValueError("tyre wear must be in [0, 1]")
        for value in self.tyre_temperature_k:
            _finite(value, "tyre_temperature_k")


@dataclass(frozen=True, slots=True)
class DriveRequest:
    acceleration_mps2: float = 0.0
    requested_mguk_force_n: float = 0.0

    def __post_init__(self) -> None:
        _finite(self.acceleration_mps2, "acceleration_mps2")
        _finite(self.requested_mguk_force_n, "requested_mguk_force_n")


@dataclass(frozen=True, slots=True)
class StepResult:
    state: HybridState
    requested_tyre_force_n: float
    delivered_ice_force_n: float
    delivered_mguk_force_n: float
    friction_brake_force_n: float
    delivered_tyre_force_n: float
    net_acceleration_mps2: float
    electrical_power_w: float
    delivered_auxiliary_power_w: float
    available_deploy_power_w: float
    available_recovery_power_w: float
    signed_energy_delta_j: float
    usable_energy_j: float
    grip_scale: float
    longitudinal_force_limit_n: float
    derated: bool
    capacity_empty: bool
    store_empty: bool
    at_reserve: bool
    deployment_unavailable: bool
    thermal_limited: bool
    derate_reason: str = ""


def _force_limit(force: float, power: float, speed: float, minimum_speed: float) -> float:
    return min(abs(force), power / max(abs(speed), minimum_speed))


def _thermal_step(temperature: float, ambient: float, heat_w: float, cooling_w_per_k: float, capacity_j_per_k: float, dt_s: float) -> float:
    if dt_s == 0.0:
        return temperature
    if cooling_w_per_k <= 0.0:
        return temperature + heat_w * dt_s / max(1.0, capacity_j_per_k)
    decay = math.exp(-cooling_w_per_k * dt_s / max(1.0, capacity_j_per_k))
    return ambient + (temperature - ambient) * decay + heat_w / cooling_w_per_k * (1.0 - decay)


def step(profile: HybridProfile, state: HybridState, request: DriveRequest, speed_mps: float, lateral_force_n: float, dt_s: float) -> StepResult:
    dt_s = _finite(dt_s, "dt_s")
    if dt_s < 0.0:
        raise ValueError("dt_s must be non-negative")
    speed_mps = _finite(speed_mps, "speed_mps")
    if speed_mps < -1e-9:
        raise ValueError("reverse speed is unsupported by this forward benchmark model")
    lateral_force_n = _finite(lateral_force_n, "lateral_force_n")
    requested = _clamp(_finite(request.acceleration_mps2, "acceleration_mps2"), -12.0, 8.0) * profile.mass_kg
    grip_values = []
    for temperature, wear in zip(state.tyre_temperature_k, state.tyre_wear):
        temp_scale = _clamp(1.0 - abs(temperature - profile.tyre_temp_optimum_k) / max(1.0, profile.tyre_temp_band_k), 0.0, 1.0)
        grip_values.append(max(0.0, (1.0 - wear) * temp_scale))
    grip_scale = _clamp(min(grip_values, default=1.0), 0.0, 1.0)
    normal_force = max(0.0, profile.tyre_normal_force_n + profile.downforce_coefficient_n_per_mps2 * speed_mps * speed_mps)
    circle_capacity = profile.tyre_mu * grip_scale * normal_force
    longitudinal_limit = math.sqrt(max(0.0, circle_capacity * circle_capacity - lateral_force_n * lateral_force_n))
    demand = _clamp(requested, -longitudinal_limit, longitudinal_limit)
    usable = max(0.0, state.stored_energy_j - profile.reserve_energy_j)
    thermal_scale = _clamp((profile.max_temperature_k - state.temperature_k) / max(1.0, profile.max_temperature_k - profile.thermal_derate_start_k), 0.0, 1.0) if state.temperature_k > profile.thermal_derate_start_k else 1.0
    thermal_limited = thermal_scale < 1.0
    drive_limit = _force_limit(profile.mguk_max_force_n, profile.mguk_max_power_w * thermal_scale, speed_mps, profile.min_force_speed_mps)
    regen_limit = _force_limit(profile.mguk_regen_force_n, profile.mguk_regen_power_w * thermal_scale, speed_mps, profile.min_force_speed_mps)
    mguk = _clamp(_finite(request.requested_mguk_force_n, "requested_mguk_force_n"), -regen_limit, drive_limit)
    if demand > 0.0:
        mguk = _clamp(mguk, 0.0, demand)
    elif demand < 0.0:
        mguk = _clamp(mguk, demand, 0.0)
    else:
        mguk = 0.0
    if mguk > 0.0 and (usable <= 0.0 or state.lap_deployed_j >= profile.max_deploy_per_lap_j or thermal_scale <= 0.0):
        mguk = 0.0
    if mguk < 0.0 and (state.lap_recovered_j >= profile.max_recovery_per_lap_j or state.stored_energy_j >= profile.battery_capacity_j or thermal_scale <= 0.0):
        mguk = 0.0
    mechanical_mguk = mguk * speed_mps
    if mechanical_mguk > 0:
        if usable <= 0.0:
            mguk = 0.0
        else:
            deploy_energy = max(0.0, usable - profile.auxiliary_power_w * dt_s)
            mguk = min(mguk, deploy_energy * profile.mguk_efficiency / max(abs(speed_mps), profile.min_force_speed_mps) / max(dt_s, 1e-9))
            mguk = min(mguk, max(0.0, profile.max_deploy_per_lap_j - state.lap_deployed_j) * profile.mguk_efficiency / max(abs(speed_mps), profile.min_force_speed_mps) / max(dt_s, 1e-9))
    elif mechanical_mguk < 0:
        mguk = max(mguk, -max(0.0, profile.max_recovery_per_lap_j - state.lap_recovered_j) / max(profile.regen_efficiency, 1e-6) / max(abs(speed_mps), profile.min_force_speed_mps) / max(dt_s, 1e-9))
        headroom = max(0.0, profile.battery_capacity_j - state.stored_energy_j)
        if headroom <= 0.0:
            mguk = 0.0
        else:
            mguk = max(mguk, -headroom / max(profile.regen_efficiency, 1e-6) / max(abs(speed_mps), profile.min_force_speed_mps) / max(dt_s, 1e-9))
    remainder = demand - mguk
    ice_limit = _force_limit(profile.ice_max_force_n, profile.ice_max_power_w, speed_mps, profile.min_force_speed_mps)
    ice = _clamp(max(0.0, remainder), 0.0, ice_limit)
    friction = min(0.0, remainder - ice)
    delivered = ice + mguk + friction
    drag = profile.drag_coefficient_n_per_mps2 * speed_mps * abs(speed_mps) + profile.rolling_resistance_n * (0 if speed_mps == 0 else (1 if speed_mps > 0 else -1))
    net_accel = (delivered - drag) / max(1e-9, profile.mass_kg)
    mech = mguk * speed_mps
    battery_rate_without_aux = -mech / max(1e-6, profile.mguk_efficiency) if mech >= 0 else -mech * profile.regen_efficiency
    available_aux = max(0.0, state.stored_energy_j - profile.reserve_energy_j) / max(dt_s, 1e-9) + battery_rate_without_aux if dt_s > 0 else 0.0
    auxiliary_power = min(profile.auxiliary_power_w, max(0.0, available_aux))
    electrical = mech / max(1e-6, profile.mguk_efficiency) + auxiliary_power if mech >= 0 else mech * profile.regen_efficiency + auxiliary_power
    requested_delta = -electrical * dt_s
    stored = _clamp(state.stored_energy_j + requested_delta, 0.0, profile.battery_capacity_j)
    delta = stored - state.stored_energy_j
    electrical = -delta / dt_s if dt_s > 0.0 else 0.0
    deployed = state.lap_deployed_j + (mech * dt_s / max(1e-6, profile.mguk_efficiency) if mech > 0 else 0)
    recovered = state.lap_recovered_j + (-mech * dt_s * profile.regen_efficiency if mech < 0 else 0)
    losses = mech * (1 / max(1e-6, profile.mguk_efficiency) - 1) if mech >= 0 else (-mech) * (1 - profile.regen_efficiency)
    temperature = _thermal_step(state.temperature_k, profile.ambient_temperature_k, losses + auxiliary_power, profile.cooling_w_per_k, profile.thermal_capacity_j_per_k, dt_s)
    tyre_heat = abs(delivered) * abs(speed_mps) * .02 + abs(lateral_force_n) * abs(speed_mps) * .02
    tyre_temps = tuple(_thermal_step(old, profile.ambient_temperature_k, tyre_heat / 4.0, profile.tyre_cooling_w_per_k, profile.tyre_heat_capacity_j_per_k, dt_s) for old in state.tyre_temperature_k)
    tyre_wear = tuple(_clamp(old + profile.wear_rate_per_j * (abs(delivered) + abs(lateral_force_n)) * abs(speed_mps) * dt_s, 0, 1) for old in state.tyre_wear)
    next_state = HybridState(stored, temperature, tyre_temps, tyre_wear, deployed, recovered, state.lap_index)
    store_empty = stored <= 1e-9
    at_reserve = stored <= profile.reserve_energy_j + 1e-9
    deployment_unavailable = at_reserve or thermal_scale <= 0.0 or state.lap_deployed_j >= profile.max_deploy_per_lap_j
    derated = at_reserve or thermal_limited or grip_scale < 1 or abs(delivered - requested) > 1e-6
    reason = "reserve" if at_reserve else "thermal" if thermal_limited else "grip" if grip_scale < 1 else "force_limit" if derated else ""
    available_deploy = drive_limit * abs(speed_mps) if not deployment_unavailable else 0.0
    available_recovery = regen_limit * abs(speed_mps) if state.stored_energy_j < profile.battery_capacity_j and state.lap_recovered_j < profile.max_recovery_per_lap_j and thermal_scale > 0.0 else 0.0
    return StepResult(next_state, requested, ice, mguk, friction, delivered, net_accel, electrical, auxiliary_power, available_deploy, available_recovery, delta, max(0, stored - profile.reserve_energy_j), grip_scale, longitudinal_limit, derated, store_empty, store_empty, at_reserve, deployment_unavailable, thermal_limited, reason)


def advance_lap(state: HybridState, lap_index: int) -> HybridState:
    if lap_index <= state.lap_index:
        raise ValueError("lap_index must strictly advance")
    if lap_index < 0:
        raise ValueError("lap_index must be non-negative")
    return replace(state, lap_index=lap_index, lap_deployed_j=0.0, lap_recovered_j=0.0)


__all__ = ["HybridProfile", "HybridState", "DriveRequest", "StepResult", "step", "advance_lap"]
