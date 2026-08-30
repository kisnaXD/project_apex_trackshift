"""Overtake Risk-Reward Index from five live-state factors."""

from __future__ import annotations

from dataclasses import dataclass

from apex.params import PackParams
from apex.physics import BatteryState
from apex.strategy import shadow_prices


WAKE_M = 20.0


@dataclass
class ROTResult:
    r_ot: float
    mode: str
    clean_air_benefit: float
    energy_cost: float
    thermal_cost: float
    tire_cost: float
    collision_cost: float
    lambda_e: float
    lambda_t: float


def _mode(r_ot: float) -> str:
    if r_ot < 1.0:
        return "harvest"
    if r_ot < 1.5:
        return "nominal"
    return "attack"


def compute_rot(state: BatteryState, opponent_aggression: float, params: PackParams) -> ROTResult:
    prices = shadow_prices(state, params)
    agg = min(1.0, max(0.0, float(opponent_aggression)))

    wake = max(0.0, 1.0 - state.gap_m / WAKE_M)
    clean_air = 0.40 + 1.80 * (wake**1.2)

    energy = (1.05 - state.soc) ** 2 * (0.55 + 0.45 * prices.lambda_e)

    margin = params.t_core_limit_c - state.t_core_c
    span = params.t_core_limit_c - 38.0
    thermal_raw = max(0.0, (span - margin) / span)
    thermal = (thermal_raw**1.4) * (0.50 + 0.50 * prices.lambda_t)

    tire = 0.15 + 0.90 * min(1.0, state.tire_wear)

    proximity = max(0.0, 1.0 - state.gap_m / 25.0)
    collision = 0.18 + 0.95 * agg * (0.35 + 0.65 * proximity)

    denom = 0.12 + 0.26 * energy + 0.26 * thermal + 0.18 * tire + 0.18 * collision
    r_ot = clean_air / denom

    return ROTResult(
        r_ot=r_ot,
        mode=_mode(r_ot),
        clean_air_benefit=clean_air,
        energy_cost=energy,
        thermal_cost=thermal,
        tire_cost=tire,
        collision_cost=collision,
        lambda_e=prices.lambda_e,
        lambda_t=prices.lambda_t,
    )
