"""Rule-based stand-in for the strategic PPO layer: sets λ_E and λ_T."""

from __future__ import annotations

from dataclasses import dataclass

from apex.params import PackParams
from apex.physics import BatteryState


@dataclass
class ShadowPrices:
    lambda_e: float
    lambda_t: float
    progress: float
    soc_target: float
    note: str = (
        "Rule-based stand-in for the reinforcement-learned PPO policy "
        "described in the full APEX architecture."
    )


def shadow_prices(state: BatteryState, params: PackParams) -> ShadowPrices:
    lap_frac = (state.time_s % params.lap_time_s) / params.lap_time_s
    progress = min(1.0, (state.lap - 1 + lap_frac) / params.n_laps)
    soc_target = 0.82 - 0.62 * progress
    soc_deficit = max(0.0, soc_target - state.soc)
    thermal_span = params.t_core_limit_c - 40.0
    thermal_stress = max(0.0, (state.t_core_c - 40.0) / thermal_span)

    lambda_e = 1.0 * (1.0 + 2.5 * soc_deficit + 1.2 * progress)
    lambda_t = 1.0 * (1.0 + 2.8 * thermal_stress)
    return ShadowPrices(
        lambda_e=lambda_e,
        lambda_t=lambda_t,
        progress=progress,
        soc_target=soc_target,
    )
