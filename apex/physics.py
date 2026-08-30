"""Two-state electro-thermal pack: core/surface temperature, SOC, lap energy."""

from __future__ import annotations

from dataclasses import dataclass

from apex.params import PackParams


@dataclass
class BatteryState:
    soc: float
    t_core_c: float
    t_surf_c: float
    e_used_lap_j: float
    time_s: float
    lap: int
    tire_wear: float
    gap_m: float


def current_from_power(p_w: float, soc: float, params: PackParams) -> float:
    """Discharge current satisfying P = (OCV - I R) I. Peak-power clipped."""
    ocv = params.ocv_v(soc)
    r = params.r_int_ohm
    p_w = float(p_w)
    if p_w <= 0.0:
        return 0.0
    p_peak = (ocv * ocv) / (4.0 * r)
    p = min(p_w, p_peak * 0.999)
    disc = ocv * ocv - 4.0 * r * p
    if disc < 0.0:
        disc = 0.0
    return (ocv - disc**0.5) / (2.0 * r)


def dq_dp(p_w: float, soc: float, params: PackParams) -> tuple[float, float, float]:
    """Heat Q = I²R and ∂Q/∂P at the operating point (for CBF linearization)."""
    i = current_from_power(p_w, soc, params)
    r = params.r_int_ohm
    q = i * i * r
    ocv = params.ocv_v(soc)
    denom = ocv - 2.0 * i * r
    if denom < 1.0:
        denom = 1.0
    dqdP = 2.0 * i * r / denom
    return q, dqdP, i


def thermal_rates(t_core_c: float, t_surf_c: float, q_gen_w: float, params: PackParams) -> tuple[float, float]:
    q_cs = (t_core_c - t_surf_c) / params.r_cs_k_per_w
    q_sa = (t_surf_c - params.t_amb_c) / params.r_sa_k_per_w
    dtc = (q_gen_w - q_cs) / params.c_core_j_per_k
    dts = (q_cs - q_sa) / params.c_surf_j_per_k
    return dtc, dts


def step(state: BatteryState, p_w: float, dt: float, params: PackParams) -> BatteryState:
    p_w = max(0.0, float(p_w))
    remaining = float(dt)
    soc = state.soc
    tc = state.t_core_c
    ts = state.t_surf_c
    e_used = state.e_used_lap_j
    sub = 0.1
    while remaining > 1e-12:
        h = sub if remaining > sub else remaining
        i = current_from_power(p_w, soc, params)
        q_gen = i * i * params.r_int_ohm
        dtc, dts = thermal_rates(tc, ts, q_gen, params)
        tc += dtc * h
        ts += dts * h
        soc -= (p_w * h) / params.capacity_j
        if soc < 0.0:
            soc = 0.0
        e_used += p_w * h
        remaining -= h
    return BatteryState(
        soc=soc,
        t_core_c=tc,
        t_surf_c=ts,
        e_used_lap_j=e_used,
        time_s=state.time_s + dt,
        lap=state.lap,
        tire_wear=state.tire_wear,
        gap_m=state.gap_m,
    )
