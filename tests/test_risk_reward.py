"""R_OT must be computed from live pack/opponent state, not free sliders."""

from apex.params import PackParams
from apex.physics import BatteryState
from apex.risk_reward import compute_rot
from apex.strategy import shadow_prices


def _state(**overrides) -> BatteryState:
    base = dict(
        soc=0.60,
        t_core_c=45.0,
        t_surf_c=42.0,
        e_used_lap_j=1.0e6,
        time_s=200.0,
        lap=3,
        tire_wear=0.20,
        gap_m=8.0,
    )
    base.update(overrides)
    return BatteryState(**base)


def test_hotter_core_lowers_rot_via_thermal_cost():
    params = PackParams()
    cool = compute_rot(_state(t_core_c=42.0), opponent_aggression=0.4, params=params)
    hot = compute_rot(_state(t_core_c=58.5), opponent_aggression=0.4, params=params)
    assert hot.r_ot < cool.r_ot
    assert hot.thermal_cost > cool.thermal_cost


def test_low_soc_lowers_rot_via_energy_cost():
    params = PackParams()
    fat = compute_rot(_state(soc=0.75), opponent_aggression=0.4, params=params)
    thin = compute_rot(_state(soc=0.22), opponent_aggression=0.4, params=params)
    assert thin.r_ot < fat.r_ot
    assert thin.energy_cost > fat.energy_cost


def test_close_gap_raises_clean_air_benefit():
    params = PackParams()
    far = compute_rot(_state(gap_m=28.0), opponent_aggression=0.3, params=params)
    close = compute_rot(_state(gap_m=4.0), opponent_aggression=0.3, params=params)
    assert close.clean_air_benefit > far.clean_air_benefit


def test_mode_thresholds_harvest_nominal_attack():
    params = PackParams()
    harvest = compute_rot(_state(soc=0.18, t_core_c=58.0, gap_m=30.0, tire_wear=0.7), 0.8, params)
    attack = compute_rot(_state(soc=0.72, t_core_c=40.0, gap_m=5.0, tire_wear=0.1), 0.2, params)
    assert harvest.mode == "harvest"
    assert harvest.r_ot < 1.0
    assert attack.mode == "attack"
    assert attack.r_ot >= 1.5


def test_shadow_prices_rise_late_in_stint_when_soc_is_low():
    params = PackParams()
    early = shadow_prices(_state(soc=0.80, t_core_c=40.0, lap=1, time_s=10.0), params)
    late = shadow_prices(_state(soc=0.22, t_core_c=40.0, lap=11, time_s=1000.0), params)
    assert late.lambda_e > early.lambda_e
