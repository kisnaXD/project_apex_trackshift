"""Two-state electro-thermal plant: heat and SOC must come from power over time."""

from apex.params import PackParams
from apex.physics import BatteryState, current_from_power, step


def _cool_pack() -> BatteryState:
    return BatteryState(
        soc=0.70,
        t_core_c=38.0,
        t_surf_c=36.0,
        e_used_lap_j=0.0,
        time_s=0.0,
        lap=1,
        tire_wear=0.10,
        gap_m=12.0,
    )


def test_high_discharge_raises_core_temperature_over_time():
    params = PackParams()
    state = _cool_pack()
    for _ in range(40):
        state = step(state, p_w=300_000.0, dt=0.5, params=params)
    assert state.t_core_c > 40.0
    assert state.t_core_c > state.t_surf_c


def test_soc_falls_with_energy_drawn_not_by_assignment():
    params = PackParams()
    start = _cool_pack()
    state = start
    dt = 1.0
    p_w = 200_000.0
    n = 20
    for _ in range(n):
        state = step(state, p_w=p_w, dt=dt, params=params)
    energy_j = p_w * dt * n
    expected_drop = energy_j / params.capacity_j
    assert start.soc - state.soc == pytest_approx(expected_drop, rel=0.15)
    assert state.e_used_lap_j == pytest_approx(energy_j, rel=0.02)


def test_surface_temperature_lags_core_under_step_load():
    params = PackParams()
    state = _cool_pack()
    early = step(state, p_w=320_000.0, dt=2.0, params=params)
    core_jump = early.t_core_c - state.t_core_c
    surf_jump = early.t_surf_c - state.t_surf_c
    assert core_jump > surf_jump
    assert core_jump > 0.0


def test_current_from_power_matches_terminal_power_identity():
    params = PackParams()
    soc = 0.65
    p_w = 180_000.0
    i_a = current_from_power(p_w, soc, params)
    ocv = params.ocv_v(soc)
    v_term = ocv - i_a * params.r_int_ohm
    assert v_term * i_a == pytest_approx(p_w, rel=1e-6)
    assert i_a > 0.0


def pytest_approx(value, rel=1e-6, abs=None):
    import pytest

    return pytest.approx(value, rel=rel, abs=abs)
