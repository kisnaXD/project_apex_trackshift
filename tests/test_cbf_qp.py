"""CBF-QP must be a real solver: clip when unsafe, pass through when safe."""

import pytest

from apex.cbf_qp import filter_power, kkt_power
from apex.params import PackParams
from apex.physics import BatteryState, step


def _state(**overrides) -> BatteryState:
    base = dict(
        soc=0.55,
        t_core_c=42.0,
        t_surf_c=40.0,
        e_used_lap_j=0.0,
        time_s=0.0,
        lap=3,
        tire_wear=0.2,
        gap_m=8.0,
    )
    base.update(overrides)
    return BatteryState(**base)


def test_moderate_request_passes_unclipped_when_cool_with_quota():
    params = PackParams()
    result = filter_power(_state(t_core_c=40.0, e_used_lap_j=0.0), p_req=180_000.0, params=params)
    assert result.solver_status == "solved"
    assert result.clipped is False
    assert result.p_safe == pytest.approx(180_000.0, rel=1e-4)
    assert result.solve_time_ms < 20.0


def test_extreme_request_near_thermal_limit_is_clipped_by_solver():
    params = PackParams()
    hot = _state(t_core_c=params.t_core_limit_c - 0.6, t_surf_c=params.t_core_limit_c - 1.2)
    result = filter_power(hot, p_req=params.p_max_w, params=params)
    assert result.solver_status == "solved"
    assert result.clipped is True
    assert result.p_safe < params.p_max_w * 0.85
    assert "thermal" in result.active_constraints


def test_extreme_request_near_energy_quota_is_clipped_by_solver():
    params = PackParams()
    almost_empty_lap = _state(e_used_lap_j=params.e_quota_lap_j * 0.985)
    result = filter_power(almost_empty_lap, p_req=300_000.0, params=params)
    assert result.solver_status == "solved"
    assert result.clipped is True
    assert result.p_safe < 80_000.0
    assert "energy" in result.active_constraints


def test_kkt_matches_filter_on_thermal_clip():
    params = PackParams()
    hot = _state(t_core_c=params.t_core_limit_c - 0.8)
    result = filter_power(hot, p_req=340_000.0, params=params)
    closed = kkt_power(hot, p_req=340_000.0, params=params)
    assert result.p_safe == pytest.approx(closed, rel=1e-3, abs=50.0)


def test_holding_filtered_power_never_breaches_thermal_limit():
    params = PackParams()
    state = _state(t_core_c=params.t_core_limit_c - 1.0, t_surf_c=params.t_core_limit_c - 1.8)
    for _ in range(80):
        safe = filter_power(state, p_req=params.p_max_w, params=params)
        state = step(state, p_w=safe.p_safe, dt=0.25, params=params)
        assert state.t_core_c <= params.t_core_limit_c + 0.15


def test_empty_pack_forces_zero_power():
    params = PackParams()
    empty = _state(soc=0.0)
    result = filter_power(empty, p_req=300_000.0, params=params)
    assert result.p_safe == pytest.approx(0.0, abs=50.0)
    assert result.clipped is True


def test_holding_filtered_power_never_breaches_lap_energy_quota():
    params = PackParams()
    state = _state(e_used_lap_j=params.e_quota_lap_j * 0.92)
    for _ in range(60):
        safe = filter_power(state, p_req=320_000.0, params=params)
        state = step(state, p_w=safe.p_safe, dt=0.5, params=params)
        assert state.e_used_lap_j <= params.e_quota_lap_j + 50.0
