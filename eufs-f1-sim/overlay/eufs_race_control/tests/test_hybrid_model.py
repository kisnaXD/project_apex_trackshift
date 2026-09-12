"""Reduced-model tests; execution is handled by the parent review."""
import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))
from eufs_race_control.models import DriveRequest, HybridProfile, HybridState, advance_lap, step


def test_force_allocation_conserves_requested_force_without_drag():
    profile = HybridProfile(rolling_resistance_n=0.0, drag_coefficient_n_per_mps2=0.0, auxiliary_power_w=0.0)
    state = HybridState(3_000_000.0, profile.ambient_temperature_k)
    result = step(profile, state, DriveRequest(2.0, 0.0), 20.0, 0.0, 0.01)
    assert result.delivered_tyre_force_n == pytest.approx(profile.mass_kg * 2.0)
    assert result.delivered_ice_force_n >= 0.0


def test_timestep_convergence_and_energy_sign():
    profile = HybridProfile(auxiliary_power_w=0.0)
    state = HybridState(3_000_000.0, profile.ambient_temperature_k)
    one = step(profile, state, DriveRequest(1.0, 1000.0), 20.0, 0.0, 1.0)
    many = state
    for _ in range(100):
        many = step(profile, many, DriveRequest(1.0, 1000.0), 20.0, 0.0, 0.01).state
    assert many.stored_energy_j == pytest.approx(one.state.stored_energy_j, rel=2e-3)
    assert one.signed_energy_delta_j < 0.0


def test_capacity_reserve_hot_pack_and_low_grip_derate():
    profile = HybridProfile()
    empty = HybridState(profile.reserve_energy_j, profile.ambient_temperature_k)
    assert step(profile, empty, DriveRequest(5.0, 4000.0), 20.0, 0.0, .1).at_reserve
    hot = HybridState(profile.battery_capacity_j, profile.max_temperature_k + 1.0)
    assert step(profile, hot, DriveRequest(2.0, 4000.0), 20.0, 0.0, .1).thermal_limited
    worn = HybridState.initial(profile)
    worn = HybridState(worn.stored_energy_j, worn.temperature_k, worn.tyre_temperature_k, (1.0, 1.0, 1.0, 1.0))
    assert step(profile, worn, DriveRequest(2.0, 0.0), 20.0, 0.0, .1).grip_scale == 0.0


def test_lap_reset_preserves_energy_and_zero_reverse_speed_is_finite():
    profile = HybridProfile()
    state = HybridState(profile.battery_capacity_j - 100.0, profile.ambient_temperature_k, lap_deployed_j=20.0, lap_recovered_j=30.0, lap_index=2)
    reset = advance_lap(state, 3)
    assert reset.stored_energy_j == state.stored_energy_j
    assert reset.lap_deployed_j == reset.lap_recovered_j == 0.0
    for speed in (0.0,):
        result = step(profile, state, DriveRequest(0.0, 1000.0), speed, 0.0, .01)
        assert math.isfinite(result.electrical_power_w)
    with pytest.raises(ValueError):
        step(profile, state, DriveRequest(0.0, 1000.0), -0.01, 0.0, .01)
    with pytest.raises(ValueError):
        step(profile, state, DriveRequest(0.0, 1000.0), 1.0, 0.0, -.01)


def test_empty_and_full_store_do_not_create_opposing_or_unaccounted_actuation():
    profile = HybridProfile(auxiliary_power_w=1000.0)
    empty = HybridState(profile.reserve_energy_j, profile.ambient_temperature_k)
    result = step(profile, empty, DriveRequest(0.1, 5000.0), 0.0, 0.0, 0.1)
    assert result.delivered_mguk_force_n == 0.0
    assert result.signed_energy_delta_j == pytest.approx(result.state.stored_energy_j - empty.stored_energy_j)
    full = HybridState(profile.battery_capacity_j, profile.ambient_temperature_k)
    regen = step(profile, full, DriveRequest(-2.0, -5000.0), 20.0, 0.0, .1)
    assert regen.delivered_mguk_force_n == 0.0
    assert regen.signed_energy_delta_j == pytest.approx(regen.state.stored_energy_j - full.stored_energy_j)
    below_ambient = HybridState(1_000_000.0, profile.ambient_temperature_k - 10.0)
    held = step(profile, below_ambient, DriveRequest(), 0.0, 0.0, 0.0)
    assert held.state.temperature_k == below_ambient.temperature_k


def test_mguk_never_opposes_requested_longitudinal_demand():
    profile = HybridProfile()
    accelerating = step(profile, HybridState.initial(profile), DriveRequest(0.1, -5000.0), 20.0, 0.0, .1)
    braking = step(profile, HybridState.initial(profile), DriveRequest(-0.1, 5000.0), 20.0, 0.0, .1)
    assert accelerating.delivered_mguk_force_n >= 0.0
    assert braking.delivered_mguk_force_n <= 0.0


def test_recovery_quota_is_stored_energy_and_efficiency_adjusted():
    profile = HybridProfile(max_recovery_per_lap_j=1000.0, regen_efficiency=.5, auxiliary_power_w=0.0)
    state = HybridState(profile.battery_capacity_j * .5, profile.ambient_temperature_k)
    result = step(profile, state, DriveRequest(-2.0, -5000.0), 20.0, 0.0, 1.0)
    assert result.state.lap_recovered_j <= 1000.0 + 1e-6
    assert result.available_recovery_power_w >= 0.0
    exhausted = HybridState(state.stored_energy_j, state.temperature_k, lap_recovered_j=profile.max_recovery_per_lap_j)
    assert step(profile, exhausted, DriveRequest(0.0, -5000.0), 0.0, 0.0, 1.0).delivered_mguk_force_n == 0.0
    with pytest.raises(ValueError):
        HybridProfile(min_force_speed_mps=0.0)


def test_combined_friction_circle_limits_longitudinal_force():
    profile = HybridProfile()
    normal = profile.tyre_normal_force_n + profile.downforce_coefficient_n_per_mps2 * 20.0 * 20.0
    result = step(profile, HybridState.initial(profile), DriveRequest(8.0, 0.0), 20.0, profile.tyre_mu * normal * .9, .1)
    assert result.longitudinal_force_limit_n < profile.mass_kg * 8.0
    assert result.derated


def test_lap_counter_requires_strict_advance():
    state = HybridState.initial(HybridProfile())
    with pytest.raises(ValueError):
        advance_lap(state, 0)
