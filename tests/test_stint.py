"""One connected stint: SOC, temperature, and R_OT evolve from the same power."""

from apex.stint import StintSim


def test_play_advances_shared_state_together():
    sim = StintSim()
    start = sim.snapshot()
    sim.set_inputs(opponent_aggression=0.35, push_bias=0.6)
    for _ in range(25):
        sim.step(1.0)
    end = sim.snapshot()
    assert end["state"]["soc"] < start["state"]["soc"]
    assert end["state"]["t_core_c"] != start["state"]["t_core_c"]
    assert end["state"]["time_s"] == 25.0
    assert len(end["history"]["t_s"]) == 26
    assert end["rot"]["r_ot"] > 0.0


def test_max_push_never_breaches_thermal_or_quota():
    sim = StintSim()
    sim.set_inputs(opponent_aggression=0.2, push_bias=1.0)
    # Start already warm so the filter must work, not luck.
    sim.state.t_core_c = 58.5
    sim.state.t_surf_c = 56.5
    sim.state.e_used_lap_j = sim.params.e_quota_lap_j * 0.90
    for _ in range(40):
        snap = sim.step(0.5)
        assert snap["state"]["t_core_c"] <= sim.params.t_core_limit_c + 0.15
        assert snap["state"]["e_used_lap_j"] <= sim.params.e_quota_lap_j + 80.0
        assert snap["cbf"]["p_safe"] <= snap["cbf"]["p_requested"] + 1.0


def test_history_stays_bounded_over_a_long_stint():
    sim = StintSim()
    for _ in range(250):
        sim.step(1.0)
    n = len(sim.snapshot()["history"]["t_s"])
    assert n <= 180
    assert n >= 2


def test_reset_restores_initial_stint():
    sim = StintSim()
    sim.step(5.0)
    sim.reset()
    snap = sim.snapshot()
    assert snap["state"]["time_s"] == 0.0
    assert snap["state"]["lap"] == 1
    assert snap["state"]["soc"] == 0.82
