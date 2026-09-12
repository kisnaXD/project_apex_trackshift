"""Focused package-level Phase 1 checks."""
from pathlib import Path
import math
import sys
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from eufs_race_control.contracts import ContractHeader, EnergyState, HybridDriveStamped, Source, to_dict
from eufs_race_control.estimation import PoseObservation, PoseSample, RaceEstimator, SyntheticPoseSensor
from eufs_race_control.geometry import CenterlinePoint, Footprint, TrackGeometry, footprints_collide, swept_collision
from eufs_race_control.supervision import CommandArbiter, CommandLimits, CommandOwner, CommandRequest, Readiness


def square_track():
    return TrackGeometry((CenterlinePoint(0, 0, 0, 0), CenterlinePoint(10, 10, 0, math.pi / 2), CenterlinePoint(20, 10, 10, math.pi), CenterlinePoint(30, 0, 10, -math.pi / 2)))


def test_contracts_reject_nonfinite_and_keep_null():
    assert to_dict(EnergyState())["stored_energy_j"] is None
    with pytest.raises(ValueError):
        HybridDriveStamped(acceleration_mps2=float("nan"))
    assert not ContractHeader(stamp=2).is_fresh(1)


def test_projection_seam_alias_and_impossible_association():
    track = square_track()
    assert track.project(0, .2, previous_progress_m=39, reachable_window_m=5).unwrapped_progress_m > 39
    with pytest.raises(ValueError):
        track.project(5, 0, previous_progress_m=30, reachable_window_m=.1)
    with pytest.raises(ValueError):
        track.project(5, 0, previous_heading_rad=math.pi, max_heading_error_rad=.01)


def test_sensor_and_causal_delayed_estimator():
    truth = PoseSample(1, 3, 4, source_id="truth")
    assert SyntheticPoseSensor(noise_std_m=.1, seed=4).observe(truth) == SyntheticPoseSensor(noise_std_m=.1, seed=4).observe(truth)
    estimator = RaceEstimator(square_track(), max_extrapolation_s=.6, stale_after_s=.6)
    estimator.add_ego(PoseSample(1, 1, 0)); estimator.add_ego(PoseSample(1.2, 1.2, 0))
    estimator.add_opponent_observation(PoseObservation(1, 1.5, PoseSample(1, 4, 0, source_id="rival")))
    assert estimator.estimate(1.2).opponent is None
    assert estimator.estimate(1.5).opponent is not None
    assert not estimator.estimate(1.8).valid


def test_estimator_closing_speed_and_reset():
    estimator = RaceEstimator(square_track())
    for stamp, ego_x, rival_x in ((1, 1, 4), (2, 1.5, 6)):
        estimator.add_ego(PoseSample(stamp, ego_x, 0)); estimator.add_opponent_observation(PoseSample(stamp, rival_x, 0, source_id="rival"))
    result = estimator.estimate(2)
    assert result.closing_speed_mps is not None and result.closing_speed_mps < 0
    estimator.reset(3)
    assert not estimator.add_ego(PoseSample(2.5, 1, 0))


def test_closing_speed_uses_progress_history_across_seam():
    track = square_track()
    estimator = RaceEstimator(track)
    samples = ((1.0, (0.0, 1.0), (1.0, 0.0)), (1.1, (0.0, 0.6), (1.2, 0.0)), (1.2, (0.0, 0.2), (1.4, 0.0)), (1.3, (0.2, 0.0), (1.6, 0.0)))
    for stamp, ego, rival in samples:
        estimator.add_ego(PoseSample(stamp, *ego, yaw_rad=-math.pi / 2))
        estimator.add_opponent_observation(PoseSample(stamp, *rival, yaw_rad=0.0, source_id="rival"))
    result = estimator.estimate(1.3)
    assert result.closing_speed_mps == pytest.approx(2.0, abs=.2)
    assert result.ego_unwrapped_progress_m is not None


def test_arbitration_priority_bounds_and_stop_latch():
    arbiter = CommandArbiter(run_id="run", limits=CommandLimits(max_acceleration_rate_mps3=100))
    header = ContractHeader(run_id="run", stamp=0, expires_at=1, source=Source.COMMAND)
    assert arbiter.submit(CommandRequest(header, CommandOwner.AUTONOMOUS, 8, .2, command_sequence=1))
    assert arbiter.arbitrate(.1, Readiness("eufs", True, True, True)).owner is CommandOwner.AUTONOMOUS
    stopped = arbiter.stop(.2, Readiness("eufs", True, True, True))
    assert stopped.command.acceleration_mps2 == -12 and stopped.command.requested_mguk_force_n == 0
    assert arbiter.arbitrate(.6, Readiness("eufs", True, True, True)).owner is CommandOwner.STOP


def test_full_rectangle_collision():
    assert footprints_collide((0, 0, 0), (4, 0, 0), Footprint())
    assert not footprints_collide((0, 0, 0), (10, 0, 0), Footprint())


def test_swept_collision_uses_shortest_yaw_and_catches_long_crossing():
    assert not swept_collision((0, 0, math.radians(179)), (0, 0, math.radians(-179)), (4, 0, 0), (4, 0, 0))
    assert swept_collision((0, 0, 0), (100, 0, 0), (6, 0, 0), (6, 0, 0))
