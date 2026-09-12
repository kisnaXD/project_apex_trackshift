"""Repository-level Phase 1 foundation checks.

The full focused suite lives with the ament package; these checks keep the
workspace test discovery independent of ROS installation layout.
"""
import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "overlay" / "eufs_race_control"))

from eufs_race_control.contracts import ContractHeader, HybridDriveStamped, Source, to_dict
from eufs_race_control.estimation import PoseObservation, PoseSample, RaceEstimator
from eufs_race_control.geometry import CenterlinePoint, TrackGeometry
from eufs_race_control.supervision import CommandArbiter, CommandOwner, CommandRequest, Readiness


def _track():
    return TrackGeometry((
        CenterlinePoint(0.0, 0.0, 0.0, 0.0),
        CenterlinePoint(10.0, 10.0, 0.0, math.pi / 2),
        CenterlinePoint(20.0, 10.0, 10.0, math.pi),
        CenterlinePoint(30.0, 0.0, 10.0, -math.pi / 2),
    ))


def test_seam_and_nearby_alias_are_continuous():
    track = _track()
    assert track.project(0.0, 0.2, previous_progress_m=39.0, reachable_window_m=3.0).unwrapped_progress_m > 39.0
    with pytest.raises(ValueError):
        track.project(5.0, 0.0, previous_progress_m=30.0, reachable_window_m=0.1)


def test_delayed_pose_causality_and_stale_fallback():
    estimator = RaceEstimator(_track(), max_extrapolation_s=0.5, stale_after_s=0.6)
    estimator.add_ego(PoseSample(1.0, 1.0, 0.0))
    estimator.add_ego(PoseSample(1.2, 1.2, 0.0))
    estimator.add_opponent_observation(PoseObservation(1.0, 1.5, PoseSample(1.0, 4.0, 0.0, source_id="rival")))
    assert estimator.estimate(1.2).opponent is None
    assert estimator.estimate(1.5).opponent is not None
    assert not estimator.estimate(1.8).valid


def test_contracts_preserve_null_and_reject_invalid_control_scalar():
    assert to_dict(ContractHeader(stamp=1.0))["known_fields"] == []
    with pytest.raises(ValueError):
        HybridDriveStamped(acceleration_mps2=float("nan"))


def test_stop_has_exclusive_owner_and_reset_epoch_rejects_old_request():
    arbiter = CommandArbiter(run_id="run")
    header = ContractHeader(run_id="run", stamp=0.0, expires_at=1.0, source=Source.COMMAND)
    assert not arbiter.submit(CommandRequest(ContractHeader(run_id="wrong", stamp=0.0, expires_at=1.0, source=Source.COMMAND), CommandOwner.AUTONOMOUS, 0.0, 0.0))
    assert arbiter.submit(CommandRequest(header, CommandOwner.AUTONOMOUS, 8.0, 0.2, command_sequence=1))
    assert arbiter.arbitrate(0.1).owner is CommandOwner.FALLBACK
    active = arbiter.arbitrate(0.2, Readiness("eufs", True, True, True))
    assert active.owner is CommandOwner.AUTONOMOUS
    stopped = arbiter.stop(0.3, Readiness("eufs", True, True, True))
    assert stopped.owner is CommandOwner.STOP
    assert stopped.command is not None and stopped.command.acceleration_mps2 < 0.0
    assert stopped.command.steering_rad == pytest.approx(active.command.steering_rad)
    assert arbiter.arbitrate(0.5, Readiness("eufs", True, True, True)).owner is CommandOwner.STOP
    arbiter.reset()
    assert not arbiter.submit(CommandRequest(header, CommandOwner.MANUAL, 0.0, 0.0))
