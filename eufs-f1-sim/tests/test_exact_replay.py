"""Deterministic unit checks for the bounded Russell replay core."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "overlay" / "eufs_race_control"))

from eufs_race_control.replay import (  # noqa: E402
    ExactKinematicReplay,
    FrozenReference,
    ReplayClockError,
    artifact_hash,
)


def _reference() -> FrozenReference:
    artifact = {
        "artifact_version": "test",
        "provenance": {"source_hashes": {"fixture": "unit"}},
        "samples": [
            {"time_s": 0.0, "progress_m": 0.0, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0,
             "speed_mps": 2.0, "acceleration_mps2": 1.0, "curvature_1pm": 0.0},
            {"time_s": 1.0, "progress_m": 2.5, "x_m": 2.5, "y_m": 0.0, "yaw_rad": 0.0,
             "speed_mps": 3.0, "acceleration_mps2": -1.0, "curvature_1pm": 0.0},
            {"time_s": 2.0, "progress_m": 5.0, "x_m": 5.0, "y_m": 0.0, "yaw_rad": 0.0,
             "speed_mps": 2.0, "acceleration_mps2": 0.0, "curvature_1pm": 0.0},
        ],
    }
    artifact["artifact_sha256"] = artifact_hash(artifact)
    return FrozenReference(artifact)


def test_interval_lookup_and_outgoing_knot_acceleration():
    reference = _reference()
    assert reference.time_sample(1.0)["acceleration_mps2"] == -1.0
    assert reference.time_sample(0.5)["acceleration_mps2"] == 1.0


def test_backward_clock_invalidates_until_reset_and_completion_is_explicit():
    replay = ExactKinematicReplay(_reference())
    replay.reset(10.0, epoch_id=4)
    assert replay.advance(11.0).x_m == pytest.approx(2.5)
    with pytest.raises(ReplayClockError, match="backwards"):
        replay.advance(10.5)
    with pytest.raises(ReplayClockError, match="reset"):
        replay.advance(12.0)
    replay.reset(10.5, epoch_id=5)
    result = replay.advance(12.5)
    assert result.completed
    assert result.reference_time_s == pytest.approx(2.0)
    assert result.observation()["epoch_id"] == 5


def test_pause_holds_reference_phase():
    replay = ExactKinematicReplay(_reference())
    replay.reset(0.0)
    replay.pause()
    paused = replay.advance(10.0)
    assert paused.reference_time_s == 0.0
    replay.resume()
    assert replay.advance(11.0).reference_time_s == pytest.approx(1.0)
