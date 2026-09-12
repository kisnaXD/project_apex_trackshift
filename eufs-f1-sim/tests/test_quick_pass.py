"""Focused quick-pass truth evaluator checks."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "overlay" / "eufs_race_control"))

from eufs_race_control.evaluation import QuickPassEvaluator  # noqa: E402
from eufs_race_control.geometry import CenterlinePoint, Footprint, TrackGeometry  # noqa: E402


def _track() -> TrackGeometry:
    return TrackGeometry([
        CenterlinePoint(-100.0, -100.0, 0.0, 0.0),
        CenterlinePoint(100.0, 100.0, 0.0, 0.0),
        CenterlinePoint(120.0, 100.0, 20.0, 1.5708),
        CenterlinePoint(140.0, -100.0, 20.0, 3.14159),
    ])


def test_clear_lateral_pass_is_held_for_two_seconds():
    evaluator = QuickPassEvaluator(_track(), Footprint(front_m=1.0, rear_m=1.0, halfwidth_m=0.3))
    evaluator.update(0.0, (0.0, 2.0, 0.0), (10.0, 0.0, 0.0))
    evaluator.update(1.0, (15.0, 2.0, 0.0), (10.0, 0.0, 0.0))
    result = evaluator.update(3.0, (15.0, 2.0, 0.0), (10.0, 0.0, 0.0))
    assert result.pass_complete
    assert not result.overlap
    assert result.hold_s >= 2.0


def test_centerline_crossing_reports_swept_overlap_and_no_pass():
    evaluator = QuickPassEvaluator(_track(), Footprint(front_m=1.0, rear_m=1.0, halfwidth_m=0.3))
    evaluator.update(0.0, (0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    result = evaluator.update(1.0, (20.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    assert result.overlap
    assert not result.pass_complete
