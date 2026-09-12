"""Pure tests for the dashboard's centerline and Frenet helpers."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / 'overlay' / 'eufs_racecar'))

from eufs_racecar.telemetry_geometry import SchemaTrackProjector


def test_cota_centerline_is_loaded_from_track_model():
    path = pathlib.Path(__file__).parents[1] / 'overlay' / 'eufs_tracks' / 'models' / 'cota' / 'centerline.csv'
    projector = SchemaTrackProjector(path)
    assert projector.length > 5_000.0
    s, lateral = projector.project(0.0, 0.0)
    assert s == 0.0
    assert lateral == 0.0


def test_signed_shortest_gap_wraps_ahead_and_behind():
    length = 100.0
    assert SchemaTrackProjector.signed_shortest_gap(2.0, 98.0, length) == 4.0
    assert SchemaTrackProjector.signed_shortest_gap(98.0, 2.0, length) == -4.0
