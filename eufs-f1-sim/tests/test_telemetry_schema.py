"""Contract tests for the nullable dashboard telemetry shape."""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / 'overlay' / 'eufs_racecar'))

from eufs_racecar.telemetry_schema import (
    ERSTelemetry,
    EgoTelemetry,
    KinematicsTelemetry,
    TelemetryFrame,
    TireChannel,
)


def test_frame_serializes_requested_nested_shape_and_nulls():
    frame = TelemetryFrame(
        timestamp=12.5,
        ego=EgoTelemetry(
            kinematics=KinematicsTelemetry(speed_kmh=0.0),
            ers=ERSTelemetry(soc_pct=0.0),
            tires=TireChannel(surface_temp_c={"FL": 80.0, "FR": None, "RL": None, "RR": None}),
        ),
    )
    data = frame.to_dict()
    assert set(data) == {"timestamp", "ego", "opponents"}
    assert data["ego"]["tires"]["surface_temp_c"]["FL"] == 80.0
    assert data["ego"]["tires"]["surface_temp_c"]["FR"] is None
    assert data["ego"]["kinematics"]["speed_kmh"] == 0.0
    assert data["ego"]["ers"]["soh_pct"] is None


def test_nonfinite_measurements_serialize_as_json_null():
    frame = TelemetryFrame(
        timestamp=math.nan,
        ego=EgoTelemetry(kinematics=KinematicsTelemetry(speed_kmh=math.inf)),
    )
    data = frame.to_dict()
    assert data["timestamp"] is None
    assert data["ego"]["kinematics"]["speed_kmh"] is None
