"""Offline reference importer tests (no FastF1/network dependency)."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parents[1] / "scripts" / "export_fastf1_reference.py"
sys.path.insert(0, str(Path(__file__).parents[1] / "overlay" / "eufs_race_control"))
spec = importlib.util.spec_from_file_location("export_fastf1_reference", SCRIPT)
assert spec and spec.loader
reference = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = reference
spec.loader.exec_module(reference)
from eufs_race_control.replay.reference import FrozenReference as RuntimeFrozenReference  # noqa: E402


def _geometry():
    radius = 10.0
    return [
        {
            "s_m": radius * angle,
            "x_m": radius * math.cos(angle),
            "y_m": radius * math.sin(angle),
            "yaw_rad": angle + math.pi / 2.0,
            "curvature_1pm": 1.0 / radius,
        }
        for angle in (2.0 * math.pi * index / 12.0 for index in range(12))
    ]


def _rows():
    car = [
        {"source_time_s": 0.0, "Speed": 36.0},
        {"source_time_s": 1.0, "Speed": 36.0},
        {"source_time_s": 2.0, "Speed": 36.0},
    ]
    pos = [{"source_time_s": value, "X": "0", "Y": "0"} for value in (0.0, 1.0, 2.0)]
    return car, pos


def test_frozen_artifact_hash_and_source_mismatch(tmp_path):
    car, pos = _rows()
    artifact = reference.build_reference(car, pos, _geometry(), source_hashes={"car_csv": "abc"})
    destination = tmp_path / "reference.json"
    reference.save_artifact(artifact, destination)
    loaded = RuntimeFrozenReference.load(destination)
    assert loaded.artifact["provenance"]["path_label"].startswith("COTA layout-reference adapted")
    with pytest.raises(reference.ReferenceError):
        RuntimeFrozenReference.load(destination, expected_hash="wrong")
    with pytest.raises(reference.ReferenceError, match="source hash"):
        RuntimeFrozenReference.load(destination, expected_source_hashes={"car_csv": "wrong"})
    on_disk = json.loads(destination.read_text())
    on_disk["samples"][0]["speed_mps"] = 999
    destination.write_text(json.dumps(on_disk))
    with pytest.raises(reference.ReferenceError):
        RuntimeFrozenReference.load(destination)


def test_monotonic_reference_and_tracking_schedule_deviation():
    car, pos = _rows()
    frozen = RuntimeFrozenReference(reference.build_reference(car, pos, _geometry()))
    samples = list(frozen.artifact["samples"])
    assert all(a["progress_m"] <= b["progress_m"] for a, b in zip(samples, samples[1:]))
    report = frozen.evaluate_schedule([{"time_s": sample["time_s"], "x_m": sample["x_m"], "y_m": sample["y_m"], "speed_mps": sample["speed_mps"]} for sample in samples])
    assert report["within_tolerance"]
    deviated = frozen.evaluate_schedule([{"time_s": 1.0, "x_m": 100.0, "y_m": 100.0, "speed_mps": 0.0}])
    assert not deviated["within_tolerance"]


def test_clock_pause_or_reset_is_explicitly_rejected_in_source():
    car, pos = _rows()
    car[2]["source_time_s"] = 0.5
    with pytest.raises(reference.ReferenceError, match="timestamps"):
        reference.build_reference(car, pos, _geometry())


def test_geometry_seam_and_curvature_bounds_are_enforced():
    car, pos = _rows()
    broken = list(_geometry())
    broken[-1] = {**broken[-1], "x_m": 999.0}
    with pytest.raises(reference.ReferenceError, match="seam"):
        reference.build_reference(car, pos, broken)
    curved = list(_geometry())
    curved[1] = {**curved[1], "curvature_1pm": 2.0}
    with pytest.raises(reference.ReferenceError, match="curvature"):
        reference.build_reference(car, pos, curved)


def test_active_lateral_cap_seam_and_pace_integral():
    car, pos = _rows()
    for pace_scale in (0.5, 1.0):
        artifact = reference.build_reference(car, pos, _geometry(), pace_scale=pace_scale, max_lateral_accel_mps2=1.0)
        samples = artifact["samples"]
        assert samples[0]["speed_mps"] == samples[-1]["speed_mps"]
        assert all(sample["speed_mps"] <= math.sqrt(10.0) + 1e-6 for sample in samples)
        integrated = sum(0.5 * (left["speed_mps"] + right["speed_mps"]) * (right["time_s"] - left["time_s"]) for left, right in zip(samples, samples[1:]))
        assert abs(integrated - samples[-1]["progress_m"]) < 1e-6
        assert all(samples[index]["source"]["source_time_s"] <= samples[index + 1]["source"]["source_time_s"] for index in range(len(samples) - 1))


def test_tighter_braking_bound_retimes_reference():
    car = [{"source_time_s": 0.0, "Speed": 36.0}, {"source_time_s": 1.0, "Speed": 180.0}, {"source_time_s": 2.0, "Speed": 36.0}]
    pos = [{"source_time_s": value, "X": "0", "Y": "0"} for value in (0.0, 1.0, 2.0)]
    relaxed = reference.build_reference(car, pos, _geometry(), max_decel_mps2=12.0, max_speed_mps=50.0, max_lateral_accel_mps2=100.0)
    conservative = reference.build_reference(car, pos, _geometry(), max_decel_mps2=0.5, max_speed_mps=50.0, max_lateral_accel_mps2=100.0)
    assert conservative["samples"][-1]["time_s"] > relaxed["samples"][-1]["time_s"]
