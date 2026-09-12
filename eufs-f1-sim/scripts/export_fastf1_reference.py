#!/usr/bin/env python3
"""Export a pinned, offline FastF1 reference artifact.

The runtime importer uses only the frozen JSON artifact and the standard
library. FastF1 is imported lazily by :func:`extract_fastf1_csv`, so this file
can be installed and used in a networking-disabled simulator image.

The generated path is a deterministic pace profile mapped to the current COTA
centerline. It is explicitly an adapted path; it must not be presented as
Russell's exact historical racing line.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sys
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence

from eufs_race_control.replay.reference import FrozenReference, ReferenceError, artifact_hash

ARTIFACT_VERSION = "1.0"
DEFAULT_EVENT = "United States Grand Prix"
DEFAULT_DRIVER = "RUS"
DEFAULT_NUMBER = "63"


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _duration_seconds(value: Any) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        raise ReferenceError("missing source timestamp")
    # FastF1 exports pandas Timedelta as ``0 days HH:MM:SS.ffffff``.
    match = re.fullmatch(r"(?:(\d+) days? )?(\d+):(\d{2}):(\d+(?:\.\d+)?)", text)
    if match:
        days, hours, minutes, seconds = match.groups()
        return (int(days or 0) * 86400 + int(hours) * 3600 + int(minutes) * 60 + float(seconds))
    try:
        return float(text)
    except ValueError as exc:
        raise ReferenceError(f"invalid source timestamp {value!r}") from exc


def _read_csv(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ReferenceError(f"empty source CSV: {path}")
    return rows


def _time_column(row: Mapping[str, Any]) -> float:
    for name in ("Time", "time", "SessionTime", "session_time"):
        if name in row and row[name] not in (None, ""):
            return _duration_seconds(row[name])
    raise ReferenceError("source row has no Time/SessionTime column")


def _row_time(row: Mapping[str, Any]) -> float:
    """Read normalized source timestamps without eagerly evaluating fallback."""

    if "source_time_s" in row and row["source_time_s"] not in (None, ""):
        return _duration_seconds(row["source_time_s"])
    return _time_column(row)


def load_source_samples(car_csv: str | Path, position_csv: str | Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, str]]:
    """Load raw channels and retain their source timestamps and hashes."""

    car_raw = _read_csv(car_csv)
    position_raw = _read_csv(position_csv)
    def prepare(rows: list[dict[str, str]], kind: str) -> list[dict[str, Any]]:
        prepared = []
        previous = None
        for row in rows:
            timestamp = _row_time(row)
            if previous is not None and timestamp <= previous:
                raise ReferenceError(f"non-monotonic {kind} source timestamps")
            previous = timestamp
            item = dict(row)
            item["source_time_s"] = timestamp
            prepared.append(item)
        return prepared
    return (
        prepare(car_raw, "car"),
        prepare(position_raw, "position"),
        {"car_csv": sha256_file(car_csv), "position_csv": sha256_file(position_csv)},
    )


def load_cota_geometry(path: str | Path) -> list[dict[str, float]]:
    rows = _read_csv(path)
    required = {"s_m", "x_m", "y_m"}
    if not required.issubset(rows[0]):
        raise ReferenceError(f"geometry must contain {sorted(required)}")
    geometry = []
    previous_s = -math.inf
    for row in rows:
        try:
            s = float(row["s_m"])
            x = float(row["x_m"])
            y = float(row["y_m"])
            yaw = float(row.get("yaw_rad", 0.0))
            curvature = float(row.get("curvature_1pm", 0.0))
        except (TypeError, ValueError) as exc:
            raise ReferenceError("non-numeric COTA geometry") from exc
        if not all(math.isfinite(v) for v in (s, x, y, yaw, curvature)) or s <= previous_s:
            raise ReferenceError("COTA geometry progress must be finite and strictly increasing")
        geometry.append({"s_m": s, "x_m": x, "y_m": y, "yaw_rad": yaw, "curvature_1pm": curvature})
        previous_s = s
    if len(geometry) < 3 or geometry[-1]["s_m"] <= 0:
        raise ReferenceError("COTA geometry is missing or too short")
    # The centerline does not duplicate its first row. Require a finite seam.
    seam = math.hypot(geometry[0]["x_m"] - geometry[-1]["x_m"], geometry[0]["y_m"] - geometry[-1]["y_m"])
    if not math.isfinite(seam) or seam > 100.0:
        raise ReferenceError(f"COTA geometry seam is not closed ({seam:.3f} m)")
    yaw_delta = abs(_angle_delta(geometry[-1].get("yaw_rad", 0.0), geometry[0].get("yaw_rad", 0.0)))
    if yaw_delta > 0.75:
        raise ReferenceError(f"COTA geometry seam heading is discontinuous ({yaw_delta:.3f} rad)")
    return geometry


def _angle_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2.0 * math.pi) - math.pi


def _interp_geometry(geometry: Sequence[Mapping[str, float]], progress: float) -> dict[str, float]:
    closing = math.hypot(geometry[0]["x_m"] - geometry[-1]["x_m"], geometry[0]["y_m"] - geometry[-1]["y_m"])
    length = geometry[-1]["s_m"] + closing
    p = min(max(progress, 0.0), length)
    pairs = list(zip(geometry, geometry[1:])) + [(geometry[-1], geometry[0])]
    for pair_index, (left, right) in enumerate(pairs):
        right_s = right["s_m"] if pair_index < len(geometry) - 1 else length
        left_s = left["s_m"]
        if p <= right_s:
            span = right_s - left_s
            alpha = 0.0 if span <= 0 else (p - left_s) / span
            yaw = left.get("yaw_rad", 0.0) + alpha * _angle_delta(left.get("yaw_rad", 0.0), right.get("yaw_rad", 0.0))
            return {
                "x_m": left["x_m"] + alpha * (right["x_m"] - left["x_m"]),
                "y_m": left["y_m"] + alpha * (right["y_m"] - left["y_m"]),
                "yaw_rad": yaw,
                "curvature_1pm": left.get("curvature_1pm", 0.0) + alpha * (right.get("curvature_1pm", 0.0) - left.get("curvature_1pm", 0.0)),
            }
    return dict(geometry[0])


def build_reference(
    car_rows: Sequence[Mapping[str, Any]],
    position_rows: Sequence[Mapping[str, Any]],
    geometry: Sequence[Mapping[str, float]],
    *,
    pace_scale: float = 1.0,
    source_hashes: Mapping[str, str] | None = None,
    metadata: Mapping[str, Any] | None = None,
    max_accel_mps2: float = 1.0,
    max_decel_mps2: float = 1.2,
    max_lateral_accel_mps2: float = 1.5,
    max_speed_mps: float = 15.0,
    wheelbase_m: float = 3.28,
    max_steering_rate_rps: float = 1.2916,
    max_curvature_1pm: float = 1.0,
) -> dict[str, Any]:
    """Create a deterministic time/progress reference on the existing path."""

    if not car_rows or not position_rows or len(geometry) < 3:
        raise ReferenceError("car, position and geometry data are all required")
    for row in position_rows:
        if not any(name in row for name in ("X", "x", "x_m")) or not any(name in row for name in ("Y", "y", "y_m")):
            raise ReferenceError("position source must contain X and Y coordinates")
        try:
            x_value = float(row.get("X", row.get("x", row.get("x_m"))))
            y_value = float(row.get("Y", row.get("y", row.get("y_m"))))
            _row_time(row)
            if not math.isfinite(x_value) or not math.isfinite(y_value):
                raise ValueError("non-finite position")
        except (TypeError, ValueError, ReferenceError) as exc:
            raise ReferenceError("invalid position source sample") from exc
    if not math.isfinite(pace_scale) or pace_scale <= 0:
        raise ReferenceError("pace_scale must be positive and finite")
    closing_length = math.hypot(geometry[0]["x_m"] - geometry[-1]["x_m"], geometry[0]["y_m"] - geometry[-1]["y_m"])
    if not math.isfinite(closing_length) or closing_length > 100.0:
        raise ReferenceError(f"COTA geometry seam is not closed ({closing_length:.3f} m)")
    if abs(_angle_delta(float(geometry[-1].get("yaw_rad", 0.0)), float(geometry[0].get("yaw_rad", 0.0)))) > 0.75:
        raise ReferenceError("COTA geometry seam heading is discontinuous")
    length = float(geometry[-1]["s_m"]) + closing_length
    if length <= 0 or any(abs(float(row.get("curvature_1pm", 0.0))) > max_curvature_1pm for row in geometry):
        raise ReferenceError("geometry curvature exceeds declared bound")
    if any(not math.isfinite(value) or value <= 0 for value in (max_speed_mps, max_accel_mps2, max_decel_mps2, max_lateral_accel_mps2, wheelbase_m, max_steering_rate_rps)):
        raise ReferenceError("invalid native speed/acceleration bounds")
    source_times: list[float] = []
    raw_speeds: list[float] = []
    raw_distance = 0.0
    for row in car_rows:
        timestamp = _row_time(row)
        if source_times and timestamp <= source_times[-1]:
            raise ReferenceError("car timestamps must be strictly increasing")
        try:
            source_speed = float(row["Speed"]) / 3.6
        except (KeyError, TypeError, ValueError) as exc:
            raise ReferenceError("car CSV must contain Speed in km/h") from exc
        if not math.isfinite(source_speed) or source_speed < 0:
            raise ReferenceError("invalid source speed")
        if source_times:
            raw_distance += 0.5 * (raw_speeds[-1] + source_speed) * (timestamp - source_times[-1])
        source_times.append(timestamp)
        raw_speeds.append(source_speed)
    if raw_distance <= 0:
        raise ReferenceError("source pace has no duration")
    # Resample the current map at <=1.5 m so curvature and the closing segment
    # are represented even when source telemetry is sparse.
    dense_count = max(3, int(math.ceil(length / 1.5)))
    progress = [length * index / dense_count for index in range(dense_count + 1)]
    poses = [_interp_geometry(geometry, value) for value in progress]
    if any(abs(float(pose["curvature_1pm"])) > max_curvature_1pm for pose in poses):
        raise ReferenceError("dense geometry curvature exceeds declared bound")
    source_distance = [0.0]
    for index in range(1, len(source_times)):
        source_distance.append(source_distance[-1] + 0.5 * (raw_speeds[index - 1] + raw_speeds[index]) * (source_times[index] - source_times[index - 1]))
    def source_speed_at(progress_m: float) -> float:
        target = progress_m / length * raw_distance
        for index in range(1, len(source_distance)):
            if target <= source_distance[index]:
                span = source_distance[index] - source_distance[index - 1]
                alpha = 0.0 if span <= 0 else (target - source_distance[index - 1]) / span
                return raw_speeds[index - 1] + alpha * (raw_speeds[index] - raw_speeds[index - 1])
        return raw_speeds[-1]
    def source_time_at(progress_m: float) -> float:
        target = progress_m / length * raw_distance
        for index in range(1, len(source_distance)):
            if target <= source_distance[index]:
                span = source_distance[index] - source_distance[index - 1]
                alpha = 0.0 if span <= 0 else (target - source_distance[index - 1]) / span
                return source_times[index - 1] + alpha * (source_times[index] - source_times[index - 1])
        return source_times[-1]
    map_speed_scale = length / raw_distance
    target_speeds = [min(max_speed_mps, source_speed_at(value) * pace_scale * map_speed_scale) for value in progress]
    curvatures = [float(pose["curvature_1pm"]) for pose in poses]
    for index, curvature in enumerate(curvatures):
        if abs(curvature) > 1e-9:
            target_speeds[index] = min(target_speeds[index], math.sqrt(max_lateral_accel_mps2 / abs(curvature)))
    # Steering-rate is a speed-dependent limit. Cap the adjacent endpoints so
    # normal map curvature transitions are adapted before acceleration passes.
    steering = [math.atan(wheelbase_m * curvature) for curvature in curvatures]
    for index in range(1, len(progress)):
        delta = abs(_angle_delta(steering[index - 1], steering[index]))
        if delta > 1e-12:
            steering_speed = max_steering_rate_rps * (progress[index] - progress[index - 1]) / delta
            target_speeds[index - 1] = min(target_speeds[index - 1], steering_speed)
            target_speeds[index] = min(target_speeds[index], steering_speed)
    # The final row is the closed-loop seam duplicate, so it must share the
    # exact speed of the first row rather than introducing a fictitious jump.
    # Keep that tie in force *throughout* the periodic passes.  Tying the
    # endpoints only after the last pass can raise the final outgoing
    # acceleration above the forward bound (the seam value was no longer part
    # of the envelope that produced the preceding row).
    seam_speed = min(target_speeds[0], target_speeds[-1])
    target_speeds[0] = target_speeds[-1] = seam_speed
    speeds = list(target_speeds)
    # Periodic forward/backward envelope in arc length. The endpoint mirrors
    # the seam speed; subsequent integration creates the closing segment.
    for _ in range(max(16, dense_count // 64)):
        speeds[0] = speeds[-1] = min(speeds[0], speeds[-1])
        for index in range(1, dense_count + 1):
            ds = progress[index] - progress[index - 1]
            speeds[index] = min(speeds[index], math.sqrt(max(0.0, speeds[index - 1] ** 2 + 2 * max_accel_mps2 * ds)))
        speeds[0] = speeds[-1] = min(speeds[0], speeds[-1])
        for index in range(dense_count - 1, -1, -1):
            ds = progress[index + 1] - progress[index]
            speeds[index] = min(speeds[index], math.sqrt(max(0.0, speeds[index + 1] ** 2 + 2 * max_decel_mps2 * ds)))
        speeds[0] = speeds[-1] = min(speeds[0], speeds[-1])
    times = [0.0]
    for index in range(1, len(progress)):
        ds = progress[index] - progress[index - 1]
        dt = 2 * ds / max(speeds[index - 1] + speeds[index], 1e-6)
        times.append(times[-1] + dt)
    # Acceleration is an outgoing interval field: sample[i] describes the
    # interval i -> i+1.  This is the convention used by the runtime
    # interpolator and avoids a one-knot stale command at a pace transition.
    accelerations = []
    for index in range(len(progress)):
        if index < dense_count:
            dt = times[index + 1] - times[index]
            accelerations.append((speeds[index + 1] - speeds[index]) / dt)
        else:
            # The duplicated seam knot has the same speed as the first knot;
            # its outgoing closed-loop interval therefore has zero acceleration.
            accelerations.append(0.0)
    # Check every outgoing interval, including the authored seam tie.  The
    # final row is a duplicate and has no second authored time interval.
    for index in range(1, len(progress)):
        interval_accel = accelerations[index - 1]
        if interval_accel > max_accel_mps2 + 1e-5 or interval_accel < -max_decel_mps2 - 1e-5:
            raise ReferenceError("adapted reference violates native acceleration envelope")
        steering_previous = math.atan(wheelbase_m * curvatures[index - 1])
        steering_current = math.atan(wheelbase_m * curvatures[index])
        if abs(steering_current - steering_previous) / max(times[index] - times[index - 1], 1e-6) > max_steering_rate_rps + 1e-5:
            raise ReferenceError("adapted reference violates steering-rate bound")
    samples: list[dict[str, Any]] = []
    for index, pose in enumerate(poses):
        samples.append({
            "time_s": times[index], "progress_m": progress[index],
            "x_m": pose["x_m"], "y_m": pose["y_m"], "yaw_rad": pose["yaw_rad"],
            "speed_mps": speeds[index], "source_speed_mps": source_speed_at(progress[index]),
            "acceleration_mps2": accelerations[index], "curvature_1pm": curvatures[index],
            "source": {"source_time_s": source_time_at(progress[index])},
        })
    car_timestamps = list(source_times)
    position_timestamps = [_row_time(row) for row in position_rows]
    provenance = dict(metadata or {})
    provenance.update({
        "event": provenance.get("event", DEFAULT_EVENT),
        "year": provenance.get("year", 2025),
        "session": provenance.get("session", "Race"),
        "driver": provenance.get("driver", DEFAULT_DRIVER),
        "driver_number": provenance.get("driver_number", DEFAULT_NUMBER),
        "candidate_lap": provenance.get("candidate_lap", 28),
        "map_profile_hash": provenance.get("map_profile_hash"),
        "vehicle_profile_hash": provenance.get("vehicle_profile_hash"),
        "source_hashes": dict(source_hashes or {}),
        "source_timestamps_s": {
            "car": car_timestamps,
            "position": position_timestamps,
        },
        "source_gaps_s": {
            "car_max": max((b - a for a, b in zip(car_timestamps, car_timestamps[1:])), default=0.0),
            "position_max": max((b - a for a, b in zip(position_timestamps, position_timestamps[1:])), default=0.0),
        },
        "pace_scale": pace_scale,
        "source_lap_duration_s": source_times[-1] - source_times[0],
        "adapted_lap_duration_s": times[-1],
        "map_speed_scale": map_speed_scale,
        "closing_segment_length_m": closing_length,
        "acceleration_bounds_mps2": {"forward": max_accel_mps2, "braking": -max_decel_mps2},
        "lateral_acceleration_bound_mps2": max_lateral_accel_mps2,
        "max_speed_mps": max_speed_mps,
        "wheelbase_m": wheelbase_m,
        "max_steering_rate_rps": max_steering_rate_rps,
        "path_label": "COTA layout-reference adapted path; approximate pace, not exact historical racing line",
        "units": {"time": "s", "progress": "m", "speed": "m/s", "source_speed": "km/h", "source_speed_mps": "m/s", "source_position": "tenths_of_metre", "curvature": "1/m"},
        "geometry_length_m": length,
        "lap_duration_s": samples[-1]["time_s"],
        "sample_count": len(samples),
    })
    artifact = {"artifact_version": ARTIFACT_VERSION, "provenance": provenance, "samples": samples}
    artifact["artifact_sha256"] = artifact_hash(artifact)
    return artifact


def save_artifact(artifact: Mapping[str, Any], output: str | Path) -> str:
    expected = artifact_hash(artifact)
    payload = dict(artifact)
    payload["artifact_sha256"] = expected
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=destination.name + ".", suffix=".tmp", dir=str(destination.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
    return expected


def extract_fastf1_csv(output_dir: str | Path, *, year: int = 2025, event: str = DEFAULT_EVENT, session_code: str = "R", driver: str = DEFAULT_DRIVER, lap_number: int = 28, cache_dir: str | Path | None = None) -> tuple[Path, Path, dict[str, Any]]:
    """Extract one selected lap with FastF1, with import/network kept offline."""

    # Lazy import: runtime and artifact readers do not depend on FastF1.
    try:
        import fastf1  # type: ignore
    except ImportError as exc:
        raise ReferenceError("FastF1 is required only for offline extraction") from exc
    if cache_dir:
        fastf1.Cache.enable_cache(str(cache_dir))
    session = fastf1.get_session(year, event, session_code)
    session.load(laps=True, telemetry=True, weather=False, messages=True)
    laps = session.laps.pick_drivers(driver)
    # Keep the declared clean-lap policy visible in the export path. A caller
    # can intentionally choose a non-clean lap only by changing this function.
    clean = laps.pick_wo_box().pick_accurate().pick_track_status("1").pick_not_deleted()
    clean = clean[clean["LapNumber"] != 1]
    selected = clean[clean["LapNumber"] == lap_number]
    if len(selected) != 1:
        raise ReferenceError(f"expected one lap {lap_number} for {driver}, found {len(selected)}")
    lap = selected.iloc[0]
    car = lap.get_car_data().add_distance()
    position = lap.get_pos_data()
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    car_path, position_path = destination / "car_candidate_lap.csv", destination / "position_candidate_lap.csv"
    car.to_csv(car_path, index=False)
    position.to_csv(position_path, index=False)
    actual_number = DEFAULT_NUMBER
    try:
        result = session.results[session.results["Abbreviation"] == driver]
        if len(result):
            actual_number = str(result.iloc[0]["DriverNumber"])
    except (AttributeError, KeyError, TypeError, IndexError):
        pass
    metadata = {"fastf1_version": getattr(fastf1, "__version__", "unknown"), "year": year, "event": event, "session": session_code, "driver": driver, "driver_number": actual_number, "candidate_lap": lap_number, "clean_lap_filters": ["pick_wo_box", "pick_accurate", "pick_track_status('1')", "pick_not_deleted", "exclude_lap_1"]}
    return car_path, position_path, metadata


def _cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--car-csv")
    parser.add_argument("--position-csv")
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata-json")
    parser.add_argument("--pace-scale", type=float, default=1.0)
    parser.add_argument("--map-profile-hash")
    parser.add_argument("--vehicle-profile-hash")
    parser.add_argument("--extract-dir")
    parser.add_argument("--cache-dir")
    args = parser.parse_args()
    if args.extract_dir:
        car, position, metadata = extract_fastf1_csv(args.extract_dir, cache_dir=args.cache_dir)
    elif args.car_csv and args.position_csv:
        car, position, metadata = args.car_csv, args.position_csv, {}
    else:
        parser.error("provide --car-csv and --position-csv, or --extract-dir")
    cars, positions, hashes = load_source_samples(car, position)
    geometry = load_cota_geometry(args.geometry)
    if args.metadata_json:
        metadata.update(json.loads(Path(args.metadata_json).read_text(encoding="utf-8")))
    if args.map_profile_hash:
        metadata["map_profile_hash"] = args.map_profile_hash
    if args.vehicle_profile_hash:
        metadata["vehicle_profile_hash"] = args.vehicle_profile_hash
    artifact = build_reference(cars, positions, geometry, pace_scale=args.pace_scale, source_hashes=hashes, metadata=metadata)
    save_artifact(artifact, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli())
