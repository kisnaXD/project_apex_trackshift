"""Resolve named EUFS tracks for load_car.launch.py.

Unknown names and missing/mismatched COTA hashes fail. There is no silent
fallback to small_track.
"""

from __future__ import annotations

import hashlib
import math
from os.path import isfile, join

import yaml
from ament_index_python.packages import get_package_share_directory

ALLOWED_TRACKS = ("cota", "small_track")


def _sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        digest.update(handle.read())
    return digest.hexdigest()


def _car_start(csv_path: str):
    with open(csv_path, encoding="utf-8") as handle:
        header = handle.readline()
        if not header:
            raise RuntimeError(f"{csv_path} is empty")
        for line in handle:
            parts = [item.strip() for item in line.split(",")]
            if parts and parts[0] == "car_start":
                return float(parts[1]), float(parts[2]), float(parts[3])
    raise RuntimeError(f"{csv_path} has no car_start row")


def _rows(csv_path: str, tag: str):
    points = []
    with open(csv_path, encoding="utf-8") as handle:
        handle.readline()
        for line in handle:
            parts = [item.strip() for item in line.split(",")]
            if parts and parts[0] == tag:
                points.append((float(parts[1]), float(parts[2])))
    return points


def _align_cota_spawn(csv_path: str, x: float, y: float, yaw: float):
    """Put the chassis on the racing line, heading along the orange S/F gate.

    EUFS car_start yaw is the centerline tangent, but a stale CSV or a mesh
    that looks cocked vs the cones is fixed here from the four big_orange
    cones every launch (csv hashes stay valid).
    """
    oranges = _rows(csv_path, "big_orange")
    if len(oranges) < 4:
        return x, y, yaw
    mean_y = sum(point[1] for point in oranges) / len(oranges)
    left = [point for point in oranges if point[1] >= mean_y]
    right = [point for point in oranges if point[1] < mean_y]
    if not left or not right:
        return x, y, yaw
    lx = sum(point[0] for point in left) / len(left)
    ly = sum(point[1] for point in left) / len(left)
    rx = sum(point[0] for point in right) / len(right)
    ry = sum(point[1] for point in right) / len(right)
    # Left→right across the gate; +90° CCW is COTA travel direction.
    fwd_x, fwd_y = -(ry - ly), (rx - lx)
    norm = math.hypot(fwd_x, fwd_y)
    if norm < 1e-6:
        return x, y, yaw
    fwd_x /= norm
    fwd_y /= norm
    gate_x = 0.5 * (lx + rx)
    gate_y = 0.5 * (ly + ry)
    # Origin is the rear axle. Nose is ~4.0 m ahead; sit 1.0 m behind the gate.
    back = 4.0 + 1.0
    aligned_x = gate_x - fwd_x * back
    aligned_y = gate_y - fwd_y * back
    aligned_yaw = math.atan2(fwd_y, fwd_x)
    return aligned_x, aligned_y, aligned_yaw


def resolve_track(name: str) -> dict:
    track = (name or "").strip()
    if track not in ALLOWED_TRACKS:
        raise RuntimeError(f"Unknown track {track!r}. Allowed: {ALLOWED_TRACKS}")

    share = get_package_share_directory("eufs_tracks")
    world = join(share, "worlds", f"{track}.world")
    model = join(share, "models", track, "model.sdf")
    csv_path = join(share, "csv", f"{track}.csv")
    missing = [path for path in (world, model, csv_path) if not isfile(path)]
    if missing:
        raise RuntimeError(
            f"track {track} is missing {missing} (no fallback to another circuit)"
        )

    if track == "cota":
        provenance_path = join(share, "models", track, "provenance.yaml")
        if not isfile(provenance_path):
            raise RuntimeError("cota is missing provenance.yaml (no fallback)")
        with open(provenance_path, encoding="utf-8") as handle:
            provenance = yaml.safe_load(handle)
        expected = provenance.get("hashes") or {}
        actual = {"csv": _sha256(csv_path), "model_sdf": _sha256(model), "world": _sha256(world)}
        mismatched = [
            key for key in ("csv", "model_sdf", "world")
            if expected.get(key) and expected[key] != actual[key]
        ]
        if mismatched:
            raise RuntimeError(f"cota hash mismatch for {mismatched} (no fallback)")

    x, y, yaw = _car_start(csv_path)
    if track == "cota":
        x, y, yaw = _align_cota_spawn(csv_path, x, y, yaw)
    return {
        "name": track,
        "world": world,
        "track_file": model,
        "csv": csv_path,
        "x": x,
        "y": y,
        "yaw": yaw,
    }
