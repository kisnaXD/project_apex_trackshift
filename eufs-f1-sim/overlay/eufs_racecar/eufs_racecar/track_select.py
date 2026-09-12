"""Resolve named EUFS tracks for load_car.launch.py.

Unknown names and missing/mismatched COTA hashes fail. There is no silent
fallback to small_track.
"""

from __future__ import annotations

import hashlib
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
    return {
        "name": track,
        "world": world,
        "track_file": model,
        "csv": csv_path,
        "x": x,
        "y": y,
        "yaw": yaw,
    }
