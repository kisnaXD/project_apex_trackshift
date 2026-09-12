"""Discover and resolve EUFS track assets for ``load_car.launch.py``.

Tracks are attached to the normal ``eufs_tracks`` package layout: a world in
``worlds/<name>.world``, model in ``models/<name>/model.sdf`` and cone CSV in
``csv/<name>.csv``.  Optional geometry/provenance files live beside the model
or in ``<name>/``.  This keeps adding a map an asset/configuration operation;
the launch code does not need a map-name branch.
"""

from __future__ import annotations

import hashlib
from glob import glob
from os.path import isfile, join, normpath

import yaml
try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # pragma: no cover - ROS supplies this in launch
    def get_package_share_directory(_package):
        raise RuntimeError("ament_index_python is unavailable")

def _tracks_share():
    return get_package_share_directory("eufs_tracks")


def _asset_paths(share: str, name: str) -> dict:
    """Return standard asset paths and optional map-owned metadata paths."""
    world = join(share, "worlds", f"{name}.world")
    model = join(share, "models", name, "model.sdf")
    csv_path = join(share, "csv", f"{name}.csv")
    # ``<name>/`` is the convenient overlay attachment point.  Keep the
    # model directory as a fallback because that is how installed EUFS models
    # are packaged and how the existing COTA metadata is installed.
    roots = (join(share, name), join(share, "models", name))
    geometry_root = next((root for root in roots if isfile(join(root, "centerline.csv"))), roots[0])
    profile_path = next((join(root, filename)
                         for root in roots
                         for filename in ("metadata.yaml", "provenance.yaml")
                         if isfile(join(root, filename))), "")
    return {
        "name": name,
        "world": world,
        "track_file": model,
        "csv": csv_path,
        "centerline": join(geometry_root, "centerline.csv"),
        "boundaries": join(geometry_root, "boundaries.csv"),
        "profile": profile_path,
    }


def _discovered_assets(share: str) -> dict:
    """Discover complete world/model/CSV triplets in deterministic order."""
    worlds = set()
    csvs = set()
    models = set()
    for filename in glob(join(share, "worlds", "*.world")):
        worlds.add(filename.rsplit("/", 1)[-1][:-len(".world")])
    for filename in glob(join(share, "csv", "*.csv")):
        csvs.add(filename.rsplit("/", 1)[-1][:-len(".csv")])
    for filename in glob(join(share, "models", "*", "model.sdf")):
        models.add(filename.rsplit("/", 2)[-2])
    return {
        name: _asset_paths(share, name)
        for name in sorted(worlds & csvs & models)
    }


def available_tracks():
    """Return complete, valid track names from the installed workspace."""
    try:
        return tuple(_discovered_assets(_tracks_share()))
    except Exception:
        # Dashboard discovery runs during startup; an unavailable ament index
        # should not make importing this pure selector fail.
        return ()


discover_tracks = available_tracks


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


def _profile(path: str) -> dict:
    if not path:
        return {}
    with open(path, encoding="utf-8") as handle:
        value = yaml.safe_load(handle) or {}
    return value if isinstance(value, dict) else {}


def _nested(profile: dict, *keys):
    value = profile
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _profile_spawn(profile: dict):
    """Read an authored spawn/alignment policy without knowing map identity."""
    # ``launch_spawn`` is an integration override for an asset whose runtime
    # pose is deliberately different from the raw CSV car_start row.  It is
    # map metadata, so adding another map never requires a selector branch.
    spawn = profile.get("launch_spawn") or profile.get("spawn")
    if not isinstance(spawn, dict):
        return None
    values = tuple(spawn.get(key) for key in ("spawn_x_m", "spawn_y_m", "spawn_yaw_rad"))
    if all(value is not None for value in values):
        return tuple(float(value) for value in values)
    return None


def _spawn_back_m(profile: dict):
    """Return optional authored grid spawn distance behind the gate."""
    for keys in (("spawn_arclength_back_m",), ("grid", "spawn_arclength_back_m"),
                 ("spawn", "arclength_back_m")):
        value = _nested(profile, *keys)
        if value is not None:
            return float(value)
    return None


def _validate_hashes(assets: dict, profile: dict):
    """Validate optional provenance hashes for any map that supplies them."""
    expected = profile.get("hashes") or {}
    if not isinstance(expected, dict):
        return
    actual = {
        key: _sha256(assets[key])
        for key in ("csv", "track_file", "world")
        if isfile(assets[key])
    }
    # Existing provenance calls the model hash ``model_sdf``; accept the
    # standard asset key too so new maps can use either spelling.
    expected_keys = {"csv": "csv", "model_sdf": "track_file", "world": "world"}
    mismatched = [key for key, asset_key in expected_keys.items()
                  if expected.get(key) and expected[key] != actual.get(asset_key)]
    if mismatched:
        raise RuntimeError(f"{assets['name']} hash mismatch for {mismatched} (no fallback)")


def _boundary_strip_path(share: str, profile: dict):
    value = _nested(profile, "cone_policy", "white_strip_mesh")
    if not value:
        value = profile.get("boundary_strips")
    if not value:
        return ""
    value = str(value)
    if value.startswith("/"):
        return value
    return normpath(join(share, value))


def resolve_track(name: str) -> dict:
    track = (name or "").strip()
    share = _tracks_share()
    discovered = _discovered_assets(share)
    assets = discovered.get(track)
    if assets is None:
        raise RuntimeError(f"Unknown track {track!r}. Available: {tuple(discovered)}")

    missing = [path for path in (assets["world"], assets["track_file"], assets["csv"])
               if not isfile(path)]
    if missing:
        raise RuntimeError(
            f"track {track} is missing {missing} (no fallback to another circuit)"
        )
    profile = _profile(assets["profile"])
    _validate_hashes(assets, profile)
    x, y, yaw = _car_start(assets["csv"])
    spawn = _profile_spawn(profile)
    if isinstance(spawn, tuple):
        x, y, yaw = spawn
    assets.update({
        "boundary_strips": _boundary_strip_path(share, profile),
        "profile_data": profile,
        "spawn_arclength_back_m": _spawn_back_m(profile),
        "x": x,
        "y": y,
        "yaw": yaw,
    })
    # Keep the resolve_track dictionary shape consumed by dashboard/launch.
    return {
        **assets,
    }
