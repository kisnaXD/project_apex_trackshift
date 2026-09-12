#!/usr/bin/env python3
"""Bind a validated frozen reference and installed EUFS mesh directory to SDF.

Gazebo Classic cannot resolve ROS ``package://`` mesh URIs in an SDF spawned
directly by ``spawn_entity.py``.  The native controls launch can call this
small preloader once per run, then spawn the resulting temporary SDF.
"""

from __future__ import annotations

import argparse
import html
import json
import re
from pathlib import Path


def _mesh_dir(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("eufs_racecar")) / "meshes"
    except Exception as exc:  # pragma: no cover - only used outside ROS
        raise RuntimeError("pass --mesh-dir when ament_index is unavailable") from exc


def _validate_reference(path: Path, expected_hash: str | None) -> str:
    from eufs_race_control.replay.reference import FrozenReference

    reference = FrozenReference.load(path, expected_hash=expected_hash)
    return str(reference.artifact["artifact_sha256"])


def prepare(template: Path, reference: Path, output: Path, mesh_dir: Path, expected_hash: str | None) -> str:
    artifact_hash = _validate_reference(reference, expected_hash)
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"mesh directory does not exist: {mesh_dir}")
    required_meshes = (
        "chassis.STL", "front_wing.STL", "rear_wing.STL",
        "left_rear_wheel.STL", "right_rear_wheel.STL",
        "left_front_wheel.STL", "right_front_wheel.STL",
    )
    missing = [name for name in required_meshes if not (mesh_dir / name).is_file()]
    if missing:
        raise FileNotFoundError("missing EUFS replay meshes: " + ", ".join(missing))
    text = template.read_text(encoding="utf-8")
    # gazebo_ros/spawn_entity.py passes file contents as a Unicode string to
    # lxml; an XML declaration carrying an encoding is invalid in that mode.
    text = re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", text, count=1)
    text = text.replace("REPLAY_REFERENCE_FILE", html.escape(str(reference.resolve()), quote=True))
    text = text.replace("REPLAY_MESH_DIR", html.escape(str(mesh_dir), quote=True))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    return artifact_hash


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mesh-dir")
    parser.add_argument("--expected-hash")
    args = parser.parse_args()
    digest = prepare(args.template, args.reference, args.output, _mesh_dir(args.mesh_dir), args.expected_hash)
    print(json.dumps({"sdf": str(args.output), "reference": str(args.reference.resolve()), "artifact_sha256": digest}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
