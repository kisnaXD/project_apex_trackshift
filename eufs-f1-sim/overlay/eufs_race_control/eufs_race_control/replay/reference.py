"""Runtime-safe frozen reference loading without FastF1 or pandas."""

from __future__ import annotations

import copy
import bisect
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping


class ReferenceError(ValueError):
    pass


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_plain(item) for item in value]
    if isinstance(value, list):
        return [_plain(item) for item in value]
    return value


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def artifact_hash(artifact: Mapping[str, Any]) -> str:
    unsigned = {key: value for key, value in artifact.items() if key != "artifact_sha256"}
    encoded = json.dumps(_plain(unsigned), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def _angle_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class FrozenReference:
    """Validated immutable artifact with interpolated time sampling."""

    artifact: Mapping[str, Any]

    def __post_init__(self) -> None:
        copied = copy.deepcopy(dict(self.artifact))
        if not isinstance(copied.get("samples"), list) or not copied["samples"]:
            raise ReferenceError("frozen reference has no samples")
        if copied.get("artifact_sha256") != artifact_hash(copied):
            raise ReferenceError("frozen reference hash mismatch")
        previous = -math.inf
        previous_progress = -math.inf
        required = ("time_s", "progress_m", "x_m", "y_m", "yaw_rad", "speed_mps", "curvature_1pm")
        for index, sample in enumerate(copied["samples"]):
            if not isinstance(sample, Mapping):
                raise ReferenceError("reference samples must be objects")
            missing = [field for field in required if field not in sample]
            if missing:
                raise ReferenceError("reference sample is missing " + ", ".join(missing))
            try:
                timestamp = float(sample["time_s"])
                progress = float(sample["progress_m"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ReferenceError("reference sample is missing time/progress") from exc
            finite_fields = ("time_s", "progress_m", "x_m", "y_m", "yaw_rad", "speed_mps", "curvature_1pm")
            try:
                values = {field: float(sample[field]) for field in finite_fields}
            except (TypeError, ValueError) as exc:
                raise ReferenceError("reference pose/speed/curvature fields must be numeric") from exc
            if any(not math.isfinite(value) for value in values.values()):
                raise ReferenceError("reference pose/speed/curvature fields must be finite")
            if values["speed_mps"] < 0.0:
                raise ReferenceError("reference speed_mps must be non-negative")
            try:
                acceleration = float(sample.get("acceleration_mps2", 0.0))
            except (TypeError, ValueError) as exc:
                raise ReferenceError("reference acceleration_mps2 must be numeric") from exc
            if not math.isfinite(acceleration):
                raise ReferenceError("reference acceleration_mps2 must be finite")
            if not math.isfinite(timestamp) or timestamp <= previous:
                raise ReferenceError("reference timestamps must be finite and strictly increasing")
            if not math.isfinite(progress) or progress < previous_progress:
                raise ReferenceError("reference progress must be finite and monotonic")
            if index == 0 and abs(timestamp) > 1e-12:
                raise ReferenceError("reference timestamps must use a finite zero origin")
            previous, previous_progress = timestamp, progress
        object.__setattr__(self, "artifact", _freeze(copied))

    @classmethod
    def load(cls, path: str | Path, *, expected_hash: str | None = None, expected_source_hashes: Mapping[str, str] | None = None) -> "FrozenReference":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        reference = cls(payload)
        actual_hash = reference.artifact["artifact_sha256"]
        if expected_hash is not None and actual_hash != expected_hash:
            raise ReferenceError("frozen reference hash mismatch")
        if expected_source_hashes is not None:
            reference.validate_source_hashes(expected_source_hashes)
        return reference

    def validate_source_hashes(self, expected: Mapping[str, str]) -> None:
        actual = (self.artifact.get("provenance") or {}).get("source_hashes") or {}
        if dict(actual) != dict(expected):
            raise ReferenceError("frozen reference source hash mismatch")

    def time_sample(self, time_s: float) -> Mapping[str, Any]:
        """Linearly interpolate a sample; out-of-range requests are errors."""

        if not math.isfinite(time_s):
            raise ValueError("time_s must be finite")
        samples = self.artifact["samples"]
        times = tuple(float(sample["time_s"]) for sample in samples)
        if time_s < times[0] or time_s > times[-1]:
            raise ValueError("time_s is outside frozen reference range")
        knot = bisect.bisect_left(times, time_s)
        if knot < len(samples) and times[knot] == time_s:
            return dict(samples[knot])
        right_index = bisect.bisect_right(times, time_s)
        left_index = right_index - 1
        left, right = samples[left_index], samples[right_index]
        left_time, right_time = times[left_index], times[right_index]
        tau = time_s - left_time
        dt = right_time - left_time
        v0 = float(left["speed_mps"])
        v1 = float(right["speed_mps"])
        acceleration = (v1 - v0) / dt
        traveled = max(0.0, v0 * tau + 0.5 * acceleration * tau * tau)
        segment_progress = max(float(right["progress_m"]) - float(left["progress_m"]), 0.0)
        alpha = min(1.0, traveled / segment_progress) if segment_progress > 0 else tau / dt
        result = dict(left)
        result["time_s"] = time_s
        for field in ("progress_m", "x_m", "y_m", "speed_mps", "curvature_1pm"):
            result[field] = float(left[field]) + alpha * (float(right[field]) - float(left[field]))
        result["speed_mps"] = v0 + acceleration * tau
        if "yaw_rad" in left and "yaw_rad" in right:
            result["yaw_rad"] = float(left["yaw_rad"]) + alpha * _angle_delta(float(left["yaw_rad"]), float(right["yaw_rad"]))
        # acceleration_mps2 is an outgoing interval field.  At an interior
        # time it therefore belongs to the left knot; at an exact knot the
        # branch above returns that knot's own outgoing value.
        result["acceleration_mps2"] = float(left.get("acceleration_mps2", acceleration))
        result["interpolated"] = True
        return result

    def evaluate_schedule(self, observed: Iterable[Mapping[str, Any]], *, position_tolerance_m: float = 5.0, speed_tolerance_mps: float = 8.0) -> dict[str, Any]:
        errors = []
        missing = 0
        for item in observed:
            try:
                expected = self.time_sample(float(item["time_s"]))
                position_error = math.hypot(float(item["x_m"]) - float(expected["x_m"]), float(item["y_m"]) - float(expected["y_m"]))
            except (KeyError, TypeError, ValueError):
                missing += 1
                continue
            speed_error = None
            speed_available = "speed_mps" in item and item["speed_mps"] is not None
            if speed_available:
                try:
                    speed_error = abs(float(item["speed_mps"]) - float(expected["speed_mps"]))
                except (TypeError, ValueError):
                    speed_available = False
            errors.append({"time_s": float(item["time_s"]), "position_error_m": position_error, "speed_error_mps": speed_error, "speed_available": speed_available})
        speed_errors = [row["speed_error_mps"] for row in errors if row["speed_error_mps"] is not None]
        return {
            "sample_count": len(errors),
            "missing_count": missing,
            "speed_unavailable_count": sum(not row["speed_available"] for row in errors),
            "max_position_error_m": max((row["position_error_m"] for row in errors), default=None),
            "max_speed_error_mps": max(speed_errors, default=None),
            "within_tolerance": bool(errors) and missing == 0 and all(row["speed_available"] for row in errors) and max(row["position_error_m"] for row in errors) <= position_tolerance_m and max(speed_errors, default=math.inf) <= speed_tolerance_mps,
            "errors": errors,
        }


def load_reference(path: str | Path, *, expected_hash: str | None = None, expected_source_hashes: Mapping[str, str] | None = None) -> FrozenReference:
    return FrozenReference.load(path, expected_hash=expected_hash, expected_source_hashes=expected_source_hashes)


ReferenceArtifact = FrozenReference
ReplayArtifact = FrozenReference

__all__ = ["FrozenReference", "ReferenceArtifact", "ReplayArtifact", "ReferenceError", "artifact_hash", "load_reference"]
