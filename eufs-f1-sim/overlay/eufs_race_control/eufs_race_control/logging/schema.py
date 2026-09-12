"""Schema helpers for causal race-control evidence.

The logging package deliberately depends only on the Python standard library.
The records are ordinary dictionaries so ROS adapters and offline tools can use
the same format without importing ROS in the control process.
"""

from __future__ import annotations

import copy
import math
import re
import time
from typing import Any, Mapping, MutableMapping

SCHEMA_VERSION = "1.0"
ID_FIELDS = (
    "state_snapshot_id",
    "decision_id",
    "candidate_id",
    "trajectory_id",
    "command_sequence",
)
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def validate_identifier(value: str, label: str = "identifier") -> str:
    """Validate a value before it can become a filename or run directory."""

    if not isinstance(value, str) or not _SAFE_ID.fullmatch(value):
        raise ValueError(
            f"unsafe {label}; expected 1-128 ASCII letters, digits, '.', '_' or '-': {value!r}"
        )
    if value in {".", ".."} or "/" in value or "\\" in value:
        raise ValueError(f"unsafe {label}: {value!r}")
    return value


def json_safe(value: Any) -> Any:
    """Return a JSON-compatible copy, replacing non-finite numbers by null.

    This also prevents custom mutable objects from crossing the recorder queue.
    Unknown objects are represented by their string form only when they are
    already scalar-like; arbitrary objects are rejected to avoid silently
    losing causal data.
    """

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    # numpy scalar values are intentionally handled without importing numpy.
    item = getattr(value, "item", None)
    if callable(item):
        try:
            return json_safe(item())
        except Exception:
            pass
    raise TypeError(f"record contains non-JSON value {type(value).__name__}")


def make_record(
    *,
    component: str,
    record_type: str,
    sim_time: float | None = None,
    wall_time: float | None = None,
    run_id: str = "unassigned",
    epoch_id: int = 0,
    scenario_id: str = "default",
    ids: Mapping[str, Any] | None = None,
    data: Mapping[str, Any] | None = None,
    schema_version: str = SCHEMA_VERSION,
) -> dict[str, Any]:
    """Construct and validate the common causal record envelope."""

    if not isinstance(component, str) or not component:
        raise ValueError("component must be a non-empty string")
    if not isinstance(record_type, str) or not record_type:
        raise ValueError("record_type must be a non-empty string")
    if not isinstance(epoch_id, int) or isinstance(epoch_id, bool) or epoch_id < 0:
        raise ValueError("epoch_id must be a non-negative integer")
    causal_ids = {field: None for field in ID_FIELDS}
    if ids:
        causal_ids.update(dict(ids))
    record = {
        "schema_version": schema_version,
        "run_id": run_id,
        "epoch_id": epoch_id,
        "scenario_id": scenario_id,
        "sim_time": sim_time,
        "wall_time": time.monotonic() if wall_time is None else wall_time,
        "component": component,
        "record_type": record_type,
        "ids": causal_ids,
        "data": {} if data is None else dict(data),
    }
    # The sanitization is also an early validation step for callers.
    return json_safe(record)


def normalize_record(
    record: Mapping[str, Any],
    *,
    defaults: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Normalize a contract dictionary while preserving all user data."""

    if not isinstance(record, Mapping):
        raise TypeError("record must be a mapping")
    source = dict(defaults or {})
    source.update(copy.deepcopy(dict(record)))
    if not source.get("component") or not source.get("record_type"):
        raise ValueError("record requires component and record_type")
    for name, value in (
        ("schema_version", SCHEMA_VERSION),
        ("run_id", "unassigned"),
        ("epoch_id", 0),
        ("scenario_id", "default"),
        ("sim_time", None),
        ("wall_time", time.monotonic()),
    ):
        source.setdefault(name, value)
    ids = {field: None for field in ID_FIELDS}
    ids.update(dict(source.get("ids") or {}))
    source["ids"] = ids
    source["data"] = dict(source.get("data") or {})
    if not isinstance(source["epoch_id"], int) or isinstance(source["epoch_id"], bool) or source["epoch_id"] < 0:
        raise ValueError("epoch_id must be a non-negative integer")
    return json_safe(source)


__all__ = ["SCHEMA_VERSION", "ID_FIELDS", "json_safe", "make_record", "normalize_record", "validate_identifier"]
