"""Headless-safe data model for the EUFS race investigation GUI.

This module deliberately contains no ROS or Qt imports.  It is the contract
between live adapters, the JSONL recorder and the presentation widgets.  The
same model is used by the replay viewer, so a paused display cannot silently
change when newer live records arrive.
"""

from __future__ import annotations

import copy
import bisect
import json
import math
import statistics
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple


RECORD_SCHEMA_VERSION = "1.0"


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _normalise_value(value: Any) -> Tuple[Any, bool, Optional[str]]:
    """Keep zero as a valid value, while marking NaN/Inf unavailable."""
    if value is None:
        return None, False, "source did not provide a value"
    if isinstance(value, float) and not math.isfinite(value):
        return None, False, "non-finite measurement"
    return value, True, None


def _path_text(parts: Sequence[Any]) -> str:
    return ".".join(str(part) for part in parts)


@dataclass(frozen=True)
class ChannelSpec:
    channel_id: str
    source: str = "record"
    car: str = ""
    units: str = ""
    shape: str = "scalar"  # scalar, vector, categorical, structured
    classification: str = "measurement"  # measurement/estimate/prediction/truth
    cadence_hz: Optional[float] = None
    description: str = ""
    bounds: Optional[Tuple[float, float]] = None
    available: bool = True
    unavailable_reason: str = ""
    field_path: str = ""
    display_name: str = ""

    @property
    def numeric(self) -> bool:
        return self.shape in ("scalar", "vector") and self.available


@dataclass(frozen=True)
class ChannelSample:
    channel_id: str
    sim_time: float
    value: Any
    valid: bool
    epoch_id: Any = 0
    run_id: str = ""
    source: str = "record"
    classification: str = "measurement"
    reason: str = ""


@dataclass(frozen=True)
class Record:
    """A tolerant representation of the common recorder contract."""

    schema_version: str
    run_id: str
    epoch_id: Any
    scenario_id: str
    sim_time: float
    wall_time: float
    component: str
    record_type: str
    ids: Mapping[str, Any] = field(default_factory=dict)
    data: Mapping[str, Any] = field(default_factory=dict)
    source: str = "recorded"
    sequence: int = 0

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any], source: str = "recorded") -> "Record":
        def required_time(name: str) -> float:
            if name not in value:
                raise ValueError(f"malformed record: missing {name}")
            try:
                parsed = float(value[name])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"malformed record: invalid {name}") from exc
            if not math.isfinite(parsed):
                raise ValueError(f"malformed record: non-finite {name}")
            return parsed

        if "epoch_id" not in value:
            raise ValueError("malformed record: missing epoch_id")
        if isinstance(value["epoch_id"], bool):
            raise ValueError("malformed record: epoch_id must be a non-negative integer")
        try:
            epoch_id: Any = int(value["epoch_id"])
        except (TypeError, ValueError) as exc:
            raise ValueError("malformed record: epoch_id must be a non-negative integer") from exc
        if isinstance(value["epoch_id"], float) and value["epoch_id"] != epoch_id:
            raise ValueError("malformed record: epoch_id must be a non-negative integer")
        if epoch_id < 0:
            raise ValueError("malformed record: epoch_id must be a non-negative integer")
        raw_data = value.get("data") or {}
        if not isinstance(raw_data, Mapping):
            raise ValueError("malformed record: data must be an object")

        return cls(
            schema_version=str(value.get("schema_version", RECORD_SCHEMA_VERSION)),
            run_id=str(value.get("run_id", "")),
            epoch_id=epoch_id,
            scenario_id=str(value.get("scenario_id", "")),
            sim_time=required_time("sim_time"),
            wall_time=required_time("wall_time"),
            component=str(value.get("component", "unknown")),
            record_type=str(value.get("record_type", "event")),
            ids=copy.deepcopy(value.get("ids") or {}),
            data=copy.deepcopy(raw_data),
            source=str(value.get("source", source)),
            sequence=int(value.get("sequence", raw_data.get("sequence", 0)) or 0),
        )

    def as_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "epoch_id": self.epoch_id,
            "scenario_id": self.scenario_id,
            "sim_time": self.sim_time,
            "wall_time": self.wall_time,
            "component": self.component,
            "record_type": self.record_type,
            "ids": copy.deepcopy(dict(self.ids)),
            "data": copy.deepcopy(dict(self.data)),
        }


def flatten_scalars(value: Any, prefix: Sequence[Any] = ()) -> Iterator[Tuple[str, Any, str]]:
    """Yield dotted scalar/vector leaves without destroying structured payloads."""
    if isinstance(value, Mapping):
        for key, child in value.items():
            yield from flatten_scalars(child, (*prefix, key))
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            yield from flatten_scalars(child, (*prefix, index))
        return
    if prefix:
        yield _path_text(prefix), value, "scalar" if is_finite_number(value) else "categorical"


class ChannelRegistry:
    """Dynamic catalogue plus indexed values for every observed data field."""

    def __init__(self, max_samples_per_channel: Optional[int] = 10000) -> None:
        self._specs: Dict[str, ChannelSpec] = {}
        self._samples: Dict[str, Deque[ChannelSample]] = defaultdict(
            lambda: deque(maxlen=max_samples_per_channel)
        )
        self._max_samples = max_samples_per_channel

    def register(self, spec: ChannelSpec, *, replace: bool = False) -> ChannelSpec:
        if replace or spec.channel_id not in self._specs:
            self._specs[spec.channel_id] = spec
        return self._specs[spec.channel_id]

    def ensure(self, channel_id: str, value: Any, *, source: str = "record", car: str = "", units: str = "",
               classification: str = "measurement", description: str = "", field_path: str = "") -> ChannelSpec:
        existing = self._specs.get(channel_id)
        if existing is not None:
            if is_finite_number(value) and existing.shape in ("categorical", "structured"):
                existing = ChannelSpec(channel_id, existing.source, existing.car, existing.units or units, "scalar",
                                       existing.classification, existing.cadence_hz, existing.description or description,
                                       existing.bounds, True, "", existing.field_path or field_path,
                                       existing.display_name or field_path)
                self._specs[channel_id] = existing
            elif not existing.available and value is not None and not (isinstance(value, float) and not math.isfinite(value)):
                existing = ChannelSpec(channel_id, existing.source, existing.car, existing.units or units,
                                       existing.shape, existing.classification, existing.cadence_hz,
                                       existing.description or description, existing.bounds, True, "",
                                       existing.field_path or field_path, existing.display_name or field_path)
                self._specs[channel_id] = existing
            return existing
        shape = "scalar" if is_finite_number(value) else "categorical"
        if isinstance(value, (list, tuple, Mapping)):
            shape = "vector" if isinstance(value, (list, tuple)) else "structured"
        available = value is not None and not (isinstance(value, float) and not math.isfinite(value))
        reason = "" if available else ("source did not provide a value" if value is None else "non-finite measurement")
        return self.register(ChannelSpec(channel_id, source, car, units, shape, classification,
                                         description=description, available=available,
                                         unavailable_reason=reason, field_path=field_path, display_name=field_path))

    @staticmethod
    def _channel_id(record: Record, field_path: str, classification: str, car: str) -> str:
        # The prefix prevents ego/opponent, estimate/truth and component
        # streams from colliding while ``field_path`` remains human-readable.
        parts = [record.component, record.record_type, classification, car, field_path]
        return "/".join(str(part) for part in parts if str(part))

    def ingest_record(self, record: Record | Mapping[str, Any]) -> List[ChannelSample]:
        if not isinstance(record, Record):
            record = Record.from_mapping(record)
        samples: List[ChannelSample] = []
        data = record.data
        classification = self._classification(record)
        meta = data.get("_channel_meta", data.get("channel_metadata", {})) if isinstance(data, Mapping) else {}
        if not isinstance(meta, Mapping):
            meta = {}
        car = str(data.get("car", data.get("vehicle", data.get("car_id", "")))) if isinstance(data, Mapping) else ""
        for key, value, shape in flatten_scalars(data):
            if key.startswith("_channel_meta.") or key.startswith("channel_metadata.") or key in {"unavailable", "classification", "source", "car", "vehicle", "car_id"}:
                continue
            channel_id = self._channel_id(record, key, classification, car)
            declaration = meta.get(key, {}) if isinstance(meta, Mapping) else {}
            declaration = declaration if isinstance(declaration, Mapping) else {}
            spec = self.ensure(channel_id, value, source=str(declaration.get("source", record.source)), car=car,
                               units=str(declaration.get("units", "")), classification=classification,
                               description=str(declaration.get("description", "")), field_path=key)
            normal, valid, reason = _normalise_value(value)
            if isinstance(value, bool):
                valid, reason = True, ""
            sample = ChannelSample(channel_id, record.sim_time, normal, valid, record.epoch_id, record.run_id,
                                   record.source, spec.classification, reason or "")
            self._samples[channel_id].append(sample)
            samples.append(sample)
        # Explicit unavailable declarations remain visible and are never made zero.
        unavailable = record.data.get("unavailable") if isinstance(record.data, Mapping) else None
        if isinstance(unavailable, Mapping):
            for key, reason in unavailable.items():
                channel_id = str(key)
                canonical = self._channel_id(record, str(key), classification, car)
                old = self._specs.get(canonical)
                self.register(ChannelSpec(canonical, record.source, car=car, shape=old.shape if old else "scalar",
                                          classification=classification, available=False,
                                          unavailable_reason=str(reason), field_path=str(key), display_name=str(key)), replace=True)
        return samples

    @staticmethod
    def _classification(record: Record) -> str:
        source = record.source.casefold()
        data_source = str(record.data.get("source", "")).casefold() if isinstance(record.data, Mapping) else ""
        if source == "truth" or data_source == "truth":
            return "truth"
        value = str(record.data.get("classification", "")) if isinstance(record.data, Mapping) else ""
        return value or ("truth" if record.source == "truth" else "measurement")

    def ingest(self, records: Iterable[Record | Mapping[str, Any]]) -> None:
        for record in records:
            self.ingest_record(record)

    def get(self, channel_id: str) -> Optional[ChannelSpec]:
        return self._specs.get(channel_id)

    def channels(self, query: str = "", *, car: Optional[str] = None, classification: Optional[str] = None) -> List[ChannelSpec]:
        term = query.casefold().strip()
        return sorted((spec for spec in self._specs.values()
                       if (not term or term in spec.channel_id.casefold() or term in spec.description.casefold())
                       and (car is None or not car or spec.car == car)
                       and (classification is None or spec.classification == classification)),
                      key=lambda spec: spec.channel_id)

    def samples(self, channel_id: str, *, run_id: Optional[str] = None, epoch_id: Optional[str] = None,
                until: Optional[float] = None) -> List[ChannelSample]:
        return [sample for sample in self._samples.get(channel_id, ())
                if (run_id is None or sample.run_id == run_id)
                and (epoch_id is None or sample.epoch_id == epoch_id)
                and (until is None or sample.sim_time <= until)]

    def latest(self, channel_id: str, **filters: Any) -> Optional[ChannelSample]:
        values = self.samples(channel_id, **filters)
        return values[-1] if values else None

    def stats(self, channel_id: str, **filters: Any) -> Dict[str, Any]:
        samples = self.samples(channel_id, **filters)
        numbers = [float(s.value) for s in samples if s.valid and is_finite_number(s.value)]
        if not numbers:
            return {"count": 0, "coverage_s": 0.0, "latest": None, "available": bool(samples)}
        times = [s.sim_time for s in samples if s.valid]
        percentiles = {}
        ordered = sorted(numbers)
        for percentile in (0.05, 0.50, 0.95):
            index = min(len(ordered) - 1, int(round(percentile * (len(ordered) - 1))))
            percentiles[str(int(percentile * 100))] = ordered[index]
        return {"count": len(numbers), "coverage_s": max(times) - min(times) if len(times) > 1 else 0.0,
                "min": min(numbers), "max": max(numbers), "mean": statistics.fmean(numbers),
                "percentiles": percentiles, "latest": numbers[-1], "available": True}

    def snapshot(self) -> Tuple[ChannelSpec, ...]:
        return tuple(self.channels())


class RecordStore:
    """Selection-aware record store; historical queries never expose future records."""

    def __init__(self, records: Iterable[Record | Mapping[str, Any]] = ()) -> None:
        self.records: List[Record] = sorted((r if isinstance(r, Record) else Record.from_mapping(r) for r in records),
                                            key=lambda item: (item.run_id, item.epoch_id, item.sim_time, item.sequence, item.wall_time))
        self.run_id = self.records[-1].run_id if self.records else ""
        self.epoch_id = self.records[-1].epoch_id if self.records else 0
        matching = [r.sim_time for r in self.records if r.run_id == self.run_id and r.epoch_id == self.epoch_id]
        self.cursor = max(matching) if matching else 0.0

    def append(self, record: Record | Mapping[str, Any]) -> Record:
        item = record if isinstance(record, Record) else Record.from_mapping(record, source="live")
        key = self._sort_key(item)
        if not self.records or key >= self._sort_key(self.records[-1]):
            self.records.append(item)
        else:
            keys = [self._sort_key(value) for value in self.records]
            self.records.insert(bisect.bisect_right(keys, key), item)
        if not self.run_id:
            self.run_id, self.epoch_id, self.cursor = item.run_id, item.epoch_id, item.sim_time
        return item

    @staticmethod
    def _sort_key(value: Record) -> Tuple[Any, Any, float, int, float]:
        return (value.run_id, value.epoch_id, value.sim_time, value.sequence, value.wall_time)

    def epochs(self) -> List[Tuple[str, str]]:
        return sorted({(item.run_id, item.epoch_id) for item in self.records})

    def select(self, run_id: str, epoch_id: Any, sim_time: Optional[float] = None) -> None:
        self.run_id, self.epoch_id = str(run_id), epoch_id
        times = [r.sim_time for r in self.records if r.run_id == self.run_id and r.epoch_id == self.epoch_id]
        self.cursor = max(times) if sim_time is None and times else (float(sim_time) if sim_time is not None else 0.0)

    def set_cursor(self, sim_time: float) -> float:
        self.cursor = float(sim_time)
        return self.cursor

    def visible(self, *, record_type: Optional[str] = None) -> List[Record]:
        return [r for r in self.records if r.run_id == self.run_id and r.epoch_id == self.epoch_id
                and r.sim_time <= self.cursor and (record_type is None or r.record_type == record_type)]

    def current(self) -> Optional[Record]:
        values = self.visible()
        return values[-1] if values else None


class DataSource:
    """Small adapter surface shared by ROS/live and JSONL/replay producers."""

    registry: ChannelRegistry
    store: RecordStore

    def visible_records(self, record_type: Optional[str] = None) -> List[Record]:
        raise NotImplementedError

    def scrub(self, sim_time: float) -> None:
        raise NotImplementedError


class ReplayDataSource(DataSource):
    """Offline JSONL source and replay clock, independent of ROS/Gazebo."""

    def __init__(self, records: Iterable[Record | Mapping[str, Any]] = (), *, manifest: Optional[Mapping[str, Any]] = None) -> None:
        self.store = RecordStore(records)
        self.manifest = copy.deepcopy(dict(manifest or {}))
        # Offline replay retains the complete indexed stream; only live
        # adapters use bounded recent-history buffers.
        self.registry = ChannelRegistry(max_samples_per_channel=None)
        self.registry.ingest(self.store.records)
        self.playing = False
        self.speed = 1.0

    @classmethod
    def from_jsonl(cls, path: str | Path) -> "ReplayDataSource":
        records = []
        with Path(path).open("r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    records.append(Record.from_mapping(json.loads(line), source="recorded"))
        return cls(records)

    @classmethod
    def from_run_dir(cls, path: str | Path) -> "ReplayDataSource":
        """Load a self-contained evidence directory without ROS or a simulator."""
        root = Path(path)
        candidates = [root / "decisions.jsonl", root / "events.jsonl", root / "observations" / "records.jsonl",
                      root / "observations.jsonl", root / "telemetry.jsonl"]
        records = []
        for candidate in candidates:
            if not candidate.is_file():
                continue
            with candidate.open("r", encoding="utf-8") as stream:
                for line in stream:
                    if line.strip():
                        records.append(Record.from_mapping(json.loads(line), source="recorded"))
        if not records:
            raise FileNotFoundError(f"no JSONL records found under {root}")
        manifest = {}
        manifest_path = root / "manifest.json"
        if manifest_path.is_file():
            with manifest_path.open("r", encoding="utf-8") as stream: manifest = json.load(stream)
        source = cls(records, manifest=manifest)
        try:
            from eufs_race_control.logging import RunReader as CanonicalRunReader
            source.run_reader = CanonicalRunReader(root)
        except (ImportError, TypeError, OSError):
            source.run_reader = None
        return source

    def load_prediction(self, identifier: str) -> Any:
        reader = getattr(self, "run_reader", None)
        if reader is None: return None
        try: return reader.prediction(identifier)
        except (AttributeError, KeyError, OSError): return None

    @property
    def run_id(self) -> str:
        return self.store.run_id

    @property
    def epoch_id(self) -> str:
        return self.store.epoch_id

    @property
    def sim_time(self) -> float:
        return self.store.cursor

    def select(self, run_id: str, epoch_id: Any, sim_time: Optional[float] = None) -> None:
        self.store.select(run_id, epoch_id, sim_time)

    def scrub(self, sim_time: float) -> None:
        self.store.set_cursor(sim_time)

    def step(self, direction: int = 1, *, by: str = "sample") -> float:
        values = [r for r in self.store.records if r.run_id == self.run_id and r.epoch_id == self.epoch_id]
        if by == "decision":
            values = [r for r in values if r.record_type in {"strategy", "strategy_decision", "decision", "tactical", "tactical_evaluation", "mpc", "mpc_solve"}]
        future = [r for r in values if r.sim_time > self.sim_time]
        previous = [r for r in values if r.sim_time < self.sim_time]
        target = future[0] if direction >= 0 and future else previous[-1] if direction < 0 and previous else None
        if target:
            self.scrub(target.sim_time)
        return self.sim_time

    def visible_records(self, record_type: Optional[str] = None) -> List[Record]:
        return self.store.visible(record_type=record_type)

    def tick(self, delta_s: float) -> float:
        if self.playing:
            times = [r.sim_time for r in self.store.records if r.run_id == self.run_id and r.epoch_id == self.epoch_id]
            target = self.sim_time + max(0.0, float(delta_s)) * self.speed
            if times: target = min(max(target, min(times)), max(times))
            self.scrub(target)
        return self.sim_time


class LiveDataSource(ReplayDataSource):
    """Bounded live source; recorded files remain available through ReplayDataSource."""

    def __init__(self, max_records: int = 30000) -> None:
        super().__init__()
        self._max_records = max_records
        self.registry = ChannelRegistry(max_samples_per_channel=max_records)
        self.follow_live = True

    def push(self, value: Record | Mapping[str, Any]) -> Record:
        record = self.store.append(value)
        self.registry.ingest_record(record)
        if len(self.store.records) > self._max_records:
            del self.store.records[:-self._max_records]
        if self.follow_live:
            # Reset epochs are a hard selection boundary. A frozen/pinned view
            # remains on its old epoch until explicitly returned to live.
            if self.store.run_id != record.run_id or self.store.epoch_id != record.epoch_id:
                self.store.run_id, self.store.epoch_id = record.run_id, record.epoch_id
            self.store.cursor = record.sim_time
        return record

    def set_follow_live(self, enabled: bool) -> None:
        self.follow_live = bool(enabled)
        if self.follow_live and self.store.records:
            latest = self.store.records[-1]
            self.store.run_id, self.store.epoch_id, self.store.cursor = latest.run_id, latest.epoch_id, latest.sim_time


class DecisionPin:
    """Immutable evidence snapshot for a selected decision."""

    def __init__(self) -> None:
        self._value: Optional[Dict[str, Any]] = None
        self._related: Tuple[Dict[str, Any], ...] = ()

    @property
    def pinned(self) -> bool:
        return self._value is not None

    @property
    def value(self) -> Optional[Dict[str, Any]]:
        return copy.deepcopy(self._value) if self._value is not None else None

    @property
    def related_records(self) -> Tuple[Dict[str, Any], ...]:
        return copy.deepcopy(self._related)

    def pin(self, record: Record | Mapping[str, Any], related_records: Iterable[Record | Mapping[str, Any]] = ()) -> Dict[str, Any]:
        source = record.as_dict() if isinstance(record, Record) else dict(record)
        self._value = copy.deepcopy(source)
        self._related = tuple(copy.deepcopy(item.as_dict() if isinstance(item, Record) else dict(item)) for item in related_records)
        return self.value or {}

    def clear(self) -> None:
        self._value = None
        self._related = ()


class EventTimeline:
    def __init__(self) -> None:
        self.events: List[Record] = []
        self._keys = set()

    def ingest(self, record: Record | Mapping[str, Any]) -> None:
        item = record if isinstance(record, Record) else Record.from_mapping(record)
        if item.record_type in {"event", "health", "outcome", "runtime"}:
            key = (item.run_id, item.epoch_id, item.sim_time, item.sequence, item.component, item.record_type)
            if key in self._keys:
                return
            self._keys.add(key)
            self.events.append(item)
            self.events.sort(key=lambda value: value.sim_time)

    def query(self, text: str = "", *, run_id: Optional[str] = None, epoch_id: Optional[str] = None,
              until: Optional[float] = None) -> List[Record]:
        term = text.casefold().strip()
        result = []
        for event in self.events:
            if run_id is not None and event.run_id != run_id:
                continue
            if epoch_id is not None and event.epoch_id != epoch_id:
                continue
            if until is not None and event.sim_time > until:
                continue
            payload = json.dumps(event.data, sort_keys=True).casefold()
            if not term or term in event.component.casefold() or term in event.record_type.casefold() or term in payload:
                result.append(event)
        return result
