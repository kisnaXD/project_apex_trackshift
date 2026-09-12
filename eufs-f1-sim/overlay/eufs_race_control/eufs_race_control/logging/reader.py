"""Indexed, causal and epoch-aware reader for recorder bundles."""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any, Iterator


class RunReader:
    """Read a bundle using a one-time bounded metadata index."""

    _STREAM_PATHS = {
        "decisions": "decisions.jsonl",
        "events": "events.jsonl",
        "observations": "observations/records.jsonl",
        "truth": "truth/records.jsonl",
    }

    def __init__(self, run_dir: str | Path) -> None:
        self.run_dir = Path(run_dir)
        if not self.run_dir.is_dir():
            raise FileNotFoundError(self.run_dir)
        manifest_path = self.run_dir / "manifest.json"
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
        self.partial = not bool(manifest_path.exists()) or not bool(self.manifest.get("complete", False))
        self.diagnostics: list[dict[str, Any]] = []
        self._index: dict[str, list[dict[str, Any]]] = {}
        self._prediction_index: list[dict[str, Any]] = []
        self._causal_index: dict[tuple[str, Any], list[dict[str, Any]]] = {}
        self._build_index()

    def _index_causal(self, entry: dict[str, Any]) -> None:
        for key, value in (entry.get("ids") or {}).items():
            if value is None:
                continue
            try:
                hash(value)
            except TypeError:
                continue
            self._causal_index.setdefault((key, value), []).append(entry)

    def _path_for(self, stream: str) -> Path:
        if stream not in self._STREAM_PATHS:
            raise ValueError(f"unknown stream {stream!r}")
        return self.run_dir / self._STREAM_PATHS[stream]

    def _scan_jsonl(self, stream: str, path: Path) -> None:
        entries: list[dict[str, Any]] = []
        if not path.exists():
            self._index[stream] = entries
            return
        with path.open("r", encoding="utf-8") as source:
            while True:
                offset = source.tell()
                line = source.readline()
                if not line:
                    break
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as exc:
                    final_partial = not line.endswith("\n")
                    self.partial = True
                    self.diagnostics.append({"path": str(path), "offset": offset, "kind": "truncated_final" if final_partial else "corrupt_line", "error": str(exc)})
                    continue
                if not isinstance(value, dict):
                    self.partial = True
                    self.diagnostics.append({"path": str(path), "offset": offset, "kind": "non_object"})
                    continue
                entry = {"stream": stream, "offset": offset, "sequence": value.get("sequence", 2**63 - 1), "sim_time": value.get("sim_time"), "epoch_id": value.get("epoch_id"), "wall_time": value.get("wall_time", 0.0), "record_type": value.get("record_type"), "ids": value.get("ids", {})}
                entries.append(entry)
                self._index_causal(entry)
        self._index[stream] = entries

    def _build_index(self) -> None:
        for stream in self._STREAM_PATHS:
            self._scan_jsonl(stream, self._path_for(stream))
        index_path = self.run_dir / "predictions" / "index.jsonl"
        if index_path.exists():
            with index_path.open("r", encoding="utf-8") as source:
                for line in source:
                    if not line.strip():
                        continue
                    try:
                        row = json.loads(line)
                        artifact_path = self.run_dir / str(row["path"])
                        artifact_path.resolve().relative_to(self.run_dir.resolve())
                        entry = {"stream": "predictions", "path": str(row["path"]), "sequence": row.get("sequence", 2**63 - 1), "key": row.get("key"), "epoch_id": row.get("epoch_id"), "sim_time": row.get("sim_time"), "wall_time": row.get("wall_time", 0.0), "record_type": row.get("record_type"), "ids": row.get("ids", {})}
                        self._prediction_index.append(entry)
                        self._index_causal(entry)
                    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                        self.partial = True
                        self.diagnostics.append({"path": str(index_path), "kind": "prediction_corrupt", "error": str(exc)})

    def _read_entries(self, entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Load only selected payloads, reusing one handle per JSONL stream."""
        handles: dict[str, Any] = {}
        records: list[dict[str, Any]] = []
        try:
            for entry in entries:
                stream = entry["stream"]
                if stream == "predictions":
                    try:
                        path = self.run_dir / str(entry["path"])
                        path.resolve().relative_to(self.run_dir.resolve())
                        payload = json.loads(path.read_text(encoding="utf-8"))
                        record = payload["record"]
                        record["prediction_values"] = payload.get("values")
                        records.append(record)
                    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                        self.partial = True
                        self.diagnostics.append({"path": str(entry.get("path")), "kind": "prediction_corrupt", "error": str(exc)})
                    continue
                try:
                    if stream not in handles:
                        handles[stream] = self._path_for(stream).open("r", encoding="utf-8")
                    handle = handles[stream]
                    handle.seek(entry["offset"])
                    value = json.loads(handle.readline())
                    if isinstance(value, dict):
                        records.append(value)
                except (OSError, json.JSONDecodeError):
                    self.partial = True
        finally:
            for handle in handles.values():
                handle.close()
        records.sort(key=lambda item: (item.get("sequence", 2**63 - 1), item.get("wall_time") or 0.0))
        return records

    def _metadata_entries(self, stream: str | None, *, include_truth: bool = False) -> list[dict[str, Any]]:
        if stream is not None and stream not in self._STREAM_PATHS and stream != "predictions":
            raise ValueError(f"unknown stream {stream!r}")
        streams = [stream] if stream else ["decisions", "events", "observations", "predictions"]
        if include_truth and stream is None:
            streams.append("truth")
        entries: list[dict[str, Any]] = []
        for name in streams:
            entries.extend(self._prediction_index if name == "predictions" else self._index.get(name, ()))
        entries.sort(key=lambda item: (item.get("sequence", 2**63 - 1), item.get("wall_time") or 0.0))
        return entries

    def iter_records(self, stream: str | None = None, *, include_truth: bool = False) -> Iterator[dict[str, Any]]:
        if stream is not None and stream not in self._STREAM_PATHS and stream != "predictions":
            raise ValueError(f"unknown stream {stream!r}")
        yield from self._read_entries(self._metadata_entries(stream, include_truth=include_truth))

    def records(self, stream: str | None = None, *, epoch_id: int | None = None, start_time: float | None = None, end_time: float | None = None, page: int = 0, page_size: int | None = None, include_truth: bool = False) -> list[dict[str, Any]]:
        if page < 0 or (page_size is not None and page_size < 1):
            raise ValueError("invalid page arguments")
        entries = []
        for entry in self._metadata_entries(stream, include_truth=include_truth):
            if epoch_id is not None and entry.get("epoch_id") != epoch_id:
                continue
            sim_time = entry.get("sim_time")
            if start_time is not None and (sim_time is None or sim_time < start_time):
                continue
            if end_time is not None and (sim_time is None or sim_time > end_time):
                continue
            entries.append(entry)
        if page_size is None:
            return self._read_entries(entries)
        start = page * page_size
        return self._read_entries(entries[start : start + page_size])

    paginate = records

    def seek(self, sim_time: float, *, epoch_id: int | None = None, stream: str | None = None, include_truth: bool = False) -> list[dict[str, Any]]:
        return self.records(stream, epoch_id=epoch_id, start_time=sim_time, include_truth=include_truth)

    def replay(self, *, epoch_id: int | None = None, until_sim_time: float | None = None, include_truth: bool = False) -> list[dict[str, Any]]:
        epochs = {entry.get("epoch_id") for entry in self._metadata_entries(None, include_truth=include_truth) if entry.get("epoch_id") is not None}
        if epoch_id is None:
            if len(epochs) > 1:
                raise ValueError("epoch_id is required to replay a bundle containing multiple epochs")
        elif epoch_id not in epochs:
            raise KeyError(f"unknown epoch_id {epoch_id}")
        return self.records(epoch_id=epoch_id, end_time=until_sim_time, include_truth=include_truth)

    replay_until = replay

    def causal_find(self, *, include_truth: bool = False, **identifiers: Any) -> list[dict[str, Any]]:
        if not identifiers:
            return []
        candidate_lists = [self._causal_index.get((key, value), []) for key, value in identifiers.items() if (key, value) in self._causal_index]
        candidates = min(candidate_lists, key=len) if candidate_lists else self._metadata_entries(None, include_truth=include_truth)
        entries = [entry for entry in candidates if all((entry.get("ids") or {}).get(key) == value or entry.get(key) == value for key, value in identifiers.items()) and (include_truth or entry.get("stream") != "truth")]
        # Legacy prediction indexes may not carry IDs; inspect only those
        # selected candidates when necessary, keeping normal causal lookup lazy.
        return self._read_entries(entries)

    find_causal = causal_find

    def events(self, **filters: Any) -> list[dict[str, Any]]:
        return [record for record in self.records("events") if all(record.get(key) == value for key, value in filters.items())]

    def prediction(self, key: str) -> dict[str, Any] | None:
        for entry in self._prediction_index:
            ids = entry.get("ids") or {}
            if key in {entry.get("key"), ids.get("trajectory_id"), ids.get("candidate_id")}:
                records = self._read_entries([entry])
                return records[0] if records else None
        return None

    def export_report(self, output: str | Path | None = None, *, include_truth: bool = True) -> str:
        rows = []
        for record in self.iter_records(include_truth=include_truth):
            rows.append("<tr><td>{}</td><td>{}</td><td>{}</td><td><code>{}</code></td></tr>".format(html.escape(str(record.get("sequence", ""))), html.escape(str(record.get("sim_time", ""))), html.escape(str(record.get("record_type", ""))), html.escape(json.dumps(record.get("data", {}), sort_keys=True))))
        status = "partial" if self.partial else "complete"
        report = "<!doctype html><meta charset='utf-8'><title>Run report</title><h1>Run {}</h1><p>Evidence status: {}</p><table><thead><tr><th>Sequence</th><th>Sim time</th><th>Type</th><th>Data</th></tr></thead><tbody>{}</tbody></table>".format(html.escape(str(self.manifest.get("run_id", self.run_dir.name))), html.escape(status), "".join(rows))
        if output is not None:
            Path(output).write_text(report, encoding="utf-8")
        return report


ReplayReader = RunReader

__all__ = ["RunReader", "ReplayReader"]
