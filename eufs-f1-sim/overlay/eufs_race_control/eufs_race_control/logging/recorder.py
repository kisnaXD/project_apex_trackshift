"""Bounded asynchronous append-only run recorder."""

from __future__ import annotations

import copy
import json
import os
import queue
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Mapping

from .schema import SCHEMA_VERSION, ID_FIELDS, json_safe, normalize_record, validate_identifier


class RecorderClosed(RuntimeError):
    """Raised when a record is submitted after :meth:`RunRecorder.close`."""


class RecorderFailed(RuntimeError):
    """Raised when the asynchronous writer has failed."""


class RunRecorder:
    """Write a self-contained run bundle from a non-blocking producer API.

    ``emit`` only normalizes, copies and enqueues a record. Disk I/O happens in
    the worker thread, so a full queue causes an explicit drop instead of
    blocking a control callback. The manifest is marked incomplete whenever a
    drop or worker failure occurs.
    """

    def __init__(
        self,
        root: str | os.PathLike[str],
        run_id: str | None = None,
        *,
        epoch_id: int = 0,
        scenario_id: str = "default",
        manifest: Mapping[str, Any] | None = None,
        queue_size: int = 2048,
    ) -> None:
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        if not isinstance(epoch_id, int) or isinstance(epoch_id, bool) or epoch_id < 0:
            raise ValueError("epoch_id must be a non-negative integer")
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.run_id = validate_identifier(run_id or uuid.uuid4().hex, "run_id")
        self.default_epoch_id = epoch_id
        self.default_scenario_id = scenario_id
        self.run_dir = self.root / f"run_{self.run_id}"
        # mkdir(exist_ok=False) is the exclusive run allocation operation.
        self.run_dir.mkdir(mode=0o755, exist_ok=False)
        (self.run_dir / "predictions").mkdir()
        (self.run_dir / "observations").mkdir()
        (self.run_dir / "truth").mkdir()
        self._queue: queue.Queue[Any] = queue.Queue(maxsize=queue_size)
        self._stop = object()
        # Serializes producer admission and close. This lock never surrounds
        # disk I/O and therefore does not block on the writer thread.
        self._producer_lock = threading.Lock()
        self._lock = threading.Lock()
        self._closed = False
        self._close_complete = threading.Event()
        self._sequence = 0
        self._written = {"decisions": 0, "events": 0, "observations": 0, "truth": 0, "predictions": 0}
        self._dropped = {name: 0 for name in self._written}
        self._worker_error: str | None = None
        self._manifest: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "epoch_id": epoch_id,
            "scenario_id": scenario_id,
            "status": "open",
            "complete": False,
            "created_wall_time": time.time(),
            "record_counts": dict(self._written),
            "drop_counts": dict(self._dropped),
            "files": {
                "decisions": "decisions.jsonl",
                "events": "events.jsonl",
                "observations": "observations/records.jsonl",
                "truth": "truth/records.jsonl",
                "predictions": "predictions/",
            },
        }
        if manifest:
            self._manifest["metadata"] = json_safe(copy.deepcopy(dict(manifest)))
        self._write_manifest()
        self._thread = threading.Thread(target=self._writer_loop, name=f"race-recorder-{self.run_id}", daemon=True)
        self._thread.start()

    @property
    def dropped_count(self) -> int:
        return sum(self._dropped.values())

    @property
    def drop_counts(self) -> dict[str, int]:
        with self._lock:
            return dict(self._dropped)

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed

    def health(self) -> dict[str, Any]:
        """Return non-blocking recorder health for runtime-health records."""
        with self._lock:
            return {
                "closed": self._closed,
                "worker_error": self._worker_error,
                "backlog": self._queue.qsize(),
                "record_counts": dict(self._written),
                "drop_counts": dict(self._dropped),
                "complete": not bool(self._worker_error) and not any(self._dropped.values()) and not self._closed,
            }

    def _write_manifest(self) -> None:
        payload = json.dumps(self._manifest, sort_keys=True, separators=(",", ":"), allow_nan=False)
        fd, temporary = tempfile.mkstemp(prefix="manifest.", suffix=".tmp", dir=str(self.run_dir))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                stream.write(payload)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.run_dir / "manifest.json")
        finally:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass

    @staticmethod
    def _stream_for(record_type: str) -> str:
        lowered = record_type.lower()
        if lowered.startswith("observation") or lowered.startswith("estimate"):
            return "observations"
        if lowered.startswith("truth") or lowered.startswith("score"):
            return "truth"
        if "decision" in lowered or lowered.startswith(("strategy", "tactical", "mpc")):
            return "decisions"
        return "events"

    def emit(self, record: Mapping[str, Any], *, stream: str | None = None) -> bool:
        """Copy and enqueue a record without waiting for disk I/O.

        Returns ``False`` when the bounded queue is full. The corresponding
        stream drop count is persisted in the manifest at close.
        """

        with self._producer_lock:
            with self._lock:
                if self._closed:
                    raise RecorderClosed("run recorder is closed")
                if self._worker_error:
                    raise RecorderFailed(self._worker_error)
                sequence = self._sequence
                self._sequence += 1
            normalized = normalize_record(
                copy.deepcopy(record),
                defaults={
                    "run_id": self.run_id,
                    "epoch_id": self.default_epoch_id,
                    "scenario_id": self.default_scenario_id,
                },
            )
            normalized["run_id"] = self.run_id
            normalized["sequence"] = sequence
            chosen = stream or self._stream_for(str(normalized["record_type"]))
            if chosen not in self._written:
                raise ValueError(f"unknown stream {chosen!r}")
            # A final deep copy makes caller-side mutation after emit harmless.
            item = (chosen, copy.deepcopy(normalized))
            try:
                self._queue.put_nowait(item)
                return True
            except queue.Full:
                with self._lock:
                    self._dropped[chosen] += 1
                return False

    def emit_prediction(
        self,
        record: Mapping[str, Any],
        values: Any,
        *,
        key: str | None = None,
    ) -> bool:
        """Enqueue an indexed JSON prediction array.

        Prediction arrays are stored as individual JSON artifacts with a
        ``predictions/index.jsonl`` entry. This is intentionally truthful about
        the format; a later exporter may convert these arrays to parquet.
        """

        with self._producer_lock:
            with self._lock:
                if self._closed:
                    raise RecorderClosed("run recorder is closed")
                if self._worker_error:
                    raise RecorderFailed(self._worker_error)
                sequence = self._sequence
                self._sequence += 1
            normalized = normalize_record(
                copy.deepcopy(record),
                defaults={"run_id": self.run_id, "epoch_id": self.default_epoch_id, "scenario_id": self.default_scenario_id},
            )
            normalized["run_id"] = self.run_id
            if key is None:
                key = normalized.get("ids", {}).get("trajectory_id") or normalized.get("ids", {}).get("candidate_id")
            if key is None:
                raise ValueError("prediction requires trajectory_id, candidate_id or key")
            key = validate_identifier(str(key), "prediction key")
            normalized["sequence"] = sequence
            try:
                self._queue.put_nowait(("predictions", (normalized, key, json_safe(copy.deepcopy(values)))))
                return True
            except queue.Full:
                with self._lock:
                    self._dropped["predictions"] += 1
                return False

    def _writer_loop(self) -> None:
        handles: dict[str, Any] = {}
        try:
            paths = {
                "decisions": self.run_dir / "decisions.jsonl",
                "events": self.run_dir / "events.jsonl",
                "observations": self.run_dir / "observations" / "records.jsonl",
                "truth": self.run_dir / "truth" / "records.jsonl",
            }
            for stream, path in paths.items():
                handles[stream] = path.open("a", encoding="utf-8")
            prediction_index = (self.run_dir / "predictions" / "index.jsonl").open("a", encoding="utf-8")
            handles["prediction_index"] = prediction_index
            while True:
                item = self._queue.get()
                try:
                    if item is self._stop:
                        return
                    stream, payload = item
                    if stream == "predictions":
                        record, key, values = payload
                        filename = f"{record['sequence']:012d}_{key}.json"
                        path = self.run_dir / "predictions" / filename
                        body = {"record": record, "values": values}
                        with path.open("x", encoding="utf-8") as output:
                            json.dump(body, output, sort_keys=True, separators=(",", ":"), allow_nan=False)
                            output.write("\n")
                        prediction_index.write(json.dumps({
                            "sequence": record["sequence"],
                            "key": key,
                            "path": f"predictions/{filename}",
                            "epoch_id": record.get("epoch_id"),
                            "sim_time": record.get("sim_time"),
                            "wall_time": record.get("wall_time"),
                            "record_type": record.get("record_type"),
                            "ids": record.get("ids", {}),
                        }, sort_keys=True, allow_nan=False) + "\n")
                        prediction_index.flush()
                    else:
                        handles[stream].write(json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
                        handles[stream].flush()
                    with self._lock:
                        self._written[stream] += 1
                finally:
                    self._queue.task_done()
        except Exception as exc:  # keep close/reporting safe after disk failure
            with self._lock:
                self._worker_error = f"{type(exc).__name__}: {exc}"
                self._manifest["complete"] = False
            # Account for records that could not be processed so queue.join()
            # cannot deadlock after a worker-side filesystem failure.
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
                else:
                    self._queue.task_done()
        finally:
            for handle in handles.values():
                try:
                    handle.flush()
                    handle.close()
                except Exception:
                    pass
            self._close_complete.set()

    def close(self, timeout: float | None = 10.0) -> None:
        """Drain queued records and atomically close the manifest.

        A failed or stalled writer cannot make shutdown wait forever. If a
        timeout expires, the run remains explicitly incomplete and the daemon
        worker is left to finish or terminate independently.
        """

        with self._producer_lock:
            with self._lock:
                if self._closed:
                    return
                self._closed = True
        deadline = None if timeout is None else time.monotonic() + max(0.0, timeout)
        # ``Queue.join`` has no timeout; poll its unfinished-task counter so a
        # broken filesystem or dead worker cannot deadlock control shutdown.
        while self._queue.unfinished_tasks:
            if deadline is not None and time.monotonic() >= deadline:
                break
            time.sleep(0.005)
        if self._thread.is_alive():
            try:
                self._queue.put_nowait(self._stop)
            except queue.Full:
                pass
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            self._thread.join(remaining)
        if not self._thread.is_alive():
            # A writer can fail between the liveness check and sentinel
            # admission. Remove any unconsumed sentinel/records so their queue
            # bookkeeping cannot make later status calculations misleading.
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
                else:
                    self._queue.task_done()
        with self._lock:
            drained = self._queue.unfinished_tasks == 0
            stopped = not self._thread.is_alive()
            complete = stopped and drained and not bool(self._worker_error) and not any(self._dropped.values())
            self._manifest["status"] = "complete" if complete else "incomplete"
            self._manifest["complete"] = complete
            self._manifest["closed_wall_time"] = time.time()
            self._manifest["record_counts"] = dict(self._written)
            self._manifest["drop_counts"] = dict(self._dropped)
            self._manifest["sequence_count"] = self._sequence
            if self._worker_error:
                self._manifest["worker_error"] = self._worker_error
            self._write_manifest()

    def __enter__(self) -> "RunRecorder":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


# Friendly aliases used by dashboard integrations.
CausalRecorder = RunRecorder
Recorder = RunRecorder

__all__ = ["RunRecorder", "CausalRecorder", "Recorder", "RecorderClosed", "RecorderFailed"]
