"""Focused acceptance tests for causal recorder evidence bundles.

These tests are intentionally kept separate from ROS/Gazebo tests; they run in
the stdlib-only offline tooling environment.
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "overlay" / "eufs_race_control"))

from eufs_race_control.logging import RecorderClosed, RecorderFailed, RunReader, RunRecorder, make_record  # noqa: E402


def _record(record_type="strategy_decision", **data):
    return make_record(component="test", record_type=record_type, sim_time=data.pop("sim_time", 1.0), data=data)


def test_emit_deep_copies_and_writes_common_schema(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="copy-test", queue_size=4)
    payload = {"nested": {"value": 1}}
    assert recorder.emit({**_record(), "data": payload}, stream="decisions")
    payload["nested"]["value"] = 99
    recorder.close()
    record = RunReader(tmp_path / "run_copy-test").records("decisions")[0]
    assert record["data"]["nested"]["value"] == 1
    assert set(("schema_version", "run_id", "epoch_id", "scenario_id", "sim_time", "wall_time", "component", "record_type", "ids", "data")) <= set(record)


def test_bounded_queue_drop_is_detectable(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="drop-test", queue_size=1)
    # A writer can consume one record, but repeatedly filling the queue must
    # still make either a drop or a complete count visible.
    outcomes = [recorder.emit(_record(sim_time=float(i)), stream="events") for i in range(100)]
    recorder.close()
    manifest = json.loads((tmp_path / "run_drop-test" / "manifest.json").read_text())
    assert manifest["drop_counts"]["events"] == outcomes.count(False)
    assert manifest["complete"] is (manifest["drop_counts"]["events"] == 0)


def test_nonfinite_values_become_json_null(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="finite-test")
    recorder.emit({**_record(), "data": {"nan": math.nan, "inf": math.inf, "neg_inf": -math.inf}}, stream="events")
    recorder.close()
    record = RunReader(tmp_path / "run_finite-test").records("events")[0]
    assert record["data"] == {"nan": None, "inf": None, "neg_inf": None}


def test_unique_run_and_safe_prediction_key(tmp_path):
    first = RunRecorder(tmp_path, run_id="same")
    first.close()
    with pytest.raises(FileExistsError):
        RunRecorder(tmp_path, run_id="same")
    recorder = RunRecorder(tmp_path, run_id="pred")
    with pytest.raises(ValueError):
        recorder.emit_prediction(_record(), [1], key="../escape")
    recorder.close()


def test_epoch_reset_replay_and_report_escaping(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="epochs")
    recorder.emit({**_record(sim_time=10), "epoch_id": 0, "data": {"text": "<unsafe>"}}, stream="events")
    recorder.emit({**_record(sim_time=1), "epoch_id": 1}, stream="events")
    recorder.close()
    reader = RunReader(tmp_path / "run_epochs")
    assert [row["sim_time"] for row in reader.replay(epoch_id=1)] == [1]
    assert [row["sim_time"] for row in reader.replay(epoch_id=0, until_sim_time=5)] == []
    assert "&lt;unsafe&gt;" in reader.export_report()


def test_records_after_close_are_rejected(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="closed")
    recorder.close()
    with pytest.raises(RecorderClosed):
        recorder.emit(_record())


def test_prediction_is_causally_queryable(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="prediction")
    record = {**_record(), "ids": {"trajectory_id": "traj-1"}}
    assert recorder.emit_prediction(record, [{"x_m": 1.0}], key="traj-1")
    recorder.close()
    reader = RunReader(tmp_path / "run_prediction")
    assert reader.causal_find(trajectory_id="traj-1")[0]["prediction_values"][0]["x_m"] == 1.0


def test_close_timeout_does_not_wait_for_stuck_writer(tmp_path):
    class StuckRecorder(RunRecorder):
        def _writer_loop(self):
            time.sleep(1.0)

    recorder = StuckRecorder(tmp_path, run_id="stuck", queue_size=1)
    recorder.emit(_record(), stream="events")
    started = time.monotonic()
    recorder.close(timeout=0.01)
    assert time.monotonic() - started < 0.5
    manifest = json.loads((tmp_path / "run_stuck" / "manifest.json").read_text())
    assert manifest["complete"] is False


def test_writer_failure_rejects_new_records_and_close_returns(tmp_path):
    class FailedRecorder(RunRecorder):
        def _writer_loop(self):
            with self._lock:
                self._worker_error = "forced failure"
            self._close_complete.set()

    recorder = FailedRecorder(tmp_path, run_id="failed", queue_size=2)
    deadline = time.monotonic() + 1.0
    while recorder._worker_error is None and time.monotonic() < deadline:
        time.sleep(0.001)
    with pytest.raises(RecorderFailed):
        recorder.emit(_record())
    recorder.close(timeout=0.1)
    assert json.loads((tmp_path / "run_failed" / "manifest.json").read_text())["complete"] is False


def test_concurrent_emit_and_close_has_ordered_admitted_sequences(tmp_path):
    recorder = RunRecorder(tmp_path, run_id="concurrent", queue_size=64)
    barrier = threading.Barrier(5)
    errors = []

    def producer():
        barrier.wait()
        for index in range(200):
            try:
                recorder.emit(_record(sim_time=float(index)), stream="events")
            except RecorderClosed:
                return
            except Exception as exc:  # pragma: no cover - failure evidence
                errors.append(exc)
                return

    threads = [threading.Thread(target=producer) for _ in range(4)]
    for thread in threads:
        thread.start()
    barrier.wait()
    time.sleep(0.002)
    recorder.close(timeout=2.0)
    for thread in threads:
        thread.join()
    assert not errors
    records = RunReader(tmp_path / "run_concurrent").records("events")
    sequences = [record["sequence"] for record in records]
    assert sequences == sorted(set(sequences))
