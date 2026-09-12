"""Contract tests for the GUI foundation (the Qt smoke test is intentionally opt-in/headless)."""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "overlay" / "eufs_race_gui"))

import pytest

from eufs_race_gui.core import ChannelRegistry, DecisionPin, LiveDataSource, Record, RecordStore, ReplayDataSource


def record(t, *, epoch=1, component="estimator", record_type="state", data=None, source="recorded", sequence=0):
    return Record.from_mapping({
        "schema_version": "1.0", "run_id": "run-a", "epoch_id": epoch,
        "scenario_id": "D1", "sim_time": t, "wall_time": 100 + t,
        "component": component, "record_type": record_type, "sequence": sequence,
        "source": source, "ids": {}, "data": data or {},
    }, source=source)


def test_registry_missing_zero_nonfinite_and_nested_fields_are_explicit():
    registry = ChannelRegistry()
    registry.ingest_record(record(0.0, data={"car": "eufs", "speed_mps": 0.0, "nan_value": float("nan"), "battery": {"usable_wh": None}}))
    zero = next(spec for spec in registry.channels() if spec.field_path == "speed_mps")
    assert zero.numeric and registry.latest(zero.channel_id).valid and registry.latest(zero.channel_id).value == 0.0
    nan = next(spec for spec in registry.channels() if spec.field_path == "nan_value")
    assert not registry.latest(nan.channel_id).valid
    assert registry.latest(nan.channel_id).reason == "non-finite measurement"
    missing = next(spec for spec in registry.channels() if spec.field_path == "battery.usable_wh")
    assert registry.latest(missing.channel_id).valid is False
    assert registry.stats(missing.channel_id)["count"] == 0


def test_registry_upgrades_none_to_numeric_and_keeps_units_from_declaration():
    registry = ChannelRegistry()
    registry.ingest_record(record(0, data={"car": "eufs", "speed_mps": None, "_channel_meta": {"speed_mps": {"units": "m/s"}}}))
    registry.ingest_record(record(1, data={"car": "eufs", "speed_mps": 12.0, "_channel_meta": {"speed_mps": {"units": "m/s"}}}))
    spec = next(spec for spec in registry.channels() if spec.field_path == "speed_mps")
    assert spec.numeric and spec.units == "m/s"


def test_channels_are_segregated_by_car_component_classification_and_truth():
    registry = ChannelRegistry()
    registry.ingest_record(record(1, component="estimator", data={"car": "eufs", "speed_mps": 3.0}))
    registry.ingest_record(record(1, component="estimator", data={"car": "eufs2", "speed_mps": 4.0}))
    registry.ingest_record(record(1, component="evaluator", data={"car": "eufs", "speed_mps": 99.0}, source="truth"))
    specs = registry.channels()
    assert len([spec for spec in specs if spec.field_path == "speed_mps"]) == 3
    values = [registry.latest(spec.channel_id).value for spec in specs if spec.field_path == "speed_mps"]
    assert sorted(values) == [3.0, 4.0, 99.0]
    assert len({spec.classification for spec in specs if spec.field_path == "speed_mps"}) == 2


def test_invalid_epoch_and_time_are_rejected_without_coercion():
    payload = record(1).as_dict()
    payload["epoch_id"] = 1.5
    with pytest.raises(ValueError):
        Record.from_mapping(payload)
    payload = record(1).as_dict(); payload["sim_time"] = float("nan")
    with pytest.raises(ValueError):
        Record.from_mapping(payload)


def test_epoch_selection_and_historical_cursor_never_leaks_future_records():
    store = RecordStore([record(1, epoch=1), record(2, epoch=1), record(3, epoch=2)])
    store.select("run-a", 1, 1.5)
    assert [r.sim_time for r in store.visible()] == [1]
    store.set_cursor(2.5)
    assert [r.sim_time for r in store.visible()] == [1, 2]
    assert all(r.epoch_id == 1 for r in store.visible())


def test_pinned_decision_is_immutable_when_live_updates_arrive():
    pin = DecisionPin(); first = record(2, record_type="decision", data={"action": "defer", "cost": {"energy": 3}})
    pin.pin(first); first.data["action"] = "attack"
    snapshot = pin.value
    assert snapshot["data"]["action"] == "defer"
    snapshot["data"]["cost"]["energy"] = 99
    assert pin.value["data"]["cost"]["energy"] == 3


def test_live_source_is_bounded_and_truth_does_not_replace_estimate():
    source = LiveDataSource(max_records=2)
    source.push(record(1, data={"car": "eufs", "x": 1.0}, source="live"))
    source.push(record(2, data={"car": "eufs", "x": 2.0}, source="live"))
    source.push(record(3, data={"car": "eufs", "x": 3.0}, source="truth"))
    assert len(source.store.records) == 2
    assert len([spec for spec in source.registry.channels() if spec.field_path == "x"]) == 2
    # Truth has a distinct classification/source, even in the same bounded run.
    assert source.registry.latest(next(spec.channel_id for spec in source.registry.channels() if spec.classification == "truth")).value == 3.0


def test_replay_step_by_decision_and_end_of_stream_are_monotonic():
    source = ReplayDataSource([
        record(1, record_type="state", data={"speed_mps": 1.0}),
        record(2, record_type="strategy_decision", data={"action": "defer"}),
        record(3, record_type="state", data={"speed_mps": 3.0}),
    ])
    source.scrub(3.0)
    assert source.step(1, by="decision") == 3.0
    source.scrub(1.0)
    assert source.step(1, by="decision") == 2.0


def test_frozen_live_source_does_not_follow_new_epoch_until_return_to_live():
    source = LiveDataSource(max_records=10)
    source.push(record(1, epoch=1, data={"car": "eufs", "speed_mps": 1.0}, source="live"))
    source.set_follow_live(False)
    source.push(record(2, epoch=2, data={"car": "eufs", "speed_mps": 2.0}, source="live"))
    assert source.epoch_id == 1 and source.sim_time == 1.0
    source.set_follow_live(True)
    assert source.epoch_id == 2 and source.sim_time == 2.0


def test_offline_replay_registry_retains_complete_history():
    source = ReplayDataSource([record(float(index), data={"car": "eufs", "speed_mps": float(index)}) for index in range(10005)])
    spec = next(spec for spec in source.registry.channels() if spec.field_path == "speed_mps")
    assert len(source.registry.samples(spec.channel_id)) == 10005


def test_ros_bridge_decodes_canonical_records_and_keeps_truth_source_separate():
    pytest.importorskip("PyQt5")
    from eufs_race_gui.ros_bridge import CanonicalEnvelopeDecoder
    payload = record(4, data={"car": "eufs", "speed_mps": 10.0}).as_dict()
    decoder = CanonicalEnvelopeDecoder()
    estimate = decoder.decode(payload, source="live_ros")
    payload_with_source = dict(payload); payload_with_source["source"] = "recorded"
    explicit = decoder.decode(payload_with_source, source="live_ros")
    truth = decoder.decode(payload, source="truth")
    assert estimate.source == "live_ros"
    assert explicit.source == "live_ros"
    assert truth.source == "truth"


def test_operator_request_is_json_envelope_without_direct_drive_topic():
    pytest.importorskip("PyQt5")
    import json
    from eufs_race_gui.ros_bridge import CanonicalEnvelopeDecoder
    envelope = json.loads(CanonicalEnvelopeDecoder.encode_operator_request("pause", {"scenario": "D1"}, run_id="r", epoch_id=3))
    assert envelope["record_type"] == "operator_request"
    assert envelope["data"]["action"] == "pause"
    assert envelope["epoch_id"] == 3


def test_odom_adapter_keeps_spawn_relative_data_out_of_map_state():
    pytest.importorskip("PyQt5")
    from types import SimpleNamespace
    from eufs_race_gui.ros_bridge import OdomMapper
    pose = SimpleNamespace(position=SimpleNamespace(x=2.0, y=3.0, z=0.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    twist = SimpleNamespace(linear=SimpleNamespace(x=4.0, y=0.0, z=0.0))
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0), frame_id="odom"), child_frame_id="base_link", pose=SimpleNamespace(pose=pose), twist=SimpleNamespace(twist=twist))
    mapped = OdomMapper().to_record(msg, "eufs")
    assert mapped.record_type == "raw_odometry"
    assert "position" not in mapped.data and mapped.data["frame_id"] == "odom"


def test_pin_stops_replay_and_retains_related_causal_payloads(monkeypatch):
    pytest.importorskip("PyQt5")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication
    from eufs_race_gui.app import MainWindow
    decision = record(2, record_type="strategy_decision", data={"action": "defer"})
    decision = Record.from_mapping({**decision.as_dict(), "ids": {"decision_id": "d1", "trajectory_id": "tr1"}})
    related = Record.from_mapping({**record(2, record_type="mpc_solve", data={"predicted": {"x": [1, 2]}}).as_dict(), "ids": {"trajectory_id": "tr1"}})
    source = ReplayDataSource([record(1), decision, related]); source.playing = True; source.load_prediction = lambda key: {"array": [1, 2]} if key == "tr1" else None
    app = QApplication.instance() or QApplication([]); window = MainWindow(source); window.engineering._pin_record(decision)
    assert source.playing is False and window.pin.pinned and window.pin.related_records
    window.close(); app.processEvents()


def test_offscreen_qt_smoke_constructs_main_window_without_starting_runtime(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
    from PyQt5.QtCore import Qt
    from eufs_race_gui.app import MainWindow
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    source = ReplayDataSource([
        record(1, record_type="state", data={"car": "eufs", "x": 1.0, "y": 2.0, "speed_mps": 3.0}),
        record(2, record_type="strategy_decision", data={"action": "defer", "reason": "reserve"}),
        record(3, record_type="event", data={"message": "ready"}),
    ])
    window = MainWindow(source)
    assert window.windowTitle().startswith("EUFS Race Intelligence")
    assert window.presentation.map is not None
    source.scrub(1.0); window.refresh()
    assert window.presentation.action_value.text() == "Unavailable"
    strategy_panel = window.engineering._record_panels[0]
    assert all(strategy_panel.records.item(i).data(Qt.UserRole) is None for i in range(strategy_panel.records.count()))
    window.close(); app.processEvents()
