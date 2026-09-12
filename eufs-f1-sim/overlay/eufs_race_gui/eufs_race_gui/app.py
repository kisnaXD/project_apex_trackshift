"""Qt application shell for live/replay EUFS race investigation."""

from __future__ import annotations

import json
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QAction, QApplication, QComboBox, QDockWidget, QFileDialog, QFormLayout, QFrame,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow,
    QPushButton, QSpinBox, QStackedWidget, QTabWidget, QTextEdit, QToolBar, QVBoxLayout,
    QWidget,
)

from .core import ChannelRegistry, ChannelSample, DecisionPin, EventTimeline, LiveDataSource, Record, ReplayDataSource
from .widgets import (
    Card, ChannelExplorer, DecisionAlternativesTable, EventTimelineWidget, ReplayControls, StructuredInspector,
    TimeSeriesPlot, TrackMapWidget, export_samples_csv,
)


STYLE = """
QMainWindow, QWidget { background: #0b1018; color: #e8f0f7; font-family: 'DejaVu Sans'; font-size: 12px; }
QFrame#card, QGroupBox { background: #111b27; border: 1px solid #26384b; border-radius: 5px; }
QLabel#sectionTitle { color: #8ea4b7; font-size: 10px; font-weight: bold; letter-spacing: 1px; }
QLabel#hero { color: #ffffff; font-size: 24px; font-weight: bold; }
QLabel#value { color: #34d5e6; font-family: 'DejaVu Sans Mono'; font-size: 17px; font-weight: bold; }
QPushButton { background: #193149; border: 1px solid #345979; border-radius: 3px; padding: 6px 12px; color: #edf5fb; }
QPushButton:hover { background: #24516e; } QPushButton:disabled { color: #536574; background: #15202a; }
QPushButton#primary { background: #b92e37; border-color: #e2565b; }
QLineEdit, QComboBox, QSpinBox { background: #0d1722; border: 1px solid #31485c; padding: 5px; color: #e8f0f7; }
QTabWidget::pane { border: 1px solid #26384b; } QTabBar::tab { padding: 8px 13px; background: #111b27; }
QTabBar::tab:selected { background: #234762; color: #ffffff; }
QTableWidget, QTreeWidget, QListWidget, QTextEdit { background: #0d1722; border: 1px solid #26384b; alternate-background-color: #101f2d; }
QHeaderView::section { background: #172839; color: #aac0d2; padding: 4px; border: 0; }
"""


class ScenarioControl(QWidget):
    """Scenario editor; callbacks are intentionally the only route to driving."""

    commandRequested = pyqtSignal(str, object)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        head = QLabel("SCENARIO CONTROL"); head.setObjectName("hero"); layout.addWidget(head)
        form = QFormLayout(); self.scenario = QComboBox(); self.scenario.addItems(["D1 · pass Russell", "D2 · decline", "D3 · wait then pass", "D4 · cancel safely", "Full lap regression"])
        self.gap = QSpinBox(); self.gap.setRange(0, 1000); self.gap.setValue(25); self.gap.setSuffix(" m")
        self.energy = QSpinBox(); self.energy.setRange(0, 100000); self.energy.setValue(2200); self.energy.setSuffix(" Wh")
        self.pace = QSpinBox(); self.pace.setRange(1, 200); self.pace.setValue(100); self.pace.setSuffix(" %")
        self.noise = QSpinBox(); self.noise.setRange(0, 1000); self.noise.setValue(0); self.noise.setSuffix(" ms")
        for label, field in (("Situation", self.scenario), ("Initial gap", self.gap), ("Usable energy", self.energy), ("Reference pace", self.pace), ("Observation delay", self.noise)): form.addRow(label, field)
        layout.addLayout(form)
        self.info = QLabel("Source/profile metadata unavailable until a run manifest is loaded.\nExpected action is scenario metadata only; planner output remains observable.")
        self.info.setWordWrap(True); self.info.setStyleSheet("color:#8ea4b7"); layout.addWidget(self.info)
        self.status = QLabel("DISCONNECTED · live controls unavailable"); layout.addWidget(self.status)
        buttons = QHBoxLayout()
        for text, action, primary in (("Start", "start", True), ("Pause", "pause", False), ("Resume", "resume", False), ("Stop", "stop", False), ("Reset", "reset", False)):
            button = QPushButton(text); button.setObjectName("primary" if primary else "")
            button.setEnabled(False); button.clicked.connect(lambda checked=False, action=action: self.commandRequested.emit(action, self.parameters()))
            setattr(self, action + "_button", button); buttons.addWidget(button)
        layout.addLayout(buttons); layout.addStretch(1)

    def parameters(self) -> Dict[str, Any]:
        return {"scenario": self.scenario.currentText(), "initial_gap_m": self.gap.value(), "energy_wh": self.energy.value(), "pace_percent": self.pace.value(), "observation_delay_ms": self.noise.value()}

    def set_manifest(self, manifest: Mapping[str, Any]) -> None:
        source = manifest.get("source_lap", manifest.get("source", "unavailable"))
        profile = manifest.get("profile_versions", manifest.get("profile", "unavailable"))
        duration = manifest.get("expected_duration_s", manifest.get("duration_s", "unavailable"))
        self.info.setText(f"Source: {source} · profile: {profile} · expected duration: {duration}\nExpected action is scenario metadata only; planner output remains observable.")

    def set_connected(self, connected: bool) -> None:
        self.status.setText("CONNECTED · command requests route through supervisor" if connected else "DISCONNECTED · live controls unavailable")
        for name in ("start", "pause", "resume", "stop", "reset"):
            getattr(self, name + "_button").setEnabled(connected)


class PresentationView(QWidget):
    def __init__(self, source: ReplayDataSource, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent); self.source = source
        layout = QVBoxLayout(self)
        heading = QHBoxLayout(); title = QLabel("LIVE RACE  /  PRESENTATION"); title.setObjectName("hero"); heading.addWidget(title); heading.addStretch(); self.badge = QLabel("RECORDED REPLAY"); heading.addWidget(self.badge); layout.addLayout(heading)
        body = QHBoxLayout(); layout.addLayout(body, 1)
        self.map = TrackMapWidget(); body.addWidget(self.map, 2)
        side = QVBoxLayout(); body.addLayout(side, 1)
        self.action = Card("Selected action"); self.action_value = QLabel("Unavailable"); self.action_value.setObjectName("value"); self.action.body.addWidget(self.action_value); self.reason = QLabel("No decision record at selected time"); self.reason.setWordWrap(True); self.action.body.addWidget(self.reason); side.addWidget(self.action)
        metrics = Card("Race status"); grid = QFormLayout(); self.metric_labels: Dict[str, QLabel] = {}
        for key, label in (("gap", "Gap"), ("closing", "Closing speed"), ("speed", "Ego speed"), ("energy", "Usable energy"), ("lifecycle", "Overtake lifecycle")):
            value = QLabel("Unavailable"); value.setObjectName("value"); self.metric_labels[key] = value; grid.addRow(label, value)
        metrics.body.addLayout(grid); side.addWidget(metrics); side.addStretch()
        self.plot = TimeSeriesPlot(); layout.addWidget(self.plot, 1)

    def refresh(self) -> None:
        records = self.source.visible_records()
        decisions = {"strategy", "strategy_decision", "decision"}
        state_types = {"state", "estimated_state", "estimated_race_state", "telemetry"}
        decision = next((record for record in reversed(records) if record.record_type in decisions), None)
        if decision:
            data = decision.data; self.action_value.setText(str(data.get("action", data.get("selected_action", "Decision recorded"))))
            self.reason.setText(str(data.get("reason", data.get("explanation", "Reason in decision record"))))
        else:
            self.action_value.setText("Unavailable")
            self.reason.setText("No strategy decision is recorded at the selected time")
        current = next((record for record in reversed(records) if record.record_type in state_types), None)
        if current:
            data = current.data
            metrics = {
                "gap": self._lookup(data, "gap_m", "gap", "opponent.gap_m", "opponent.signed_gap_m"),
                "closing": self._lookup(data, "closing_speed_mps", "closing_speed", "opponent.closing_speed_mps"),
                "speed": self._lookup(data, "speed_mps", "ego.vx_mps", "ego.speed_mps"),
                "energy": self._lookup(data, "usable_energy_wh", "energy_wh", "energy.usable_energy_j", "energy.usable_energy_wh"),
                "lifecycle": self._lookup(data, "overtake_lifecycle", "lifecycle", "strategy.lifecycle"),
            }
            energy_units = "Wh" if self._lookup(data, "usable_energy_wh", "energy_wh") is not None else "J"
            for key, value in metrics.items(): self.metric_labels[key].setText(self._metric_text(key, value, energy_units))
            self._refresh_map(records)
        else:
            for label in self.metric_labels.values(): label.setText("Unavailable")
            self.map.set_data()
        self._refresh_plot()
        self.badge.setText("RECORDED REPLAY" if self.source.store.records and not isinstance(self.source, LiveDataSource) else "LIVE")

    @staticmethod
    def _lookup(data: Mapping[str, Any], *paths: str) -> Any:
        for path in paths:
            value: Any = data
            try:
                for part in path.split("."): value = value[part]
            except (KeyError, TypeError):
                continue
            if value is not None: return value
        return None

    @staticmethod
    def _metric_text(key: str, value: Any, energy_units: str = "J") -> str:
        if value is None: return "Unavailable"
        if isinstance(value, (int, float)):
            units = {"gap": "m", "closing": "m/s", "speed": "m/s", "energy": energy_units}.get(key, "")
            return f"{float(value):.2f} {units}".strip()
        return str(value)

    def _refresh_map(self, records: Sequence[Record]) -> None:
        cars: Dict[str, Dict[str, Any]] = {}; trails: Dict[str, list] = {}; paths: Dict[str, Sequence[Sequence[float]]] = {}; occupancy = []; track = ()
        for record in records:
            data = record.data
            candidate_track = data.get("track", data.get("track_points", data.get("centerline")))
            if isinstance(candidate_track, (list, tuple)) and candidate_track:
                track = [(item.get("x_m", item.get("x")), item.get("y_m", item.get("y"))) if isinstance(item, Mapping) else item for item in candidate_track]
            trajectory = data.get("trajectory", data.get("path"),) or data.get("points")
            if isinstance(trajectory, (list, tuple)):
                points = []
                for item in trajectory:
                    if isinstance(item, Mapping):
                        item = (item.get("x_m", item.get("x")), item.get("y_m", item.get("y")))
                    if isinstance(item, (list, tuple)) and len(item) >= 2 and item[0] is not None and item[1] is not None: points.append((float(item[0]), float(item[1])))
                if points: paths[str(data.get("candidate_id", record.ids.get("candidate_id", record.record_type)))] = points
            candidate_occupancy = data.get("opponent_occupancy", data.get("occupancy", ()))
            if isinstance(candidate_occupancy, (list, tuple)): occupancy.extend(item for item in candidate_occupancy if isinstance(item, Mapping))
            entities = data.get("cars") if isinstance(data.get("cars"), Mapping) else {}
            if not entities:
                entities = {key: data[key] for key in ("ego", "opponent", "eufs", "eufs2") if isinstance(data.get(key), Mapping)}
            if entities:
                entity_items = entities.items()
            else:
                entity_items = [(str(data.get("car", data.get("vehicle", data.get("car_id", "")))), data)]
            for car, entity in entity_items:
                position = entity.get("position", entity.get("pose")) if isinstance(entity, Mapping) else None
                if position is None and isinstance(entity, Mapping) and entity.get("x_m") is not None and entity.get("y_m") is not None: position = (entity.get("x_m"), entity.get("y_m"))
                if position is None and isinstance(entity, Mapping) and entity.get("x") is not None and entity.get("y") is not None: position = (entity.get("x"), entity.get("y"))
                if isinstance(position, Mapping): position = (position.get("x_m", position.get("x")), position.get("y_m", position.get("y")))
                if not car or not isinstance(position, (list, tuple)) or len(position) < 2 or position[0] is None or position[1] is None: continue
                point = (float(position[0]), float(position[1])); truth = record.source == "truth" or data.get("classification") == "truth" or entity.get("classification") == "truth"
                display_car = f"{car} · truth" if truth else str(car)
                trails.setdefault(display_car, []).append(point); cars[display_car] = {"position": point, "truth": truth}
        self.map.set_data(track=track, cars=cars, trails=trails, paths=paths, occupancy=occupancy)

    def _refresh_plot(self) -> None:
        selected = []
        unit_group: Optional[str] = None
        for spec in self.source.registry.channels():
            field = spec.field_path.casefold()
            if spec.numeric and any(term in field for term in ("speed", "gap", "energy", "closing")):
                if unit_group is None: unit_group = spec.units
                if spec.units != unit_group: continue
                samples = self.source.registry.samples(spec.channel_id, run_id=self.source.run_id, epoch_id=self.source.epoch_id, until=self.source.sim_time)
                if samples: selected.append((spec.channel_id, samples))
            if len(selected) >= 4: break
        self.plot.set_series(dict(selected), title="Selected race telemetry", y_label="SI units")


class EngineeringView(QWidget):
    """Full engineering surface shared by all streams and record types."""

    def __init__(self, source: ReplayDataSource, registry: ChannelRegistry, timeline: EventTimeline, pin: DecisionPin, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent); self.source, self.registry, self.timeline, self.pin = source, registry, timeline, pin
        self._record_panels = []
        self._timeline_seen = set()
        self._timeline_record_count = 0
        self._selected_channel: Optional[str] = None
        layout = QVBoxLayout(self); tabs = QTabWidget(); layout.addWidget(tabs)
        self.explorer = ChannelExplorer(registry); tabs.addTab(self.explorer, "Telemetry explorer")
        self.plot = TimeSeriesPlot(); self.plot_card = QWidget(); plot_layout = QVBoxLayout(self.plot_card); plot_layout.addWidget(self.plot); self.plot_info = QLabel("Select any numeric channel to plot every recorded sample."); plot_layout.addWidget(self.plot_info); tabs.addTab(self.plot_card, "Live graphs")
        self.timeline_widget = EventTimelineWidget(timeline); tabs.addTab(self.timeline_widget, "Events / health")
        self.inspector = StructuredInspector(); tabs.addTab(self.inspector, "Decision inspector")
        self.explorer.channelSelected.connect(self.show_channel); self.timeline_widget.eventSelected.connect(self.inspect)
        self.tabs = tabs
        for title, descriptor, record_types in (("Strategy decisions", "Every hold, decline, defer, replan and abort with alternatives, value and uncertainty.", ("strategy", "strategy_decision", "decision")), ("Tactical paths", "Follow / left / right / defer candidate trajectories and feasibility reasons.", ("tactical", "tactical_evaluation", "candidate", "trajectory")), ("MPC and actuation", "Reference, prediction, requested / delayed / applied controls, limits and solver health.", ("mpc", "mpc_solve", "control", "actuation")), ("Energy and tyres", "Battery, power, reserve, thermal state, four-wheel tyre channels and derates.", ("energy", "tyre", "thermal")), ("Opponent / perception", "Observed opponent pose, covariance, delay, identity confidence and separate truth overlay.", ("opponent", "observation", "perception")), ("Replay / compare", "Run/epoch selection, event bookmarks, aligned prediction versus execution and evidence export.", ("outcome", "comparison", "event"))):
            widget = self._record_panel(title, descriptor, record_types); tabs.addTab(widget, title); self._record_panels.append(widget)

    def show_channel(self, channel_id: str) -> None:
        self._selected_channel = channel_id
        spec = self.registry.get(channel_id); samples = self.registry.samples(channel_id, run_id=self.source.run_id, epoch_id=self.source.epoch_id, until=self.source.sim_time)
        if spec and spec.numeric:
            self.plot.set_series({channel_id: samples}, title=channel_id, y_label=spec.units); self.tabs.setCurrentWidget(self.plot_card)
            self.plot_info.setText(f"{channel_id} · {spec.classification} · {spec.source} · {len(samples)} samples")
        elif spec:
            self.inspect({"channel_id": channel_id, "shape": spec.shape, "available": spec.available, "reason": spec.unavailable_reason})

    def refresh(self) -> None:
        records_for_timeline = self.source.store.records if len(self.source.store.records) < self._timeline_record_count else self.source.store.records[self._timeline_record_count:]
        for record in records_for_timeline:
            key = (record.run_id, record.epoch_id, record.sim_time, record.sequence, record.component, record.record_type)
            if key not in self._timeline_seen:
                self.timeline.ingest(record); self._timeline_seen.add(key)
        self._timeline_record_count = len(self.source.store.records)
        self.explorer.refresh()
        for panel in self._record_panels:
            self._refresh_record_panel(panel)
        self._refresh_selected_plot()
        self.timeline_widget.refresh(until=self.source.sim_time, run_id=self.source.run_id, epoch_id=self.source.epoch_id)

    def _refresh_selected_plot(self) -> None:
        if not self._selected_channel:
            return
        spec = self.registry.get(self._selected_channel)
        if spec is None or not spec.numeric:
            self.plot.set_series({}, title=self._selected_channel, y_label="")
            return
        samples = self.registry.samples(self._selected_channel, run_id=self.source.run_id,
                                        epoch_id=self.source.epoch_id, until=self.source.sim_time)
        self.plot.set_series({self._selected_channel: samples}, title=self._selected_channel, y_label=spec.units)
        self.plot_info.setText(f"{self._selected_channel} · {spec.classification} · {spec.source} · {len(samples)} samples")

    def _refresh_record_panel(self, panel: QWidget) -> None:
        visible = [record for record in self.source.visible_records() if self._matches_type(record.record_type, panel.record_types)]
        keys = [(r.run_id, r.epoch_id, r.sim_time, r.sequence, r.component, r.record_type) for r in visible]
        existing = [panel.records.item(i).data(Qt.UserRole) for i in range(panel.records.count()) if panel.records.item(i).data(Qt.UserRole) is not None]
        if keys != existing:
            selected_key = panel.records.currentItem().data(Qt.UserRole) if panel.records.currentItem() and panel.records.currentItem().data(Qt.UserRole) else None
            panel.records.blockSignals(True); panel.records.clear()
            for record, key in zip(visible, keys):
                item = QListWidgetItem(f"{record.sim_time:.3f}s  {record.record_type}  {record.component}")
                item.setData(Qt.UserRole, key); item.setData(Qt.UserRole + 1, record); panel.records.addItem(item)
            if not visible: panel.records.addItem("No records at selected epoch / time — unavailable")
            panel.records.blockSignals(False)
            if selected_key in keys: panel.records.setCurrentRow(keys.index(selected_key))
        current = panel.records.currentItem()
        record = current.data(Qt.UserRole + 1) if current and current.data(Qt.UserRole + 1) is not None else None
        if isinstance(record, Record):
            panel.inspector.set_payload(record.as_dict())
            self._update_domain_panel(panel, record)
        else:
            panel.inspector.set_payload({"status": "unavailable at selected time", "run_id": self.source.run_id,
                                         "epoch_id": self.source.epoch_id, "sim_time": self.source.sim_time})
            if getattr(panel, "domain", None) is not None:
                if isinstance(panel.domain, TrackMapWidget): panel.domain.set_data()
                elif isinstance(panel.domain, DecisionAlternativesTable): panel.domain.setRowCount(0)
                else: panel.domain.set_series({})

    def _record_panel(self, title: str, descriptor: str, record_types: Sequence[str]) -> QWidget:
        widget = QWidget(); box = QVBoxLayout(widget)
        title_label = QLabel(title.upper()); title_label.setObjectName("hero"); box.addWidget(title_label)
        text = QLabel(descriptor + "  Select a record to freeze its exact payload and causal identifiers."); text.setWordWrap(True); text.setStyleSheet("color:#8ea4b7"); box.addWidget(text)
        domain = None
        if title == "Strategy decisions": domain = DecisionAlternativesTable()
        elif title == "Tactical paths": domain = TrackMapWidget()
        elif title in ("MPC and actuation", "Energy and tyres"): domain = TimeSeriesPlot()
        if domain is not None: box.addWidget(domain)
        row = QHBoxLayout(); records = QListWidget(); inspector = StructuredInspector(); row.addWidget(records, 1); right = QVBoxLayout(); right.addWidget(inspector, 1); row.addLayout(right, 2); box.addLayout(row, 1)
        widget.record_types = tuple(record_types); widget.records = records; widget.inspector = inspector; widget.domain = domain; widget.domain_kind = title
        def selected(item: QListWidgetItem) -> None:
            value = item.data(Qt.UserRole + 1) or item.data(Qt.UserRole)
            if isinstance(value, Record):
                self._pin_record(value); inspector.set_payload(value.as_dict())
        records.itemClicked.connect(selected)
        return widget

    def _update_domain_panel(self, panel: QWidget, record: Record) -> None:
        domain = getattr(panel, "domain", None)
        if isinstance(domain, DecisionAlternativesTable):
            domain.set_decision(record.data); return
        if isinstance(domain, TrackMapWidget):
            trajectory = record.data.get("trajectory", record.data.get("path", record.data.get("points", ())))
            points = []
            for point in trajectory if isinstance(trajectory, (list, tuple)) else ():
                if isinstance(point, Mapping): point = (point.get("x_m", point.get("x")), point.get("y_m", point.get("y")))
                if isinstance(point, (list, tuple)) and len(point) >= 2 and point[0] is not None and point[1] is not None: points.append((float(point[0]), float(point[1])))
            occupancy = record.data.get("opponent_occupancy", record.data.get("occupancy", ()))
            domain.set_data(paths={str(record.data.get("candidate_id", "selected")): points} if points else {}, occupancy=occupancy if isinstance(occupancy, (list, tuple)) else ())
            return
        if isinstance(domain, TimeSeriesPlot):
            arrays = record.data.get("predicted", record.data.get("prediction", {})) if panel.domain_kind.startswith("MPC") else record.data.get("channels", {})
            series = {}
            if panel.domain_kind == "Energy and tyres" and not isinstance(arrays, Mapping): arrays = {}
            if panel.domain_kind == "Energy and tyres" and not arrays:
                unit_group = None
                for spec in self.registry.channels():
                    field = spec.field_path.casefold()
                    if spec.numeric and any(term in field for term in ("energy", "power", "temp", "tyre", "tire", "grip", "wear")):
                        if unit_group is None: unit_group = spec.units
                        if spec.units != unit_group: continue
                        samples = self.registry.samples(spec.channel_id, run_id=self.source.run_id, epoch_id=self.source.epoch_id, until=self.source.sim_time)
                        if samples: series[spec.field_path] = samples
                        if len(series) >= 8: break
            if isinstance(arrays, Mapping):
                for name, values in arrays.items():
                    if not isinstance(values, (list, tuple)) or not all(isinstance(value, (int, float)) for value in values): continue
                    series[str(name)] = [ChannelSample(str(name), float(index), float(value), True, record.epoch_id, record.run_id, record.source, "prediction") for index, value in enumerate(values)]
            if panel.domain_kind.startswith("MPC"):
                actual = record.data.get("actual", record.data.get("executed", {}))
                if isinstance(actual, Mapping):
                    for name, values in actual.items():
                        if isinstance(values, (list, tuple)) and all(isinstance(value, (int, float)) for value in values):
                            series[f"actual/{name}"] = [ChannelSample(f"actual/{name}", float(index), float(value), True, record.epoch_id, record.run_id, record.source, "measurement") for index, value in enumerate(values)]
            domain.set_series(series, title=panel.domain_kind, y_label="declared units")

    @staticmethod
    def _matches_type(record_type: str, accepted: Sequence[str]) -> bool:
        return any(record_type == item or record_type.startswith(item + "_") for item in accepted)

    def inspect(self, payload: Any) -> None:
        record = payload if isinstance(payload, Record) else payload
        if isinstance(record, Record):
            self._pin_record(record); self.inspector.set_payload(record.as_dict())
        else: self.inspector.set_payload(record)
        self.tabs.setCurrentWidget(self.inspector)

    def _pin_record(self, record: Record) -> None:
        """Freeze the shared cursor and retain linked causal records immutably."""
        self.source.playing = False
        self.source.scrub(record.sim_time)
        if isinstance(self.source, LiveDataSource):
            self.source.set_follow_live(False)
        ids = {str(value) for value in record.ids.values() if value not in (None, "")}; related = []; seen = set()
        # Follow the causal identifier graph through state, candidates,
        # trajectories, commands and actuation records at or before the pin.
        changed = True
        while changed:
            changed = False
            for candidate in self.source.store.records:
                key = (candidate.run_id, candidate.epoch_id, candidate.sim_time, candidate.sequence, candidate.component, candidate.record_type)
                if candidate is record or key in seen or candidate.run_id != record.run_id or candidate.epoch_id != record.epoch_id or candidate.sim_time > record.sim_time: continue
                candidate_ids = {str(value) for value in candidate.ids.values() if value not in (None, "")}
                if ids and ids.intersection(candidate_ids):
                    related.append(candidate); seen.add(key); prior = len(ids); ids.update(candidate_ids); changed = changed or len(ids) != prior
        related_predictions = []
        for identifier in ids:
            loader = getattr(self.source, "load_prediction", None)
            if loader is not None:
                loaded = loader(identifier)
                if loaded is not None: related_predictions.append({"prediction_id": identifier, "payload": loaded})
        self.pin.pin(record, [*related, *related_predictions])


class MainWindow(QMainWindow):
    """1080p-friendly presentation/engineering shell with one shared cursor."""

    def __init__(self, source: Optional[ReplayDataSource] = None, *, command_callback: Optional[Callable[[str, Mapping[str, Any]], None]] = None) -> None:
        super().__init__(); self.setWindowTitle("EUFS Race Intelligence · Telemetry & Evidence"); self.resize(1440, 900); self.setStyleSheet(STYLE)
        self.source = source or ReplayDataSource(); self.registry = self.source.registry; self.timeline = EventTimeline(); self.pin = DecisionPin(); self.command_callback = command_callback; self._pending_live_refresh = 0; self._display_drop_count = 0
        self._build_toolbar(); self._build_views(); self._timer = QTimer(self); self._timer.setInterval(50); self._timer.timeout.connect(self._tick); self._timer.start()

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("GUI mode"); toolbar.setMovable(False); self.addToolBar(toolbar)
        toolbar.addWidget(QLabel("  EUFS RACE INTELLIGENCE  "))
        self.mode = QComboBox(); self.mode.addItems(["Presentation", "Engineering"]); self.mode.currentTextChanged.connect(self.set_mode); toolbar.addWidget(self.mode)
        toolbar.addSeparator(); self.freeze = QPushButton("Freeze view"); self.freeze.setCheckable(True); self.freeze.toggled.connect(self._freeze_changed); toolbar.addWidget(self.freeze)
        self.return_live = QPushButton("Return to live"); self.return_live.clicked.connect(self._return_to_live); toolbar.addWidget(self.return_live)
        toolbar.addWidget(QLabel("   Source: ")); self.source_label = QLabel("live ROS" if isinstance(self.source, LiveDataSource) else "recorded replay"); toolbar.addWidget(self.source_label)
        export = QAction("Export evidence…", self); export.triggered.connect(self.export_evidence); toolbar.addAction(export)

    def _build_views(self) -> None:
        self.stack = QStackedWidget(); self.setCentralWidget(self.stack)
        self.presentation = PresentationView(self.source); self.engineering = EngineeringView(self.source, self.registry, self.timeline, self.pin)
        self.control = ScenarioControl(); self.control.commandRequested.connect(self._command); self.control.set_connected(False)
        self.control.set_manifest(getattr(self.source, "manifest", {}))
        self.stack.addWidget(self.presentation); self.stack.addWidget(self.engineering)
        dock = QDockWidget("Scenario control", self); dock.setWidget(self.control); dock.setObjectName("scenarioControl"); self.addDockWidget(Qt.LeftDockWidgetArea, dock)
        self.replay = ReplayControls(self.source); self.replay.cursorChanged.connect(self.refresh); self.replay.stepRequested.connect(self._step); replay_dock = QDockWidget("Replay", self); replay_dock.setWidget(self.replay); replay_dock.setObjectName("replayControls"); self.addDockWidget(Qt.BottomDockWidgetArea, replay_dock)
        self.engineering.refresh()

    def set_mode(self, mode: str) -> None:
        self.stack.setCurrentWidget(self.presentation if mode == "Presentation" else self.engineering)

    def _tick(self) -> None:
        if self.freeze.isChecked(): return
        self.source.tick(0.05); self.refresh()

    def _freeze_changed(self, frozen: bool) -> None:
        if isinstance(self.source, LiveDataSource):
            self.source.set_follow_live(not frozen)

    def attach_live_bridge(self, bridge: Any) -> None:
        """Attach a threaded ROS bridge; callbacks arrive as queued Qt signals."""
        if not isinstance(self.source, LiveDataSource):
            raise TypeError("live bridge requires MainWindow(LiveDataSource(...))")
        self._live_bridge = bridge
        self.command_callback = bridge.send_operator_request
        bridge.recordReceived.connect(self._ingest_live_record)
        bridge.healthChanged.connect(self._bridge_health)
        bridge.connectedChanged.connect(self._bridge_connected)
        bridge.errorRaised.connect(lambda message: self.source_label.setText(f"LIVE ROS · {message}"))
        bridge.start()

    def _ingest_live_record(self, record: Record) -> None:
        if isinstance(self.source, LiveDataSource):
            self.source.push(record)
            if self._pending_live_refresh >= 1000: self._display_drop_count += 1
            else: self._pending_live_refresh += 1

    def _bridge_connected(self, connected: bool) -> None:
        self.source_label.setText("LIVE ROS · connected" if connected else "LIVE ROS · disconnected")
        self.control.set_connected(False)

    def _bridge_health(self, payload: Mapping[str, Any]) -> None:
        ready = bool(payload.get("ready", payload.get("healthy", False)))
        self.source_label.setText("LIVE ROS · orchestrator ready" if ready else "LIVE ROS · orchestrator not ready")
        self.control.set_connected(ready)

    def _return_to_live(self) -> None:
        self.pin.clear()
        self.freeze.setChecked(False)
        if isinstance(self.source, LiveDataSource):
            self.source.set_follow_live(True)
        else:
            times = [r.sim_time for r in self.source.store.records if r.run_id == self.source.run_id and r.epoch_id == self.source.epoch_id]
            if times: self.source.scrub(max(times))
        self.refresh()

    def _step(self, direction: int) -> None:
        self.source.step(direction); self.refresh()

    def refresh(self) -> None:
        self.replay.refresh(); self.presentation.refresh(); self.engineering.refresh(); self._pending_live_refresh = 0
        if self._display_drop_count:
            self.source_label.setToolTip(f"{self._display_drop_count} display refreshes coalesced; recorder/history retained all received records")

    def _command(self, action: str, parameters: Mapping[str, Any]) -> None:
        if self.command_callback is not None:
            self.command_callback(action, parameters)

    def export_evidence(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export evidence", "eufs_evidence.json", "JSON (*.json)")
        if path:
            payload = {"run_id": self.source.run_id, "epoch_id": self.source.epoch_id, "sim_time": self.source.sim_time, "pinned_decision": self.pin.value, "records_visible": len(self.source.visible_records())}
            with open(path, "w", encoding="utf-8") as stream: json.dump(payload, stream, indent=2, default=str)

    def closeEvent(self, event: Any) -> None:
        bridge = getattr(self, "_live_bridge", None)
        if bridge is not None:
            bridge.stop()
        super().closeEvent(event)


def create_application(argv: Optional[Sequence[str]] = None) -> QApplication:
    app = QApplication(list(argv or [])); app.setApplicationName("EUFS Race Intelligence"); return app


def run(argv: Optional[Sequence[str]] = None) -> int:
    app = create_application(argv); window = MainWindow(); window.show(); return app.exec_()
