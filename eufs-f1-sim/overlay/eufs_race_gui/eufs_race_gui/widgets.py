"""Reusable PyQt5 widgets for the race GUI.

The widgets are intentionally data-oriented: every panel can be rendered from
the replay/live adapters and remains useful with partial logs.  A missing
channel is displayed as unavailable instead of being replaced with a zero.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QAbstractItemView, QComboBox, QFileDialog, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
    QPushButton, QScrollArea, QSizePolicy, QSlider, QSpinBox, QStackedWidget,
    QTableWidget, QTableWidgetItem, QTextEdit, QTreeWidget, QTreeWidgetItem,
    QVBoxLayout, QWidget,
)

from .core import ChannelRegistry, ChannelSample, ChannelSpec, EventTimeline, Record, ReplayDataSource


COLORS = {
    "bg": QColor("#0b1018"), "panel": QColor("#111b27"), "line": QColor("#26384b"),
    "text": QColor("#e8f0f7"), "muted": QColor("#8ea4b7"), "red": QColor("#ef4444"),
    "cyan": QColor("#34d5e6"), "amber": QColor("#f5b83d"), "green": QColor("#58d68d"),
    "purple": QColor("#ae8bff"), "blue": QColor("#5b9dff"),
}


def _display(value: Any, units: str = "") -> str:
    if value is None:
        return "Unavailable"
    if isinstance(value, float) and not math.isfinite(value):
        return "Unavailable"
    if isinstance(value, float):
        return f"{value:.3f} {units}".strip()
    return f"{value} {units}".strip()


class Card(QFrame):
    def __init__(self, title: str = "", parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        if title:
            label = QLabel(title.upper())
            label.setObjectName("sectionTitle")
            layout.addWidget(label)
        self.body = layout


class TimeSeriesPlot(QWidget):
    """Dependency-free plot supporting multiple scalar channels and gaps."""

    cursorChanged = pyqtSignal(float)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(150)
        self.series: Dict[str, Sequence[ChannelSample]] = {}
        self.y_label = ""
        self.title = ""
        self.cursor: Optional[float] = None
        self.setMouseTracking(True)

    def set_series(self, series: Mapping[str, Sequence[ChannelSample]], *, title: str = "", y_label: str = "") -> None:
        self.series = dict(series)
        self.title, self.y_label = title, y_label
        self.update()

    def mouseMoveEvent(self, event: Any) -> None:
        if not self.series:
            return
        values = [sample.sim_time for points in self.series.values() for sample in points]
        if not values:
            return
        left, right = 42, max(43, self.width() - 15)
        self.cursor = min(values) + (max(values) - min(values)) * max(0.0, min(1.0, (event.x() - left) / max(1, right - left)))
        nearest = []
        for name, samples in self.series.items():
            valid = [sample for sample in samples if sample.valid and isinstance(sample.value, (int, float))]
            if valid:
                sample = min(valid, key=lambda item: abs(item.sim_time - self.cursor)); nearest.append(f"{name.split('/')[-1]}={float(sample.value):.3g}")
        self.setToolTip(f"t={self.cursor:.3f}s  " + "  ".join(nearest))
        self.cursorChanged.emit(self.cursor)
        self.update()

    def paintEvent(self, _event: Any) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), COLORS["panel"])
        painter.setRenderHint(QPainter.Antialiasing)
        left, top, right, bottom = 42, 25, self.width() - 15, self.height() - 24
        painter.setPen(QPen(COLORS["line"], 1))
        for fraction in (0.0, .25, .5, .75, 1.0):
            y = top + fraction * (bottom - top)
            painter.drawLine(left, int(y), right, int(y))
        all_samples = [sample for points in self.series.values() for sample in points]
        valid = [sample for sample in all_samples if sample.valid and isinstance(sample.value, (int, float))]
        if not valid:
            painter.setPen(COLORS["muted"])
            painter.drawText(QRectF(left, top, right - left, bottom - top), Qt.AlignCenter, "No valid samples in selected interval")
            return
        t_min, t_max = min(s.sim_time for s in valid), max(s.sim_time for s in valid)
        y_values = [float(s.value) for s in valid]
        y_min, y_max = min(y_values), max(y_values)
        if y_min == y_max:
            y_min, y_max = y_min - 1.0, y_max + 1.0
        painter.setPen(COLORS["muted"])
        painter.setFont(QFont("DejaVu Sans", 8))
        painter.drawText(3, 14, self.title)
        painter.drawText(3, top + 4, f"{y_max:.3g}")
        painter.drawText(3, bottom, f"{y_min:.3g}")
        painter.drawText(left, bottom + 15, f"{t_min:.2f}s")
        painter.drawText(int((left + right) / 2 - 20), bottom + 15, f"{(t_min + t_max) / 2:.2f}s")
        painter.drawText(right - 55, bottom + 15, f"{t_max:.2f}s")
        painter.drawText(right - 60, top + 4, self.y_label)
        palette = [COLORS["cyan"], COLORS["amber"], COLORS["purple"], COLORS["green"], COLORS["blue"]]
        for index, (name, samples) in enumerate(self.series.items()):
            painter.setPen(QPen(palette[index % len(palette)], 2))
            previous = None
            for sample in samples:
                if not sample.valid or not isinstance(sample.value, (int, float)):
                    previous = None  # gaps are intentionally not bridged
                    continue
                x = left if t_max == t_min else left + (sample.sim_time - t_min) / (t_max - t_min) * (right - left)
                y = bottom - (float(sample.value) - y_min) / (y_max - y_min) * (bottom - top)
                point = QPointF(x, y)
                if previous is not None:
                    painter.drawLine(previous, point)
                previous = point
            painter.drawText(right - 170, top + 14 * index, name.split("/")[-1][-24:])
        if self.cursor is not None and t_min <= self.cursor <= t_max:
            x = left + (self.cursor - t_min) / max(1e-9, t_max - t_min) * (right - left)
            painter.setPen(QPen(COLORS["text"], 1, Qt.DashLine))
            painter.drawLine(int(x), top, int(x), bottom)

    def export_png(self, path: str | Path) -> None:
        self.grab().save(str(path), "PNG")


class TrackMapWidget(QWidget):
    """Small, scalable 2D map with distinct truth and estimate styling."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(400, 260)
        self.track: Sequence[Sequence[float]] = ()
        self.cars: Dict[str, Mapping[str, Any]] = {}
        self.trails: Dict[str, Sequence[Sequence[float]]] = {}
        self.paths: Dict[str, Sequence[Sequence[float]]] = {}
        self.occupancy: Sequence[Mapping[str, Any]] = ()
        self.labels: Sequence[Mapping[str, Any]] = ()

    def set_data(self, *, track: Sequence[Sequence[float]] = (), cars: Mapping[str, Mapping[str, Any]] = {},
                 trails: Mapping[str, Sequence[Sequence[float]]] = {}, paths: Mapping[str, Sequence[Sequence[float]]] = {},
                 labels: Sequence[Mapping[str, Any]] = (), occupancy: Sequence[Mapping[str, Any]] = ()) -> None:
        self.track, self.cars, self.trails, self.paths, self.labels, self.occupancy = track, dict(cars), dict(trails), dict(paths), labels, occupancy
        self.update()

    def _transform(self, point: Sequence[float], bounds: QRectF, extent: Sequence[float]) -> QPointF:
        x, y = float(point[0]), float(point[1])
        xmin, xmax, ymin, ymax = extent
        sx = bounds.left() + (x - xmin) / max(1e-9, xmax - xmin) * bounds.width()
        sy = bounds.bottom() - (y - ymin) / max(1e-9, ymax - ymin) * bounds.height()
        return QPointF(sx, sy)

    def paintEvent(self, _event: Any) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), COLORS["panel"])
        all_points = list(self.track)
        for points in list(self.trails.values()) + list(self.paths.values()):
            all_points.extend(points)
        for car in self.cars.values():
            if car.get("position"):
                all_points.append(car["position"])
        for item in self.occupancy:
            point = item.get("position", item.get("center")) if isinstance(item, Mapping) else None
            if isinstance(point, Mapping): point = (point.get("x_m", point.get("x")), point.get("y_m", point.get("y")))
            if isinstance(point, (list, tuple)) and len(point) >= 2 and point[0] is not None and point[1] is not None: all_points.append(point)
        if not all_points:
            painter.setPen(COLORS["muted"])
            painter.drawText(self.rect(), Qt.AlignCenter, "Circuit geometry unavailable")
            return
        xs, ys = [float(p[0]) for p in all_points], [float(p[1]) for p in all_points]
        pad_x, pad_y = max(1.0, (max(xs) - min(xs)) * .08), max(1.0, (max(ys) - min(ys)) * .08)
        extent = (min(xs) - pad_x, max(xs) + pad_x, min(ys) - pad_y, max(ys) + pad_y)
        bounds = QRectF(14, 24, self.width() - 28, self.height() - 38)
        painter.setRenderHint(QPainter.Antialiasing)
        if self.track:
            painter.setPen(QPen(QColor("#52677e"), 9, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            transformed = [self._transform(p, bounds, extent) for p in self.track]
            for first, second in zip(transformed, transformed[1:]):
                painter.drawLine(first, second)
            painter.setPen(QPen(QColor("#c6d4df"), 2))
            for first, second in zip(transformed, transformed[1:]):
                painter.drawLine(first, second)
        palette = [COLORS["cyan"], COLORS["amber"], COLORS["purple"]]
        for index, (name, points) in enumerate(self.trails.items()):
            painter.setPen(QPen(palette[index % len(palette)], 2, Qt.DashLine))
            transformed = [self._transform(p, bounds, extent) for p in points]
            for first, second in zip(transformed, transformed[1:]):
                painter.drawLine(first, second)
        for name, points in self.paths.items():
            painter.setPen(QPen(COLORS["green"], 2, Qt.DotLine))
            transformed = [self._transform(p, bounds, extent) for p in points]
            for first, second in zip(transformed, transformed[1:]):
                painter.drawLine(first, second)
        painter.setPen(QPen(COLORS["amber"], 1, Qt.DashLine)); painter.setBrush(Qt.NoBrush)
        for item in self.occupancy:
            point = item.get("position", item.get("center")) if isinstance(item, Mapping) else None
            if isinstance(point, Mapping): point = (point.get("x_m", point.get("x")), point.get("y_m", point.get("y")))
            if isinstance(point, (list, tuple)) and len(point) >= 2 and point[0] is not None and point[1] is not None:
                center = self._transform(point, bounds, extent); radius = max(5.0, float(item.get("radius_m", item.get("radius", 2.0))) * bounds.width() / max(1.0, extent[1] - extent[0])); painter.drawEllipse(center, radius, radius)
        painter.setFont(QFont("DejaVu Sans", 8))
        for label in self.labels:
            position = label.get("position")
            if position:
                point = self._transform(position, bounds, extent)
                painter.setPen(COLORS["muted"])
                painter.drawText(point + QPointF(4, -4), str(label.get("name", "")))
        for index, (name, car) in enumerate(self.cars.items()):
            position = car.get("position")
            if not position:
                continue
            point = self._transform(position, bounds, extent)
            color = COLORS["cyan"] if name in ("ego", "eufs") else COLORS["amber"]
            if car.get("truth"):
                painter.setPen(QPen(COLORS["purple"], 2, Qt.DashLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(point, 9, 9)
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(point, 6, 6)
            painter.setPen(COLORS["text"])
            painter.drawText(point + QPointF(10, 4), str(name))
        painter.setPen(COLORS["muted"])
        painter.drawText(16, 16, "CIRCUIT / ESTIMATE + TRUTH OVERLAYS")


class ChannelExplorer(QWidget):
    channelSelected = pyqtSignal(str)

    def __init__(self, registry: ChannelRegistry, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.registry = registry
        layout = QVBoxLayout(self)
        filters = QHBoxLayout()
        self.search = QLineEdit()
        self.search.setPlaceholderText("Search channels, source fields, units…")
        self.car_filter = QComboBox()
        self.car_filter.addItems(["All cars", "ego", "opponent", "eufs", "eufs2"])
        self.class_filter = QComboBox()
        self.class_filter.addItems(["All sources", "measurement", "estimate", "prediction", "truth"])
        filters.addWidget(self.search, 1); filters.addWidget(self.car_filter); filters.addWidget(self.class_filter)
        layout.addLayout(filters)
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["Channel", "Car", "Units", "Shape", "Source", "Value", "Status"])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self._selected)
        layout.addWidget(self.table)
        self._channel_keys = []
        self.search.textChanged.connect(self.refresh); self.car_filter.currentTextChanged.connect(self.refresh)
        self.class_filter.currentTextChanged.connect(self.refresh)
        self.refresh()

    def refresh(self) -> None:
        car = self.car_filter.currentText(); car = None if car == "All cars" else car
        classification = self.class_filter.currentText(); classification = None if classification == "All sources" else classification
        specs = self.registry.channels(self.search.text(), car=car, classification=classification)
        keys = [spec.channel_id for spec in specs]
        if keys != self._channel_keys:
            self.table.setRowCount(len(specs)); self._channel_keys = keys
        elif self.table.rowCount() != len(specs):
            self.table.setRowCount(len(specs))
        for row, spec in enumerate(specs):
            latest = self.registry.latest(spec.channel_id)
            values = [spec.channel_id, spec.car or "—", spec.units or "—", spec.shape, spec.source,
                      _display(latest.value, spec.units) if latest and latest.valid else "Unavailable",
                      "OK" if spec.available and (latest is None or latest.valid) else (spec.unavailable_reason or "stale")]
            for col, value in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(str(value)))

    def _selected(self) -> None:
        rows = self.table.selectionModel().selectedRows()
        if rows:
            self.channelSelected.emit(self.table.item(rows[0].row(), 0).text())


class StructuredInspector(QTreeWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setHeaderLabels(["Field", "Value"])
        self.setColumnWidth(0, 230)

    def set_payload(self, payload: Any) -> None:
        self.clear()
        def add(parent: Any, key: str, value: Any) -> None:
            item = QTreeWidgetItem([str(key), _display(value)])
            parent.addChild(item) if hasattr(parent, "addChild") else self.addTopLevelItem(item)
            if isinstance(value, Mapping):
                for child_key, child_value in value.items(): add(item, child_key, child_value)
            elif isinstance(value, (list, tuple)):
                for index, child_value in enumerate(value): add(item, index, child_value)
        if isinstance(payload, Mapping):
            for key, value in payload.items(): add(self.invisibleRootItem(), key, value)
        else:
            add(self.invisibleRootItem(), "value", payload)
        self.expandToDepth(1)


class DecisionAlternativesTable(QTableWidget):
    """Compact causal comparison for strategy alternatives."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(0, 7, parent)
        self.setHorizontalHeaderLabels(["Candidate", "Feasible", "Reason", "Time", "Energy", "Exit reserve", "Next-best margin"])
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)

    def set_decision(self, payload: Mapping[str, Any]) -> None:
        alternatives = payload.get("alternatives", payload.get("candidates", ()))
        if not isinstance(alternatives, (list, tuple)):
            alternatives = ()
        self.setRowCount(len(alternatives))
        for row, candidate in enumerate(alternatives):
            candidate = candidate if isinstance(candidate, Mapping) else {"candidate": candidate}
            values = [candidate.get("candidate", candidate.get("action", candidate.get("id", "—"))),
                      candidate.get("feasible", "unavailable"), candidate.get("reason", candidate.get("reason_code", "—")),
                      candidate.get("time_s", candidate.get("predicted_time_s", "—")), candidate.get("energy_j", candidate.get("deploy_cost_j", "—")),
                      candidate.get("exit_reserve_j", candidate.get("exit_reserve", "—")), candidate.get("next_best_margin", payload.get("next_best_margin", "—"))]
            for column, value in enumerate(values): self.setItem(row, column, QTableWidgetItem(str(value)))


class EventTimelineWidget(QWidget):
    eventSelected = pyqtSignal(object)

    def __init__(self, timeline: EventTimeline, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.timeline = timeline
        layout = QVBoxLayout(self)
        self.search = QLineEdit(); self.search.setPlaceholderText("Filter events, health and runtime records…")
        self.list = QListWidget(); layout.addWidget(self.search); layout.addWidget(self.list)
        self.search.textChanged.connect(self.refresh); self.list.itemClicked.connect(self._clicked)
        self.refresh()

    def refresh(self, until: Optional[float] = None, run_id: Optional[str] = None, epoch_id: Any = None) -> None:
        events = self.timeline.query(self.search.text(), run_id=run_id, epoch_id=epoch_id, until=until)
        keys = [(event.run_id, event.epoch_id, event.sim_time, event.sequence, event.component, event.record_type) for event in events]
        existing = [self.list.item(index).data(Qt.UserRole + 1) for index in range(self.list.count()) if self.list.item(index).data(Qt.UserRole + 1) is not None]
        if keys == existing:
            return
        selected = self.list.currentItem().data(Qt.UserRole + 1) if self.list.currentItem() else None
        self.list.blockSignals(True); self.list.clear()
        for event, key in zip(events, keys):
            item = QListWidgetItem(f"{event.sim_time:9.3f}s  {event.record_type:<10} {event.component}: {event.data}")
            item.setData(Qt.UserRole, event); item.setData(Qt.UserRole + 1, key); self.list.addItem(item)
        self.list.blockSignals(False)
        if selected in keys: self.list.setCurrentRow(keys.index(selected))

    def _clicked(self, item: QListWidgetItem) -> None:
        self.eventSelected.emit(item.data(Qt.UserRole))


class ReplayControls(QWidget):
    playChanged = pyqtSignal(bool)
    cursorChanged = pyqtSignal(float)
    stepRequested = pyqtSignal(int)

    def __init__(self, source: ReplayDataSource, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.source = source
        layout = QHBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        self.play = QPushButton("▶ Play"); self.play.setCheckable(True); self.play.toggled.connect(self._toggle)
        back = QPushButton("‹ Step"); forward = QPushButton("Step ›")
        back.clicked.connect(lambda: self.stepRequested.emit(-1)); forward.clicked.connect(lambda: self.stepRequested.emit(1))
        self.slider = QSlider(Qt.Horizontal); self.slider.setRange(0, 1000); self.slider.valueChanged.connect(self._scrub)
        self.time_label = QLabel("Replay / unavailable")
        layout.addWidget(self.play); layout.addWidget(back); layout.addWidget(forward); layout.addWidget(self.slider, 1); layout.addWidget(self.time_label)
        self.refresh()

    def refresh(self) -> None:
        records = self.source.store.records
        if records:
            times = [r.sim_time for r in records if r.run_id == self.source.run_id and r.epoch_id == self.source.epoch_id]
            if times:
                lo, hi = min(times), max(times)
                self.slider.blockSignals(True); self.slider.setValue(1000 if hi == lo else int((self.source.sim_time - lo) / (hi - lo) * 1000)); self.slider.blockSignals(False)
                self.time_label.setText(f"RECORDED  {self.source.sim_time:.3f}s")
                return
        self.time_label.setText("REPLAY / no records")

    def _toggle(self, state: bool) -> None:
        self.source.playing = state; self.playChanged.emit(state); self.play.setText("❚❚ Pause" if state else "▶ Play")

    def _scrub(self, value: int) -> None:
        records = [r for r in self.source.store.records if r.run_id == self.source.run_id and r.epoch_id == self.source.epoch_id]
        if not records: return
        lo, hi = min(r.sim_time for r in records), max(r.sim_time for r in records)
        self.source.scrub(lo + (hi - lo) * value / 1000.0); self.cursorChanged.emit(self.source.sim_time); self.refresh()


def export_samples_csv(registry: ChannelRegistry, channel_id: str, path: str | Path) -> None:
    samples = registry.samples(channel_id)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream); writer.writerow(["run_id", "epoch_id", "sim_time", "value", "valid", "source", "reason"])
        for sample in samples:
            writer.writerow([sample.run_id, sample.epoch_id, sample.sim_time, sample.value, sample.valid, sample.source, sample.reason])
