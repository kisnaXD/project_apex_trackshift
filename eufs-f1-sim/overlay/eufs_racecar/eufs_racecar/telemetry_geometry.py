"""Small dependency-free COTA Frenet projection for dashboard telemetry."""

from __future__ import annotations

import csv
import math
from pathlib import Path


class TrackProjector:
    def __init__(self, centerline: str | Path):
        self.points = []
        with Path(centerline).open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                try:
                    x = float(row.get("x_m", row.get("x", "nan")))
                    y = float(row.get("y_m", row.get("y", "nan")))
                    s = float(row.get("s_m", row.get("s", "nan")))
                except (TypeError, ValueError):
                    continue
                if all(math.isfinite(value) for value in (x, y, s)):
                    self.points.append((x, y, s))
        if len(self.points) < 3:
            raise ValueError("centerline needs at least three points")
        self.closure = math.hypot(self.points[0][0] - self.points[-1][0],
                                  self.points[0][1] - self.points[-1][1])
        self.length = self.points[-1][2] - self.points[0][2] + self.closure
        if self.length <= 0.0:
            self.length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                              for a, b in zip(self.points, self.points[1:])) + self.closure
        offset = self.points[0][2]
        self.points = [(x, y, s - offset) for x, y, s in self.points]

    def project(self, x: float, y: float, previous_s: float | None = None):
        if not all(math.isfinite(float(value)) for value in (x, y)):
            raise ValueError("projection coordinates must be finite")
        best = None
        for index, (ax, ay, s0) in enumerate(self.points):
            bx, by, next_s = self.points[(index + 1) % len(self.points)]
            span = self.length - s0 if index == len(self.points) - 1 else next_s - s0
            dx, dy = bx - ax, by - ay
            length2 = dx * dx + dy * dy
            t = ((x - ax) * dx + (y - ay) * dy) / length2 if length2 > 1e-12 else 0.0
            t = max(0.0, min(1.0, t))
            qx, qy = ax + t * dx, ay + t * dy
            distance = math.hypot(x - qx, y - qy)
            if best is None or distance < best[0]:
                tangent = max(math.hypot(dx, dy), 1e-12)
                s = (s0 + t * span) % self.length
                lateral = (-dy * (x - qx) + dx * (y - qy)) / tangent
                best = (distance, s, lateral)
        return best[1], best[2]


class SchemaTrackProjector(TrackProjector):
    """Project the schema's world coordinates onto an EUFS centerline.

    The alias is intentionally a concrete class so callers can depend on the
    telemetry-facing name while the small geometric implementation remains
    reusable in tests.
    """

    @staticmethod
    def signed_shortest_gap(opponent_s: float, ego_s: float, length: float) -> float:
        if not all(math.isfinite(float(value)) for value in (opponent_s, ego_s, length)):
            raise ValueError("Frenet values must be finite")
        if length <= 0.0:
            raise ValueError("track length must be positive")
        gap = (float(opponent_s) - float(ego_s)) % float(length)
        if gap > 0.5 * length:
            gap -= length
        return gap
