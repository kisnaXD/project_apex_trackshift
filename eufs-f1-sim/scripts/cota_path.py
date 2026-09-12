#!/usr/bin/env python3
"""Dependency-free closed-loop COTA route geometry and lap controller."""
from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

STEERING_LIMIT = 0.6458
STEERING_RATE = 1.2916
WHEELBASE = 3.28
MAX_SPEED = 15.0
MAX_LATERAL_ACCEL = 1.5
MAX_BRAKE = 1.2


@dataclass(frozen=True)
class RoutePoint:
    x: float
    y: float
    yaw: float
    curvature: float
    s: float


@dataclass(frozen=True)
class Projection:
    s: float
    lateral_error: float
    heading_error: float
    distance: float


@dataclass(frozen=True)
class ControllerStep:
    """One feedback result from :meth:`LapController.step`."""
    steer: float
    targetspeed: float
    accel: float
    progress: float
    clearance: float
    done: bool = False
    failure: str | None = None

    @property
    def target_speed(self) -> float:
        return self.targetspeed

    @property
    def steering(self) -> float:
        return self.steer

    def __getitem__(self, key):
        if isinstance(key, str):
            return getattr(self, key)
        return tuple(self)[key]

    def __iter__(self):
        yield self.steer
        yield self.targetspeed
        yield self.accel
        yield self.progress
        yield self.clearance
        yield self.done
        yield self.failure


@dataclass(frozen=True)
class _BoundaryPoint:
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    s: float


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _read_centerline(path: Path) -> list[RoutePoint]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    out: list[RoutePoint] = []
    for row in rows:
        out.append(RoutePoint(
            float(row.get("x_m", row.get("x", 0.0))),
            float(row.get("y_m", row.get("y", 0.0))),
            float(row.get("yaw_rad", row.get("yaw", 0.0))),
            float(row.get("curvature_1pm", row.get("curvature", 0.0))),
            float(row.get("s_m", row.get("s", 0.0))),
        ))
    return out


def _read_boundaries(path: Path) -> list[_BoundaryPoint]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    points = [_BoundaryPoint(
        float(row["left_x_m"]), float(row["left_y_m"]),
        float(row["right_x_m"]), float(row["right_y_m"]),
        float(row.get("s_m", i)),
    ) for i, row in enumerate(rows)]
    if points:
        first_s = points[0].s
        points = [_BoundaryPoint(p.left_x, p.left_y, p.right_x, p.right_y,
                                 p.s - first_s) for p in points]
    return points


def _segment_projection(px: float, py: float, ax: float, ay: float,
                        bx: float, by: float) -> tuple[float, float, float, float]:
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = ((px - ax) * dx + (py - ay) * dy) / length2 if length2 > 1e-12 else 0.0
    t = max(0.0, min(1.0, t))
    qx, qy = ax + t * dx, ay + t * dy
    return t, qx, qy, math.hypot(px - qx, py - qy)


class CotaRoute:
    """A closed, uniformly resampled route with optional real boundaries."""

    def __init__(self, centerline: Path | str, boundaries: Path | str | None = None,
                 spacing: float = 1.0):
        if spacing <= 0.0:
            raise ValueError("route spacing must be positive")
        raw = _read_centerline(Path(centerline))
        if len(raw) < 3:
            raise ValueError("COTA centerline needs at least three points")
        self.raw = raw
        self.spacing = float(spacing)
        self.closure_length = math.hypot(raw[0].x - raw[-1].x, raw[0].y - raw[-1].y)
        source_length = raw[-1].s - raw[0].s
        if source_length <= 0.0:
            source_length = sum(math.hypot(b.x - a.x, b.y - a.y)
                                for a, b in zip(raw, raw[1:]))
        self.length = source_length + self.closure_length
        self._source = [RoutePoint(p.x, p.y, p.yaw, p.curvature, p.s - raw[0].s)
                        for p in raw]
        self._source.append(RoutePoint(raw[0].x, raw[0].y, raw[0].yaw,
                                       raw[0].curvature, self.length))
        self._source_s = [p.s for p in self._source]
        self.points = self._resample(self.spacing)
        self._s = [p.s for p in self.points] + [self.length]
        if boundaries is None:
            sibling = Path(centerline).with_name("boundaries.csv")
            boundaries = sibling if sibling.exists() else None
        self.boundaries = _read_boundaries(Path(boundaries)) if boundaries else None
        self._boundary_left: list[tuple[float, float]] = []
        self._boundary_right: list[tuple[float, float]] = []
        if self.boundaries and len(self.boundaries) >= 2:
            self._boundary_left = [(p.left_x, p.left_y) for p in self.boundaries]
            self._boundary_right = [(p.right_x, p.right_y) for p in self.boundaries]
            self._boundary_left.append(self._boundary_left[0])
            self._boundary_right.append(self._boundary_right[0])

    def _raw_at(self, s: float) -> RoutePoint:
        s = max(0.0, min(self.length, s))
        i = bisect.bisect_right(self._source_s, s) - 1
        i = max(0, min(i, len(self._source) - 2))
        a, b = self._source[i], self._source[i + 1]
        span = b.s - a.s
        t = (s - a.s) / span if span > 1e-12 else 0.0
        return RoutePoint(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                          _wrap(a.yaw + t * _wrap(b.yaw - a.yaw)),
                          a.curvature + t * (b.curvature - a.curvature), s)

    def _resample(self, spacing: float) -> list[RoutePoint]:
        count = max(3, int(math.ceil(self.length / spacing)))
        return [self._raw_at(min(i * spacing, self.length - 1e-9)) for i in range(count)]

    def at(self, s: float) -> RoutePoint:
        s = float(s) % self.length
        i = bisect.bisect_right(self._s[:-1], s) - 1
        i = max(0, min(i, len(self.points) - 1))
        a = self.points[i]
        b = self.points[(i + 1) % len(self.points)]
        b_s = self.length if i == len(self.points) - 1 else b.s
        span = b_s - a.s
        t = (s - a.s) / span if span > 1e-12 else 0.0
        return RoutePoint(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                          _wrap(a.yaw + t * _wrap(b.yaw - a.yaw)),
                          a.curvature + t * (b.curvature - a.curvature), s)

    def _candidate_segments(self, previous_s: float | None, window: float) -> Iterable[int]:
        n = len(self.points)
        if previous_s is None:
            return range(n)
        center = previous_s % self.length
        count = max(1, int(math.ceil(window / self.spacing)))
        center_i = bisect.bisect_right(self._s[:-1], center) - 1
        return ((center_i + j) % n for j in range(-count - 1, count + 2))

    def project(self, x: float, y: float, previous_s: float | None = None,
                window: float = 25.0, yaw: float | None = None) -> Projection:
        """Project onto exact segments, locally after the first projection."""
        best: tuple[float, float, float, float] | None = None
        for i in self._candidate_segments(previous_s, max(window, self.spacing)):
            a = self.points[i]
            b = self.points[(i + 1) % len(self.points)]
            t, qx, qy, distance = _segment_projection(x, y, a.x, a.y, b.x, b.y)
            b_s = self.length if i == len(self.points) - 1 else b.s
            s = a.s + t * (b_s - a.s)
            dx, dy = b.x - a.x, b.y - a.y
            lateral = (-dy * (x - qx) + dx * (y - qy)) / max(math.hypot(dx, dy), 1e-12)
            if best is None or distance < best[0]:
                best = (distance, s, lateral, math.atan2(dy, dx))
        if best is None:
            raise RuntimeError("route has no projectable segments")
        distance, s, lateral, route_yaw = best
        heading_error = _wrap(yaw - route_yaw) if yaw is not None else 0.0
        return Projection(s % self.length, lateral, heading_error, distance)

    def target(self, s: float, lookahead: float) -> RoutePoint:
        return self.at(s + max(0.5, lookahead))

    @staticmethod
    def _polyline_distance(x: float, y: float, line: Sequence[tuple[float, float]]) -> float:
        return min(_segment_projection(x, y, a[0], a[1], b[0], b[1])[3]
                   for a, b in zip(line, line[1:]))

    @staticmethod
    def _inside_polygon(x: float, y: float,
                        polygon: Sequence[tuple[float, float]]) -> bool:
        inside = False
        for (ax, ay), (bx, by) in zip(polygon, polygon[1:]):
            if (ay > y) != (by > y):
                cross_x = (bx - ax) * (y - ay) / (by - ay) + ax
                if x < cross_x:
                    inside = not inside
        return inside

    def boundary_clearance(self, x: float, y: float,
                           previous_s: float | None = None) -> float:
        """Signed distance to the nearest real boundary (positive inside)."""
        if not self._boundary_left:
            return float("inf")
        # A polygon point-in-test incorrectly rejects the small seam region
        # behind the first CSV point, even though that region is inside the
        # physically closed track.  Project locally onto the centerline and
        # compare the signed lateral coordinate with the two real boundary
        # polylines at that same progress instead.
        projection = self.project(x, y, previous_s=previous_s)
        center = self.at(projection.s)
        left, right = self._boundary_at(projection.s)
        tx, ty = math.cos(center.yaw), math.sin(center.yaw)
        left_width = -(ty * (left[0] - center.x)) + tx * (left[1] - center.y)
        right_width = -(ty * (right[0] - center.x)) + tx * (right[1] - center.y)
        left_width = max(0.0, left_width)
        right_width = min(0.0, right_width)
        if right_width <= projection.lateral_error <= left_width:
            return min(left_width - projection.lateral_error,
                       projection.lateral_error - right_width)
        if projection.lateral_error > left_width:
            return -(projection.lateral_error - left_width)
        return -(right_width - projection.lateral_error)

    def _boundary_at(self, s: float) -> tuple[tuple[float, float], tuple[float, float]]:
        """Interpolate left/right boundary points over the closed CSV polyline."""
        s = s % self.length
        rows = self.boundaries or []
        if len(rows) < 2:
            return ((0.0, 0.0), (0.0, 0.0))
        if not hasattr(self, "_boundary_s"):
            self._boundary_s = [p.s for p in rows]
        i = bisect.bisect_right(self._boundary_s, s) - 1
        i = max(0, min(i, len(rows) - 1))
        j = (i + 1) % len(rows)
        a, b = rows[i], rows[j]
        b_s = self.length if j == 0 else b.s
        span = b_s - a.s
        t = (s - a.s) / span if span > 1e-12 else 0.0
        return ((a.left_x + t * (b.left_x - a.left_x),
                 a.left_y + t * (b.left_y - a.left_y)),
                (a.right_x + t * (b.right_x - a.right_x),
                 a.right_y + t * (b.right_y - a.right_y)))

    def body_clearance(self, x: float, y: float, yaw: float,
                       previous_s: float | None = None) -> float:
        """Minimum clearance for corners and edge samples of the footprint."""
        if not self._boundary_left:
            return float("inf")
        c, sn = math.cos(yaw), math.sin(yaw)
        samples: list[tuple[float, float]] = []
        for i in range(9):
            along = -0.7 + 5.1 * i / 8.0
            for side in (-1.05, 1.05):
                samples.append((x + c * along - sn * side,
                                y + sn * along + c * side))
        return min(self.boundary_clearance(px, py, previous_s=previous_s)
                   for px, py in samples)


class LapController:
    """Pure-pursuit feedback with progress and boundary safeguards."""

    def __init__(self, route: CotaRoute, start_s: float | None = None):
        self.route = route
        self.start_s = start_s
        self.s: float | None = None
        self.progress = 0.0
        self._last_s: float | None = None
        self._steering = 0.0
        self.done = False
        self.failure: str | None = None
        self.last_lookahead = 3.0

    def _ds(self, new_s: float, old_s: float) -> float:
        d = new_s - old_s
        if d > 0.5 * self.route.length:
            d -= self.route.length
        elif d < -0.5 * self.route.length:
            d += self.route.length
        return d

    def speed_limit(self, s: float) -> float:
        kappa = abs(self.route.at(s).curvature)
        if kappa < 1e-5:
            return MAX_SPEED
        return max(2.0, min(MAX_SPEED, math.sqrt(MAX_LATERAL_ACCEL / kappa)))

    def target_speed(self, s: float, progress: float, current_speed: float = 0.0) -> float:
        """Curvature speed with braking preview over stopping distance."""
        local = self.speed_limit(s)
        stopping = max(15.0, current_speed * current_speed / (2.0 * MAX_BRAKE) + 15.0)
        preview = min(self.route.length, stopping)
        result = local
        d = 0.0
        while d <= preview + 1e-9:
            ahead = progress + d
            if ahead >= self.route.length:
                limit, distance = 0.0, self.route.length - progress
            else:
                limit, distance = self.speed_limit(s + d), d
            result = min(result, math.sqrt(max(0.0, limit * limit + 2.0 * MAX_BRAKE * distance)))
            d += 1.0
        return max(0.0, min(MAX_SPEED, result))

    def _steering_with_rate_limit(self, requested: float, dt: float) -> float:
        """Apply only the configured rate limit.

        The native vehicle plugin already models steering actuator delay, so
        this controller must not add a second delay in the command stream.
        """
        requested = max(-STEERING_LIMIT, min(STEERING_LIMIT, requested))
        max_delta = STEERING_RATE * max(0.0, dt)
        self._steering += max(-max_delta, min(max_delta, requested - self._steering))
        return self._steering

    def step(self, mapx: float, mapy: float, yaw: float, vx: float, dt: float) -> ControllerStep:
        if self.done or self.failure:
            return ControllerStep(0.0, 0.0, -MAX_BRAKE, self.progress,
                                  self.route.body_clearance(mapx, mapy, yaw, self.s),
                                  self.done, self.failure)
        projection = self.route.project(mapx, mapy, self.s, yaw=yaw)
        if self.s is None:
            self.s = projection.s
            self.start_s = projection.s if self.start_s is None else self.start_s % self.route.length
            self._last_s = projection.s
        else:
            assert self._last_s is not None
            delta = self._ds(projection.s, self._last_s)
            if delta < -5.0:
                self.failure = "reversed progress"
            else:
                self.progress = max(0.0, self.progress + delta)
            self.s = projection.s
            self._last_s = projection.s
        clearance = self.route.body_clearance(mapx, mapy, yaw, self.s)
        if self.failure is None and abs(projection.lateral_error) > 7.0:
            self.failure = "off route"
        if self.failure is None and clearance < 0.5:
            self.failure = "insufficient boundary clearance"
        if self.progress >= self.route.length:
            self.done = True
        remaining = max(0.0, self.route.length - self.progress)
        lookahead = max(3.0, 0.6 * abs(vx) + 3.0)
        if abs(self.route.at(self.s).curvature) > 0.08:
            lookahead = min(lookahead, 3.5)
        self.last_lookahead = lookahead
        target = self.route.target(self.s, lookahead)
        distance = max(0.5, math.hypot(target.x - mapx, target.y - mapy))
        alpha = _wrap(math.atan2(target.y - mapy, target.x - mapx) - yaw)
        requested_steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), distance)
        steer = self._steering_with_rate_limit(requested_steer, dt)
        target_speed = self.target_speed(self.s, self.progress, abs(vx))
        if clearance < 0.5 or abs(projection.lateral_error) > 6.0:
            target_speed = min(target_speed, 2.0)
        if remaining < 1.0:
            target_speed = 0.0
        drag = (0.8269 / 788.0) * vx * abs(vx)
        accel = max(-MAX_BRAKE, min(1.0, 0.8 * (target_speed - vx) + drag))
        if self.failure or self.done:
            accel = -MAX_BRAKE
            target_speed = 0.0
        return ControllerStep(steer, target_speed, accel, self.progress, clearance,
                              self.done, self.failure)


__all__ = ["CotaRoute", "RoutePoint", "Projection", "ControllerStep", "LapController",
           "STEERING_LIMIT", "_wrap"]
