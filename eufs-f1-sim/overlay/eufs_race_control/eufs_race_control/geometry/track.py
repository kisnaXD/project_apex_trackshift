"""Continuous COTA map/Frenet geometry and conservative footprint checks."""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _wrap(value: float, length: float) -> float:
    return value % length


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True, slots=True)
class CenterlinePoint:
    s_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    curvature_1pm: float = 0.0


@dataclass(frozen=True, slots=True)
class Projection:
    progress_m: float
    unwrapped_progress_m: float
    lateral_m: float
    distance_m: float
    segment_index: int
    heading_error_rad: Optional[float] = None
    valid: bool = True
    reason: Optional[str] = None

    @property
    def s_m(self) -> float:
        return self.progress_m

    @property
    def y_m(self) -> float:
        return self.lateral_m


@dataclass(frozen=True, slots=True)
class FrenetPose:
    x_m: float
    y_m: float
    yaw_rad: float
    progress_m: float
    lateral_m: float


class TrackGeometry:
    """Piecewise-linear map with continuous projection around a closed seam.

    ``previous_progress_m`` is unwrapped progress. Candidate projections are
    scored inside a reachable progress window and, when provided, by heading.
    This prevents a nearest Euclidean point on a neighboring parallel section
    from stealing the track association.
    """

    def __init__(self, centerline: str | Path | Sequence[CenterlinePoint], boundaries: str | Path | None = None, map_id: str = "cota"):
        self.map_id = map_id
        if isinstance(centerline, (str, Path)):
            self.points = self._read_centerline(Path(centerline))
        else:
            self.points = tuple(centerline)
        if len(self.points) < 3:
            raise ValueError("centerline needs at least three points")
        self._s0 = self.points[0].s_m
        self.points = tuple(CenterlinePoint(p.s_m - self._s0, p.x_m, p.y_m, p.yaw_rad, p.curvature_1pm) for p in self.points)
        self._segments = tuple(self._build_segments())
        self.length_m = sum(segment[3] for segment in self._segments)
        if self.length_m <= 0.0:
            raise ValueError("centerline length must be positive")
        self.boundary_widths = self._read_boundaries(Path(boundaries)) if boundaries else ()

    @staticmethod
    def _read_centerline(path: Path) -> Tuple[CenterlinePoint, ...]:
        rows = []
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                try:
                    x = _finite(row.get("x_m", row.get("x", "nan")), "x")
                    y = _finite(row.get("y_m", row.get("y", "nan")), "y")
                    s = _finite(row.get("s_m", row.get("s", "nan")), "s")
                    yaw = _finite(row.get("yaw_rad", "0"), "yaw")
                    curvature = _finite(row.get("curvature_1pm", "0"), "curvature")
                except (TypeError, ValueError):
                    continue
                rows.append(CenterlinePoint(s, x, y, yaw, curvature))
        return tuple(rows)

    @staticmethod
    def _read_boundaries(path: Path) -> Tuple[Tuple[float, float], ...]:
        result = []
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                try:
                    s = _finite(row.get("s_m", "nan"), "boundary s")
                    lx = _finite(row.get("left_x_m", "nan"), "left x")
                    ly = _finite(row.get("left_y_m", "nan"), "left y")
                    rx = _finite(row.get("right_x_m", "nan"), "right x")
                    ry = _finite(row.get("right_y_m", "nan"), "right y")
                except (TypeError, ValueError):
                    continue
                result.append((s, math.hypot(lx - rx, ly - ry) * 0.5))
        return tuple(result)

    def _build_segments(self):
        start_s = 0.0
        for index, a in enumerate(self.points):
            b = self.points[(index + 1) % len(self.points)]
            dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            # Progress is geometric arclength.  Authored CSV ``s_m`` is kept
            # as source metadata but cannot be mixed with Euclidean spans.
            yield (a, b, start_s, length, math.atan2(dy, dx))
            start_s += length

    def _segment_at(self, wrapped_s: float):
        s = _wrap(wrapped_s, self.length_m)
        accumulated = 0.0
        for segment in self._segments:
            if s <= segment[2] + segment[3] + 1e-9:
                return segment, (s - segment[2]) / max(segment[3], 1e-12)
        return self._segments[-1], 1.0

    def interpolate(self, progress_m: float) -> FrenetPose:
        progress_m = _finite(progress_m, "progress_m")
        segment, t = self._segment_at(progress_m)
        a, b, _, _, tangent = segment
        x = a.x_m + t * (b.x_m - a.x_m)
        y = a.y_m + t * (b.y_m - a.y_m)
        lateral = 0.0
        yaw = tangent
        return FrenetPose(x, y, yaw, _wrap(progress_m, self.length_m), lateral)

    def frenet_to_map(self, progress_m: float, lateral_m: float = 0.0, yaw_offset_rad: float = 0.0) -> FrenetPose:
        base = self.interpolate(progress_m)
        lateral_m = _finite(lateral_m, "lateral_m")
        x = base.x_m - math.sin(base.yaw_rad) * lateral_m
        y = base.y_m + math.cos(base.yaw_rad) * lateral_m
        return FrenetPose(x, y, _angle_delta(base.yaw_rad + yaw_offset_rad, 0.0), base.progress_m, lateral_m)

    def _candidate(self, x: float, y: float, index: int, segment, heading: Optional[float], previous: Optional[float]):
        a, b, start_s, length, tangent = segment
        dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
        t = ((x - a.x_m) * dx + (y - a.y_m) * dy) / (length * length)
        t = max(0.0, min(1.0, t))
        qx, qy = a.x_m + t * dx, a.y_m + t * dy
        distance = math.hypot(x - qx, y - qy)
        wrapped = _wrap(start_s + t * length, self.length_m)
        if previous is None:
            unwrapped = wrapped
        else:
            unwrapped = wrapped + round((previous - wrapped) / self.length_m) * self.length_m
        lateral = (-dy * (x - qx) + dx * (y - qy)) / length
        error = None if heading is None else abs(_angle_delta(heading, tangent))
        return Projection(wrapped, unwrapped, lateral, distance, index, error)

    def project(self, x_m: float, y_m: float, previous_progress_m: Optional[float] = None, previous_heading_rad: Optional[float] = None, reachable_window_m: float = 80.0, max_heading_error_rad: float = math.pi, max_distance_m: Optional[float] = 8.0) -> Projection:
        x_m, y_m = _finite(x_m, "x_m"), _finite(y_m, "y_m")
        max_heading_error_rad = _finite(max_heading_error_rad, "max_heading_error_rad")
        if max_heading_error_rad < 0.0:
            raise ValueError("max_heading_error_rad must be non-negative")
        if max_distance_m is not None:
            max_distance_m = _finite(max_distance_m, "max_distance_m")
            if max_distance_m < 0.0:
                raise ValueError("max_distance_m must be non-negative")
        if previous_heading_rad is not None:
            previous_heading_rad = _finite(previous_heading_rad, "previous_heading_rad")
        if previous_progress_m is not None:
            previous_progress_m = _finite(previous_progress_m, "previous_progress_m")
            reachable_window_m = _finite(reachable_window_m, "reachable_window_m")
            if reachable_window_m < 0.0:
                raise ValueError("reachable_window_m must be non-negative")
        segment_items = list(enumerate(self._segments))
        if previous_progress_m is not None and reachable_window_m < self.length_m * 0.5:
            center = _wrap(previous_progress_m, self.length_m)
            segment_items = [(i, segment) for i, segment in segment_items if abs(((segment[2] - center + 0.5 * self.length_m) % self.length_m) - 0.5 * self.length_m) <= reachable_window_m + segment[3]]
        candidates = [self._candidate(x_m, y_m, i, segment, previous_heading_rad, previous_progress_m) for i, segment in segment_items]
        if previous_progress_m is not None:
            candidates = [candidate for candidate in candidates if abs(candidate.unwrapped_progress_m - previous_progress_m) <= reachable_window_m + 1e-9]
            if not candidates:
                raise ValueError("no reachable track association")
        if previous_heading_rad is not None:
            candidates = [candidate for candidate in candidates if candidate.heading_error_rad is not None and candidate.heading_error_rad <= max_heading_error_rad]
            if not candidates:
                raise ValueError("no heading-consistent track association")
        result = min(candidates, key=lambda candidate: (candidate.distance_m, candidate.heading_error_rad or 0.0))
        if max_distance_m is not None and result.distance_m > max_distance_m:
            raise ValueError("track projection residual exceeds limit")
        return result

    def project_continuous(self, *args, **kwargs) -> Projection:
        return self.project(*args, **kwargs)

    def width_at(self, progress_m: float, default_half_width_m: Optional[float] = None) -> float:
        if not self.boundary_widths:
            if default_half_width_m is None:
                raise ValueError("track boundaries are unavailable; corridor width is uncertified")
            if default_half_width_m <= 0.0 or not math.isfinite(default_half_width_m):
                raise ValueError("default_half_width_m must be positive and finite")
            return default_half_width_m
        wrapped = _wrap(progress_m, self.length_m)
        return min(self.boundary_widths, key=lambda item: abs(item[0] - wrapped))[1]


@dataclass(frozen=True, slots=True)
class Footprint:
    front_m: float = 4.4
    rear_m: float = 0.7
    halfwidth_m: float = 1.05

    def __post_init__(self):
        for name in ("front_m", "rear_m", "halfwidth_m"):
            value = _finite(getattr(self, name), name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")

    def corners(self, x_m: float, y_m: float, yaw_rad: float):
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        result = []
        for longitudinal, lateral in ((self.front_m, self.halfwidth_m), (self.front_m, -self.halfwidth_m), (-self.rear_m, -self.halfwidth_m), (-self.rear_m, self.halfwidth_m)):
            result.append((x_m + c * longitudinal - s * lateral, y_m + s * longitudinal + c * lateral))
        return tuple(result)


def _axes(poly):
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length > 1e-12:
            yield (-dy / length, dx / length)


def polygons_overlap(first, second, clearance_m: float = 0.0) -> bool:
    """SAT overlap for convex quadrilaterals, with optional clearance."""
    clearance_m = _finite(clearance_m, "clearance_m")
    if clearance_m < 0.0:
        raise ValueError("clearance_m must be non-negative")
    for axis in tuple(_axes(first)) + tuple(_axes(second)):
        amin = min(p[0] * axis[0] + p[1] * axis[1] for p in first)
        amax = max(p[0] * axis[0] + p[1] * axis[1] for p in first)
        bmin = min(p[0] * axis[0] + p[1] * axis[1] for p in second)
        bmax = max(p[0] * axis[0] + p[1] * axis[1] for p in second)
        if amax + clearance_m < bmin or bmax + clearance_m < amin:
            return False
    return True


def swept_footprint_collision(first_pose, second_pose, footprint: Footprint = Footprint(), samples: int = 8, clearance_m: float = 0.0, other_start=None, other_end=None) -> bool:
    """Conservatively detect overlap over a pose interval by interpolation.

    ``first_pose``/``second_pose`` describe the first car at the start/end of
    the interval.  For two moving cars pass ``other_start`` and ``other_end``;
    when omitted, the second car is stationary at ``second_pose`` only for the
    convenient static-overlap use case.
    """
    if samples < 1:
        raise ValueError("samples must be positive")
    first_pose = tuple(float(value) for value in first_pose)
    second_pose = tuple(float(value) for value in second_pose)
    if len(first_pose) != 3 or len(second_pose) != 3:
        raise ValueError("poses must be (x_m, y_m, yaw_rad)")
    if other_start is None:
        other_start = other_end = second_pose
    if other_end is None:
        other_end = other_start
    if len(other_start) != 3 or len(other_end) != 3:
        raise ValueError("other poses must be (x_m, y_m, yaw_rad)")
    poses = first_pose + second_pose + tuple(float(value) for value in other_start) + tuple(float(value) for value in other_end)
    if any(not math.isfinite(value) for value in poses):
        raise ValueError("swept poses must be finite")
    clearance_m = _finite(clearance_m, "clearance_m")
    if clearance_m < 0.0:
        raise ValueError("clearance_m must be non-negative")
    def angle_delta(a, b):
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi
    translation = max(math.hypot(second_pose[0] - first_pose[0], second_pose[1] - first_pose[1]), math.hypot(other_end[0] - other_start[0], other_end[1] - other_start[1]))
    rotation = max(abs(angle_delta(second_pose[2], first_pose[2])), abs(angle_delta(other_end[2], other_start[2])))
    radius = math.hypot(max(footprint.front_m, footprint.rear_m), footprint.halfwidth_m)
    # Adapt subdivision to translation and shortest-angle rotation.  The
    # additional bound below accounts for motion between samples, so this is
    # conservative even for long straight crossings and near-full turns.
    subdivisions = max(int(samples), int(math.ceil(translation / max(0.25, footprint.halfwidth_m))) + 1, int(math.ceil(rotation / (math.pi / 36.0))) + 1)
    for step in range(subdivisions + 1):
        t = step / subdivisions
        pose_a = (first_pose[0] + t * (second_pose[0] - first_pose[0]), first_pose[1] + t * (second_pose[1] - first_pose[1]), first_pose[2] + t * angle_delta(second_pose[2], first_pose[2]))
        a = footprint.corners(*pose_a)
        pose_b = (other_start[0] + t * (other_end[0] - other_start[0]), other_start[1] + t * (other_end[1] - other_start[1]), other_start[2] + t * angle_delta(other_end[2], other_start[2]))
        motion_bound = (math.hypot(second_pose[0] - first_pose[0], second_pose[1] - first_pose[1]) + math.hypot(other_end[0] - other_start[0], other_end[1] - other_start[1])) / max(1.0, subdivisions) + radius * rotation / max(1.0, subdivisions)
        if polygons_overlap(a, footprint.corners(*pose_b), clearance_m + motion_bound):
            return True
    return False


def footprints_collide(pose_a, pose_b, footprint: Footprint = Footprint(), clearance_m: float = 0.0) -> bool:
    return polygons_overlap(footprint.corners(*pose_a), footprint.corners(*pose_b), clearance_m)


def swept_collision(first_start, first_end, second_start, second_end, footprint: Footprint = Footprint(), samples: int = 8, clearance_m: float = 0.0) -> bool:
    return swept_footprint_collision(first_start, first_end, footprint, samples, clearance_m, second_start, second_end)


__all__ = ["CenterlinePoint", "Projection", "FrenetPose", "TrackGeometry", "Footprint", "polygons_overlap", "swept_footprint_collision", "swept_collision", "footprints_collide"]
