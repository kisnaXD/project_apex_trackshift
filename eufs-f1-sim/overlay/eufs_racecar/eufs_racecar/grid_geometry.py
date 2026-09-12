"""Pure map geometry adapter used by the bounded multi-car grid.

Maps provide the standard EUFS asset paths plus optional ordered centerline,
boundary CSVs, and a visual boundary-strip mesh.  The grid generator remains
usable on maps without geometry metadata, where callers can report that
curved-track validation is unavailable.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from os.path import isfile


def _rows(path, fields):
    if not path or not isfile(path):
        return []
    with open(path, newline='', encoding='utf-8') as stream:
        return [tuple(float(row[field]) for field in fields)
                for row in csv.DictReader(stream)]


GRID_ROW_SPACING_M = 12.0
GRID_COLUMN_STAGGER_M = 6.0
GRID_LATERAL_M = 2.5


@dataclass(frozen=True)
class GridMapAdapter:
    """Map-owned geometry and spawn metadata for grid placement."""

    name: str
    centerline: tuple = ()
    boundaries: str = ''
    boundary_strips: str = ''
    profile: dict = None
    spawn_arclength_back_m: float | None = None

    @classmethod
    def from_assets(cls, assets: dict):
        return cls(
            name=str(assets.get('name', 'track')),
            centerline=tuple(_rows(
                assets.get('centerline', ''), ('s_m', 'x_m', 'y_m', 'yaw_rad'),
            )),
            boundaries=str(assets.get('boundaries', '') or ''),
            boundary_strips=str(assets.get('boundary_strips', '') or ''),
            profile=assets.get('profile_data') or {},
            spawn_arclength_back_m=assets.get('spawn_arclength_back_m'),
        )

    @property
    def has_boundary_validation(self):
        """Whether ordered left/right boundary CSVs are attached."""
        return bool(self.boundaries and isfile(self.boundaries))

    @property
    def has_boundary_strips(self):
        """Whether the map supplies an optional static visual strip mesh."""
        return bool(self.boundary_strips and isfile(self.boundary_strips))

    def cycle_length(self):
        if len(self.centerline) < 2:
            return 0.0
        last, first = self.centerline[-1], self.centerline[0]
        return last[0] + math.hypot(last[1] - first[1], last[2] - first[2])

    def nominal_spawn_s(self):
        if self.spawn_arclength_back_m is None or not self.centerline:
            return None
        return self.cycle_length() - float(self.spawn_arclength_back_m)

    def spawn_arclength(self, x, y):
        """Return authored spawn arclength or nearest raw-map projection."""
        if not self.centerline:
            return None
        nominal = self.nominal_spawn_s()
        if nominal is not None:
            px, py, _ = self.sample(nominal)
            if math.hypot(float(x) - px, float(y) - py) <= 1.0:
                return nominal
        cycle = self.cycle_length()
        best = (float('inf'), 0.0)
        segments = list(zip(self.centerline, self.centerline[1:]))
        segments.append((self.centerline[-1],
                         (cycle, self.centerline[0][1], self.centerline[0][2], self.centerline[0][3])))
        for first, second in segments:
            dx, dy = second[1] - first[1], second[2] - first[2]
            scale = max(1.0e-9, dx * dx + dy * dy)
            fraction = max(0.0, min(1.0,
                ((float(x) - first[1]) * dx + (float(y) - first[2]) * dy) / scale))
            px, py = first[1] + fraction * dx, first[2] + fraction * dy
            distance = math.hypot(float(x) - px, float(y) - py)
            if distance < best[0]:
                best = (distance, first[0] + fraction * (second[0] - first[0]))
        return best[1]

    def sample(self, s):
        """Interpolate ``(x, y, yaw)`` on the ordered periodic centerline."""
        cycle = self.cycle_length()
        if cycle <= 0.0:
            raise ValueError('centerline has no usable arclength')
        s %= cycle
        segments = list(zip(self.centerline, self.centerline[1:]))
        segments.append((self.centerline[-1],
                         (cycle, self.centerline[0][1], self.centerline[0][2], self.centerline[0][3])))
        for first, second in segments:
            if first[0] <= s <= second[0]:
                fraction = (s - first[0]) / max(1.0e-9, second[0] - first[0])
                yaw_delta = math.atan2(math.sin(second[3] - first[3]),
                                       math.cos(second[3] - first[3]))
                return (first[1] + fraction * (second[1] - first[1]),
                        first[2] + fraction * (second[2] - first[2]),
                        first[3] + fraction * yaw_delta)
        raise RuntimeError('centerline interpolation failed')


def build_grid_poses(assets: dict, count: int, spawn=None):
    """Build the bounded two-column grid from resolved map assets.

    ``spawn`` may be an ``(x, y, yaw)`` override.  Ordered centerline
    metadata makes the placement follow corners; maps without it retain the
    historical spawn-relative tangent grid.  The function deliberately does
    not infer cone order or synthesize missing map boundaries.
    """
    if not 1 <= int(count) <= 20:
        raise ValueError('grid count must be between 1 and 20')
    count = int(count)
    x, y, yaw = spawn or (assets.get('x'), assets.get('y'), assets.get('yaw'))
    if x is None or y is None or yaw is None:
        raise ValueError('grid assets need x, y, and yaw spawn values')
    x, y, yaw = float(x), float(y), float(yaw)
    adapter = assets if isinstance(assets, GridMapAdapter) else GridMapAdapter.from_assets(assets)
    if count == 1:
        return [(x, y, yaw)]
    poses = []
    for index in range(count):
        column = index % 2
        row = index // 2
        side = -1.0 if column == 0 else 1.0
        longitudinal = GRID_ROW_SPACING_M * row + GRID_COLUMN_STAGGER_M * column
        if adapter.centerline:
            nominal = adapter.nominal_spawn_s()
            if nominal is None:
                # No authored gate offset: project the resolved CSV spawn on
                # the map-owned centreline instead of guessing a distance.
                spawn_s = adapter.spawn_arclength(x, y)
            else:
                nominal_x, nominal_y, nominal_yaw = adapter.sample(nominal)
                if (abs(x - nominal_x) <= 1.0 and abs(y - nominal_y) <= 1.0 and
                        abs(math.atan2(math.sin(yaw - nominal_yaw),
                                       math.cos(yaw - nominal_yaw))) <= 0.1):
                    spawn_s = nominal
                else:
                    spawn_s = adapter.spawn_arclength(x, y)
            cx, cy, tangent = adapter.sample(spawn_s - longitudinal)
            poses.append((cx + side * GRID_LATERAL_M * -math.sin(tangent),
                          cy + side * GRID_LATERAL_M * math.cos(tangent), tangent))
        else:
            forward = (math.cos(yaw), math.sin(yaw))
            left = (-math.sin(yaw), math.cos(yaw))
            poses.append((x - longitudinal * forward[0] + side * GRID_LATERAL_M * left[0],
                          y - longitudinal * forward[1] + side * GRID_LATERAL_M * left[1], yaw))
    return poses
