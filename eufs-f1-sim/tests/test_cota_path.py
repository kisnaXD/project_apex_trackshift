from __future__ import annotations

import math
from pathlib import Path

import pytest

from scripts.cota_path import CotaRoute, LapController


ROOT = Path(__file__).resolve().parents[1]
TRACK = ROOT / "overlay/eufs_tracks/cota"


def test_real_track_contains_closing_segment_and_wrap_projection():
    route = CotaRoute(TRACK / "centerline.csv", TRACK / "boundaries.csv", spacing=1.7)
    assert route.length == pytest.approx(5513.0, abs=1e-3)
    assert route.closure_length == pytest.approx(28.709184, abs=1e-4)
    end = route.at(route.length - 1e-4)
    first = route.at(0.0)
    assert math.hypot(end.x - first.x, end.y - first.y) < 0.01
    wrapped = route.project(first.x, first.y, previous_s=route.length - 0.4)
    assert wrapped.s == pytest.approx(0.0, abs=0.5)


def test_projection_is_segment_exact_and_signed():
    route = CotaRoute(TRACK / "centerline.csv", TRACK / "boundaries.csv")
    p = route.at(100.0)
    nx, ny = -math.sin(p.yaw), math.cos(p.yaw)
    q = route.project(p.x + 2.0 * nx, p.y + 2.0 * ny, previous_s=100.0)
    assert q.s == pytest.approx(100.0, abs=0.1)
    assert q.lateral_error == pytest.approx(2.0, abs=0.01)
    assert q.distance == pytest.approx(2.0, abs=0.01)


def test_real_boundaries_report_margin_for_body_samples():
    route = CotaRoute(TRACK / "centerline.csv", TRACK / "boundaries.csv")
    p = route.at(100.0)
    assert route.boundary_clearance(p.x, p.y) > 6.0
    assert route.body_clearance(p.x, p.y, p.yaw) > 5.0
    assert route.boundary_clearance(p.x + 20.0 * (-math.sin(p.yaw)),
                                    p.y + 20.0 * math.cos(p.yaw)) < 0.0


def test_braking_preview_reduces_speed_before_finish():
    route = CotaRoute(TRACK / "centerline.csv", TRACK / "boundaries.csv")
    controller = LapController(route)
    cruise = controller.target_speed(route.at(5000).s, 5000.0, 15.0)
    finish_preview = controller.target_speed(route.at(5500).s, 5500.0, 15.0)
    assert cruise == pytest.approx(15.0)
    assert finish_preview < cruise
    assert finish_preview > 0.0


def test_controller_pure_pursuit_sign_and_command_limits():
    route = CotaRoute(TRACK / "centerline.csv", TRACK / "boundaries.csv")
    p = route.at(100.0)
    # Put the body right of the route while pointing along the route: target is
    # to the left, so positive steering is required.
    nx, ny = -math.sin(p.yaw), math.cos(p.yaw)
    controller = LapController(route)
    out = controller.step(p.x - nx, p.y - ny, p.yaw, 5.0, 1.0)
    assert out.steer > 0.0
    assert abs(out.steer) <= 0.6458
    assert -1.2 <= out.accel <= 1.0
    assert out.failure is None
