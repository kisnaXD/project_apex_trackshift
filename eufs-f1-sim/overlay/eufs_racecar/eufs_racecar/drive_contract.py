"""Shared bounded-drive contract used by the dashboard and CLI smoke test."""

from __future__ import annotations

import time

MAX_SPEED_MPS = 2.0
MAX_ACCEL_MPS2 = 0.5
RATE_HZ = 20.0
DURATION_S = 10.0
WALL_TIMEOUT_S = 60.0
BRAKE_S = 0.5
CLOCK_EPSILON_S = 1e-6
ODOM_STALE_S = 1.0


def clamp_command(speed: float, acceleration: float) -> tuple[float, float]:
    return (
        max(-MAX_SPEED_MPS, min(MAX_SPEED_MPS, float(speed))),
        max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, float(acceleration))),
    )


def braking_acceleration(speed: float) -> float:
    """Return bounded acceleration opposing signed speed, or zero at rest."""
    if abs(float(speed)) < 0.05:
        return 0.0
    return -MAX_ACCEL_MPS2 if speed > 0.0 else MAX_ACCEL_MPS2


def forward_feedback_acceleration(measured_speed: float | None,
                                  target_speed: float = MAX_SPEED_MPS) -> float:
    speed = float(measured_speed or 0.0)
    error = float(target_speed) - speed
    if error <= 0.0:
        return braking_acceleration(speed) if error < -0.05 else 0.0
    return min(MAX_ACCEL_MPS2, error * 1.5)


def telemetry_fresh(last_wall: float | None, now_wall: float, max_age=ODOM_STALE_S) -> bool:
    return last_wall is not None and now_wall - last_wall <= max_age


class SimTimedDrive:
    """Wall-safe timer whose completion is measured in simulation seconds."""

    def __init__(self, duration=DURATION_S, wall_timeout=WALL_TIMEOUT_S):
        self.duration = float(duration)
        self.wall_timeout = float(wall_timeout)
        self.baseline_sim = None
        self.started_sim = None
        self.started_wall = None
        self.cancelled = False

    def arm(self, sim_time):
        self.baseline_sim = sim_time
        self.started_sim = None
        self.started_wall = time.monotonic()
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    def state(self, sim_time, now_wall=None):
        now_wall = time.monotonic() if now_wall is None else now_wall
        if self.cancelled:
            return 'cancelled'
        if self.started_wall is not None and now_wall - self.started_wall >= self.wall_timeout:
            return 'timeout'
        if (
            sim_time is None
            or self.baseline_sim is None
            or sim_time <= self.baseline_sim + CLOCK_EPSILON_S
        ):
            return 'waiting'
        if self.started_sim is None:
            self.started_sim = sim_time
        if sim_time - self.started_sim >= self.duration:
            return 'done'
        return 'active'
