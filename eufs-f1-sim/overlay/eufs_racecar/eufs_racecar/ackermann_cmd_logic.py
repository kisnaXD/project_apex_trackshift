"""Pure speed target/ramp logic for the physical Ackermann alternate."""

from __future__ import annotations


class SpeedController:
    """Translate Ackermann speed/acceleration fields into a bounded Twist vx."""

    def __init__(
        self,
        max_speed_mps: float = 80.0,
        accel_only_speed_mps: float = 2.0,
        command_timeout_s: float = 0.5,
        stale_decel_mps2: float = 2.0,
    ):
        self.max_speed = abs(float(max_speed_mps))
        self.accel_only_speed = min(abs(float(accel_only_speed_mps)), self.max_speed)
        self.command_timeout = max(0.0, float(command_timeout_s))
        self.stale_decel = abs(float(stale_decel_mps2))
        self.speed = 0.0
        self.target = 0.0
        self.acceleration = 0.0
        self.last_command_time = None
        self.last_update_time = None

    def reset(self, now: float):
        """Reset timer history without integrating across a clock reset."""
        self.last_update_time = float(now)

    def accept(self, speed: float, acceleration: float, now: float):
        speed = float(speed)
        acceleration = float(acceleration)
        self.acceleration = acceleration
        self.last_command_time = float(now)
        self.last_update_time = float(now) if self.last_update_time is None else self.last_update_time
        if abs(speed) > 1e-3:
            self.target = max(-self.max_speed, min(self.max_speed, speed))
        elif acceleration > 1e-6:
            # Acceleration-only commands get a bounded smoke-test target;
            # zero speed plus negative acceleration is a brake-to-zero command.
            self.target = self.accel_only_speed
        else:
            self.target = 0.0

    def update(self, now: float):
        now = float(now)
        if self.last_update_time is None:
            self.last_update_time = now
            return None
        dt = now - self.last_update_time
        self.last_update_time = now
        if dt <= 0.0:
            return self.speed

        target = self.target
        acceleration = self.acceleration
        if (
            self.last_command_time is not None
            and now - self.last_command_time > self.command_timeout
        ):
            target = 0.0
            acceleration = -self.stale_decel

        delta = target - self.speed
        if abs(delta) <= 1e-9:
            self.speed = target
        elif abs(acceleration) > 1e-6:
            step = abs(acceleration) * dt
            self.speed += max(-step, min(step, delta))
        else:
            self.speed = target
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed))
        return self.speed
