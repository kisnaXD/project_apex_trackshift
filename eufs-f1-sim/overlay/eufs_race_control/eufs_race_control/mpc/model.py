"""ROS-independent Layer 3 vehicle MPC model.

The model is a prediction/solver boundary rather than a command publisher.
Native DynamicBicycle conventions are retained: acceleration is a pre-drag
tyre-force request divided by mass, and the historical implementation uses
``l*w_front`` as the lever arm for both slip-angle branches.  That quirk is
documented and intentionally not recalibrated here.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import ctypes
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple

from ..models.hybrid import DriveRequest, HybridProfile, HybridState, step as hybrid_step


# The default runtime horizon is deliberately small enough for a 10--20 Hz
# controls loop.  Longer non-uniform schedules remain accepted for offline
# callers by supplying ``MPCConfig(horizon_dt=...)``.
HORIZON_INTERVALS: Tuple[float, ...] = (0.10,) * 20
STATE_NAMES = ("s_m", "n_m", "heading_error_rad", "vx_mps", "vy_mps", "yaw_rate_rps", "steering_rad", "energy_j", "pack_temperature_k")
# Layer 3 optimises the actuator rate.  The command boundary still carries a
# native steering angle target (see ``MPCResult.first_command``); this keeps the
# 0.2 s atomic delay shared by steering, ICE and MGU-K allocation while avoiding
# the non-smooth target/tau state equation inside the OCP.
CONTROL_NAMES = ("steering_rate_rps", "predrag_acceleration_mps2", "signed_mguk_force_n")


def _finite(value: Any, name: str) -> float:
    try: value = float(value)
    except (TypeError, ValueError) as exc: raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(value): raise ValueError(f"{name} must be finite")
    return value


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def footprint_lateral_bounds(heading_error_rad: float, half_width_m: float, front_m: float, rear_m: float) -> Tuple[float, float]:
    """Lateral extrema of the four vehicle footprint corners.

    Front and rear lengths are intentionally kept asymmetric.  A heading
    error therefore produces different left/right clearances, which a scalar
    ``abs(n) <= width`` check cannot represent.
    """
    c, s = math.cos(heading_error_rad), math.sin(heading_error_rad)
    values = [s * x + c * y for x in (front_m, -rear_m) for y in (-half_width_m, half_width_m)]
    return min(values), max(values)


@dataclass(frozen=True, slots=True)
class VehicleState9:
    s_m: float = 0.0
    n_m: float = 0.0
    heading_error_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    steering_rad: float = 0.0
    energy_j: float = 0.0
    pack_temperature_k: float = 298.15

    def as_tuple(self) -> Tuple[float, ...]:
        return tuple(float(getattr(self, name)) for name in STATE_NAMES)

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "VehicleState9":
        if len(values) != len(STATE_NAMES): raise ValueError("MPC state requires nine values")
        return cls(*(_finite(value, name) for value, name in zip(values, STATE_NAMES)))

    def __post_init__(self) -> None:
        for name in STATE_NAMES: _finite(getattr(self, name), name)


@dataclass(frozen=True, slots=True)
class ReferencePoint:
    s_m: float
    n_m: float = 0.0
    heading_error_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    curvature_inv_m: float = 0.0
    steering_rad: float = 0.0
    energy_j: Optional[float] = None
    pack_temperature_k: Optional[float] = None
    minimum_exit_energy_j: Optional[float] = None
    energy_price_s_per_mj: float = 0.0
    corridor_left_m: float = -7.0
    corridor_right_m: float = 7.0
    footprint_half_width_m: float = 1.05
    footprint_front_m: float = 4.40
    footprint_rear_m: float = 0.70
    source_time_s: Optional[float] = None

    def __post_init__(self) -> None:
        for name in ("s_m", "n_m", "heading_error_rad", "vx_mps", "vy_mps", "yaw_rate_rps", "curvature_inv_m", "steering_rad", "corridor_left_m", "corridor_right_m", "footprint_half_width_m", "energy_price_s_per_mj"):
            _finite(getattr(self, name), name)
        for name in ("energy_j", "pack_temperature_k", "minimum_exit_energy_j", "source_time_s"):
            if getattr(self, name) is not None: _finite(getattr(self, name), name)
        if self.corridor_left_m >= self.corridor_right_m: raise ValueError("corridor bounds must be ordered")

    @property
    def left_n_m(self) -> float: return self.corridor_left_m + self.footprint_half_width_m

    @property
    def right_n_m(self) -> float: return self.corridor_right_m - self.footprint_half_width_m


@dataclass(frozen=True, slots=True)
class Corridor:
    points: Tuple[ReferencePoint, ...]

    def __post_init__(self) -> None:
        if not self.points: raise ValueError("corridor/reference must not be empty")

    def at(self, index: int) -> ReferencePoint:
        return self.points[min(max(int(index), 0), len(self.points) - 1)]


@dataclass(frozen=True, slots=True)
class MPCConfig:
    horizon_dt: Tuple[float, ...] = HORIZON_INTERVALS
    mass_kg: float = 788.0
    inertia_z_kgm2: float = 1500.0
    wheelbase_m: float = 3.28
    w_front: float = 0.45
    axle_width_m: float = 1.50
    max_steering_rad: float = 0.6458
    max_steering_rate_rps: float = 1.2916
    steering_actuator_tau_s: float = 0.20
    command_delay_s: float = 0.20
    min_vx_mps: float = 0.0
    max_vx_mps: float = 80.0
    max_predrag_acceleration_mps2: float = 8.0
    min_predrag_acceleration_mps2: float = -12.0
    # Full collision footprint; axle half-width is used only by tyre slip.
    footprint_half_width_m: float = 1.05
    footprint_front_m: float = 4.40
    footprint_rear_m: float = 0.70
    boundary_margin_m: float = 0.10
    max_pack_temperature_k: float = 373.15
    deadline_s: float = 0.05
    integration_substep_s: float = 0.025
    terminal_weight: float = 5.0
    energy_price_default_s_per_mj: float = 0.0
    energy_state_scale_mj: float = 1.0e6
    validation_tolerance_m: float = 1.0e-7

    def __post_init__(self) -> None:
        if not 1 <= len(self.horizon_dt) <= 50 or any(dt <= 0.0 or not math.isfinite(dt) for dt in self.horizon_dt): raise ValueError("MPC requires 1..50 positive finite intervals")
        for name in ("mass_kg", "inertia_z_kgm2", "wheelbase_m", "w_front", "axle_width_m", "max_steering_rad", "max_steering_rate_rps", "steering_actuator_tau_s", "command_delay_s", "max_vx_mps", "max_predrag_acceleration_mps2", "deadline_s", "integration_substep_s", "footprint_half_width_m", "footprint_front_m", "footprint_rear_m", "energy_state_scale_mj", "validation_tolerance_m"):
            if _finite(getattr(self, name), name) <= 0.0: raise ValueError(f"{name} must be positive")
        if not 0.0 < self.w_front < 1.0: raise ValueError("w_front must be in (0,1)")
        if self.max_steering_rad <= 0.0 or self.max_steering_rate_rps <= 0.0: raise ValueError("steering limits must be positive")
        if self.min_predrag_acceleration_mps2 >= self.max_predrag_acceleration_mps2: raise ValueError("acceleration limits must be ordered")


@dataclass(frozen=True, slots=True)
class MPCInput:
    run_id: str
    epoch_id: int
    state_snapshot_id: str
    stamp_s: float
    received_monotonic_s: float
    state: VehicleState9
    reference: Corridor
    steering_history: Tuple[float, ...] = ()
    steering_rate_history: Tuple[float, ...] = ()
    applied_acceleration_mps2: float = 0.0
    applied_mguk_force_n: float = 0.0
    expires_at_s: Optional[float] = None
    trajectory_id: str = ""
    minimum_exit_energy_j: Optional[float] = None
    energy_price_s_per_mj: Optional[float] = None
    # Commands already issued to the native plant.  Each entry is
    # ``(issue_stamp_s, steering_angle_target_rad, acceleration_mps2,
    # signed_mguk_force_n)`` and is consumed atomically after command_delay_s.
    pending_commands: Tuple[Tuple[float, float, float, float], ...] = ()

    def __post_init__(self) -> None:
        if isinstance(self.epoch_id, bool) or self.epoch_id < 0: raise ValueError("epoch_id must be non-negative")
        _finite(self.stamp_s, "stamp_s"); _finite(self.received_monotonic_s, "received_monotonic_s")
        if self.expires_at_s is not None: _finite(self.expires_at_s, "expires_at_s")
        _finite(self.applied_acceleration_mps2, "applied_acceleration_mps2"); _finite(self.applied_mguk_force_n, "applied_mguk_force_n")
        for command in self.pending_commands:
            if len(command) != 4: raise ValueError("pending command requires stamp and three controls")
            for value in command: _finite(value, "pending command")


@dataclass(frozen=True, slots=True)
class MPCResult:
    status: str
    fallback: bool
    run_id: str
    epoch_id: int
    state_snapshot_id: str
    trajectory_id: str
    generation_monotonic_s: float
    solve_time_s: float
    controls: Tuple[Tuple[float, float, float], ...] = ()
    predicted_states: Tuple[Tuple[float, ...], ...] = ()
    residuals: Mapping[str, float] = field(default_factory=dict)
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    rejection_reason: str = ""

    @property
    def first_command(self) -> Optional[Tuple[float, float, float]]:
        """Native command tuple (angle target, pre-drag acceleration, MGU-K force)."""
        if self.fallback or not self.controls: return None
        index = int(self.diagnostics.get("first_action_index", 0))
        index = min(max(index, 0), len(self.controls) - 1)
        angle, accel, mguk = self.controls[index]
        # ``controls`` are rates internally; predicted_states[0] is the
        # measured state and one 50 ms knot is the command publish interval.
        steering = self.predicted_states[0][6] if self.predicted_states else 0.0
        return (steering + angle * 0.05, accel, mguk)


class DelayHistory:
    """Epoch-aware command history for the native 0.2 s input delay."""

    def __init__(self, delay_s: float = 0.20) -> None:
        self.delay_s = _finite(delay_s, "delay_s"); self._epoch: Optional[int] = None; self._entries: deque[Tuple[float, Tuple[float, float, float]]] = deque()

    def reset(self, epoch_id: int) -> None:
        self._epoch = int(epoch_id); self._entries.clear()

    def append(self, stamp_s: float, control: Sequence[float], epoch_id: int) -> None:
        if self._epoch != epoch_id: self.reset(epoch_id)
        values = tuple(_finite(value, "control") for value in control)
        if len(values) != 3: raise ValueError("control requires steering rate, acceleration and MGU-K force")
        stamp = _finite(stamp_s, "stamp_s")
        if self._entries and stamp < self._entries[-1][0]:
            raise ValueError("command history must be monotonic")
        self._entries.append((stamp, values))
        while len(self._entries) > 1000: self._entries.popleft()

    def extend(self, entries: Iterable[Tuple[float, Sequence[float]]], epoch_id: int) -> None:
        """Replace/append a runtime pending queue in issue-time order."""
        for stamp, control in sorted(entries, key=lambda item: float(item[0])):
            self.append(stamp, control, epoch_id)

    def clear(self) -> None:
        self._entries.clear()

    def delayed(self, target_stamp_s: float, default: Sequence[float] = (0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
        target = _finite(target_stamp_s, "target_stamp_s") - self.delay_s; selected = tuple(float(value) for value in default)
        for stamp, control in self._entries:
            if stamp <= target: selected = control
            else: break
        return selected

    def command_at(self, action_stamp_s: float, default: Sequence[float] = (0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
        """Return the command affecting a simulated action time.

        History timestamps are command issue times.  The native plant applies
        the latest issue that is at least ``delay_s`` before the action time.
        Keeping this conversion here prevents wall-clock deadlines from being
        mixed with the simulation clock used for prediction.
        """
        return self.delayed(action_stamp_s, default)

    @property
    def entries(self) -> Tuple[Tuple[float, Tuple[float, float, float]], ...]:
        return tuple(self._entries)


class NativeDynamicBicycle:
    """Differentiable-friendly Python mirror of the DynamicBicycle equations."""

    tire_B = 12.56; tire_C = -1.38; tire_D = 1.60; tire_E = -0.58
    downforce_coefficient = 1.50; drag_coefficient = 0.8269; gravity = 9.81; rolling_resistance_n = 120.0

    def __init__(self, config: MPCConfig = MPCConfig(), hybrid_profile: Optional[HybridProfile] = None) -> None:
        self.config = config; self.hybrid_profile = hybrid_profile or HybridProfile.from_vehicle_defaults()

    @property
    def front_lever_arm_m(self) -> float: return self.config.wheelbase_m * self.config.w_front

    @property
    def rear_lever_arm_m(self) -> float: return self.config.wheelbase_m * (1.0 - self.config.w_front)

    def slip_angles(self, state: VehicleState9) -> Tuple[float, float]:
        vx = max(1.0, state.vx_mps); lever = self.front_lever_arm_m; denominator = vx - 0.5 * self.config.axle_width_m * state.yaw_rate_rps
        rear = math.atan((state.vy_mps - lever * state.yaw_rate_rps) / denominator)
        front = math.atan((state.vy_mps + lever * state.yaw_rate_rps) / denominator) - state.steering_rad
        return front, rear

    def lateral_forces(self, state: VehicleState9) -> Tuple[float, float]:
        front_alpha, rear_alpha = self.slip_angles(state); normal = self.hybrid_profile.tyre_normal_force_n + self.downforce_coefficient * state.vx_mps * state.vx_mps
        def force(alpha: float, front: bool) -> float:
            axle = 0.5 * (self.config.w_front if front else 1.0 - self.config.w_front) * normal
            mu = self.tire_D * math.sin(self.tire_C * math.atan(self.tire_B * (1.0 - self.tire_E) * alpha + self.tire_E * math.atan(self.tire_B * alpha)))
            return axle * mu
        return force(front_alpha, True), force(rear_alpha, False)

    def derivative(self, state: VehicleState9, control: Sequence[float], reference: ReferencePoint, hybrid: HybridState, dt_s: float) -> Tuple[VehicleState9, HybridState, Mapping[str, float]]:
        values = tuple(control)
        if len(values) != 3:
            raise ValueError("control requires steering rate, acceleration and MGU-K force")
        steering_rate, acceleration, mguk = (_finite(value, name) for value, name in zip(values, CONTROL_NAMES))
        delta = _clamp(state.steering_rad, -self.config.max_steering_rad, self.config.max_steering_rad)
        steer_rate = _clamp(steering_rate, -self.config.max_steering_rate_rps, self.config.max_steering_rate_rps)
        current = VehicleState9(*state.as_tuple()[:-3], delta, state.energy_j, state.pack_temperature_k)
        front, rear = self.lateral_forces(current); lateral_force = 2.0 * (math.cos(delta) * front + rear)
        hybrid_result = hybrid_step(self.hybrid_profile, hybrid, DriveRequest(acceleration, mguk), max(0.0, state.vx_mps), lateral_force, dt_s)
        drag = self.drag_coefficient * state.vx_mps * abs(state.vx_mps) + (self.rolling_resistance_n if state.vx_mps > 0.0 else 0.0)
        Fx = hybrid_result.delivered_tyre_force_n - drag; m = self.config.mass_kg; iz = self.config.inertia_z_kgm2; lf = self.front_lever_arm_m; lr = self.rear_lever_arm_m
        vx_dot = state.yaw_rate_rps * state.vy_mps + (Fx - math.sin(delta) * 2.0 * front) / m
        vy_dot = (math.cos(delta) * 2.0 * front + 2.0 * rear) / m - state.yaw_rate_rps * state.vx_mps
        r_dot = (math.cos(delta) * 2.0 * front * lf - 2.0 * rear * lr) / iz
        denom = max(1e-4, 1.0 - reference.curvature_inv_m * state.n_m)
        s_dot = (state.vx_mps * math.cos(state.heading_error_rad) - state.vy_mps * math.sin(state.heading_error_rad)) / denom
        n_dot = state.vx_mps * math.sin(state.heading_error_rad) + state.vy_mps * math.cos(state.heading_error_rad)
        heading_dot = state.yaw_rate_rps - reference.curvature_inv_m * s_dot
        values = (s_dot, n_dot, heading_dot, vx_dot, vy_dot, r_dot, steer_rate, hybrid_result.signed_energy_delta_j / max(dt_s, 1e-9), (hybrid_result.state.temperature_k - state.pack_temperature_k) / max(dt_s, 1e-9))
        predicted = VehicleState9(*(old + rate * dt_s for old, rate in zip(state.as_tuple(), values)))
        speed = math.hypot(state.vx_mps, state.vy_mps); blend = _clamp(0.5 * (speed - 1.5), 0.0, 1.0)
        low_vx = state.vx_mps + dt_s * (Fx / m); low_vy = math.tan(delta) * predicted.vx_mps * self.config.w_front; low_r = math.tan(delta) * predicted.vx_mps / self.config.wheelbase_m
        predicted = VehicleState9(predicted.s_m, predicted.n_m, predicted.heading_error_rad, _clamp(blend * predicted.vx_mps + (1.0 - blend) * low_vx, self.config.min_vx_mps, self.config.max_vx_mps), blend * predicted.vy_mps + (1.0 - blend) * low_vy, blend * predicted.yaw_rate_rps + (1.0 - blend) * low_r, _clamp(predicted.steering_rad, -self.config.max_steering_rad, self.config.max_steering_rad), hybrid_result.state.stored_energy_j, hybrid_result.state.temperature_k)
        return predicted, hybrid_result.state, {"lateral_force_n": lateral_force, "longitudinal_force_limit_n": hybrid_result.longitudinal_force_limit_n, "delivered_force_n": hybrid_result.delivered_tyre_force_n, "grip_scale": hybrid_result.grip_scale}

    def integrate(self, state: VehicleState9, control: Sequence[float], reference: ReferencePoint, hybrid: HybridState, interval_s: float) -> Tuple[VehicleState9, HybridState, Mapping[str, float]]:
        current, hybrid_state, diagnostics, _ = self.integrate_trace(state, control, reference, hybrid, interval_s)
        return current, hybrid_state, diagnostics

    def integrate_trace(self, state: VehicleState9, control: Sequence[float], reference: ReferencePoint, hybrid: HybridState, interval_s: float) -> Tuple[VehicleState9, HybridState, Mapping[str, float], Tuple[VehicleState9, ...]]:
        remaining = _finite(interval_s, "interval_s"); current, hybrid_state, diagnostics = state, hybrid, {}
        trace = []
        while remaining > 1e-12:
            dt = min(remaining, self.config.integration_substep_s)
            current, hybrid_state, diagnostics = self.derivative(current, control, reference, hybrid_state, dt)
            trace.append(current); remaining -= dt
        return current, hybrid_state, diagnostics, tuple(trace)


def _import_acados_template() -> Any:
    """Import acados_template from an installed package or the workspace toolchain."""
    candidates = []
    for key in ("ACADOS_TEMPLATE_PATH", "ACADOS_SOURCE_DIR"):
        value = os.environ.get(key)
        if value:
            root = pathlib.Path(value)
            candidates.extend((root, root / "interfaces" / "acados_template"))
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        candidates.extend((parent / ".tmp" / "acados" / "interfaces" / "acados_template", parent / ".tmp" / "controls-solver-venv" / "lib" / "python3.10" / "site-packages"))
    for candidate in candidates:
        path = str(candidate)
        if candidate.exists() and path not in sys.path:
            sys.path.insert(0, path)
    # The repository keeps the compiled renderer/libraries in the separate
    # install prefix used by the controls solver image.
    for parent in pathlib.Path(__file__).resolve().parents:
        install = parent / ".tmp" / "acados-install"
        renderer = install / "bin" / "t_renderer"
        if renderer.exists():
            os.environ.setdefault("TERA_PATH", str(renderer))
            os.environ.setdefault("ACADOS_INSTALL_DIR", str(install))
            lib_dir = str(install / "lib")
            old_ld = os.environ.get("LD_LIBRARY_PATH", "")
            if lib_dir not in old_ld.split(":"):
                os.environ["LD_LIBRARY_PATH"] = lib_dir + ((":" + old_ld) if old_ld else "")
            # ctypes resolves DT_NEEDED dependencies at load time; preload the
            # bundled solver dependencies so generated Python solvers work even
            # when the parent process did not export LD_LIBRARY_PATH early.
            for name in ("libblasfeo.so", "libhpipm.so"):
                library = install / "lib" / name
                if library.exists():
                    try:
                        ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
                    except OSError:
                        pass
            break
    import acados_template  # type: ignore
    return acados_template


class AcadosSolverAdapter:
    """Generated acados SQP/RTI solver with lazy toolchain loading.

    Generation is deliberately performed on first solve, so importing the ROS
    package does not compile C code.  ``code_export_directory`` can point at a
    persisted generated tree; otherwise a workspace-local cache is used.
    """

    def __init__(self, model: NativeDynamicBicycle, *, code_export_directory: Optional[str] = None, auto_generate: bool = True) -> None:
        self.model = model
        self.solver = None
        self.error = ""
        self.generated = False
        self.auto_generate = bool(auto_generate)
        default_dir = pathlib.Path(os.environ.get("EUFS_MPC_ACADOS_DIR", ".tmp/eufs_mpc_acados"))
        self.code_export_directory = pathlib.Path(code_export_directory) if code_export_directory else default_dir
        self._acados = None
        self._warm_x = None
        self._warm_u = None
        try:
            self._acados = _import_acados_template()
            self.available = True
        except Exception as exc:
            self.available = False
            self.error = str(exc)

    def reset(self) -> None:
        """Discard warm starts after a run/epoch transition."""
        self._warm_x = None
        self._warm_u = None

    def _build_ocp(self) -> Tuple[Any, Any]:
        if self._acados is None:
            self._acados = _import_acados_template()
        import casadi as ca  # type: ignore
        import numpy as np  # type: ignore
        ac = self._acados
        model = self.model
        x = ca.SX.sym("x", 9)
        xdot = ca.SX.sym("xdot", 9)
        u = ca.SX.sym("u", 3)
        p = ca.SX.sym("p", 17)
        ref = p
        # The generated OCP uses a deliberately small kinematic model.  The
        # native nine-state plant remains the publication validator, while
        # this model avoids tyre-force clipping and dynamic slip branches that
        # made SQP unusable for a 20-knot runtime horizon.
        delta, vx, vy, yaw = x[6], x[3], x[4], x[5]
        requested = model.config.mass_kg * u[1]
        ice_limit = ca.fmin(model.hybrid_profile.ice_max_force_n, model.hybrid_profile.ice_max_power_w / ca.fmax(model.hybrid_profile.min_force_speed_mps, ca.fabs(vx)))
        mguk_limit = ca.fmin(model.hybrid_profile.mguk_max_force_n, model.hybrid_profile.mguk_max_power_w / ca.fmax(model.hybrid_profile.min_force_speed_mps, ca.fabs(vx)))
        signed_mguk = ca.if_else(requested >= 0.0, ca.fmin(ca.fmax(u[2], 0.0), mguk_limit), ca.fmax(ca.fmin(u[2], 0.0), -model.hybrid_profile.mguk_regen_force_n))
        signed_mguk = ca.if_else(x[7] * model.config.energy_state_scale_mj <= model.hybrid_profile.reserve_energy_j, ca.fmin(signed_mguk, 0.0), signed_mguk)
        ice = ca.fmin(ca.fmax(requested - signed_mguk, -ice_limit), ice_limit)
        delivered = ice + signed_mguk
        # Native semantics do not apply rolling resistance to a stopped car;
        # keeping that branch explicit prevents the vx >= 0 bound from being
        # violated during a zero-command standstill warmup.
        drag = model.drag_coefficient * vx * ca.fabs(vx) + ca.if_else(vx > 1e-3, model.rolling_resistance_n, 0.0)
        sdot = (vx * ca.cos(x[2]) - vy * ca.sin(x[2])) / ca.fmax(1e-4, 1.0 - ref[6] * x[1])
        yaw_target = vx * ca.tan(delta) / model.config.wheelbase_m
        energy_dot = ca.if_else(signed_mguk >= 0.0, -signed_mguk * ca.fabs(vx) / model.hybrid_profile.mguk_efficiency, -signed_mguk * ca.fabs(vx) * model.hybrid_profile.regen_efficiency) - model.hybrid_profile.auxiliary_power_w
        # Temperature is carried through the public nine-state contract and
        # validated by the native rollout; the reduced OCP keeps it constant
        # to avoid coupling a slow thermal branch into the realtime QP.
        temp_dot = 0.0
        # x[7] is MJ in the OCP; public VehicleState9 keeps joules.
        f_expl = ca.vertcat(sdot, vx * ca.sin(x[2]) + vy * ca.cos(x[2]), yaw - ref[6] * sdot, (delivered - drag) / model.config.mass_kg, 8.0 * (vx * ca.tan(delta) * model.config.w_front - vy), 8.0 * (yaw_target - yaw), u[0], energy_dot / model.config.energy_state_scale_mj, temp_dot)
        ac_model = ac.AcadosModel()
        ac_model.name = "eufs_kinematic_energy_mpc_v2"
        ac_model.x, ac_model.xdot, ac_model.u, ac_model.p = x, xdot, u, p
        ac_model.f_expl_expr = f_expl
        ac_model.f_impl_expr = f_expl - xdot
        tracking = ca.vertcat(x[0] - p[0], x[1] - p[1], x[2] - p[2], x[3] - p[3], x[4] - p[4], x[5] - p[5], x[6] - p[7], x[7] - p[8], x[8] - p[9], u)
        tracking_e = ca.vertcat(x[0] - p[0], x[1] - p[1], x[2] - p[2], x[3] - p[3], x[4] - p[4], x[5] - p[5], x[6] - p[7], x[7] - p[8], x[8] - p[9])
        weights = ca.DM(np.diag([8e-3, 5.0, 2.0, 0.12, 0.02, 0.05, 0.2, 0.5, 1e-3, 0.04, 0.01, 0.01]))
        weights_e = ca.DM(np.diag([2e-2, 15.0, 6.0, 0.25, 0.04, 0.1, 0.3, 0.8, 2e-3]))
        ac_model.cost_y_expr = ca.vertcat(x, u, p[13] * ca.fmax(0.0, p[8] - x[7]))
        ac_model.cost_y_expr_e = ca.vertcat(x, p[13] * ca.fmax(0.0, p[8] - x[7]))
        # Combined tyre capacity, corridor footprint and terminal energy.
        lateral_extent = ca.fabs(ca.sin(x[2])) * max(model.config.footprint_front_m, model.config.footprint_rear_m) + ca.fabs(ca.cos(x[2])) * model.config.footprint_half_width_m
        h = ca.vertcat(ice_limit + mguk_limit - ca.fabs(requested), x[1] - (ref[10] + lateral_extent), (ref[11] - lateral_extent) - x[1], x[7] - ref[12])
        ac_model.con_h_expr = h
        ocp = ac.AcadosOcp()
        ocp.model = ac_model
        ocp.parameter_values = np.zeros(17)
        lib_hint = os.environ.get("ACADOS_LIB_PATH")
        if not lib_hint and os.environ.get("ACADOS_INSTALL_DIR"):
            lib_hint = str(pathlib.Path(os.environ["ACADOS_INSTALL_DIR"]) / "lib")
        if lib_hint and hasattr(ocp.code_gen_options, "acados_lib_path"):
            ocp.code_gen_options.acados_lib_path = lib_hint
        include_hint = os.environ.get("ACADOS_INCLUDE_PATH")
        if not include_hint and os.environ.get("ACADOS_INSTALL_DIR"):
            include_hint = str(pathlib.Path(os.environ["ACADOS_INSTALL_DIR"]) / "include")
        if include_hint and hasattr(ocp.code_gen_options, "acados_include_path"):
            ocp.code_gen_options.acados_include_path = include_hint
        N = len(self.model.config.horizon_dt)
        ocp.solver_options.N_horizon = N
        ocp.solver_options.time_steps = np.asarray(self.model.config.horizon_dt, dtype=float)
        ocp.solver_options.tf = float(sum(self.model.config.horizon_dt))
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        # The reduced OCP is intended for a realtime loop.  SQP remains
        # selectable for offline diagnostics, while RTI avoids spending the
        # full iteration budget on a stale command.
        ocp.solver_options.nlp_solver_type = os.environ.get("MPC_ACADOS_NLP_SOLVER", "SQP_RTI")
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 2
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag([8e-3, 5.0, 2.0, 0.12, 0.02, 0.05, 0.2, 0.5, 1e-3, 0.04, 0.01, 0.01, 1e-3])
        ocp.cost.W_e = np.diag([2e-2, 15.0, 6.0, 0.25, 0.04, 0.1, 0.3, 0.8, 2e-3, 1e-3])
        ocp.cost.yref = np.zeros(13)
        ocp.cost.yref_e = np.zeros(10)
        ocp.constraints.idxbx = np.array([3, 6, 7, 8], dtype=int)
        ocp.constraints.lbx = np.array([model.config.min_vx_mps, -model.config.max_steering_rad, 0.0, model.hybrid_profile.ambient_temperature_k])
        ocp.constraints.ubx = np.array([model.config.max_vx_mps, model.config.max_steering_rad, model.hybrid_profile.battery_capacity_j / model.config.energy_state_scale_mj, model.config.max_pack_temperature_k])
        ocp.constraints.idxbu = np.arange(3, dtype=int)
        ocp.constraints.lbu = np.array([-model.config.max_steering_rate_rps, model.config.min_predrag_acceleration_mps2, -model.hybrid_profile.mguk_regen_force_n])
        ocp.constraints.ubu = np.array([model.config.max_steering_rate_rps, model.config.max_predrag_acceleration_mps2, model.hybrid_profile.mguk_max_force_n])
        ocp.constraints.lh = np.zeros(4)
        ocp.constraints.uh = np.full(4, ac.ACADOS_INFTY if hasattr(ac, "ACADOS_INFTY") else 1e9)
        ocp.constraints.x0 = np.zeros(9)
        ocp.solver_options.nlp_solver_max_iter = 20
        return ocp, ac_model

    def ensure_solver(self) -> None:
        if self.solver is not None:
            return
        if not self.available:
            raise RuntimeError(self.error or "acados_template unavailable")
        try:
            self.code_export_directory.mkdir(parents=True, exist_ok=True)
            ocp, _ = self._build_ocp()
            ocp.code_export_directory = str(self.code_export_directory)
            json_file = str(self.code_export_directory / "eufs_kinematic_energy_mpc_v2.json")
            # AcadosOcpSolver performs generation and compilation if this tree is
            # new, and loads the generated shared library on subsequent solves.
            self.solver = self._acados.AcadosOcpSolver(ocp, json_file=json_file, verbose=False)
            self.generated = True
        except Exception as exc:
            self.error = f"acados generation/loading failed: {exc}"
            raise

    def _params(self, point: ReferencePoint, request: MPCInput, *, terminal: bool = False) -> Any:
        import numpy as np  # type: ignore
        minimum = request.minimum_exit_energy_j if terminal and request.minimum_exit_energy_j is not None else self.model.hybrid_profile.reserve_energy_j
        scale = self.model.config.energy_state_scale_mj
        return np.asarray([point.s_m, point.n_m, point.heading_error_rad, point.vx_mps, point.vy_mps, point.yaw_rate_rps, point.curvature_inv_m, point.steering_rad, (point.energy_j if point.energy_j is not None else request.state.energy_j) / scale, point.pack_temperature_k if point.pack_temperature_k is not None else request.state.pack_temperature_k, point.corridor_left_m, point.corridor_right_m, (point.minimum_exit_energy_j if terminal and point.minimum_exit_energy_j is not None else minimum) / scale, point.energy_price_s_per_mj if request.energy_price_s_per_mj is None else request.energy_price_s_per_mj, point.footprint_half_width_m, self.model.hybrid_profile.max_deploy_per_lap_j / scale, self.model.hybrid_profile.max_recovery_per_lap_j / scale], dtype=float)

    def _solver_state(self, state: Sequence[float]) -> Any:
        import numpy as np  # type: ignore
        values = np.asarray(state, dtype=float).copy()
        values[7] /= self.model.config.energy_state_scale_mj
        return values

    def _public_state(self, state: Sequence[float]) -> Tuple[float, ...]:
        values = list(float(value) for value in state)
        values[7] *= self.model.config.energy_state_scale_mj
        return tuple(values)

    def solve(self, request: MPCInput) -> MPCResult:
        start = time.monotonic()
        try:
            self.ensure_solver()
        except Exception as exc:
            return MPCResult("solver_unavailable", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), time.monotonic() - start, rejection_reason=str(exc))
        try:
            import numpy as np  # type: ignore
            solver = self.solver
            solver.set(0, "lbx", self._solver_state(request.state.as_tuple())); solver.set(0, "ubx", self._solver_state(request.state.as_tuple()))
            # Controls inside the physical delay are fixed to the command that
            # is currently in flight.  This is atomic for steering/accel/MGU-K.
            if request.pending_commands:
                _, delayed, delayed_accel, delayed_mguk = request.pending_commands[-1]
            else:
                delayed = request.steering_history[-1] if request.steering_history else request.state.steering_rad
                delayed_accel = request.applied_acceleration_mps2
                delayed_mguk = request.applied_mguk_force_n
            delayed_rate = _clamp((delayed - request.state.steering_rad) / max(self.model.config.steering_actuator_tau_s, 1e-6), -self.model.config.max_steering_rate_rps, self.model.config.max_steering_rate_rps)
            hold = np.asarray([delayed_rate, delayed_accel, delayed_mguk], dtype=float)
            elapsed = 0.0
            if self._warm_x is None or len(self._warm_x) != len(self.model.config.horizon_dt) + 1:
                warm_x = np.tile(self._solver_state(request.state.as_tuple()), (len(self.model.config.horizon_dt) + 1, 1))
            else:
                warm_x = np.asarray(self._warm_x, dtype=float)
            if self._warm_u is None or len(self._warm_u) != len(self.model.config.horizon_dt):
                warm_u = np.tile(hold, (len(self.model.config.horizon_dt), 1))
            else:
                warm_u = np.asarray(self._warm_u, dtype=float)
            for i in range(len(self.model.config.horizon_dt) + 1):
                solver.set(i, "x", warm_x[min(i, len(warm_x) - 1)])
            for i, dt in enumerate(self.model.config.horizon_dt):
                point = request.reference.at(i)
                solver.set(i, "p", self._params(point, request, terminal=False))
                xref = (point.s_m, point.n_m, point.heading_error_rad, point.vx_mps, point.vy_mps, point.yaw_rate_rps, point.steering_rad, (point.energy_j if point.energy_j is not None else request.state.energy_j) / self.model.config.energy_state_scale_mj, point.pack_temperature_k if point.pack_temperature_k is not None else request.state.pack_temperature_k)
                solver.set(i, "yref", np.asarray((*xref, 0.0, 0.0, 0.0, 0.0), dtype=float))
                if elapsed < self.model.config.command_delay_s - 1e-9:
                    solver.set(i, "lbu", hold); solver.set(i, "ubu", hold)
                solver.set(i, "u", warm_u[min(i, len(warm_u) - 1)])
                elapsed += dt
            end = request.reference.at(len(self.model.config.horizon_dt) - 1)
            solver.set(len(self.model.config.horizon_dt), "p", self._params(end, request, terminal=True))
            end_ref = (end.s_m, end.n_m, end.heading_error_rad, end.vx_mps, end.vy_mps, end.yaw_rate_rps, end.steering_rad, (end.energy_j if end.energy_j is not None else request.state.energy_j) / self.model.config.energy_state_scale_mj, end.pack_temperature_k if end.pack_temperature_k is not None else request.state.pack_temperature_k)
            solver.set(len(self.model.config.horizon_dt), "yref", np.asarray((*end_ref, 0.0), dtype=float))
            status = int(solver.solve())
            controls = tuple(tuple(float(v) for v in np.asarray(solver.get(i, "u")).reshape(-1)) for i in range(len(self.model.config.horizon_dt)))
            solver_states = tuple(tuple(float(v) for v in np.asarray(solver.get(i, "x")).reshape(-1)) for i in range(len(self.model.config.horizon_dt) + 1))
            states = tuple(self._public_state(state) for state in solver_states)
            finite_solution = all(math.isfinite(value) for control in controls for value in control) and all(math.isfinite(value) for state in states for value in state)
            first_action_index = next((index for index, elapsed_knot in enumerate(__import__('itertools').accumulate(self.model.config.horizon_dt, initial=0.0)) if elapsed_knot >= self.model.config.command_delay_s - 1e-9), len(controls) - 1)
            stats = {"status_code": status, "iterations": int(solver.get_stats("sqp_iter")) if hasattr(solver, "get_stats") else 0, "first_action_index": first_action_index, "delay_prefix_s": sum(self.model.config.horizon_dt[:first_action_index])}
            if hasattr(solver, "get_stats"):
                for stat_name in ("cost_value", "res_stat", "res_eq", "res_ineq", "res_comp", "qp_iter"):
                    try: stats[stat_name] = float(solver.get_stats(stat_name))
                    except Exception: pass
            if status != 0 or not finite_solution:
                self.reset()
                reason = f"acados status {status}" if status != 0 else "non-finite solver output"
                return MPCResult("solver_failed", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), time.monotonic() - start, controls, states, diagnostics=stats, rejection_reason=reason)
            self._warm_u = controls[1:] + (controls[-1],) if controls else None
            self._warm_x = solver_states[1:] + (solver_states[-1],) if solver_states else None
            # Recheck state/resource/corridor constraints before publication.
            check = self.model_rollout(request, controls)
            if check.status == "rejected":
                return MPCResult("constraint_rejected", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), time.monotonic() - start, controls, states, diagnostics=stats, rejection_reason=check.rejection_reason)
            return MPCResult("solved", False, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), time.monotonic() - start, controls, states, diagnostics=stats)
        except Exception as exc:
            return MPCResult("solver_failed", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), time.monotonic() - start, rejection_reason=str(exc))

    def model_rollout(self, request: MPCInput, controls: Sequence[Sequence[float]]) -> MPCResult:
        # A local rollout is used only for publication-time validation; the
        # controller owns the complete version below when this adapter is used.
        controller = MPCController(self.model.config, solver=self)
        return controller.prediction_rollout(request, controls)


def build_casadi_model(model: NativeDynamicBicycle) -> Any:
    """Build the solver-independent CasADi dynamics used by acados and tests.

    ``p`` has 17 entries: reference state (0:9), pack/corridor/resource values
    (9:13), energy price (13), footprint half-width (14), and lap energy quota
    (15:17).
    """
    import casadi as ca  # type: ignore
    x = ca.SX.sym("x", 9); u = ca.SX.sym("u", 3); ref = ca.SX.sym("ref", 17)
    delta, vx, vy, yaw_rate, kappa = x[6], x[3], x[4], x[5], ref[6]
    denom = ca.fmax(1.0, vx - 0.5 * model.config.axle_width_m * yaw_rate)
    alpha_f = ca.atan((vy + model.front_lever_arm_m * yaw_rate) / denom) - delta
    alpha_r = ca.atan((vy - model.rear_lever_arm_m * yaw_rate) / denom)
    normal = model.hybrid_profile.tyre_normal_force_n + model.downforce_coefficient * vx * vx
    def fy(alpha: Any, front: bool) -> Any:
        axle = 0.5 * (model.config.w_front if front else 1.0 - model.config.w_front) * normal
        mu = model.tire_D * ca.sin(model.tire_C * ca.atan(model.tire_B * (1.0 - model.tire_E) * alpha + model.tire_E * ca.atan(model.tire_B * alpha)))
        return axle * mu
    fyf, fyr = fy(alpha_f, True), fy(alpha_r, False)
    lateral = 2.0 * (ca.cos(delta) * fyf + fyr)
    capacity = ca.sqrt(ca.fmax(1.0, (model.hybrid_profile.tyre_mu * normal) ** 2 - lateral ** 2))
    requested = model.config.mass_kg * u[1]
    longitudinal = ca.fmin(ca.fmax(requested, -capacity), capacity)
    thermal_scale = ca.if_else(x[8] > model.hybrid_profile.thermal_derate_start_k, ca.fmin(1.0, ca.fmax(0.0, (model.hybrid_profile.max_temperature_k - x[8]) / (model.hybrid_profile.max_temperature_k - model.hybrid_profile.thermal_derate_start_k))), 1.0)
    raw_mguk = ca.fmin(ca.fmax(u[2], -model.hybrid_profile.mguk_regen_force_n * thermal_scale), model.hybrid_profile.mguk_max_force_n * thermal_scale)
    signed_mguk = ca.if_else(longitudinal >= 0.0, ca.fmax(0.0, raw_mguk), ca.fmin(0.0, raw_mguk))
    signed_mguk = ca.if_else(x[7] * model.config.energy_state_scale_mj <= model.hybrid_profile.reserve_energy_j, 0.0, signed_mguk)
    ice_limit = ca.fmin(model.hybrid_profile.ice_max_force_n, model.hybrid_profile.ice_max_power_w / ca.fmax(model.hybrid_profile.min_force_speed_mps, ca.fabs(vx)))
    ice = ca.fmin(ice_limit, ca.fmax(0.0, longitudinal - signed_mguk))
    friction = ca.fmin(0.0, longitudinal - signed_mguk - ice)
    delivered = ice + signed_mguk + friction
    drag = model.drag_coefficient * vx * ca.fabs(vx) + model.rolling_resistance_n
    sdot = (vx * ca.cos(x[2]) - vy * ca.sin(x[2])) / ca.fmax(1e-4, 1.0 - kappa * x[1])
    energy_dot = ca.if_else(signed_mguk >= 0.0, -signed_mguk * ca.fabs(vx) / model.hybrid_profile.mguk_efficiency, -signed_mguk * ca.fabs(vx) * model.hybrid_profile.regen_efficiency) - model.hybrid_profile.auxiliary_power_w
    loss_power = ca.if_else(signed_mguk >= 0.0, ca.fabs(signed_mguk * vx) * (1.0 / model.hybrid_profile.mguk_efficiency - 1.0), ca.fabs(signed_mguk * vx) * (1.0 - model.hybrid_profile.regen_efficiency))
    temp_dot = (loss_power + model.hybrid_profile.auxiliary_power_w - model.hybrid_profile.cooling_w_per_k * (x[8] - model.hybrid_profile.ambient_temperature_k)) / model.hybrid_profile.thermal_capacity_j_per_k
    xdot = ca.vertcat(sdot, vx * ca.sin(x[2]) + vy * ca.cos(x[2]), yaw_rate - kappa * sdot, yaw_rate * vy + (delivered - ca.sin(delta) * 2 * fyf - drag) / model.config.mass_kg, (ca.cos(delta) * 2 * fyf + 2 * fyr) / model.config.mass_kg - yaw_rate * vx, (ca.cos(delta) * 2 * fyf * model.front_lever_arm_m - 2 * fyr * model.rear_lever_arm_m) / model.config.inertia_z_kgm2, u[0], energy_dot / model.config.energy_state_scale_mj, temp_dot)
    return ca.Function("native_dynamic_bicycle_step", [x, u, ref], [xdot], ["x", "u", "ref"], ["xdot"])


class MPCController:
    def __init__(self, config: MPCConfig = MPCConfig(), *, solver: Optional[AcadosSolverAdapter] = None, hybrid_profile: Optional[HybridProfile] = None) -> None:
        self.config = config
        self.model = NativeDynamicBicycle(config, hybrid_profile)
        self.solver = solver or AcadosSolverAdapter(self.model)
        self.delay = DelayHistory(config.command_delay_s)
        self._last_epoch: Optional[int] = None
        self._last_run_id: Optional[str] = None
        self._seeded_snapshot_id: Optional[str] = None

    def reset(self, epoch_id: int, run_id: Optional[str] = None) -> None:
        self._last_epoch = int(epoch_id); self._last_run_id = run_id; self._seeded_snapshot_id = None
        self.delay.reset(epoch_id)
        if hasattr(self.solver, "reset"):
            self.solver.reset()

    def _seed_delay_history(self, request: MPCInput) -> None:
        if self._seeded_snapshot_id == request.state_snapshot_id:
            return
        if request.pending_commands:
            self.delay.extend(((entry[0], entry[1:]) for entry in request.pending_commands), request.epoch_id)
        elif request.steering_history:
            base = request.stamp_s - self.config.command_delay_s
            count = len(request.steering_history)
            for index, angle in enumerate(request.steering_history):
                stamp = base + (index + 1) * self.config.command_delay_s / max(1, count)
                self.delay.append(stamp, (angle, request.applied_acceleration_mps2, request.applied_mguk_force_n), request.epoch_id)
        self._seeded_snapshot_id = request.state_snapshot_id

    def _inflight_rate(self, target_angle: float, steering_angle: float) -> float:
        return _clamp((target_angle - steering_angle) / max(self.config.steering_actuator_tau_s, 1e-6), -self.config.max_steering_rate_rps, self.config.max_steering_rate_rps)

    def command_from_result(self, result: MPCResult, steering_angle_rad: Optional[float] = None) -> Optional[Tuple[float, float, float]]:
        """Convert the first optimised rate knot to the native angle command."""
        if result.fallback or not result.controls: return None
        current = steering_angle_rad
        if current is None and result.predicted_states:
            current = result.predicted_states[0][6]
        if current is None: return None
        index = int(result.diagnostics.get("first_action_index", 0))
        index = min(max(index, 0), len(result.controls) - 1)
        rate, acceleration, mguk = result.controls[index]
        target = _clamp(current + rate * 0.05, -self.config.max_steering_rad, self.config.max_steering_rad)
        return target, acceleration, mguk

    def solve(self, request: MPCInput, *, now_monotonic_s: Optional[float] = None) -> MPCResult:
        now = time.monotonic() if now_monotonic_s is None else _finite(now_monotonic_s, "now_monotonic_s")
        if self._last_epoch != request.epoch_id or self._last_run_id != request.run_id: self.reset(request.epoch_id, request.run_id)
        if request.expires_at_s is not None and request.stamp_s > request.expires_at_s: return self._reject(request, "input expiry precedes stamp")
        if now - request.received_monotonic_s > self.config.deadline_s: return self._reject(request, "input stale")
        if any(not math.isfinite(value) for point in request.reference.points for value in (point.s_m, point.n_m, point.heading_error_rad, point.vx_mps)): return self._reject(request, "reference non-finite")
        self._seed_delay_history(request)
        result = self.solver.solve(request)
        if result.status == "solved" and result.solve_time_s > self.config.deadline_s:
            return MPCResult("deadline_exceeded", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, result.generation_monotonic_s, result.solve_time_s, rejection_reason=f"solve exceeded deadline ({result.solve_time_s:.6f}s > {self.config.deadline_s:.6f}s)", diagnostics=result.diagnostics)
        return result

    def _reject(self, request: MPCInput, reason: str) -> MPCResult:
        return MPCResult("rejected", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), 0.0, rejection_reason=reason)

    def prediction_rollout(self, request: MPCInput, controls: Sequence[Sequence[float]]) -> MPCResult:
        start = time.monotonic()
        self.reset(request.epoch_id, request.run_id)
        self._seed_delay_history(request)
        if len(controls) > len(self.config.horizon_dt): return self._reject(request, "control horizon exceeds configured horizon")
        normalised = []
        for command in controls:
            values = tuple(_finite(value, "control") for value in command)
            if len(values) != 3: return self._reject(request, "control shape invalid")
            normalised.append(values)
        hybrid = HybridState(request.state.energy_j, request.state.pack_temperature_k)
        state = request.state
        predicted = [state.as_tuple()]
        diagnostics: Mapping[str, float] = {}
        elapsed_sim = 0.0
        for index, dt in enumerate(self.config.horizon_dt):
            command = normalised[min(index, len(normalised) - 1)] if normalised else (0.0, 0.0, 0.0)
            reference = request.reference.at(index)
            action_stamp = request.stamp_s + elapsed_sim
            # Commands issued before this snapshot are angles in the native
            # queue.  Optimised knots become active after the delay and are
            # already rates, so they do not get target/tau transformed.
            queued = self.delay.command_at(action_stamp, (state.steering_rad, request.applied_acceleration_mps2, request.applied_mguk_force_n))
            if elapsed_sim < self.config.command_delay_s - 1e-9:
                applied = (self._inflight_rate(queued[0], state.steering_rad), queued[1], queued[2])
            else:
                applied = command
            state, hybrid, diagnostics, trace = self.model.integrate_trace(state, applied, reference, hybrid, dt)
            for sample in trace:
                reason = self._validate_prediction_state(sample, reference, terminal=False)
                if reason: return self._reject(request, reason)
            elapsed_sim += dt
            if state.energy_j < (request.minimum_exit_energy_j if index == len(self.config.horizon_dt) - 1 and request.minimum_exit_energy_j is not None else self.model.hybrid_profile.reserve_energy_j) - 1e-6:
                return self._reject(request, "energy lower bound infeasible")
            predicted.append(state.as_tuple())
        final_ref = request.reference.at(len(self.config.horizon_dt) - 1)
        terminal_reason = self._validate_prediction_state(state, final_ref, terminal=True)
        if terminal_reason: return self._reject(request, terminal_reason)
        requested_floor = request.minimum_exit_energy_j if request.minimum_exit_energy_j is not None else self.model.hybrid_profile.reserve_energy_j
        if state.energy_j < requested_floor - 1e-6: return self._reject(request, "terminal energy corridor infeasible")
        elapsed = time.monotonic() - start
        return MPCResult("prediction_only", True, request.run_id, request.epoch_id, request.state_snapshot_id, request.trajectory_id, time.monotonic(), elapsed, tuple(normalised), tuple(predicted), diagnostics, "")

    def _validate_prediction_state(self, state: VehicleState9, reference: ReferencePoint, *, terminal: bool) -> str:
        low, high = footprint_lateral_bounds(state.heading_error_rad, self.config.footprint_half_width_m, self.config.footprint_front_m, self.config.footprint_rear_m)
        if state.n_m + low < reference.corridor_left_m + self.config.boundary_margin_m - self.config.validation_tolerance_m or state.n_m + high > reference.corridor_right_m - self.config.boundary_margin_m + self.config.validation_tolerance_m:
            return "full footprint outside corridor"
        if state.pack_temperature_k > self.config.max_pack_temperature_k + 1e-9:
            return "pack temperature limit"
        if terminal and state.energy_j < (reference.minimum_exit_energy_j if reference.minimum_exit_energy_j is not None else self.model.hybrid_profile.reserve_energy_j) - 1e-6:
            return "terminal energy corridor infeasible"
        values = state.as_tuple()
        if any(not math.isfinite(value) for value in values): return "prediction non-finite"
        return ""


def standalone_single_solve(request: MPCInput, controls: Optional[Sequence[Sequence[float]]] = None, *, controller: Optional[MPCController] = None) -> MPCResult:
    """Reproducible headless entrypoint: adapter status plus optional prediction."""
    controller = controller or MPCController()
    if controls is None: return controller.solve(request)
    return controller.prediction_rollout(request, controls)
