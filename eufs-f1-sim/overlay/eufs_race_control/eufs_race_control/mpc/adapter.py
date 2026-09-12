"""Small pure-contract adapter from estimation/tactical outputs to Layer 3.

The adapter owns only ego projection and reference interpolation.  It does not
read opponent truth or invent opponent constraints; tactical corridor values
already present on ``TrajectoryPoint`` are the only corridor inputs forwarded
to the MPC.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple

from ..contracts import EstimatedRaceState, TrajectoryPlan, TrajectoryPoint
from ..geometry import TrackGeometry
from .model import Corridor, MPCInput, ReferencePoint, VehicleState9


def _finite(value: Any, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _angle_error(angle: float, tangent: float) -> float:
    return (angle - tangent + math.pi) % (2.0 * math.pi) - math.pi


def _field(value: Any, *names: str, default: Any = None) -> Any:
    if isinstance(value, Mapping):
        for name in names:
            if name in value:
                return value[name]
    for name in names:
        if hasattr(value, name):
            return getattr(value, name)
    return default


def _curvature_at(track: TrackGeometry, progress_m: float) -> float:
    """Use authored centreline curvature nearest to the projected progress."""
    points = getattr(track, "points", ())
    if not points:
        return 0.0
    wrapped = progress_m % track.length_m
    point = min(points, key=lambda item: abs((item.s_m % track.length_m) - wrapped))
    return float(getattr(point, "curvature_1pm", 0.0))


def _own_projection(estimate: EstimatedRaceState, track: TrackGeometry):
    ego = estimate.ego
    if ego.x_m is not None and ego.y_m is not None:
        return track.project(
            ego.x_m,
            ego.y_m,
            previous_progress_m=estimate.unwrapped_progress_m,
            previous_heading_rad=ego.yaw_rad,
            reachable_window_m=max(20.0, track.length_m * 0.25),
            max_heading_error_rad=math.pi,
            max_distance_m=None,
        )
    progress = estimate.unwrapped_progress_m
    if progress is None:
        progress = estimate.progress_m if estimate.progress_m is not None else 0.0
    pose = track.interpolate(progress)
    return track.project(pose.x_m, pose.y_m, previous_progress_m=progress, max_distance_m=None)


def _point_at(points: Sequence[TrajectoryPoint], stamp_s: float) -> Optional[TrajectoryPoint]:
    if not points:
        return None
    if stamp_s <= points[0].stamp:
        return points[0]
    if stamp_s >= points[-1].stamp:
        return points[-1]
    for left, right in zip(points, points[1:]):
        if left.stamp <= stamp_s <= right.stamp:
            span = max(right.stamp - left.stamp, 1e-9)
            fraction = (stamp_s - left.stamp) / span

            def blend(name: str) -> Optional[float]:
                a, b = getattr(left, name), getattr(right, name)
                if a is None: return b
                if b is None: return a
                return float(a) + fraction * (float(b) - float(a))

            return TrajectoryPoint(
                stamp_s,
                x_m=blend("x_m"), y_m=blend("y_m"), yaw_rad=blend("yaw_rad"),
                progress_m=blend("progress_m"), lateral_m=blend("lateral_m"),
                speed_mps=blend("speed_mps"), curvature_1pm=blend("curvature_1pm"),
                corridor_left_m=blend("corridor_left_m"), corridor_right_m=blend("corridor_right_m"),
                energy_reference_j=blend("energy_reference_j"), acceleration_mps2=blend("acceleration_mps2"),
                steering_rad=blend("steering_rad"),
            )
    return points[-1]


def _normalise_pending(commands: Iterable[Any], sim_time_s: float) -> Tuple[Tuple[float, float, float, float], ...]:
    result = []
    for command in commands:
        stamp = _field(command, "stamp_s", "issue_stamp_s", "stamp", default=sim_time_s)
        angle = _field(command, "steering_angle_rad", "steering_rad", "steering_target_rad", default=0.0)
        acceleration = _field(command, "predrag_acceleration_mps2", "acceleration_mps2", default=0.0)
        mguk = _field(command, "signed_mguk_force_n", "requested_mguk_force_n", "mguk_force_n", default=0.0)
        result.append((_finite(stamp, "pending stamp"), _finite(angle, "pending steering"), _finite(acceleration, "pending acceleration"), _finite(mguk, "pending MGU-K")))
    return tuple(sorted(result, key=lambda item: item[0]))


def build_mpc_input(
    estimate: EstimatedRaceState,
    trajectory: TrajectoryPlan,
    track: TrackGeometry,
    *,
    now_monotonic_s: float,
    actual_pending_commands: Iterable[Any] = (),
    sim_time_s: Optional[float] = None,
    horizon_dt: Sequence[float] = (0.10,) * 20,
) -> MPCInput:
    """Build one causal ``MPCInput`` from pure estimation/tactical contracts.

    ``estimate.header.stamp`` is simulation time.  ``now_monotonic_s`` is only
    the local receipt/deadline clock and is never used to advance references.
    Pending entries are timestamped native atomic commands, not future tactical
    opponent predictions.
    """
    if not isinstance(track, TrackGeometry):
        raise TypeError("track must be TrackGeometry")
    sim_stamp = _finite(estimate.header.stamp if sim_time_s is None else sim_time_s, "sim_time_s")
    received = _finite(now_monotonic_s, "now_monotonic_s")
    projection = _own_projection(estimate, track)
    tangent = track.interpolate(projection.unwrapped_progress_m).yaw_rad
    ego = estimate.ego
    energy = estimate.energy
    vx = ego.vx_mps if ego.vx_mps is not None else (ego.track_speed_mps if ego.track_speed_mps is not None else 0.0)
    vy = ego.vy_mps if ego.vy_mps is not None else (ego.lateral_speed_mps if ego.lateral_speed_mps is not None else 0.0)
    yaw_rate = ego.yaw_rate_rps if ego.yaw_rate_rps is not None else 0.0
    steering = ego.steering_rad if ego.steering_rad is not None else 0.0
    stored_energy = energy.stored_energy_j if energy.stored_energy_j is not None else (energy.reserve_energy_j if energy.reserve_energy_j is not None else 400_000.0)
    temperature = energy.temperature_k if energy.temperature_k is not None else 298.15
    state = VehicleState9(projection.unwrapped_progress_m, projection.lateral_m, _angle_error(ego.yaw_rad, tangent) if ego.yaw_rad is not None else 0.0, vx, vy, yaw_rate, steering, stored_energy, temperature)

    # Tactical timestamps are normally absolute simulation stamps.  If a plan
    # is relative (0, .1, ...), anchor it at its header stamp or current sim
    # time without changing its causal ordering.
    points = trajectory.points
    target_stamps = tuple(sim_stamp + sum(float(dt) for dt in horizon_dt[:index]) for index in range(len(horizon_dt)))
    relative_plan_offset = 0.0
    if points and points[-1].stamp < sim_stamp - 1e-9 and trajectory.header.stamp <= sim_stamp:
        relative_plan_offset = trajectory.header.stamp if trajectory.header.stamp else sim_stamp
    references = []
    half_width = 7.0
    for target in target_stamps:
        point = _point_at(points, target - relative_plan_offset)
        if point is None:
            progress = projection.unwrapped_progress_m + max(0.0, vx) * (target - sim_stamp)
            lateral, heading, speed, curvature = 0.0, 0.0, vx, _curvature_at(track, progress)
            left, right, energy_ref, steer = -half_width, half_width, None, steering
        else:
            progress = point.progress_m
            if progress is None and point.x_m is not None and point.y_m is not None:
                progress = track.project(point.x_m, point.y_m, previous_progress_m=projection.unwrapped_progress_m, max_distance_m=None).unwrapped_progress_m
            if progress is None:
                progress = projection.unwrapped_progress_m + max(0.0, vx) * (target - sim_stamp)
            frenet = track.interpolate(progress)
            lateral = point.lateral_m if point.lateral_m is not None else (track.project(point.x_m, point.y_m, previous_progress_m=progress, max_distance_m=None).lateral_m if point.x_m is not None and point.y_m is not None else 0.0)
            tangent = frenet.yaw_rad
            heading = _angle_error(point.yaw_rad, tangent) if point.yaw_rad is not None else 0.0
            speed = point.speed_mps if point.speed_mps is not None else vx
            curvature = point.curvature_1pm if point.curvature_1pm is not None else _curvature_at(track, progress)
            left = point.corridor_left_m if point.corridor_left_m is not None else -track.width_at(progress, default_half_width_m=half_width)
            right = point.corridor_right_m if point.corridor_right_m is not None else track.width_at(progress, default_half_width_m=half_width)
            energy_ref, steer = point.energy_reference_j, point.steering_rad if point.steering_rad is not None else steering
        references.append(ReferencePoint(progress, lateral, heading, speed, 0.0, speed * math.tan(steer) / 3.28, curvature, steer, energy_ref, temperature, energy.reserve_energy_j, 0.0, left, right))
    pending = _normalise_pending(actual_pending_commands, sim_stamp)
    steering_history = tuple(entry[1] for entry in pending)
    return MPCInput(
        run_id=estimate.header.run_id,
        epoch_id=estimate.header.epoch_id,
        state_snapshot_id=estimate.header.snapshot_id or f"state:{sim_stamp:.6f}",
        stamp_s=sim_stamp,
        received_monotonic_s=received,
        state=state,
        reference=Corridor(tuple(references)),
        steering_history=steering_history,
        applied_acceleration_mps2=pending[-1][2] if pending else 0.0,
        applied_mguk_force_n=pending[-1][3] if pending else 0.0,
        expires_at_s=estimate.header.expires_at,
        trajectory_id=estimate.header.trajectory_id or trajectory.header.trajectory_id or trajectory.candidate_id,
        minimum_exit_energy_j=energy.reserve_energy_j,
        pending_commands=pending,
    )


__all__ = ["build_mpc_input"]
