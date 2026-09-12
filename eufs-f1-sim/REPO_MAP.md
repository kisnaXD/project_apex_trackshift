# Repo map (EUFS F1 sim)

This document explains where the main pieces live in `eufs-f1-sim/`, and which
ROS 2 topics/services they use. It is meant for humans onboarding onto the code.

For “how to run it” and operator workflow, also see:

- `RUN_GRID_TELEMETRY.md`
- `HANDOVER.md`
- `GRID_TELEMETRY_PLAN.md`

## Layout

- `Dockerfile`
  - Full build image (clones EUFS upstream sources, applies overlays/patches, builds everything).
- `Dockerfile.grid`
  - Fast “overlay-only” rebuild on top of the known-good simulator image.
- `docker-compose.yml`
  - Container settings used by `scripts/start-stack.sh`.
- `scripts/`
  - Operator scripts (start, lap helper, 10s helper, build utilities).
- `overlay/`
  - All durable source changes that must survive a clean rebuild.
  - `overlay/eufs_racecar/` is the authored ROS package (dashboard + helpers + launch).
  - `overlay/eufs_tracks/` contains the COTA world/model/CSV assets and cone models.
- `tests/`
  - Focused Python tests for telemetry, map attachment, stack lifecycle, and contracts.
- `ws/`
  - Prepared colcon workspace clone/build output. This is runtime/build artifact; do not treat it
    as the “source of truth” for changes.

## Main entrypoints

- `overlay/eufs_racecar/eufs_racecar/start_dashboard.py`
  - The PyQt dashboard (Start/Stop/Shutdown) and the in-process telemetry view/model.
- `overlay/eufs_racecar/launch/load_car.launch.py`
  - Starts `gzserver` (paused), spawns 1–20 cars, launches per-car nodes (bridge, TF, tyres),
    and optionally starts the dashboard.
- `scripts/start-stack.sh`
  - Host helper that starts/reuses the canonical `eufs-f1-sim` container and opens the dashboard.

## Namespacing: cars 1..20

The canonical “car id” is the namespace string:

- Car 1: `eufs`
- Car N (2..20): `eufs2` .. `eufs20`

Per-car ROS namespaces use `/<car_id>/...`, for example `/<car_id>/cmd`.

TF frame prefixes:

- Ego frames: `map → odom → base_link`
- Opponent frames: `map → <car_id>/odom → <car_id>/base_link`

## ROS topics/services: what publishes/subscribes to what

Below, `ns` means the car namespace `/<car_id>` (for example `/eufs` or `/eufs4`).

### Dashboard telemetry subscriptions (StartDashboard)

File: `overlay/eufs_racecar/eufs_racecar/start_dashboard.py`

The dashboard subscribes to:

- `ns/cmd_vel` (`geometry_msgs/msg/Twist`) — telemetry only (effective target).
- `/clock` (`rosgraph_msgs/msg/Clock`) — sim time.
- `/ros_can/state` (`eufs_msgs/msg/CanState`) — global mission/state (ego only).
- `ns/odom` (`nav_msgs/msg/Odometry`) — kinematics source.
- `ns/joint_states` (`sensor_msgs/msg/JointState`) — wheel RPM (when available).
- `ns/forgez/battery_state` (`sensor_msgs/msg/BatteryState`) — pack voltage/current/SOC/temp.
- `ns/forgez/charge_level_wh` (`std_msgs/msg/Float64`)
- `ns/forgez/deploy_power_w` (`std_msgs/msg/Float64`)
- `ns/forgez/signed_power_w` (`std_msgs/msg/Float64`)
- `ns/forgez/lap_energy_remaining_wh` (`std_msgs/msg/Float64`)
- `ns/forgez/derate_reason` (`std_msgs/msg/String`)
- `ns/tyres/temps` (`std_msgs/msg/Float32MultiArray`)
- `ns/tyres/wheel_rpm` (`std_msgs/msg/Float32MultiArray`)
- `ns/tyres/degradation_rate` (`std_msgs/msg/Float32`)
- `ns/tyres/lap_degradation` (`std_msgs/msg/Float32`)
- `ns/tyres/life` (`std_msgs/msg/Float32`)

For opponents, the dashboard also subscribes to:

- `/<opponent_id>/odom` (`nav_msgs/msg/Odometry`) for each `opponent_id` in `eufs2..eufsN`

and uses TF (`/tf`, `/tf_static`) internally to map each odometry sample into `map`.

The dashboard publishes:

- `ns/cmd` (`ackermann_msgs/msg/AckermannDriveStamped`) — **authoritative control** for the 10s demo.

The dashboard calls services:

- `/pause_physics` (`std_srvs/srv/Empty`)
- `/unpause_physics` (`std_srvs/srv/Empty`)
- `/ros_can/set_mission` (`eufs_msgs/srv/SetCanState`) — mission/manual mode request (ego only).

### Ackermann → Twist bridge (AckermannCmdBridge)

File: `overlay/eufs_racecar/eufs_racecar/ackermann_cmd_bridge.py`

Per car (`ns`):

- Subscribes: `ns/cmd` (`ackermann_msgs/msg/AckermannDriveStamped`)
- Publishes: `ns/cmd_vel` (`geometry_msgs/msg/Twist`)

This is primarily used so the plant/tyre model can operate in a `cmd_vel`-shaped interface, and so
the dashboard can treat `cmd_vel` as telemetry.

### Tyre telemetry publisher (TyreStatePublisher)

File: `overlay/eufs_racecar/eufs_racecar/tyre_state_publisher.py`

Per car (`ns`):

- Subscribes:
  - `ns/odom` (`nav_msgs/msg/Odometry`)
  - `ns/cmd_vel` (`geometry_msgs/msg/Twist`)
- Publishes:
  - `ns/tyres/temps` (`std_msgs/msg/Float32MultiArray`)
  - `ns/tyres/wheel_rpm` (`std_msgs/msg/Float32MultiArray`)
  - `ns/tyres/degradation_rate` (`std_msgs/msg/Float32`)
  - `ns/tyres/lap_degradation` (`std_msgs/msg/Float32`)
  - `ns/tyres/life` (`std_msgs/msg/Float32`)

### map→odom TF publisher (OdomTfPublisher)

File: `overlay/eufs_racecar/eufs_racecar/odom_tf_publisher.py`

Per car (`ns`):

- Subscribes:
  - `/clock` (`rosgraph_msgs/msg/Clock`)
  - `ns/odom` (`nav_msgs/msg/Odometry`)
- Publishes:
  - TF frames on `/tf` so `map→(ns)odom→(ns)base_link` is available even when Gazebo is paused.

### Track markers (cones + optional boundary ribbons)

File: `overlay/eufs_racecar/eufs_racecar/track_marker_publisher.py`

- Publishes: `/track_markers` (`visualization_msgs/msg/MarkerArray`)

This reads the currently selected track’s SDF and emits RViz markers for cones. If the track’s
metadata/provenance opts in, it also publishes two `LINE_STRIP` markers for left/right boundaries.

### Forgez battery topics

Sources:

- `overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo`
- `overlay/gazebo_ros_battery/*` (battery plugin)

Per car (`ns`), the Forgez plugin provides:

- `ns/forgez/battery_state` (`sensor_msgs/msg/BatteryState`)
- `ns/forgez/charge_level_wh` (`std_msgs/msg/Float64`)
- `ns/forgez/deploy_power_w` (`std_msgs/msg/Float64`)
- `ns/forgez/signed_power_w` (`std_msgs/msg/Float64`)
- `ns/forgez/lap_energy_remaining_wh` (`std_msgs/msg/Float64`)
- `ns/forgez/derate_reason` (`std_msgs/msg/String`)

### Native vehicle plugin + CAN topics

URDF/SDF wiring is in `overlay/eufs_racecar/eufs_racecar/urdf/racecar.gazebo`.

Key point: ego and opponents are **not** symmetric for CAN routing:

- Ego uses global CAN topics and services: `/ros_can/*` and `/race_car_model/command_mode`
- Opponents are remapped to `/<car_id>/ros_can/*` and `/<car_id>/race_car_model/command_mode`

The dashboard reads mission state from `/ros_can/state` (ego) and can request manual mode via
`/ros_can/set_mission`.

## Map / track attachment

Files:

- `overlay/eufs_racecar/eufs_racecar/track_select.py` — discovery and `resolve_track(name)` contract.
- `overlay/eufs_racecar/eufs_racecar/grid_geometry.py` — `build_grid_poses(assets, count, ...)`.
- `overlay/eufs_racecar/launch/load_car.launch.py` — uses the resolved assets to spawn the grid.
- `overlay/eufs_tracks/` — map assets (world, model, csv, centerline, boundaries, provenance).

Maps are discovered via the `eufs_tracks` package layout:

```text
worlds/<name>.world
models/<name>/model.sdf
csv/<name>.csv
```

Optional map-owned metadata can be placed under `<name>/` or `models/<name>/`:

- `centerline.csv` (ordered arclength samples)
- `boundaries.csv` (ordered left/right boundary samples)
- `metadata.yaml` or `provenance.yaml` (spawn overrides, grid profile, hash pins, strip mesh)

## Tests

All tests live in `tests/` and are designed to run without ROS being installed on the host (pure
Python path injection). ROS message import tests require running inside the container image that
has ROS installed.

