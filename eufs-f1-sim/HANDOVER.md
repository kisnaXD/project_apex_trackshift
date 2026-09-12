# EUFS F1 handover

This checkout is the EUFS F1 demo on ROS 2 Humble with Gazebo Classic 11. The
current default map is COTA. The source is under
`/home/gera/Desktop/EUFS_FORK_CONTAINERIZED/eufs-f1-sim`; the handoff point is
branch `feat/modular-grid-telemetry`, commit `7c08f72`. Changes are local until authenticated GitHub push is
performed; do not assume this commit is published.

## Starting and operating it

The normal host entrypoint is:

```bash
cd /home/gera/Desktop/EUFS_FORK_CONTAINERIZED/eufs-f1-sim
xhost +local:
DISPLAY=:0 TRACK=cota CARS=1 ./scripts/start-stack.sh
```

`start-stack.sh` reuses or starts the canonical `eufs-f1-sim` container by
default. Use `./scripts/start-stack.sh --build` when an image rebuild is
needed. It preserves a replaced canonical container as
`eufs-f1-sim-before-grid-<timestamp>` and its image as a rollback tag; see
[`RUN_GRID_TELEMETRY.md`](RUN_GRID_TELEMETRY.md). The managed dashboard is the
container command. Its Start button attaches Gazebo and RViz after spawning,
Stop pauses physics, and Shutdown closes the owned backend and viewers while
leaving the GUI ready to Start again.

The Cars selector accepts 1–20. Cars are placed in a two-column staggered grid
using map geometry when available; the one-car spawn remains the authored
single-car pose. Start with a changed map or car count restarts the managed
backend and auto-starts when ready, so it does not require a second click.

For a headless ROS launch inside a sourced ROS/container shell, use one
backend and disable the dashboard when another dashboard is not wanted:

```bash
ros2 launch eufs_racecar load_car.launch.py \
  track:=cota cars:=4 show_dashboard:=false
```

This launch starts paused. To release physics, use
`ros2 service call /unpause_physics std_srvs/srv/Empty '{}'`.

Do not run this command alongside the managed dashboard unless the existing
dashboard is intentionally disabled; two backends on the same default graph
will collide. The defaults are Gazebo master `http://127.0.0.1:11345` and ROS
domain `0` (one backend per default host session).

## Cars, namespaces, and commands

| Car | Gazebo entity / ROS namespace | Main command topic |
|---|---|---|
| 1 | `eufs` | `/eufs/cmd` |
| 2 | `eufs2` | `/eufs2/cmd` |
| 3 | `eufs3` | `/eufs3/cmd` |
| 4 | `eufs4` | `/eufs4/cmd` |

Here `eufsN` means `eufs2` through `eufs20`; the ego car is `eufs`. `/eufsN/cmd` is
`ackermann_msgs/msg/AckermannDriveStamped`; the native DynamicBicycle path uses
acceleration and steering angle in radians. `/eufsN/cmd_vel` is a
`geometry_msgs/msg/Twist` bridge/telemetry path and is not the dashboard's
authoritative control interface. The dashboard publishes Ackermann commands on
`/eufs/cmd`.

Per-car topics to verify are `/eufsN/odom` (`nav_msgs/msg/Odometry`),
`/eufsN/joint_states` (`sensor_msgs/msg/JointState`), and
`/eufsN/robot_description` (`std_msgs/msg/String`, transient-local
durability). The authored telemetry publishers also expose:

```text
/eufsN/forgez/battery_state          sensor_msgs/msg/BatteryState
/eufsN/forgez/charge_level_wh        std_msgs/msg/Float64
/eufsN/forgez/deploy_power_w         std_msgs/msg/Float64
/eufsN/forgez/signed_power_w         std_msgs/msg/Float64
/eufsN/forgez/lap_energy_remaining_wh std_msgs/msg/Float64
/eufsN/forgez/derate_reason          std_msgs/msg/String
/eufsN/tyres/temps                    std_msgs/msg/Float32MultiArray
/eufsN/tyres/wheel_rpm                std_msgs/msg/Float32MultiArray
/eufsN/tyres/degradation_rate         std_msgs/msg/Float32
/eufsN/tyres/lap_degradation          std_msgs/msg/Float32
/eufsN/tyres/life                     std_msgs/msg/Float32
```

Shared topics are `/clock`, `/tf`, `/tf_static`, and `/track_markers`. The ego
frame chain is `map` → `odom` → `base_link`; an opponent uses
`map` → `eufsN/odom` → `eufsN/base_link`. Ego CAN topics are global under
`/ros_can/*`; opponent CAN topics are under `/eufsN/ros_can/*`. The command
mode query is `/race_car_model/command_mode` for ego and
`/eufsN/race_car_model/command_mode` for opponents. Native launch defaults are
`vehicleModel:=DynamicBicycle` and `commandMode:=acceleration`, with
acceleration in m/s² and steering in radians. Mission readiness uses
`/ros_can/set_mission` (`SetCanState.Request.ami_state = CanState.AMI_MANUAL`)
or an observed `AS_DRIVING` state. These are routing/readiness contracts; they
do not define a control algorithm.

## Telemetry and extension points

The typed public frame is implemented in
`overlay/eufs_racecar/eufs_racecar/telemetry_schema.py` and assembled by
`start_dashboard.py`. `TelemetryFrame` is an in-process
`StartDashboard.telemetry_frame().to_dict()` result, not an aggregate ROS
topic. It covers session/lap state, Frenet `s/y`, world pose,
speed, longitudinal/lateral acceleration, yaw rate, ERS/pack values, tire
channels, and opponent gaps. `None` means no authoritative source and is
serialized as JSON `null`; zero remains a real measurement. Tire temperatures,
degradation, and some battery values are model estimates or optional sources.
Unsupported fields such as SOH, MGUK power, lap deployment, carcass
temperature, fuel, DRS, and gear remain unavailable rather than being filled
with phantom values.

Map attachment and grid placement are in
`track_select.py`, `grid_geometry.py`, and `load_car.launch.py`. A resolved map
provides world/model/CSV assets, spawn, optional `centerline.csv`,
`boundaries.csv`, and optional boundary strips. `resolve_track()` and
`build_grid_poses()` use that metadata; without centerline metadata the grid
falls back to a spawn-relative tangent layout, and without boundaries curved
track validation is unavailable. The attachment format is documented in
[`GRID_TELEMETRY_PLAN.md`](GRID_TELEMETRY_PLAN.md). RViz grid generation is in
`rviz_grid.py`.

The optional `scripts/drive_straight_10s.py`, `scripts/cota_lap.py`, and their
shell helpers are bounded demonstration helpers. The COTA lap helper is
COTA-specific and is not a general controller.

Forgez battery and tire topics are optional and appear only when relevant
plugins publish them; the native path does not guarantee every optional field.
Known validation includes live 4-car COTA and live 2-car `small_track`, plus
geometry checks for 1, 4, and 20 cars, stock geometry, and the native wheel
path. The current testing work is external to this handoff; do
not start or alter the runtime while another operator is testing it.
