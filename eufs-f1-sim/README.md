# EUFS F1 Sim — Lean Container Slice

ROS 2 Humble + Gazebo Classic 11 simulation stack for EUFS tracks with a GrabCAD Mercedes F1 concept visual on `eufs_racecar` and a Humble-ported [ctu-vras/gazebo_ros_battery](https://github.com/ctu-vras/gazebo_ros_battery) including Forgez `T_core`, `E_lap`, and `R_OT` modes (Harvest / Nominal / Attack).

## F1 vehicle mesh (source + license)

1. Model: Mercedes-AMG Petronas F1 Concept 2 (GrabCAD community CAD).
2. URL: https://grabcad.com/library/mercedes-amg-petronas-f1-concept-2
3. STEP file: `Assem step.STEP` (from user Downloads).
4. License: GrabCAD Community terms — educational/concept model; not official team CAD.
5. Conversion tool: **cadquery-ocp** (Open CASCADE 7.x) — `STEPControl` read + `StlAPI` mesh export via `scripts/step_to_urdf_ocp.py`.
6. Per-link visuals: chassis body, front wing, rear wing, four wheels (~5.1 m length, 3.28 m wheelbase).
7. Meshes install flat to `share/eufs_racecar/meshes/`; URDF in `share/eufs_racecar/urdf/`.
8. Ackermann drive and Forgez battery plugins; perception sensors are omitted from the demo so the car and track remain the focus.
9. Regenerate: `python3 scripts/step_to_urdf_ocp.py --step "/path/to/Assem step.STEP"` (needs `cadquery-ocp`).
10. Attribution: `overlay/eufs_racecar/eufs_racecar/meshes/F1_MODEL_SOURCE.txt`.

## Locks

| Component | Version / source |
|-----------|------------------|
| EUFS sim | `gitlab.com/eufs/public/eufs_sim` tag `v2.1.0` |
| EUFS msgs | `gitlab.com/eufs/public/eufs_msgs` tag `v2.0.0` |
| Vehicle | GrabCAD Mercedes F1 STEP + `eufs_racecar` ackermann stack |
| Battery | `ctu-vras/gazebo_ros_battery` (Humble port in `overlay/gazebo_ros_battery`) |
| Platform | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 |

Excluded by design: ADMM, DRL, AURIX, extra BMS stacks, extra services.

## Prerequisites (host)

- Ubuntu 22.04
- Docker Engine + Compose plugin (or use the commands below after installing Docker)
- For GUI: working X11 (`xhost +local:` once per session)

## Build

```bash
cd eufs-f1-sim
docker compose build
```

## Run

Single start path (dashboard only until you click Start):

```bash
cd eufs-f1-sim
xhost +local:
export DISPLAY=:0
sg docker -c './scripts/start-stack.sh'
```

Compose CMD is `ros2 launch eufs_racecar load_car.launch.py` with
`track:=cota cars:=1`. That one file starts headless gzserver, the track map,
and the stock PyQt5 start window, then publishes `/eufs/robot_description` and
spawns that URDF once. Click **Start** to open gzclient + RViz (`eufs_f1.rviz`,
fixed frame `map`). Do not start `eufs_tracks/*.launch` alongside it.
Shutdown: `docker compose down`.

The demo publishes 71 stock RViz cone markers on `/track_markers`, uses `map` as
the fixed frame, and publishes the car description on `/eufs/robot_description`.
Gazebo odometry is already in world coordinates, so `map -> odom` is identity.
Drive with a `geometry_msgs/Twist` on `/eufs/cmd_vel`; send a zero command when
releasing control.

Forgez modes and parameters: `overlay/eufs_racecar/config/forgez_battery.yaml`.

### Battery topics

- `/eufs/forgez/battery_state` (`sensor_msgs/BatteryState`)
- `/eufs/forgez/charge_level_wh` (`std_msgs/Float64`)

## Layout

```
eufs-f1-sim/
  Dockerfile
  docker-compose.yml
  scripts/          prepare-workspace.sh, entrypoint.sh, step_to_urdf_ocp.py
  overlay/
    eufs_racecar/   F1 visual meshes + EUFS racecar URDF/plugins
    gazebo_ros_battery/  Humble battery plugin + Forgez params
  ws/src/           populated at build time by prepare-workspace.sh
```

## Native build (without Docker)

```bash
./scripts/prepare-workspace.sh
cd ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export EUFS_MASTER=$PWD
xhost +local:
ros2 launch eufs_racecar load_car.launch.py track:=cota cars:=1
```

Requires `ros-humble-gazebo-ros-pkgs` (Gazebo Classic 11) on the host.
