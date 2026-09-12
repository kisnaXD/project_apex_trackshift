# EUFS F1 Sim — Runbook

## Inspect (why RViz had the car and Gazebo did not)

1. RViz RobotModel is fed by `robot_state_publisher` on `/eufs/robot_description` (one xacro, `package://eufs_racecar/meshes/*.STL`). That topic is enough for RViz; it does not prove Gazebo spawned anything.
2. Docker CMD used a second launch: `eufs_tracks/small_track.launch` starts `gazebo.launch.py` then includes `load_car`. Two files, two env graphs.
3. That XML `set_env` **replaces** `GAZEBO_PLUGIN_PATH` with `/opt/ros/humble/lib` + `install/eufs_plugins` (no `/lib`, no `gazebo_ros_battery`). Forgez/energy-gate plugins fail; `spawn_entity` dies; Gazebo stays empty.
4. The same `set_env` replaces `GAZEBO_MODEL_PATH` with `eufs_tracks/models` only, so `model://eufs_tracks/meshes/...` cones miss and gzclient can stick on splash / never show the car.
5. `load_car` already published RSP + JSP (`use_sim_time`) and spawned from the namespaced topic, but it did **not** start gzserver/gzclient. RViz and Gazebo were not one process tree.
6. Meshes are binary STL; RViz can load them. Gazebo never got a successful spawn of that same URDF.
7. `gz model -l` vs `/eufs/robot_description` vs RViz RobotModel were therefore different: description present, model absent.
8. `/clock` `/tf` `/eufs/odom` `/eufs/cmd_vel` must be one graph with `use_sim_time:=true` on RViz and RSP.
9. Fix: **one** launch (`eufs_racecar/load_car.launch.py`) starts gzserver+gzclient, writes one URDF, publishes it, spawns that file once, RSP+JSP, RViz `-d` eufs_f1.rviz, rqt.
10. Do not start `eufs_tracks/*.launch` or `eufs_launcher` for this demo — they are a second stack.

## Prerequisites (host)

- Ubuntu 22.04
- Docker Engine + Compose plugin
- Working X11 display (allow local Docker access once per session):

```bash
xhost +local:
```

## Layout

| Path | Purpose |
|------|---------|
| `docker-compose.yml` | Single service `eufs-f1-sim` |
| `Dockerfile` | Image `eufs-f1-sim:lean` |
| `scripts/entrypoint.sh` | ROS/Gazebo env setup |
| `scripts/start-stack.sh` | Host helper: xhost + build + up + GUI check |
| `scripts/prepare-workspace.sh` | Populates `ws/src` at build time |
| `scripts/step_to_urdf_ocp.py` | GrabCAD STEP → grouped STL meshes + URDF joints |
| `overlay/` | F1 meshes, battery plugin, patches |
| `overlay/eufs_racecar/launch/load_car.launch.py` | **The** launch: Gazebo + map + start dashboard + RViz + one spawn |
| `overlay/eufs_racecar/eufs_racecar/start_dashboard.py` | Stock PyQt5 Track / Cars / Start / Stop window |

## Start (single command)

```bash
cd eufs-f1-sim
xhost +local:
export DISPLAY=:0
sg docker -c './scripts/start-stack.sh'
```

That is the only start path. It builds and starts the `eufs-f1-sim` compose service. Compose CMD is:

`ros2 launch eufs_racecar load_car.launch.py gazebo_gui:=false show_rqt_gui:=false rviz:=false track:=cota num_cars:=1`

On this start **only** the stock PyQt **EUFS F1 Start** window (~360x160, Track / Cars / Start / Stop) should appear. Gazebo (`gzclient`) and RViz do **not** open until you click **Start**. `load_car.launch.py` is the only launch: headless `gzserver` on `cota.world` (paused), one spawn, `track_marker_publisher`, and the dashboard Node. Do not start `eufs_tracks/small_track.launch`, `eufs_launcher`, or any second Gazebo stack.

## Track args

| Arg | Values | Default |
|-----|--------|---------|
| `track` | `cota` (Circuit of The Americas Grand Prix layout) or `small_track` | `cota` |
| `num_cars` | `1` | `1` |

```bash
ros2 launch eufs_racecar load_car.launch.py track:=cota num_cars:=1
ros2 launch eufs_racecar load_car.launch.py track:=small_track num_cars:=1
```

`track:=cota` loads `eufs_tracks` world `cota.world` and model `models/cota/model.sdf`. The four `big_orange` cones sit on the orange start/finish line; the car spawns with its front bumper behind that gate. Missing COTA files or hash mismatch fail the launch (no fallback to `small_track`).

| GUI | Process | When |
|-----|---------|------|
| Start window | `start_dashboard` | On compose up. Track `cota` / `small_track`, Cars=1, Start/Stop |
| Gazebo client | `gzclient` | After **Start**. Same `gzserver` / `cota.world` / one `eufs` model |
| RViz2 | `rviz2 -d eufs_f1.rviz` | After **Start**. Fixed frame `map`, RobotModel `/eufs/robot_description`, `/track_markers` |

**Start** opens `gzclient` + RViz on the existing paused `gzserver` (same `DISPLAY`) and calls `/unpause_physics`. **Stop** publishes a zero `/eufs/cmd_vel` and calls `/pause_physics`. Neither button starts a second Gazebo, `eufs_tracks/*.launch`, or `eufs_launcher`.

- **Service name:** `eufs-f1-sim`
- **Container name:** `eufs-f1-sim`
- **Image:** `eufs-f1-sim:lean`

## Dashboard

`load_car.launch.py` starts a stock PyQt5 window as the `start_dashboard` Node on the **same** launch as `track:=cota` (not a second process tree). It uses the container `DISPLAY` (compose passes `${DISPLAY:-:0}` and `/tmp/.X11-unix`). Widgets: Track (`cota` / `small_track`), Cars=1, Start, Stop. No extra styling. Size is about 360x160.

Humble launch YAML stringifies integers. `cars` is declared and passed as a **string** (`'1'`), then coerced with `int()` in the node. An INTEGER default with a string override kills the process before `window.show()`.

Start opens `gzclient` + `rviz2 -d eufs_f1.rviz` (`use_sim_time:=true`, fixed frame `map`) against the gzserver this launch already started, then `/unpause_physics`. Stop sends a zero `/eufs/cmd_vel` and `/pause_physics`. The buttons do not start `gzserver`, `eufs_tracks/*.launch`, or `eufs_launcher`.

## Verify

```bash
docker compose ps
docker exec eufs-f1-sim bash -lc 'echo DISPLAY=$DISPLAY'
docker exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui|start_dashboard'"
DISPLAY="${DISPLAY:-:0}" xwininfo -root -tree | grep -E 'EUFS F1 Start|Gazebo|RViz'
docker exec eufs-f1-sim bash -lc "gz model -m eufs -p"
```

After compose up, expect **only** `start_dashboard` / **EUFS F1 Start** on `DISPLAY` — no `gzclient`, no `rviz2`. Headless `gzserver` is on `cota.world` with one `eufs` model (`timeout 6 gz model -m eufs -p` near `-4.2 3.3`). After **Start**, expect `gzclient` + `rviz2` on the same `DISPLAY`, RobotModel on `/eufs/robot_description`, cones on `/track_markers` (fixed frame `map`), and one topic graph (`/clock`, `/tf`, `/eufs/odom`, `/eufs/cmd_vel`). RViz empty while Gazebo is live usually means the config still points at `/robot_description` or fixed frame `base_link` without the namespaced car TF — not a second Gazebo.

`load_car` sets `GAZEBO_MODEL_DATABASE_URI` empty and spawns the car with `file://` STLs. Do not point gzclient at models.gazebosim.org — that is what pins the orange "Preparing your world" splash. This Gazebo Classic `gz model` has no `-l` list flag; `timeout 6 gz model -m eufs -p` is enough.

In **gzclient** the chassis and wings should read **silver** (RViz `0.75 0.75 0.78`) and the tires **black**. Classic binds STL color from `EUFSF1/Silver` / `EUFSF1/TireBlack` in `gazebo.material` (entrypoint appends `eufs_f1.material`) plus the same RGBA on each visual. Scene ambient is `0.40` so silver does not wash to white.

## Shutdown

The dashboard Stop button pauses physics only. To tear down the container and all GUIs:

```bash
cd eufs-f1-sim
docker compose down
```

Do **not** run `docker system prune`, disk wipes, or unbounded Docker operations as part of normal use.

## RViz

The dashboard Start button loads `overlay/eufs_racecar/config/eufs_f1.rviz` with `rviz2 -d` and `use_sim_time:=true`. Fixed frame is `map` (same frame as `/track_markers` and `map`→`odom`→`base_link`). RobotModel reads `/eufs/robot_description`, not `/robot_description`.

| Display | Topic / frame |
|---------|----------------|
| Fixed frame | `map` |
| View target | `base_link` (third-person Orbit view) |
| RobotModel | `/eufs/robot_description` (Transient Local) |
| TF | all frames |
| Track | `/track_markers` (COTA EUFS cones from the same SDF Gazebo loaded) |
| Odometry | `/eufs/odom` |

## Driving

```bash
docker exec eufs-f1-sim bash -lc 'source /opt/ros/humble/setup.bash; source /opt/eufs_ws/install/setup.bash; ros2 topic pub -r 10 /eufs/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"'
```

Stop with Ctrl-C, then one zero command on `/eufs/cmd_vel`.

## F1 mesh from GrabCAD STEP

The GrabCAD STEP is **Y-up**. `scripts/step_to_urdf_ocp.py` maps CAD → ROS as `X=-Z`, `Y=-X`, `Z=+Y`. Meshes are **binary STL** at `package://eufs_racecar/meshes/...` so both RViz and Gazebo load the same files.

```bash
cd eufs-f1-sim
python3 scripts/step_to_urdf_ocp.py --step "/home/gera/Downloads/Assem step.STEP"
sg docker -c './scripts/start-stack.sh'
```

## Battery topics

- `/eufs/forgez/battery_state` (`sensor_msgs/BatteryState`)
- `/eufs/forgez/charge_level_wh` (`std_msgs/Float64`)

Forgez modes: `overlay/eufs_racecar/config/forgez_battery.yaml` (`forgez_mode:=Harvest|Nominal|Attack` on the same launch).

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
ros2 launch eufs_racecar load_car.launch.py track:=cota num_cars:=1
```

## Version locks

| Component | Source |
|-----------|--------|
| EUFS sim | `gitlab.com/eufs/public/eufs_sim` tag `v2.1.0` |
| EUFS msgs | `gitlab.com/eufs/public/eufs_msgs` tag `v2.0.0` |
| Battery | `ctu-vras/gazebo_ros_battery` (Humble port in overlay) |
| Platform | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 |
