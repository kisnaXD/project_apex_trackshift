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
| `overlay/eufs_racecar/launch/load_car.launch.py` | **The** launch: Gazebo + RViz + one spawn |

## Start (single command)

```bash
cd eufs-f1-sim
xhost +local:
export DISPLAY=:0
sg docker -c './scripts/start-stack.sh'
```

That is the only start path. Compose CMD is:

`ros2 launch eufs_racecar load_car.launch.py gazebo_gui:=true show_rqt_gui:=true rviz:=true`

| GUI | Process | Purpose |
|-----|---------|---------|
| Gazebo | `gzserver` + `gzclient` | small_track world + one `eufs` model |
| RViz2 | `rviz2 -d eufs_f1.rviz` | RobotModel on `/eufs/robot_description` |
| rqt | `rqt_gui` | EUFS Robot Steering + Mission Control |

- **Service name:** `eufs-f1-sim`
- **Container name:** `eufs-f1-sim`
- **Image:** `eufs-f1-sim:lean`

## Verify

```bash
docker compose ps
docker exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui'"
docker exec eufs-f1-sim bash -lc "gz model -m eufs -p"
docker exec eufs-f1-sim bash -lc "source /opt/ros/humble/setup.bash; source /opt/eufs_ws/install/setup.bash; ros2 node list; ros2 topic list"
```

Expect one `eufs` model on the track, one `robot_state_publisher`, RobotModel OK in RViz (chase `base_link`), and one topic graph (`/clock`, `/tf`, `/eufs/odom`, `/eufs/cmd_vel`, `/eufs/robot_description`).

In **gzclient** the chassis and wings should read **silver** (RViz `0.75 0.75 0.78`) and the tires **black**. Classic binds STL color from `EUFSF1/Silver` / `EUFSF1/TireBlack` in `gazebo.material` (entrypoint appends `eufs_f1.material`) plus the same RGBA on each visual. Scene ambient is `0.40` so silver does not wash to white.

## Stop

```bash
cd eufs-f1-sim
docker compose down
```

Do **not** run `docker system prune`, disk wipes, or unbounded Docker operations as part of normal use.

## RViz

Launch always loads `overlay/eufs_racecar/config/eufs_f1.rviz` with `rviz2 -d` and `use_sim_time:=true`.

| Display | Topic / frame |
|---------|----------------|
| Fixed frame | `map` |
| View target | `base_link` (third-person Orbit view) |
| RobotModel | `/eufs/robot_description` (Transient Local) |
| TF | all frames |
| Track | `/track_markers` (71 transient-local cone markers) |
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
ros2 launch eufs_racecar load_car.launch.py gazebo_gui:=true show_rqt_gui:=true rviz:=true
```

## Version locks

| Component | Source |
|-----------|--------|
| EUFS sim | `gitlab.com/eufs/public/eufs_sim` tag `v2.1.0` |
| EUFS msgs | `gitlab.com/eufs/public/eufs_msgs` tag `v2.0.0` |
| Battery | `ctu-vras/gazebo_ros_battery` (Humble port in overlay) |
| Platform | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 |
