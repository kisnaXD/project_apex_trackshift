# EUFS F1 Sim — Runbook

Lean ROS 2 Humble + Gazebo Classic 11 container for EUFS tracks with the Mercedes F1 visual and Forgez battery plugin.

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

## Build (only when image is missing or sources changed)

```bash
cd eufs-f1-sim
docker compose build
```

## Start (default: small_track + Gazebo + RViz + rqt manual control)

```bash
cd eufs-f1-sim
xhost +local:
export DISPLAY=:0
sg docker -c './scripts/start-stack.sh'
```

Or manually:

```bash
cd eufs-f1-sim
xhost +local:
sg docker -c 'docker compose up -d'
```

- **Service name:** `eufs-f1-sim`
- **Container name:** `eufs-f1-sim`
- **Image:** `eufs-f1-sim:lean`
- **Default CMD:** `ros2 launch eufs_tracks small_track.launch gazebo_gui:=true show_rqt_gui:=true rviz:=true vehicleModelConfig:=configDry.yaml`

Every stack start launches three GUIs:

| GUI | Process | Purpose |
|-----|---------|---------|
| Gazebo | `gzclient` | 3D sim + F1 car visual |
| RViz2 | `rviz2` | Laser/TF visualization |
| rqt | `rqt_gui` | EUFS Robot Steering + Mission Control |

### If Gazebo GUI (`gzclient`) does not appear

```bash
cd eufs-f1-sim
xhost +local:
docker compose restart
```

Or launch the client manually inside the running container:

```bash
docker exec -e DISPLAY=$DISPLAY eufs-f1-sim bash -lc "gzclient"
```

## Verify

```bash
docker compose ps
docker compose logs -f --tail=50
docker exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui'"
```

Expect `eufs-f1-sim` **Up** and all three GUI processes running (`gzclient`, `rviz2`, and `python3 ... rqt_gui`). Meshes are **binary STL** so RViz2 can load the RobotModel (ASCII STL is rejected).

## Stop

```bash
cd eufs-f1-sim
docker compose down
```

Do **not** run `docker system prune`, disk wipes, or unbounded Docker operations as part of normal use.

## RViz

Launch always loads `overlay/eufs_racecar/config/eufs_f1.rviz` (not `~/.rviz2/default.rviz`).

| Display | Topic / frame |
|---------|----------------|
| Fixed frame | `odom` (ackermann plugin publishes `odom` → `base_link`) |
| RobotModel | `/robot_description` |
| TF | all frames |
| Grid | XY in `odom` |
| LaserScan | `/scan` (Humble remaps `gazebo_ros_ray_sensor` `~/out` → `/scan`) |
| Odometry | `/odom` |

Do not use `base_footprint` — this racecar URDF has `base_link` only.

## F1 mesh from GrabCAD STEP

The GrabCAD STEP is **Y-up**. `scripts/step_to_urdf_ocp.py` maps CAD → ROS as `X=-Z`, `Y=-X`, `Z=+Y` (proper rotation, +Z up) so wheels sit on the ground under the body. Regenerating STLs updates visuals, wheel collisions, and lidar/camera height together — do not apply a visual-only RPY flip.

Each corner wheel mesh is the GrabCAD `WHEEL` instance only (tire + rim). Leftover `Formula 1` body solids at the corners — uprights, wishbones, and brake ducts — are dropped so nothing pokes out of the rubber. Front/rear wings are already part of the chassis body solid.

**Colors (Gazebo + RViz):** chassis and wings are metallic silver (`ambient`/`diffuse` `0.75 0.75 0.78`, high specular). Tires are black (`0.05 0.05 0.05`). Rims share the tire mesh (dark). There is no default-white STL and no leftover `Gazebo/Red` on the body.

```bash
cd eufs-f1-sim
python3 scripts/step_to_urdf_ocp.py --step "/home/gera/Downloads/Assem step.STEP"
sg docker -c 'docker compose build && docker compose up -d --force-recreate'
```

## Alternate launches

One-off runs without changing the default service CMD:

```bash
docker compose run --rm eufs-f1-sim bash -lc \
  'ros2 launch eufs_tracks skidpad.launch gazebo_gui:=true show_rqt_gui:=true rviz:=true vehicleModelConfig:=configDry.yaml'

docker compose run --rm eufs-f1-sim bash -lc \
  'ros2 launch eufs_tracks small_track.launch forgez_mode:=Attack gazebo_gui:=true show_rqt_gui:=true rviz:=true vehicleModelConfig:=configDry.yaml'
```

Forgez battery modes and params: `overlay/eufs_racecar/config/forgez_battery.yaml`.

## Battery topics

- `/eufs/forgez/battery_state` (`sensor_msgs/BatteryState`)
- `/eufs/forgez/charge_level_wh` (`std_msgs/Float64`)

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
ros2 launch eufs_tracks small_track.launch gazebo_gui:=true show_rqt_gui:=true rviz:=true vehicleModelConfig:=configDry.yaml
```

## Version locks

| Component | Source |
|-----------|--------|
| EUFS sim | `gitlab.com/eufs/public/eufs_sim` tag `v2.1.0` |
| EUFS msgs | `gitlab.com/eufs/public/eufs_msgs` tag `v2.0.0` |
| Battery | `ctu-vras/gazebo_ros_battery` (Humble port in overlay) |
| Platform | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 |
