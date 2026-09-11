# EUFS F1 Sim — Lean Container Slice

ROS 2 Humble + Gazebo Classic 11 simulation stack for EUFS tracks with an MIT racecar (`eufs_racecar`) and a Humble-ported [ctu-vras/gazebo_ros_battery](https://github.com/ctu-vras/gazebo_ros_battery) including Forgez `T_core`, `E_lap`, and `R_OT` modes (Harvest / Nominal / Attack).

## Locks

| Component | Version / source |
|-----------|------------------|
| EUFS sim | `gitlab.com/eufs/public/eufs_sim` tag `v2.1.0` |
| EUFS msgs | `gitlab.com/eufs/public/eufs_msgs` tag `v2.0.0` |
| Vehicle | `CihatAltiparmak/mit_racecar_gazebo_ros2` packaged as `eufs_racecar` |
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

```bash
xhost +local:
docker compose up
```

Default launch: EUFS `small_track` with the MIT racecar and Forgez battery (Nominal mode).

### Other launches

```bash
docker compose run --rm eufs-f1-sim bash -lc \
  'ros2 launch eufs_tracks skidpad.launch.py gazebo_gui:=true show_rqt_gui:=false vehicleModelConfig:=configDry.yaml'

docker compose run --rm eufs-f1-sim bash -lc \
  'ros2 launch eufs_tracks small_track.launch.py forgez_mode:=Attack gazebo_gui:=true show_rqt_gui:=false vehicleModelConfig:=configDry.yaml'
```

Forgez modes and parameters: `overlay/eufs_racecar/config/forgez_battery.yaml`.

### Battery topics

- `/eufs/forgez/battery_state` (`sensor_msgs/BatteryState`)
- `/eufs/forgez/charge_level_wh` (`std_msgs/Float64`)

## Layout

```
eufs-f1-sim/
  Dockerfile
  docker-compose.yml
  scripts/          prepare-workspace.sh, entrypoint.sh
  overlay/
    eufs_racecar/   MIT racecar as EUFS racecar package
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
ros2 launch eufs_tracks small_track.launch.py gazebo_gui:=true show_rqt_gui:=false vehicleModelConfig:=configDry.yaml
```

Requires `ros-humble-gazebo-ros-pkgs` (Gazebo Classic 11) on the host.
