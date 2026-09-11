#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /opt/eufs_ws/install/setup.bash

export EUFS_MASTER=/opt/eufs_ws
export GAZEBO_MODEL_PATH="${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${EUFS_MASTER}/install/eufs_sensors/share/eufs_sensors/meshes:${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/meshes:${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/materials:${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="${EUFS_MASTER}/install/eufs_plugins/lib:${EUFS_MASTER}/install/gazebo_ros_battery/lib:${GAZEBO_PLUGIN_PATH:-}"

exec "$@"
