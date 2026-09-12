#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/eufs_ws/install/setup.bash
set -u

export EUFS_MASTER=/opt/eufs_ws

# setup.py once installed meshes under share/eufs_racecar/eufs_racecar/meshes;
# URDF/xacro expect share/eufs_racecar/meshes.
_wrong_meshes="${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar/eufs_racecar/meshes"
_right_meshes="${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar/meshes"
if [[ -d "${_wrong_meshes}" ]]; then
  mkdir -p "${_right_meshes}"
  cp -rn "${_wrong_meshes}/." "${_right_meshes}/" 2>/dev/null || true
fi

_world="${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/worlds/small_track.world"
if [[ -f "${_world}" ]]; then
  # Frame the default car spawn at (-13, 10.3) while retaining enough track
  # context for the Gazebo demo camera.
  sed -i "s|-40 40 20 0 0.667643 -1|-15 18 12 0 0.55 -1.35|g" "${_world}" || true
fi

_f1_mat="${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar/materials/scripts"
# Keep the package share root in the model search path as well as its model
# directory.  Cone meshes are packaged below share/eufs_tracks/meshes and are
# referenced by model://eufs_tracks/meshes/... from the cone SDFs.
export GAZEBO_MODEL_PATH="${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/models:${EUFS_MASTER}/install/eufs_tracks/share:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MATERIAL_PATH="${_f1_mat}:/usr/share/gazebo-11/media/materials/scripts:${GAZEBO_MATERIAL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${EUFS_MASTER}/install/eufs_sensors/share/eufs_sensors/meshes:${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/meshes:${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/materials:${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar:${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar/materials:${_f1_mat}:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="${EUFS_MASTER}/install/eufs_plugins/lib:${EUFS_MASTER}/install/gazebo_ros_battery/lib:${GAZEBO_PLUGIN_PATH:-}"
# Empty URI: do not query models.gazebosim.org (that freezes gzclient on splash).
export GAZEBO_MODEL_DATABASE_URI=""
export LIBGL_DRI3_DISABLE=1

mkdir -p /root/.gazebo
if [[ ! -f /root/.gazebo/gui.ini ]]; then
  printf '[geometry]\nx=0\ny=0\n' > /root/.gazebo/gui.ini
fi

# Allow container GUI apps on the host X server (host must run `xhost +local:` once per session).
export DISPLAY="${DISPLAY:-:0}"

exec "$@"
