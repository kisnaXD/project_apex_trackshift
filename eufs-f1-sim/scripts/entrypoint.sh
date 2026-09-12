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
  sed -i "s|-40 40 20 0 0.667643 -1|-22 8 6 0 0.45 0.55|g" "${_world}" || true
  # Full-white ambient washes EUFSF1/Silver to the same white as an unbound STL.
  sed -i "s|<ambient>1.0 1.0 1.0 1.0</ambient>|<ambient>0.40 0.40 0.43 1.0</ambient>|g" "${_world}" || true
fi

_f1_mat="${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar/materials/scripts"
# URDF→SDF hardcodes file://media/materials/scripts/gazebo.material. Put
# EUFSF1/Silver and EUFSF1/TireBlack in that file so gzclient actually paints.
_gz_mat="/usr/share/gazebo-11/media/materials/scripts/gazebo.material"
if [[ -f "${_f1_mat}/eufs_f1.material" && -f "${_gz_mat}" ]] && ! grep -q 'material EUFSF1/Silver' "${_gz_mat}"; then
  cat "${_f1_mat}/eufs_f1.material" >> "${_gz_mat}"
fi
export GAZEBO_MODEL_PATH="${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MATERIAL_PATH="${_f1_mat}:/usr/share/gazebo-11/media/materials/scripts:${GAZEBO_MATERIAL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${EUFS_MASTER}/install/eufs_sensors/share/eufs_sensors/meshes:${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/meshes:${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/materials:${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar:${EUFS_MASTER}/install/eufs_racecar/share/eufs_racecar/materials:${_f1_mat}:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="${EUFS_MASTER}/install/eufs_plugins/lib:${EUFS_MASTER}/install/gazebo_ros_battery/lib:${GAZEBO_PLUGIN_PATH:-}"

# Allow container GUI apps on the host X server (host must run `xhost +local:` once per session).
export DISPLAY="${DISPLAY:-:0}"

exec "$@"
