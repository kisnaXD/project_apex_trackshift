#!/usr/bin/env bash
# Assemble colcon workspace from locked sources + overlays.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${EUFS_MASTER:-${ROOT}/ws}"
SRC="${WS}/src"

mkdir -p "${SRC}"

clone_if_missing() {
  local dest="$1" url="$2" ref="$3"
  if [[ ! -d "${dest}/.git" ]]; then
    git clone --depth 1 --branch "${ref}" "${url}" "${dest}"
  fi
}

clone_if_missing "${SRC}/eufs_sim" "https://gitlab.com/eufs/public/eufs_sim.git" "v2.1.0"
clone_if_missing "${SRC}/eufs_msgs" "https://gitlab.com/eufs/public/eufs_msgs.git" "v2.0.0"

rm -rf "${SRC}/eufs_sim/eufs_racecar"
rm -rf "${SRC}/eufs_racecar" "${SRC}/gazebo_ros_battery"
cp -a "${ROOT}/overlay/eufs_racecar" "${SRC}/eufs_racecar"
cp -a "${ROOT}/overlay/gazebo_ros_battery" "${SRC}/gazebo_ros_battery"
cp "${ROOT}/overlay/patches/eufs_plugins_CMakeLists.txt" "${SRC}/eufs_sim/eufs_plugins/CMakeLists.txt"
sed -i "s|-40 40 20 0 0.667643 -1|-22 8 6 0 0.45 0.55|g" \
  "${SRC}/eufs_sim/eufs_tracks/worlds/small_track.world" || true

# COTA Grand Prix cone track, selected by load_car.launch.py track:=cota
_COTA_OVERLAY="${ROOT}/overlay/eufs_tracks"
_TRACKS="${SRC}/eufs_sim/eufs_tracks"
if [[ -d "${_COTA_OVERLAY}" ]]; then
  mkdir -p "${_TRACKS}/csv" "${_TRACKS}/worlds" "${_TRACKS}/models/cota" "${_TRACKS}/cota"
  cp -a "${_COTA_OVERLAY}/csv/." "${_TRACKS}/csv/" 2>/dev/null || true
  cp -a "${_COTA_OVERLAY}/worlds/." "${_TRACKS}/worlds/" 2>/dev/null || true
  cp -a "${_COTA_OVERLAY}/models/." "${_TRACKS}/models/" 2>/dev/null || true
  if [[ -d "${_COTA_OVERLAY}/cota" ]]; then
    cp -a "${_COTA_OVERLAY}/cota/." "${_TRACKS}/cota/"
  fi
  if [[ -d "${_COTA_OVERLAY}/source" ]]; then
    mkdir -p "${_TRACKS}/source"
    cp -a "${_COTA_OVERLAY}/source/." "${_TRACKS}/source/"
  fi
fi

# Gazebo Classic looks up <material>EUFSF1/Silver</material> on GAZEBO_MATERIAL_PATH.
_mat_line='  <set_env name="GAZEBO_MATERIAL_PATH" value="$(find-pkg-share eufs_racecar)/materials/scripts:/usr/share/gazebo-11/media/materials/scripts"/>'
for _launch in "${SRC}/eufs_sim/eufs_tracks/launch/"*.launch; do
  if grep -q 'GAZEBO_RESOURCE_PATH' "${_launch}" && ! grep -q 'GAZEBO_MATERIAL_PATH' "${_launch}"; then
    sed -i "/set_env name=\"GAZEBO_RESOURCE_PATH\"/a\\${_mat_line}" "${_launch}"
  fi
done

echo "Workspace prepared at ${WS}"
