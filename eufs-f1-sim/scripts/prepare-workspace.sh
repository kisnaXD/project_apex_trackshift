#!/usr/bin/env bash
# Assemble colcon workspace from locked sources + overlays.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/ws"
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

echo "Workspace prepared at ${WS}"
