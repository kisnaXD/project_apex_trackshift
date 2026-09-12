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
sed -i "s|-40 40 20 0 0.667643 -1|-15 18 12 0 0.55 -1.35|g" \
  "${SRC}/eufs_sim/eufs_tracks/worlds/small_track.world" || true

# Upstream cone SDFs used file:// relative to cwd, then model://eufs_tracks
# which is not a Gazebo model name. gzclient then hits the online model DB
# and sits on the splash. Copy each DAE into its model directory and point
# the SDF at model://<cone>/<file> (no Fuel). Collision becomes a cylinder.
python3 "${ROOT}/overlay/patches/relocate_cone_meshes.py" \
  "${SRC}/eufs_sim/eufs_tracks"

# Gazebo Classic looks up <material>EUFSF1/Silver</material> on GAZEBO_MATERIAL_PATH.
_mat_line='  <set_env name="GAZEBO_MATERIAL_PATH" value="$(find-pkg-share eufs_racecar)/materials/scripts:/usr/share/gazebo-11/media/materials/scripts"/>'
for _launch in "${SRC}/eufs_sim/eufs_tracks/launch/"*.launch; do
  if grep -q 'GAZEBO_RESOURCE_PATH' "${_launch}" && ! grep -q 'GAZEBO_MATERIAL_PATH' "${_launch}"; then
    sed -i "/set_env name=\"GAZEBO_RESOURCE_PATH\"/a\\${_mat_line}" "${_launch}"
  fi
  # model://eufs_tracks/meshes/... is resolved from the package share root;
  # the upstream launch files otherwise replace entrypoint's environment
  # with only the models directory and Gazebo cannot find those meshes.
  sed -i \
    's|$(find-pkg-share eufs_tracks)/models"|$(find-pkg-share eufs_tracks)/models:$(find-pkg-share eufs_tracks)/.."|g' \
    "${_launch}" || true
  sed -i \
    's|$(find-pkg-share eufs_tracks)/materials:/usr/share|$(find-pkg-share eufs_tracks)/materials:$(find-pkg-share eufs_tracks)/..:/usr/share|g' \
    "${_launch}" || true
done

echo "Workspace prepared at ${WS}"
