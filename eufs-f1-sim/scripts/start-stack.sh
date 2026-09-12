#!/usr/bin/env bash
# Start EUFS F1 sim: only the PyQt dashboard until you click Start.
#
# This script deliberately never runs `docker compose down`: the canonical
# container may be the user's known-good rollback point.  With --build, the
# new image is built first; only after a successful build is the old container
# renamed and stopped so the canonical name can be reused.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

xhost +local: >/dev/null 2>&1 || true
export DISPLAY="${DISPLAY:-:0}"
export TRACK="${TRACK:-cota}"
export CARS="${CARS:-1}"

BUILD=0
for arg in "$@"; do
  case "${arg}" in
    --build) BUILD=1 ;;
    *) echo "usage: $0 [--build]" >&2; exit 2 ;;
  esac
done

docker_do() {
  if groups | grep -qw docker; then
    docker "$@"
    return
  fi
  local command arg
  command="docker"
  for arg in "$@"; do
    printf -v arg '%q' "${arg}"
    command+=" ${arg}"
  done
  sg docker -c "${command}"
}

STAMP="${EUFS_STACK_STAMP:-$(date +%Y%m%d-%H%M%S)}"

if docker_do inspect eufs-f1-sim >/dev/null 2>&1 && [[ "${BUILD}" -eq 0 ]]; then
  running="$(docker_do inspect -f '{{.State.Running}}' eufs-f1-sim)"
  if [[ "${running}" == "true" ]]; then
    echo "Reusing running canonical container eufs-f1-sim"
  else
    docker_do start eufs-f1-sim >/dev/null
    echo "Started existing canonical container eufs-f1-sim"
  fi
else
  if [[ "${BUILD}" -eq 1 ]]; then
    # Build before touching the working canonical container.
    docker_do compose build
  elif ! docker_do image inspect eufs-f1-sim:lean >/dev/null 2>&1; then
    echo "eufs-f1-sim:lean is absent; building it before first start"
    docker_do compose build
  fi

  if docker_do inspect eufs-f1-sim >/dev/null 2>&1; then
    old_image="$(docker_do inspect -f '{{.Image}}' eufs-f1-sim)"
    rollback_image="eufs-f1-sim:rollback-${STAMP}"
    backup_name="eufs-f1-sim-before-grid-${STAMP}"
    if docker_do inspect "${backup_name}" >/dev/null 2>&1; then
      echo "Refusing to replace existing rollback container ${backup_name}; choose a new timestamp." >&2
      exit 1
    fi
    docker_do tag "${old_image}" "${rollback_image}"
    docker_do rename eufs-f1-sim "${backup_name}"
    docker_do stop "${backup_name}" >/dev/null || true
    echo "Preserved ${backup_name} using ${rollback_image}"
  fi

  docker_do run --detach --init --name eufs-f1-sim \
    --network host --ipc host --cpus 4 --memory 8g --privileged \
    --env "DISPLAY=${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env EUFS_MASTER=/opt/eufs_ws \
    --env ROS_LOCALHOST_ONLY=1 \
    --env GAZEBO_IP=127.0.0.1 \
    --env GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
    --env "TRACK=${TRACK}" --env "CARS=${CARS}" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --interactive --tty eufs-f1-sim:lean \
    ros2 run eufs_racecar start_dashboard --ros-args -p manage_stack:=true
fi

echo "Waiting for stack..."
sleep 20
docker_do ps --filter name=eufs-f1-sim
echo
echo "Launch: load_car.launch.py track:=${TRACK} cars:=${CARS}"
echo "DISPLAY (host=${DISPLAY}, container):"
docker_do exec eufs-f1-sim bash -lc 'echo DISPLAY=$DISPLAY QT_X11_NO_MITSHM=$QT_X11_NO_MITSHM'
echo
echo "GUI processes (expect start_dashboard only; gzclient/rviz2 after Start):"
docker_do exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui|start_dashboard' || true"
echo
echo "X11 windows on ${DISPLAY} (expect only 'EUFS F1 Demo' before Start):"
DISPLAY="${DISPLAY}" xwininfo -root -tree 2>/dev/null | grep -E 'EUFS F1 Demo|EUFS F1 Start|Gazebo|RViz|rqt' || true
echo
echo "Gazebo model eufs (headless gzserver):"
docker_do exec eufs-f1-sim bash -lc "timeout 6 gz model -m eufs -p || true"
