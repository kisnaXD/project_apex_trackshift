#!/usr/bin/env bash
# Start EUFS F1 sim with host X11 access: Gazebo, RViz, rqt, and the start dashboard.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

xhost +local: >/dev/null 2>&1 || true
export DISPLAY="${DISPLAY:-:0}"
export TRACK="${TRACK:-cota}"
export CARS="${CARS:-1}"

_compose_up() {
  docker compose down 2>/dev/null || true
  docker compose build
  TRACK="${TRACK}" CARS="${CARS}" docker compose up -d
}

if groups | grep -qw docker; then
  _compose_up
else
  sg docker -c "cd '${ROOT}' && export DISPLAY='${DISPLAY}' TRACK='${TRACK}' CARS='${CARS}' && docker compose down 2>/dev/null || true && docker compose build && docker compose up -d"
fi

echo "Waiting for stack..."
sleep 20
docker compose ps
echo
echo "Launch: load_car.launch.py track:=${TRACK} cars:=${CARS}"
echo "GUI processes (expect gzclient, rviz2, rqt_gui, start_dashboard):"
docker exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui|start_dashboard' || true"
echo
echo "Gazebo model eufs:"
docker exec eufs-f1-sim bash -lc "timeout 5 gz model -l || true"
