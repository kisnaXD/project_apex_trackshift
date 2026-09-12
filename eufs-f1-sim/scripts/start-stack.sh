#!/usr/bin/env bash
# Start EUFS F1 sim: only the PyQt dashboard until you click Start.
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
echo "DISPLAY (host=${DISPLAY}, container):"
docker exec eufs-f1-sim bash -lc 'echo DISPLAY=$DISPLAY QT_X11_NO_MITSHM=$QT_X11_NO_MITSHM'
echo
echo "GUI processes (expect start_dashboard only; gzclient/rviz2 after Start):"
docker exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui|start_dashboard' || true"
echo
echo "X11 windows on ${DISPLAY} (expect only 'EUFS F1 Demo' before Start):"
DISPLAY="${DISPLAY}" xwininfo -root -tree 2>/dev/null | grep -E 'EUFS F1 Demo|EUFS F1 Start|Gazebo|RViz|rqt' || true
echo
echo "Gazebo model eufs (headless gzserver):"
docker exec eufs-f1-sim bash -lc "timeout 6 gz model -m eufs -p || true"
