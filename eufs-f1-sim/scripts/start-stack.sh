#!/usr/bin/env bash
# Start EUFS F1 sim with host X11 access and all three GUIs (Gazebo, RViz, rqt).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

xhost +local: >/dev/null 2>&1 || true
export DISPLAY="${DISPLAY:-:0}"

if groups | grep -qw docker; then
  docker compose down 2>/dev/null || true
  docker compose build
  docker compose up -d
else
  sg docker -c "cd '${ROOT}' && docker compose down 2>/dev/null || true && docker compose build && docker compose up -d"
fi

echo "Waiting for stack..."
sleep 20
docker compose ps
echo
echo "GUI processes (expect gzclient, rviz2, rqt_gui):"
docker exec eufs-f1-sim bash -lc "pgrep -af 'gzclient|rviz2|rqt_gui' || true"
echo
echo "Gazebo model eufs:"
docker exec eufs-f1-sim bash -lc "gz model -m eufs -p || true"
