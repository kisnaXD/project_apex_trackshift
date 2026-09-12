#!/usr/bin/env bash
set -euo pipefail

container="eufs-f1-sim"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -qx true; then
  echo "container $container is not running" >&2
  exit 2
fi

# Do not start a second lap process.  This check only reads process state.
if docker exec "$container" bash -lc "pgrep -f '[.]*/cota_lap[.]py( |$)' >/dev/null"; then
  echo "a COTA lap runner is already running in $container" >&2
  exit 3
fi

docker cp "$script_dir/cota_path.py" "$container:/tmp/cota_path.py"
docker cp "$script_dir/cota_lap.py" "$container:/tmp/cota_lap.py"

exec docker exec -i "$container" bash -lc \
  'source /opt/ros/humble/setup.bash
   source /opt/eufs_ws/install/setup.bash
   exec python3 /tmp/cota_lap.py \
     --centerline /opt/eufs_ws/install/eufs_tracks/share/eufs_tracks/models/cota/centerline.csv \
     --boundaries /opt/eufs_ws/install/eufs_tracks/share/eufs_tracks/models/cota/boundaries.csv \
     --trace /tmp/cota_lap_run.json "$@"' bash "$@"
