# Running the managed dashboard

From `eufs-f1-sim/`, `scripts/start-stack.sh` reuses or starts the canonical
container by default. Use `scripts/start-stack.sh --build` to build a new
image before changing the running container; if the image is absent, the first
start builds it automatically. It preserves an existing canonical container under
`eufs-f1-sim-before-grid-<timestamp>` and tags its image as
`eufs-f1-sim:rollback-<timestamp>`. It never runs `docker compose down` or
removes unrelated containers. The replacement starts with direct `docker run`
so Compose labels cannot select a renamed rollback container. Set `TRACK`,
`CARS`, or `EUFS_STACK_STAMP` in the environment before starting when needed.

The final managed restart check passed with four cars: readiness was 74.02 s
initially and 63.54 s after restart; fresh TF appeared in 0.36 s with zero
old-TF data. Stop paused physics, Shutdown completed in 2.12 s, and the final
process audit found no ROS, Gazebo, or RViz processes. The evidence is in
`.tmp/restart-final-check/managed_dashboard_smoke.json` and
`.tmp/restart-check-output.log`. The preserved canonical container is
`eufs-f1-sim-before-grid-20260912`; the verified final image is
`grid-telemetry-20260912` with SHA256
`34b9b3099bf2da06cab9e9201f0d24a952ec4185054d5a72a62e2713ce7b2cec`.

The dashboard opens with one car paused. The Cars selector supports 1–20 and
the selected count is used for the two-column spawn grid. Start loads or
restarts the selected backend and automatically attaches Gazebo and RViz once
the selected cars have published their command and odometry topics. Stop
pauses physics and keeps the backend available. Shutdown closes the owned
Gazebo/RViz viewers and backend while leaving the dashboard ready for a later
Start. The dashboard's telemetry tabs show unavailable values as `null` in
the schema and as an em dash in the UI; no value is fabricated for an absent
source.

The map adapter resolves each named map's world, model, spawn, centerline,
boundaries, and optional boundary-strip mesh. `build_grid_poses()` consumes
those assets to place the two-column grid on the map centerline when geometry
is available and retains the map's authored spawn fallback otherwise. RViz
generates one RobotModel display per car and fits the multi-car camera while
preserving the authored one-car view.

The telemetry schema exposes optional ego and opponent fields for session and
lap state, Frenet `s/y`, world pose, speed, acceleration, yaw rate, ERS/pack
channels, tire channels, and opponent gaps. `None` serializes to JSON `null`;
zero remains a measured zero. A future stack can attach through the standard
namespace topics: `<namespace>/cmd`, `<namespace>/cmd_vel`,
`<namespace>/odom`, `<namespace>/joint_states`, and the corresponding TF
frames. The dashboard does not require a control algorithm to provide these
telemetry channels.

## Rollback

After a replacement, list the saved container and rollback image:

```bash
docker ps -a --filter 'name=eufs-f1-sim-before-grid-'
docker image ls 'eufs-f1-sim:rollback-*'
```

To restore a saved container, stop the new canonical container, rename it to
a timestamped saved name, rename the planned backup to the canonical name,
and start it:

```bash
docker stop eufs-f1-sim
EUFS_SAVED_CONTAINER="eufs-f1-sim-grid-saved-$(date +%Y%m%d-%H%M%S)"
docker rename eufs-f1-sim "$EUFS_SAVED_CONTAINER"
docker rename eufs-f1-sim-before-grid-20260912 eufs-f1-sim
docker start eufs-f1-sim
```

The command stores the replaced container in `EUFS_SAVED_CONTAINER`; inspect
that name with `docker ps -a` if you need to return to it. The explicit
`eufs-f1-sim-before-grid-20260912` name is the planned dated rollback
container. The old image is retained by its `rollback-<timestamp>` tag.
