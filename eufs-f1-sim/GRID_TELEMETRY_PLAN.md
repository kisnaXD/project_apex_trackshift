# Grid and telemetry handoff

The bounded multi-car backend is implemented in the launch/Xacro path. The
existing one-car contract remains the compatibility baseline; multi-car runs
use namespaced commands, odometry, descriptions, TF, tyre state, and native
EUFS services. `rviz_grid.py` generates one RobotModel display per car. Its
public entry points are:

```python
from eufs_racecar.rviz_grid import generate_robot_model_displays, generate_rviz_config
from eufs_racecar.grid_geometry import GridMapAdapter, build_grid_poses
```

Map geometry is supplied by the standard EUFS asset registry returned from
`resolve_track()`: `worlds/<name>.world`, `models/<name>/model.sdf`, and
`csv/<name>.csv`. Optional ordered `centerline.csv`, `boundaries.csv`, and
profile/provenance YAML can live under `<name>/` or `models/<name>/`. Maps
without centerline metadata use the generic spawn-relative grid and report
that curved-track validation is unavailable. `build_grid_poses()` accepts the
resolved asset dictionary (and an optional `(x, y, yaw)` override) and applies
the authored centerline profile when present. If centerline metadata is
missing, it uses a spawn-tangent grid without boundary validation; this does
not guarantee that any car is inside an arbitrary map.

The minimal attachment contract for a new map is:

```text
eufs_tracks/
  worlds/example.world
  models/example/model.sdf
  csv/example.csv
  example/                         # optional map-owned metadata
    centerline.csv                 # optional
    boundaries.csv                 # optional
    metadata.yaml                  # optional (or provenance.yaml)
    example_boundary_strips.dae    # optional visual mesh
```

The standard CSV headers are:

```text
cone CSV:       tag,x,y,direction,x_variance,y_variance,xy_covariance
centerline:     s_m,x_m,y_m,z_m,yaw_rad,curvature_1pm
boundaries:     s_m,left_x_m,left_y_m,right_x_m,right_y_m
```

`metadata.yaml` can supply explicit `launch_spawn.spawn_x_m`,
`launch_spawn.spawn_y_m`, `launch_spawn.spawn_yaw_rad`, and
`grid.spawn_arclength_back_m`. A boundary-strip opt-in uses the ordered SDF
cone convention, for example:

```yaml
cone_policy:
  white_strip_mesh: models/example/example_boundary_strips.dae
  white_strip_source: ordered_blue_yellow_cones
  white_strip_marker_namespace: example_track_boundary
```

The strip adapter and cone marker publisher expose this same metadata path;
they do not generate strips or reorder cone includes. White marker ribbons
therefore require blue and yellow cone includes already ordered along the SDF
track path. COTA supplies its authored five-metre spawn arclength profile and
explicit runtime launch pose.
`available_tracks()` / `discover_tracks()` expose the registry to telemetry/UI
code without embedding map names in the UI.

Validation completed:

- Four-car isolated control/pose test passed; result is
  `.tmp/grid-isolation-result.json`.
- N=1, N=4, and N=20 grid geometry passed full footprint boundary and pairwise
  overlap checks using independent left/right boundary-loop XOR validation;
  the final artifact is `.tmp/grid-final-geometry.json`.
- RViz grid generation produces per-car description topics and bare RViz TF
  prefixes (`eufs2`), matching RSP's `eufs2/` frame prefix.
- 27 focused tests passed, including synthetic new-map discovery and profile
  attachment. Stock `small_track` also spawned two cars with both native
  vehicle plugins.
- The full COTA lap completed `5513.0986 m` in `545.687 s` and stopped at zero
  final velocity. Independent trace validation found zero footprint violations
  across all 4808 samples; see `.tmp/lap_validation/cota_lap_full.json` and
  `.tmp/lap_validation/live_trace_footprint_summary.json`.

White RobotModel rendering is corrected in the generated RViz config. The
final restart check passed: initial four-car readiness was 74.02 s, restart
readiness was 63.54 s, fresh TF appeared in 0.36 s, and the log contained zero
old-TF data. The check covered fresh mapped ego plus three opponents, live
Gazebo/RViz viewers, Stop pause, Shutdown completion in 2.12 s, and a clean
post-shutdown process audit. See `.tmp/restart-final-check/managed_dashboard_smoke.json`
and `.tmp/restart-check-output.log`. Native plugin source/checksum and
wheel/physics assets remain unchanged.

## Rollback

The source snapshot immediately before the grid/telemetry work is archived at
`.tmp/pre-grid-telemetry-20260912.tar.gz`. The known-good image is
`eufs-f1-sim:forward-rolling-20260912`. The preserved canonical container is
`eufs-f1-sim-before-grid-20260912`; the final image is
`grid-telemetry-20260912` (SHA256
`34b9b3099bf2da06cab9e9201f0d24a952ec4185054d5a72a62e2713ce7b2cec`).
