# COTA EUFS Track Integration Plan

Date: 2026-09-12  
Status: planning only. This document defines an asset and validation plan; it does not add a COTA track, download data, change a launch file, or start the simulator.

The goal is a COTA benchmark that can be selected by the existing EUFS track flow and consumed by both Gazebo and RViz. The car should begin on the approach to an orange start/finish gate, with its front bumper behind the gate and its heading aligned with the chosen direction of travel. The geometry must support track-relative progress, collision and out-of-bounds checks, and later two-car experiments.

## What counts as COTA fidelity

Use three explicit fidelity labels in metadata and experiment output:

| Label | Geometry and intended use |
| --- | --- |
| `layout_reference` | A metric approximation of the published COTA plan view. It preserves the 20-turn ordering, start/finish relationship, broad sector structure, and approximate lap length. It is suitable for software integration and qualitative strategy demos. |
| `boundary_validated` | A sampled centerline plus measured or independently sourced left/right boundaries, width envelope, elevation samples, and error report. It is suitable for collision, track-limit, and tactical planning tests. |
| `calibrated_benchmark` | The validated geometry plus a documented vehicle, tyre, energy, timing, and elevation calibration. It is required before making lap-time or overtake-performance claims. |

The FIA 2017 United States Grand Prix preview gives a 5.513 km lap and identifies the uphill approach to Turn 1 and the 20-turn layout; the FIA 2024 media kit provides a circuit map with turns and sector/DRS annotations. These are reference facts and visual control points, not a machine-readable boundary dataset: [FIA 2017 COTA preview](https://www.fia.com/sites/default/files/2017_usa_preview_0.pdf), [FIA 2024 US Grand Prix media kit](https://www.fia.com/sites/default/files/final_formula1crypto.commiamigrandprix2024_lo_0.pdf). Store the exact document URL, event/year, retrieval date, and any image digitisation error in the asset provenance.

Do not call a public image or racing line an official COTA boundary. A candidate open-data source is [TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database), which describes GPS-derived center lines and track widths. Before inclusion, inspect its per-file license, source attribution, coordinate system, sampling, and whether a COTA asset is actually present. A racing line remains a line-selection dataset, not a track centerline or boundary; it must not be used as either without an explicit transform and error report. If no redistributable metric source passes review, digitize a layout approximation from the FIA map and label it `layout_reference`.

## Canonical asset and coordinate frame

Keep one canonical metric asset and derive every runtime representation from it. Use a local ENU-style map frame in metres: `x` east/right in the chosen image or survey frame, `y` north/up in that frame, `z` elevation, and yaw counter-clockwise from `+x`. Record the origin, rotation, scale, source CRS, transform, and sign convention in `provenance.yaml`; do not silently mix WGS84 degrees, image pixels, or simulator metres.

The canonical file should contain at least:

```text
track_id: cota_<fidelity>_<version>
frame: cota_local_enu
centerline: [s_m, x_m, y_m, z_m, yaw_rad, curvature_1pm]
boundaries: [s_m, left_x_m, left_y_m, right_x_m, right_y_m]
width: [s_m, left_width_m, right_width_m]
events: [id, type, s_m, x_m, y_m, heading_rad, notes]
provenance: source_urls, license_note, transform, scale, retrieved_at, fit_error_m
```

Resample a source polyline by arc length only after preserving the original points. Reject duplicate points, large gaps, self-intersections, and discontinuous heading. Compute cumulative unwrapped `s` from the resampled centerline and close the lap explicitly. Do not space cones by row number when the source sampling is irregular.

For a plan-view approximation, use a declared constant or piecewise width envelope and mark `z_m` as unknown/zero. For a boundary-validated asset, retain measured left/right points and fit a smooth centerline from the boundaries. Elevation is a separate channel: a 2D asset must not imply that COTA gradients have been reproduced. The FIA description of the uphill Turn 1 approach supports an event annotation, but does not supply an elevation profile.

## EUFS cone semantics and gate

Reuse the existing EUFS CSV and generated SDF conventions rather than introducing a new colour vocabulary:

- `blue`: one side boundary;
- `yellow`: the other side boundary;
- `orange`: ordinary orange cones if needed;
- `big_orange`: the four-cone start/finish gate;
- `car_start`: reference pose only, with the final spawn checked against vehicle geometry.

The existing CSV header is `tag,x,y,direction,x_variance,y_variance,xy_covariance`. The current `small_track.csv` uses `blue`, `yellow`, four `big_orange` rows, and one `car_start` row. The existing marker publisher maps yellow to `(1.0, 0.85, 0.0)`, orange/big-orange to `(1.0, 0.35, 0.0)`, and blue to `(0.05, 0.25, 1.0)`, with 0.24 m diameter and 0.30 m marker height. COTA should use these same values unless an asset review documents a deliberate change.

Generate boundary cones by arc-length distance along each boundary, with a documented spacing policy and a maximum chord/curvature error. Use the source boundary locations where available; do not sample a racing line and offset it by an unverified constant. Keep the exact generated CSV as the audit artifact so the SDF and RViz output can be compared row-for-row.

The gate is a semantic event as well as four `big_orange` models. Put two cones on each side of the start/finish crossing, with a recorded gate center, tangent heading, normal, width, and `s_m`. Use the same crossing for lap counting and the planner event. If the reference map only shows a line and not cone placement, label the gate placement as an authored approximation.

## Spawn pose: front bumper behind the gate

`car_start` must not be treated as “put the vehicle origin on the gate.” The spawn calculation should use the collision envelope and the selected heading:

1. Choose the gate center `g`, forward unit vector `f`, and signed track direction.
2. Choose a required bumper-to-gate clearance `c_gate` that exceeds the cone collision radius and the simulator contact margin.
3. Determine the vehicle’s longitudinal front extent `l_front` from the collision geometry in the vehicle frame, including any front wing or bumper collision envelope.
4. Set the vehicle reference origin to `p_spawn = g - f * (l_front + c_gate)` for a start behind the gate. Apply the same transform to any `car_start` CSV metadata.
5. Set yaw to the gate tangent in the selected travel direction, then validate the sign by projecting a short forward motion onto increasing `s`.
6. Check both front corners and the full collision envelope against the gate cones and boundaries before launch.

For the existing small track, the four `big_orange` rows are approximately at `x=-9.6/-10.0`, `y=8.39/13.0`, and the reference `car_start` is `(-13.0,10.3,yaw=0)`. This is useful evidence of the desired visual relationship, but its clearance must be recomputed from the actual F1 collision envelope before copying the pattern to COTA. Store `front_extent_m`, `gate_clearance_m`, `spawn_s_m`, `spawn_x_m`, `spawn_y_m`, `spawn_yaw_rad`, and a direction-check result in metadata.

## Gazebo, RViz, and packaging

Package one named COTA model/world and one CSV asset under the track package. The generated SDF should use shared lightweight cone mesh/material resources and static or otherwise low-overhead geometry. Thousands of individually detailed visual meshes can make Gazebo startup and rendering expensive; preserve coloured appearance with the existing small cone models and avoid duplicating mesh data per instance. Use collision geometry only where the benchmark needs it: boundary cones should retain simple collision for track-limit/contact tests, while a separate `visual_only` mode may omit collision for large visual smoke tests. Record the selected mode in the world metadata.

Keep these outputs derived from the same canonical asset:

- `cota_<version>.csv`: cone rows and `car_start` metadata;
- `models/cota_<version>/model.sdf`: Gazebo includes and collision/visual policy;
- `worlds/cota_<version>.world`: ground, lighting, and track model;
- `provenance.yaml`: source, license, transform, scale, fidelity, fit errors, and generator version;
- `events.yaml`: start/finish, corner sectors, braking/activation/detection zones;
- optional `centerline.csv` and `boundaries.csv`: planner geometry in the same local frame.

Add a track selector that resolves a named asset to these files. The selector should fail on missing or mismatched hashes rather than silently falling back to `small_track`. The launch path should pass the selected SDF to the existing marker publisher so RViz reads exactly the same cone placements Gazebo loads. Do not add COTA to the default launch until the staged checks below pass.

## Collision and track-limit policy

Define the track as the closed corridor between the validated left and right boundaries. A car is within limits only when its configured footprint, or the selected reference points, remains inside that corridor; document which rule is used. Cone contact is a separate event from a track-limit violation. Keep the F1 visual mesh and physics collision envelope distinct, as the current repository already treats the visual CAD as presentation geometry.

For a 2D approximation, use conservative widths and report that they are authored. For measured boundaries, compute the minimum distance from every footprint sample to both sides, test corner-cutting at high curvature, and validate the wraparound segment. Include a coarse `visual_only` mode for map review and a collision-enabled mode for vehicle tests; never compare timing results across the two modes without stating the difference.

## Staged verification

1. **Source and licensing review:** record URLs, documents, repository commit/hash, license terms, transform, and known limitations. No asset enters the package without a redistributability decision.
2. **Geometry checks:** verify units, closed-loop continuity, no self-intersections, arc-length monotonicity, turn order 1–20, declared lap length, start/finish location, width bounds, and fit error.
3. **Asset consistency:** regenerate CSV, SDF, world, and event metadata from one input; compare cone count, tags, coordinates, colours, and hashes. Confirm no irregular source sampling was used directly for spacing.
4. **Marker-only review:** load the SDF into the existing marker publisher or an offline parser and verify blue/yellow/orange counts, gate placement, and 2D rendering in the local frame.
5. **Gazebo visual smoke test:** use shared lightweight meshes and confirm the full circuit loads without duplicate models, missing materials, or unacceptable startup/render cost.
6. **Single-car spawn test:** place the car using the collision-derived front extent; verify the front bumper is behind the orange gate, yaw advances `s`, all wheels are on the ground, and no initial cone/boundary collision occurs.
7. **Repeated-lap track test:** run one car over repeated laps, checking `s` wrap, gate crossing, projection continuity, track limits, and no collision leaks at the seam.
8. **Two-car readiness:** only after single-car checks, spawn an opponent at a declared `s` gap and validate independent namespaces, collision policy, and observation frames. Mark any line, speed, elevation, or opponent response as modelled rather than sourced.

The first acceptance report should state `layout_reference`, `boundary_validated`, or `calibrated_benchmark` explicitly. Until boundary and vehicle calibration are complete, COTA results are integration and strategy-development results, not claims about real COTA lap time or racing performance.
