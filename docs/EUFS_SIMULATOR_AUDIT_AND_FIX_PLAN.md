# EUFS Simulator Audit and Fix Plan

Date: 2026-09-12

Scope: read-only audit of `/home/gera/Desktop/EUFS_FORK_CONTAINERIZED/eufs-f1-sim` for use as the simulator behind a 2026 F1 hybrid energy and overtaking strategy benchmark. No simulator source, config, build, or launch files were changed.

Project assumptions:

- Target rules: 2026 FIA Formula 1 hybrid and overtaking rules.
- Delta Energy: change in our own battery energy over time.
- Opponent input: estimated opponent pose/history and derived speed gap. No opponent battery telemetry.
- Roadmap: credible single-car energy model, then two-car strategic benchmark on COTA, then strategy/MPC/advisory.

## Coverage

Inspected:

- Docker and build/run scripts: `Dockerfile`, `docker-compose.yml`, `scripts/prepare-workspace.sh`, `scripts/entrypoint.sh`, `scripts/start-stack.sh`.
- CAD helper scripts: `scripts/build_f1_meshes.py`, `scripts/step_to_urdf_ocp.py`, `scripts/inspect_step.py`.
- Overlay packages: `overlay/eufs_racecar`, `overlay/gazebo_ros_battery`, `overlay/patches`.
- Populated workspace sources: `ws/src/eufs_sim`, `ws/src/eufs_msgs`, `ws/src/eufs_racecar`, `ws/src/gazebo_ros_battery`.
- Vehicle URDF/xacro, Gazebo plugins, battery plugin code, EUFS dynamic bicycle model, launch files, track assets, package install data, build/run logs.

Inventory counts from read-only search:

- `eufs-f1-sim`: 5437 files excluding `.git` contents.
- `eufs-f1-sim/overlay`: 232 files.
- `eufs-f1-sim/ws/src`: 496 files excluding nested `.git` contents.

These counts are repository inventory coverage, not a claim that every generated, vendored, built, or installed file was line-by-line audited. Detailed line evidence is limited to the files cited in the findings below.

Not performed:

- No Gazebo/ROS runtime launch.
- No Docker build or container start.
- No simulator code execution.
- No package installation.
- No CAD conversion run.

Existing logs were inspected as evidence, but they were not generated during this audit.

## Current Repository State

The working tree was already dirty before this documentation pass:

- Modified: `eufs-f1-sim/docker-compose.yml`, battery plugin source files, `scripts/prepare-workspace.sh`, and nested `ws/src/eufs_sim`.
- Untracked: `.build.log`, `.run.log`, `.tmp/`, `overlay/patches/`, Python cache, and `scripts/build_f1_meshes.py`.

This audit did not revert or normalize those files.

## Severity Summary

| Severity | Finding | Verified status |
|---|---|---|
| P0 | Runtime startup needs fresh verification | Historical `.run.log` failure verified; current entrypoint mitigation present; no fresh launch performed |
| P0 | Battery state is not coupled back into vehicle performance | Verified by static wiring/code inspection |
| P0 | Mechanical energy consumer uses world potential energy, not drivetrain power | Verified by static code inspection |
| P0 | Two-car operation is not namespace-safe | Verified static wiring risk; runtime two-car collision not tested |
| P0 | COTA is absent from inspected assets | Verified by bounded file/text search |
| P1 | F1 CAD pipeline is visual-first, source-specific, and mutates simulator files | Verified by script/source inspection |
| P1 | Authored overlay, copied workspace source, and installed files are stale relative to each other | Verified by static file comparison from energy audit |
| P1 | `vehicleModel` and `commandMode` launch arguments are inactive on the authored Ackermann path | Verified by static launch/xacro inspection |
| P1 | Dynamic bicycle config is incomplete | Verified for EUFS dynamics path; current overlay Ackermann path appears not to consume it |
| P1 | Competing vehicle-control paths need a benchmark choice | Verified by static launch/xacro/plugin inspection |
| P1 | Upstream race-car plugin has an out-of-bounds vector bug | Verified by static code inspection |
| P1 | Battery lap/mode parameters are loaded but not enforced | Verified by static code/config inspection |
| P1 | Thermal model is single-lump and appears timestep-inconsistent | Verified by static code inspection |
| P1 | Physical battery capacity scale conflicts with configured lap budgets | Verified by static config/code inspection |
| P1 | Entrypoint mutates installed world/assets at runtime | Verified by static script inspection |
| P2 | Dependency/source provenance is partially pinned | Verified by static script/Dockerfile inspection |
| P2 | Ground truth and estimated observations need experiment separation | Verified by static topic/config inspection |

## Prioritised Findings

### P0: Runtime startup needs fresh verification after a historical nounset failure

Evidence:

- Historical `.run.log` shows `/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable` at line 5 and `eufs-f1-sim exited with code 1` at line 7.
- File timestamp check showed `.run.log` last modified at `2026-09-12 00:30:15 +0530`.
- Current `scripts/entrypoint.sh` disables nounset before sourcing ROS setup files and re-enables it afterward at lines 4-7.
- Current `scripts/entrypoint.sh` was last modified later, at `2026-09-12 12:25:50 +0530`.

Impact:

The log proves a historical observed startup failure, not a current reproducible failure. The current entrypoint appears to mitigate that specific nounset issue, but no fresh launch was performed in this audit. Runtime startup remains unverified, which blocks topic inspection, two-car validation, and COTA smoke tests.

Proposed remedy:

Keep the nounset mitigation in place and run a fresh headless startup smoke test. If the failure recurs, initialise `AMENT_TRACE_SETUP_FILES` or further isolate ROS setup sourcing from strict shell options.

Acceptance criteria:

- Container reaches ROS launch.
- `ros2 node list` and `ros2 topic list` work inside the container.
- Gazebo server starts headless for a smoke test without immediate exit.

### P0: Battery state is not coupled back into vehicle performance

Evidence:

- Battery plugin publishes `battery_state` and `charge_level_wh` in `battery_discharge.cpp` lines 39-42 and 230-234.
- The vehicle path in `racecar.gazebo` uses `gazebo_ros_ackermann_drive` with `max_speed` 20 m/s at lines 126-153.
- No inspected code reduces allowed speed, acceleration, torque, or power based on battery state.

Impact:

The simulator can display changing battery values without proving energy-aware lap time, acceleration, or overtake behaviour. Strategy evaluation would be invalid because spending or saving energy may not change the plant response.

Proposed remedy:

Choose one active vehicle plant path and connect energy limits into it. For strategy development, prefer an acceleration/power command interface where battery power limits constrain available longitudinal force. Battery discharge, recovery, and thermal limits must feed back into allowable propulsion.

Acceptance criteria:

- Same throttle/acceleration command produces different speed traces at different battery power limits.
- Battery empty or power-limited state caps acceleration.
- Recovery increases usable energy only in allowed braking/coast regions.
- Lap time changes are physically explainable from energy deployment differences.
- Published telemetry exposes own absolute energy, signed own Delta Energy, deploy power, recovery power, derate reason, and validity flags.
- The selected propulsion path uses the same power limit that the battery accounting reports.

### P0: Mechanical energy consumer uses world potential energy, not drivetrain power

Evidence:

- `mechanical_energy_consumer.cpp` initialises and updates `last_energy_` from `model->GetWorldEnergyPotential()` at lines 44 and 54.
- It computes power from `delta_energy / dt` at lines 55-60.
- It clamps power to at least idle power at line 61.

Impact:

On a flat track, potential energy misses acceleration, kinetic energy changes, aerodynamic drag, rolling resistance, and tyre losses. The idle clamp also prevents this consumer from representing regenerative power. It is unsuitable as the main F1 hybrid energy model.

Proposed remedy:

Replace this consumer with a drivetrain/ERS power model:

- Battery energy update from deploy power, regen power, efficiencies, auxiliaries, and heat.
- Longitudinal force from available mechanical power and tyre grip.
- Explicit sign convention for deploy versus regen.
- Separate physical state of charge from FIA accounting counters.

Acceptance criteria:

- Constant-speed flat running consumes drag/rolling/auxiliary energy.
- Acceleration consumes additional energy.
- Braking/coast recovery is possible only when configured and bounded.
- Energy conservation checks pass on simple synthetic cases.
- The sign convention is verified with a plugin-level test: positive propulsion discharges, regenerative braking charges, and auxiliary load always discharges.

### P0: Two-car operation is not namespace-safe

Evidence:

- `load_car.launch.py` accepts `namespace` at line 28 but creates `robot_state_publisher` and `joint_state_publisher` without a ROS namespace at lines 60-95.
- `spawn_entity.py` reads `robot_description` from a shared topic at line 77.
- `joint_state_publisher` remaps `/joint_states` to absolute `/eufs/joint_states` at line 94.
- Battery xacro hard-codes `<namespace>/eufs</namespace>` at `forgez_battery.gazebo` lines 19-23 and 45-48.
- Lidar remaps output to absolute `/scan` in `racecar.gazebo` line 203.
- Upstream EUFS plugin xacro uses absolute topics such as `/ground_truth/state`, `/odometry_integration/car_state`, `/ros_can/wheel_speeds`, and `/cmd` in `eufs_plugins.gazebo.xacro` lines 27-31 and `gazebo_ros_race_car_model.cpp` line 85.

Impact:

Two spawned cars will collide in topic names, frames, robot descriptions, joint states, and battery topics. This prevents independent ego/opponent simulation and corrupts opponent pose estimation.

Proposed remedy:

Make every car instance namespace-clean:

- Per-car robot description topic.
- Per-car command, odom, battery, joint state, scan, and state topics.
- Per-car frame prefix or distinct TF tree.
- Per-car spawn entity name and node names.
- One deliberate cross-car observation interface for the planner.

Acceptance criteria:

- Two cars can be spawned as `/ego` and `/opp_1`.
- Each car accepts independent commands.
- Topic list has no unintended shared control/state topics.
- TF trees do not overwrite each other.
- Planner receives opponent estimate through a named observation topic, not by subscribing to the opponent's private control state.

### P0: COTA is not present in the inspected assets

Evidence:

- Workspace tracks include Formula Student style worlds: `acceleration`, `small_track`, `skidpad`, `rectangle`, `peanut`, and similar.
- Bounded searches covered `eufs-f1-sim/overlay`, `eufs-f1-sim/ws/src/eufs_sim`, and `eufs-f1-sim/scripts` for `COTA|cota`, plus world/model/launch file inventories. They found no COTA map, model, launch file, or text reference.

Impact:

The accepted roadmap requires a COTA two-car benchmark. The current repository cannot provide that by changing launch arguments alone.

Proposed remedy:

Create a COTA track package or asset set with:

- Gazebo world.
- Centerline and boundaries.
- Track-coordinate projection.
- Start grid/spawn points.
- Named events for turns, straights, braking zones, detection/activation/manual override zones.
- Validation metadata for scale and lap length.

Acceptance criteria:

- Lap length and turn ordering match the intended COTA reference.
- One car can complete repeated laps without track-coordinate discontinuities.
- Two cars can spawn at valid grid/rolling-start offsets.
- Event annotations are available to the strategy planner.
- World metres, mesh scale, track-coordinate arc length, and lap progress are separately validated and logged.
- Single-car validation precedes selecting any benchmark COTA speed profile.

### P1: Current F1 CAD pipeline is visual-first, source-specific, and mutates simulator files

Evidence:

- `F1_MODEL_SOURCE.txt` identifies the source as a GrabCAD Mercedes-AMG Petronas F1 Concept 2 community model, not official team CAD, lines 1-5.
- The same file says corner solids such as uprights, wishbones, and brake ducts were dropped at lines 8-9.
- `macros.xacro` uses hand-written inertial values such as chassis mass 740 kg and inertia tensor at lines 6-11.
- Wheel collision is simple cylinder geometry at lines 113-124.
- The URDF gives most visible body parts visuals but no collision on chassis/front/rear wings in `racecar.xacro` lines 10-59.
- `build_f1_meshes.py` hard-codes GLB scene node names such as `car_body_5`, `front_wheels_7`, and `back_wheels_1` at lines 81-84.
- `build_f1_meshes.py` deletes existing STL and legacy DAE meshes in the output directory at lines 134-142 before exporting replacements.
- `step_to_urdf_ocp.py` classifies STEP solids from bounding boxes and volume at lines 110-169, with comments stating suspension/upright/brake-duct leftovers are dropped at lines 113-119.
- `step_to_urdf_ocp.py` defaults to `/home/gera/Downloads/Assem step.STEP` at lines 403-407 and defaults output into the overlay mesh directory at lines 409-413.
- `step_to_urdf_ocp.py` patches `racecar.xacro` and `macros.xacro` by default at lines 420-430 and performs those writes at lines 589-594.
- `inspect_step.py` depends on FreeCAD's Python API and prints assembly object bounds; it is an inspection helper, not a repeatable validation test.

Impact:

The CAD can help visual presentation, but it should not be treated as physically plausible F1 geometry, mass distribution, collision, aero, or tyre contact. The helper scripts are also tailored to one known asset structure and can overwrite simulator meshes/xacros as a side effect, which is risky while the CAD import remains unstable.

Proposed remedy:

Separate "visual mesh" from "physics model". Keep simplified collision and inertial models, but derive them deliberately from target dimensions and vehicle parameters. Convert CAD into a generated staging directory first, inspect metadata, then explicitly promote outputs into the overlay. Disable default xacro patching unless the caller passes an explicit mutation flag.

Acceptance criteria:

- Visual scale is verified against target length, width, wheelbase, and tyre size.
- Physics collision envelope does not snag or float on COTA.
- Inertial parameters are documented and plausible for the benchmark, even if simplified.
- CAD conversion can run in dry-run/staging mode without modifying overlay files.
- Conversion metadata records source file, scale, wheelbase, wheel centres, dropped solids, and generated mesh bounds.

### P1: Vehicle configuration is insufficient for EUFS dynamic bicycle use

Evidence:

- `configDry.yaml` has only `vehicle.mass` and `vehicle.wheelbase` at lines 1-4.
- EUFS `vehicle_param.hpp` expects `inertia`, `kinematics`, `tire`, `aero`, and `input_ranges` sections at lines 38-42.

Impact:

The current overlay robot includes `racecar.gazebo` through `racecar.xacro` lines 299-301 and therefore uses the Gazebo Ackermann plugin path. The stub `configDry.yaml` does not appear to be consumed by that active overlay Ackermann plant path. It is still a blocker for switching to the upstream EUFS race-car model or dynamic bicycle benchmark path, where the parameter loader expects the full schema.

Proposed remedy:

Create a full F1 benchmark vehicle parameter file. Include mass, yaw inertia, wheelbase, CG distribution, axle width, tyre model coefficients, tyre radius, drag, downforce, acceleration limits, speed limits, and steering limits.

Acceptance criteria:

- Dynamic bicycle plugin loads the config without parameter exceptions.
- Simple acceleration, braking, and constant-radius tests produce plausible traces.
- The same config is referenced by both simulation and strategy model generation.

### P1: `vehicleModel` and `commandMode` arguments are inactive on the authored Ackermann path

Evidence:

- EUFS track launch files pass `vehicleModel` and `commandMode` into the racecar launch path; for example the energy audit cites `eufs_tracks/launch/small_track.launch` lines 63-64.
- Authored `load_car.launch.py` declares `vehicleModel` and `commandMode` at lines 115-116.
- The same authored launch file processes xacro with mappings only for `config_file`, `forgez_mode`, `forgez_T_core`, `forgez_E_lap`, and `forgez_R_OT` at lines 47-55.
- Authored `robot.urdf.xacro` includes `racecar.xacro` and `forgez_battery.gazebo` at lines 10-11, not the upstream `eufs_plugins.gazebo.xacro`.
- Authored `racecar.xacro` includes `racecar.gazebo` at lines 299-301.
- Authored `racecar.gazebo` uses `libgazebo_ros_ackermann_drive.so` at line 126.

Impact:

Launches can appear to select `DynamicBicycle` or `acceleration` mode while the spawned robot still uses the Ackermann velocity-control plugin. This can mislabel experiments and hide the fact that the energy model is not coupled to the selected vehicle dynamics.

Proposed remedy:

Make plant selection explicit. Either remove these arguments from the authored Ackermann path or add a separate benchmark launch mode that includes the EUFS race-car plugin with a complete compatible config. The generated robot description should be inspected before every benchmark run.

Acceptance criteria:

- A benchmark launch emits the selected plant path in logs and metadata.
- The generated robot description contains exactly one expected motion plugin.
- `vehicleModel` and `commandMode` either affect the spawned robot or are rejected as unsupported for that launch mode.

### P1: There are two competing vehicle-control paths

Evidence:

- Overlay `racecar.gazebo` uses `gazebo_ros_ackermann_drive` at lines 126-171.
- Upstream EUFS has `gazebo_race_car_model` in `eufs_plugins.gazebo.xacro` lines 11-37.
- The upstream plugin sets model pose and velocity directly at `gazebo_ros_race_car_model.cpp` lines 347-365 and updates the vehicle model at lines 565-603.

Impact:

Using both paths or switching between them without a contract can create inconsistent dynamics, topics, and energy accounting. The Ackermann velocity plugin is easier to launch but less suitable for power-limited F1 energy work.

Proposed remedy:

Pick one plant path for benchmark validity:

- Short-term: dynamic bicycle plugin with an acceleration command interface and energy-limited longitudinal force.
- Keep Ackermann velocity plugin only for visual/smoke-test mode.

Acceptance criteria:

- Exactly one vehicle plant plugin controls motion in benchmark launches.
- Command topics and state outputs are documented.
- Energy state is updated from the same longitudinal-force/power path that moves the car.

### P1: Upstream race-car plugin has an out-of-bounds vector bug

Evidence:

- `ToQuaternion()` reserves four vector elements at line 644, then writes `q[0]` through `q[3]` at lines 645-648. `reserve()` does not change vector size.

Impact:

If this plugin is used, quaternion generation has undefined behaviour and can corrupt state publication or crash.

Proposed remedy:

Resize the vector or return a fixed-size array/message directly.

Acceptance criteria:

- Unit or smoke test calls quaternion conversion without sanitizer errors.
- Published orientation is correct for known yaw values.

### P1: Battery lap budget and mode parameters are loaded but not enforced

Evidence:

- `battery_discharge.cpp` loads `forgez_mode`, `forgez_E_lap_wh`, `forgez_T_core`, and `forgez_R_OT` at lines 77-81.
- It logs them at lines 128-131.
- The inspected update logic publishes charge and temperature but does not enforce the loaded lap budget or `max_power_w`.
- `forgez_battery.yaml` defines `E_lap` at lines 10, 15, and 20.
- `forgez_battery.yaml` defines `max_power_w` per mode at lines 12, 17, and 22.
- `load_car.launch.py` passes `T_core`, `E_lap`, and `R_OT` to xacro at lines 52-54, but does not pass `max_power_w`.

Impact:

Modes look meaningful in configuration but do not yet govern deployment power, recovery power, or strategy constraints. That is dangerous for experiments because "Attack" and "Harvest" can become labels rather than behaviours.

Proposed remedy:

Move mode handling into explicit energy-policy inputs:

- Mode-specific power limits.
- Optional mode-specific thermal targets.
- Lap/race rule counters.
- Strategy/MPC interface for requested deployment and recovery.

Acceptance criteria:

- Harvest, Nominal, and Attack produce different allowed deployment/recovery behaviours.
- Rule-counter violations are detectable.
- Battery topics expose both physical state and rule-accounting state.
- Lowering `max_power_w` measurably reduces achievable acceleration or terminal speed through the chosen vehicle plant.
- Setting a low `E_lap` produces either a deployment limit or a clearly labelled rule violation before excess deployment is consumed.

### P1: Regeneration and current sign conventions are not validated

Evidence:

- Battery charging support exists in `battery_discharge.cpp` lines 171 and 177 for negative loads when charging is allowed.
- The active mechanical consumer adds a friction term from absolute power at `mechanical_energy_consumer.cpp` line 60 and then clamps power to at least idle at line 61.
- `forgez_battery.gazebo` sets `consumer_idle_power` to `120.0` at line 54.

Impact:

The battery plugin has some support for charging, but the active mechanical drive consumer cannot produce negative regenerative power after the idle clamp. Without a sign-convention test, later fixes can accidentally invert discharge/charge semantics.

Proposed remedy:

Split auxiliary load from propulsion and recovery. Verify the Gazebo battery sign convention with a small plugin-level test before wiring strategy/MPC energy accounting to it.

Acceptance criteria:

- Positive propulsion power decreases own usable energy.
- Regenerative braking increases own usable energy when state of charge, temperature, tyre grip, speed, and rules allow it.
- Auxiliary load decreases own usable energy even during coasting.
- Telemetry reports signed own Delta Energy consistently across deploy, coast, and regen cases.

### P1: Thermal model is single-lump and appears timestep-inconsistent

Evidence:

- Energy audit found one battery temperature state `t_` and one scalar `heat_energy_` in `battery_discharge.hh` lines 60-61.
- Heat generation multiplies power by `dt` in `battery_discharge.cpp` line 207.
- Heat dissipation is computed at line 209 and subtracted at line 210 without multiplying by `dt`.
- Temperature computation is enabled in `forgez_battery.gazebo` at line 34.

Impact:

If `heat_dissipation_rate` is intended as W/K, cooling becomes dependent on simulation step rate. A one-lump model also cannot represent core/surface lag, which matters when high-power deployment and thermal derate are part of the strategy.

Proposed remedy:

Fix timestep consistency first. Then introduce at least a two-node core/surface thermal model if thermal derate is used by the planner.

Acceptance criteria:

- Running the same power profile at different physics step sizes produces materially similar temperature traces.
- Battery temperature can derate available deployment power through the chosen vehicle plant.
- Thermal telemetry distinguishes measured/modelled state from planner assumptions.

### P1: Physical battery capacity scale conflicts with configured lap budgets

Evidence:

- `forgez_battery.gazebo` sets nominal voltage to `22.2` V at line 12.
- `initial_charge`, `capacity`, and `design_capacity` are all `5.5` at lines 29-31.
- Nominal mode `E_lap` is `220.0` Wh in `forgez_battery.yaml` line 15.
- `charge_level_wh` is published as `q_ * et` in `battery_discharge.cpp` line 233.

Impact:

The physical capacity implied by charge and voltage appears smaller than the nominal lap budget. Even if these are placeholders, the scale is misleading for strategy thresholds and makes F1-style energy accounting hard to interpret.

Proposed remedy:

Define usable physical energy in Wh or MJ in one research config, convert to Gazebo battery units only at plugin boundaries, and keep physical capacity separate from FIA accounting counters.

Acceptance criteria:

- Full-charge published energy matches configured usable energy within a documented tolerance.
- Lap/event budget counters can be smaller, larger, or independent from physical state of charge without unit confusion.

### P1: Authored overlay, copied workspace source, and installed files are stale relative to each other

Evidence:

- `prepare-workspace.sh` copies overlay packages into `ws/src` and patches EUFS plugin CMake at lines 21-25.
- Energy audit found authored overlay chassis mass `740.0` kg in `overlay/eufs_racecar/eufs_racecar/urdf/macros.xacro` line 8, while current `ws/src/eufs_racecar/eufs_racecar/urdf/macros.xacro` line 8 and installed `ws/install/eufs_racecar/share/eufs_racecar/urdf/macros.xacro` line 8 still show `4.0` kg.
- Energy audit found authored battery link `base_link` in `overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo` line 10, while the installed/generated battery link is `chassis` in `ws/install/eufs_racecar/share/eufs_racecar/urdf/forgez_battery.gazebo` line 10.
- Energy audit did not find expected plugin shared libraries in the local `ws/install` tree.

Impact:

The files being edited and the files that a local runtime may use are not necessarily the same. This can invalidate experiments because mass, battery link, meshes, and plugin binaries may come from stale generated or installed output.

Proposed remedy:

Make the authored overlay the source of truth, regenerate `ws/src` from it, rebuild, and add an audit script that compares key authored/generated/installed files before experiments.

Acceptance criteria:

- After clean prepare/build, key `ws/src` and installed share files match the overlay or documented generated output.
- Expected plugin `.so` files exist in install paths.
- Experiment metadata records source commit, overlay file hashes, generated file hashes, and install file hashes.

### P1: Entrypoint mutates installed world/assets at runtime

Evidence:

- `entrypoint.sh` copies meshes from an old installed path to a new path at lines 11-18.
- It edits the installed `small_track.world` with `sed` at lines 20-23.
- `prepare-workspace.sh` also patches `small_track.world` using `sed` at lines 26-27.

Impact:

Runtime mutation makes reproducibility weaker and can hide packaging mistakes. It also suggests the install layout and world files are not fully controlled by source.

Proposed remedy:

Fix package install paths and world source files so runtime does not need corrective mutation.

Acceptance criteria:

- Container entrypoint only sources environments and starts the requested command.
- Installed mesh/world paths match source package declarations.
- Rebuilding from a clean workspace produces the same installed files.

### P2: Dependency/source provenance is only partially pinned

Evidence:

- `prepare-workspace.sh` clones `eufs_sim` tag `v2.1.0` and `eufs_msgs` tag `v2.0.0` at lines 18-19.
- `Dockerfile` uses `FROM osrf/ros:humble-desktop-full` at line 1, which is a moving image tag unless pinned by digest.
- Apt packages are not version-pinned in `Dockerfile` lines 9-25.
- README identifies the battery package as a Humble port in overlay but does not pin an upstream commit for that port.

Impact:

Rebuilding later can produce different ROS/Gazebo/apt dependency versions. Strategy experiments need reproducibility.

Proposed remedy:

Pin base image digest for serious experiments, record upstream commits for overlays, and save `rosdep`/apt environment metadata in experiment artifacts.

Acceptance criteria:

- Clean rebuild from documented sources reproduces package versions.
- Experiment logs include image digest, git commit, overlay commit/source, and config hashes.

### P2: Ground-truth semantics must be separated from estimated observations

Evidence:

- EUFS publishes ground truth topics such as `/ground_truth/state`, `/ground_truth/odom`, `/ground_truth/cones`, and `/ground_truth/track` in plugin xacros.
- The user requirement is opponent pose estimation from observations/history and speed gap, not privileged opponent state.

Impact:

Using ground truth directly in the planner can overstate performance and hide perception latency, association, and uncertainty problems.

Proposed remedy:

Provide two labelled observation modes:

- Oracle mode for debugging and upper-bound studies.
- Estimated mode for planner validation, using controlled noise, latency, covariance, and dropout.

Acceptance criteria:

- Every experiment records observation mode.
- Advisory/strategy validation uses estimated mode by default.
- Oracle results are never mixed with estimated-observation results.

## Dependency-Ordered Fix Plan

This is a proposal only. No fixes were executed in this audit.

### Phase 0: Make the platform launchable and reproducible

1. Verify current `entrypoint.sh` nounset mitigation with a fresh headless startup smoke test.
2. If startup still fails, fix the remaining shell/environment issue with exact log evidence.
3. Remove runtime mesh/world mutation by fixing install/source packaging.
4. Regenerate `ws/src` from authored overlays and rebuild install outputs from a clean state.
5. Add a pre-experiment drift check for authored overlay, generated workspace source, installed share files, and plugin libraries.
6. Record exact source and dependency provenance.
7. Add a headless smoke-test launch target.

Gate:

- Fresh container starts headless Gazebo/ROS.
- `ros2 topic list` is available.
- Existing single car spawns on `small_track`.
- Authored overlay, `ws/src`, and installed files are either matching or explicitly documented as generated differences.
- Required plugin libraries exist in the install tree and load successfully.

### Phase 1: Single-car physical credibility

1. Choose one authoritative benchmark vehicle dynamics model and disable all competing motion-control plugins in benchmark launches.
2. Restore or create a full F1 benchmark vehicle parameter file.
3. Replace potential-energy battery consumer with drivetrain/ERS energy accounting.
4. Couple battery power and temperature limits into longitudinal performance.
5. Wire `max_power_w`, `E_lap`, physical energy capacity, and rule counters into the same energy-limit model.
6. Validate deploy/regen/current sign conventions.
7. Fix timestep consistency in thermal dynamics and add derate feedback if thermal limits are used.
8. Add simple synthetic tests for flat constant speed, acceleration, braking recovery, and thermal growth.

Gate:

- Exactly one plant owns vehicle pose/velocity updates in benchmark mode.
- Ackermann velocity control and the EUFS race-car plugin cannot both be active for the same car.
- The selected plant exposes a documented command contract, state contract, and energy-power coupling point.
- Deployment policy changes speed trace and lap time.
- Energy accounting passes simple physical checks.
- Delta Energy is own battery energy change over time.
- `max_power_w` changes achievable acceleration through the selected plant.
- `E_lap` or rule-counter limits produce either deployment limiting or labelled violations.
- Same power profile at different physics steps gives consistent thermal traces.
- Full-charge published energy matches configured usable energy within tolerance.

### Phase 2: Namespace-clean two-car simulation

1. Make launch and xacro per-car namespace safe.
2. Spawn `/ego` and `/opp_1` with independent command/state/battery topics.
3. Add per-car TF/frame naming.
4. Add oracle opponent pose feed for debugging.
5. Add estimated opponent pose/history feed with noise and latency.
6. Confirm `vehicleModel` and `commandMode` are either active in the chosen plant mode or rejected for that launch.

Gate:

- Two cars can run independently without topic/frame collisions.
- Strategy consumes estimated opponent observations and derived speed gap.
- Oracle feed is available only under labelled debug mode.

### Phase 3: COTA benchmark

1. Add validated COTA world/map assets.
2. Add centerline, boundaries, spawn poses, lap distance, and event annotations.
3. Add configurable 2026 FIA deployment/overtake zones and leave event-specific settings external.
4. Validate one-car repeated laps and two-car spawn/interaction.
5. Validate world metres, mesh scale, track-coordinate arc length, and lap progress independently.
6. Choose any COTA benchmark speed profile only after the single-car plant and energy model pass validation.

Gate:

- COTA lap projection is stable.
- Turn/event sequence is correct.
- Two-car scenarios can be reset and replayed.

### Phase 3A: Opponent Observation Pipeline

1. Keep simulator truth for evaluation and debugging only.
2. Generate timestamped noisy and delayed opponent pose observations through an explicit observation model.
3. Project observations into track coordinates and estimate opponent speed from pose history.
4. Derive speed gap from estimated opponent speed and known ego speed.
5. Feed the planner only the observation-model output, never hidden opponent state.
6. Later replace the synthetic observation model with a detector/tracker while preserving the same planner interface.

Gate:

- Every experiment logs whether it used oracle truth, synthetic estimated observations, or detector/tracker observations.
- The planner interface is identical across synthetic and detector/tracker observation modes.
- Hidden simulator truth is not available to the planner in validation mode.
- Estimation latency, covariance, dropout, and association errors are recorded.

### Phase 4: Strategy, MPC, and advisory

1. Implement event-based strategy model.
2. Generate hold/prepare/attack/defend/abort alternatives.
3. Add tactical feasibility/MPC feedback.
4. Add driver advisory message lifecycle with expiry and cancel conditions.
5. Add a strategy engineer action/outcome explorer that exposes evaluated scope, deadline, freshness, uncertainty, and feasible tradeoffs.
6. Compare finite candidate sets against richer or exhaustive discretised oracle searches where tractable.
7. Compare against baselines.

Gate:

- System declines a feasible early pass when energy value is too high.
- It prepares a later pass and improves retained position or race outcome.
- The explanation cites gap, speed gap, energy delta, exit reserve, and uncertainty.
- Candidate-set adequacy is measured with missed beneficial actions, regret against the richer oracle, and sensitivity to action templates, discretisation, horizon length, and opponent-response assumptions.
- Probability calibration and ranking robustness are reported separately from vehicle/state/model uncertainty.
- Rare-case fallback behaviour is tested: infeasible attack, stale opponent estimate, high covariance, unexpected defensive move, energy derate, thermal derate, and missed planner deadline.
- The engineer display never claims "all possibilities" unless the finite search space is explicitly enumerated and exhausted.
- Unevaluated actions are marked unevaluated, not assigned zero probability.
- Driver-facing advice remains concise and accounts for human reaction delay and imperfect execution.

## Verification Plan

Minimum checks before using this for papers/demos:

- Clean build from pinned sources.
- Headless launch smoke test.
- Single-car energy conservation scenarios.
- Single-car lap replay on COTA.
- Two-car namespace test.
- Opponent observation calibration test.
- Strategy regression scenarios with fixed seeds.
- Runtime deadline logging for strategy, tactical planner, and MPC.
- Baseline comparisons using matched seeds.

## Open Questions

- Which vehicle plant should become the benchmark plant: patched EUFS dynamic bicycle or a new F1-specific plant plugin?
- What fidelity is required for tyre temperature and wear in the first milestone?
- What official COTA geometry source is acceptable for the project?
- Will the advisory be trackside-only, onboard, or simulation-only? Real F1 legality depends on the exact communication path and rule interpretation.
- Should manual override zones be modelled only as rule constraints, or also as driver-facing advisory events?

## Main Conclusion

EUFS is a reasonable integration base for ROS/Gazebo experiments, but the current local simulator is not yet valid for energy-aware F1 overtaking strategy. The immediate work is fresh runtime verification, energy-to-performance coupling, namespace-clean two-car operation, and a validated COTA map. The F1 CAD should remain a visual asset until the physics parameters, collision geometry, and vehicle dynamics are deliberately rebuilt around benchmark requirements.
