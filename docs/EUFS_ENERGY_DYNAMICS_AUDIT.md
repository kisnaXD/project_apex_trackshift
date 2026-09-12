# EUFS Energy And Dynamics Audit

Static audit only. I did not build, launch Gazebo, start Docker, install packages, or execute a remediation plan. I inspected authored overlay files, generated/copied workspace source, installed share files, launch plumbing, vehicle/battery plugins, EUFS vehicle models, messages, and track assets.

No `AGENTS.md` was present under `/home/gera/Desktop/EUFS_FORK_CONTAINERIZED`.

## Inventory And Scope

Inspected areas:

- `eufs-f1-sim/overlay/gazebo_ros_battery`: authored battery discharge and mechanical energy consumer plugins.
- `eufs-f1-sim/overlay/eufs_racecar`: authored F1 racecar URDF, battery URDF plugin wiring, launch file, meshes/config.
- `eufs-f1-sim/ws/src/gazebo_ros_battery`: generated/copied battery source currently in the local colcon workspace.
- `eufs-f1-sim/ws/src/eufs_racecar`: generated/copied racecar package currently in the local colcon workspace.
- `eufs-f1-sim/ws/src/eufs_sim/eufs_plugins`: upstream EUFS Gazebo plugins and vehicle model plugin.
- `eufs-f1-sim/ws/src/eufs_sim/eufs_models`: upstream point-mass/dynamic-bicycle models and noise model.
- `eufs-f1-sim/ws/src/eufs_msgs`: message definitions relevant to vehicle state, wheel speeds, forces, and telemetry.
- `eufs-f1-sim/ws/src/eufs_sim/eufs_tracks`: available EUFS tracks.
- `eufs-f1-sim/Dockerfile`, `scripts/prepare-workspace.sh`, `scripts/entrypoint.sh`, and `scripts/start-stack.sh`.
- `/home/gera/Downloads/strategy.pdf`, converted to text for PDF-specific design corrections.

Unavailable or not verified:

- I did not verify a running container or Gazebo runtime.
- I did not verify installed binary symbols or plugin load logs.
- I did not find COTA assets in the workspace.
- I did not inspect any imported external battery-management ROS package beyond files present in this workspace.
- The local `ws/src` and `ws/install` appear stale relative to `overlay`; findings distinguish authored overlay intent from generated/runtime state where possible.

## Critical Findings

### 1. Mechanical energy consumer ignores flat-track acceleration, drag, and braking energy

Severity: critical.

Evidence:

- `eufs-f1-sim/overlay/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:44` initializes from `model->GetWorldEnergyPotential()`.
- `eufs-f1-sim/overlay/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:54` reads `GetWorldEnergyPotential()` again.
- `eufs-f1-sim/overlay/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:55` computes `delta_energy` from potential energy only.
- `eufs-f1-sim/overlay/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:59` converts that potential-energy delta into power.
- The generated copy has the same mechanism at `eufs-f1-sim/ws/src/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:38` and `eufs-f1-sim/ws/src/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:48`.

Mechanism:

The consumer uses gravitational/world potential energy, not kinetic energy change, drivetrain work, wheel torque, aerodynamic drag, or commanded propulsion power. On a flat circuit, accelerating from 0 to high speed can produce almost no battery demand through this plugin except idle load.

Likely effect on research:

An energy-aware overtake strategy cannot be validated because the main cost of attacking on a straight is absent. The planner may learn or appear to prove strategies that exploit a nonphysical battery model.

Proposed remedy:

Replace the potential-energy consumer with a propulsion energy interface driven by commanded or achieved longitudinal force:

```text
P_mech = F_longitudinal * v_progress
P_batt = P_mech / eta_drive + P_aux
P_regen = eta_regen * min(braking_power, recovery_limit)
```

Couple this to the controller so battery power limits reduce available longitudinal force.

Acceptance check:

On a flat straight, a fixed acceleration run must reduce battery energy by approximately the kinetic energy increase plus drag/rolling losses divided by efficiency. A braking recovery run must increase usable battery energy within configured recovery and state-of-charge limits.

Remaining uncertainty:

Gazebo's internal `Battery` sign convention should be verified with a small plugin-level test before implementation.

### 2. Regeneration is effectively clipped by the mechanical consumer idle floor

Severity: critical.

Evidence:

- `eufs-f1-sim/overlay/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:60` adds a friction term based on absolute power.
- `eufs-f1-sim/overlay/gazebo_ros_battery/src/mechanical_energy_consumer.cpp:61` clamps power to at least `consumer_idle_power_`.
- `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:54` sets `consumer_idle_power` to `120.0`.
- Battery charging support exists in `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:171` and `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:177`, but the active mechanical consumer cannot publish negative power after the idle clamp.

Mechanism:

Even if potential energy drops or another calculation produced negative mechanical power, line 61 forces it to be at least positive idle load. The battery plugin can process negative loads, but this consumer never supplies one.

Likely effect on research:

Recovery zones, braking strategy, lift-and-coast, and COTA heavy braking sections cannot be represented through the current drive consumer.

Proposed remedy:

Model propulsion and recovery as separate signed flows or separate consumers. Keep auxiliary/idle load as a separate always-positive consumer.

Acceptance check:

During a simulated braking event with recovery enabled, `charge_level_wh` must rise unless the battery is full or recovery is limited by rules, tyre force, temperature, or speed.

Remaining uncertainty:

If a separate imported BMS package later adds a regen consumer, this specific consumer would still be wrong for drive energy but not the only charge path.

### 3. Battery state does not constrain propulsion

Severity: critical.

Evidence:

- Authored racecar URDF uses `libgazebo_ros_ackermann_drive.so` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/racecar.gazebo:126`.
- The Ackermann plugin receives `cmd_vel` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/racecar.gazebo:129`.
- Its maximum speed is fixed by URDF at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/racecar.gazebo:152`.
- Battery plugins are wired separately in `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:17` and `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:45`.
- There is no subscription or call path from `battery_state`, charge, temperature, or power limit into the Ackermann controller in the inspected authored URDF/plugin wiring.

Mechanism:

Battery telemetry changes independently of the vehicle controller. The controller can continue to request/achieve its velocity command even if battery state is low, hot, or rule-limited.

Likely effect on research:

The global planner cannot test the central tradeoff. Energy can be logged, but it does not alter lap time, acceleration, top speed, or ability to overtake.

Proposed remedy:

Insert an energy-aware propulsion layer between strategy/MPC commands and wheel force. It should expose available deploy power, recovery power, thermal derate, and rule derate, then limit achievable acceleration or torque accordingly.

Acceptance check:

With identical `cmd_vel`/speed targets, lower available battery power must produce measurably lower acceleration or lower terminal speed on a straight.

Remaining uncertainty:

The upstream EUFS dynamic vehicle plugin also exists in `ws/src`, but the authored overlay path currently favours the Gazebo ROS Ackermann plugin.

### 4. `E_lap` and `max_power_w` configuration is inert

Severity: high.

Evidence:

- `eufs-f1-sim/overlay/eufs_racecar/config/forgez_battery.yaml:10`, `:15`, and `:20` define `E_lap`.
- `eufs-f1-sim/overlay/eufs_racecar/config/forgez_battery.yaml:12`, `:17`, and `:22` define `max_power_w`.
- `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:52` to `:54` pass `T_core`, `E_lap`, and `R_OT` to xacro, but not `max_power_w`.
- `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:78` loads `forgez_E_lap_wh`.
- `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:128` to `:131` only logs the Forgez mode values.
- No inspected battery plugin code enforces `forgez_e_lap_wh_` or a max deploy power.

Mechanism:

The strategy-looking config values do not affect battery update, propulsion, or controller limits.

Likely effect on research:

Changing Harvest/Nominal/Attack may alter initial temperature/resistance but does not enforce lap energy or deployment power. Experiments that compare modes will not test the intended energy policy.

Proposed remedy:

Promote the rule and mode limits into an explicit `EnergyLimits` model used by both battery accounting and propulsion limiting. Pass `max_power_w` through xacro or ROS parameters if it remains mode-specific.

Acceptance check:

Setting `max_power_w` lower must reduce achievable acceleration. Setting a low lap counter must trigger a clear limit or violation flag before excess deployment.

Remaining uncertainty:

Future imported BMS packages may provide this, but no such active path was present in inspected files.

### 5. Thermal model is single-lump and dissipates without multiplying by timestep

Severity: high.

Evidence:

- Battery plugin has one temperature state `t_` at `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.hh:60`.
- Heat is accumulated in one scalar `heat_energy_` at `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.hh:61`.
- Heat generation multiplies power by `dt` at `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:207`.
- Heat dissipation is computed as `(t - ambient) * heat_dissipation_rate_` at `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:209`.
- The dissipation value is subtracted from energy at `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:210` without multiplying by `dt`.
- URDF enables temperature computation at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:34`.

Mechanism:

If `heat_dissipation_rate` has normal units of W/K, the model subtracts joules-per-second as joules every update, making cooling dependent on step rate. The model also cannot represent core/surface temperature lag, which matters for high-power hybrid deployment.

Likely effect on research:

Thermal constraints, pre-cooling, attack mode, and CBF-style thermal claims cannot be trusted.

Proposed remedy:

Use at least a two-node core/surface thermal model with timestep-consistent heat transfer:

```text
C_core * dT_core/dt = I^2 R - (T_core - T_surface) / R_core_surface
C_surface * dT_surface/dt = (T_core - T_surface) / R_core_surface - (T_surface - T_ambient) / R_surface_ambient
```

Acceptance check:

Running the same power profile at different physics step sizes should produce nearly the same temperature trace.

Remaining uncertainty:

The intended unit of `heat_dissipation_rate` is not documented in the inspected files.

### 6. Physical battery capacity is inconsistent with configured lap budgets

Severity: high.

Evidence:

- URDF battery nominal voltage is `22.2` V at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:12`.
- `initial_charge`, `capacity`, and `design_capacity` are all `5.5` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:29` to `:31`.
- Default nominal `E_lap` is `220.0` Wh at `eufs-f1-sim/overlay/eufs_racecar/config/forgez_battery.yaml:15`.
- `charge_level_wh` is published as `q_ * et` at `eufs-f1-sim/overlay/gazebo_ros_battery/src/battery_discharge.cpp:233`.

Mechanism:

If charge is in amp-hours and voltage is around 25 V, total stored energy is roughly 137 Wh, while the nominal lap budget is 220 Wh. The values may be placeholders, but they cannot support a 2026 F1 hybrid model or even the configured nominal lap budget.

Likely effect on research:

Energy budgets and state of charge will have misleading scales, causing the planner to learn thresholds that do not transfer to the intended car.

Proposed remedy:

Define physical battery capacity in joules or Wh directly for the research model, and convert to Gazebo `Battery` units only at the plugin boundary. Keep F1-rule accounting counters separate from physical capacity.

Acceptance check:

The published absolute energy at full charge must match the configured usable battery energy within a documented tolerance.

Remaining uncertainty:

Gazebo `Battery` model details should be checked before final unit conversion.

### 7. Workspace source and install tree are stale relative to the authored overlay

Severity: high.

Evidence:

- `scripts/prepare-workspace.sh:21` to `:25` copies overlay packages into `ws/src` and patches EUFS plugin CMake.
- Authored overlay chassis mass is `740.0` kg at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/macros.xacro:8`.
- Current `ws/src` chassis mass is `4.0` kg at `eufs-f1-sim/ws/src/eufs_racecar/eufs_racecar/urdf/macros.xacro:8`.
- Current `ws/install` chassis mass is also `4.0` kg at `eufs-f1-sim/ws/install/eufs_racecar/share/eufs_racecar/urdf/macros.xacro:8`.
- Authored battery link is `base_link` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:10`, while current installed/generated battery link is `chassis` at `eufs-f1-sim/ws/install/eufs_racecar/share/eufs_racecar/urdf/forgez_battery.gazebo:10`.
- `find eufs-f1-sim/ws/install` did not reveal expected plugin shared libraries such as `libgazebo_ros_battery_discharge.so`, `libgazebo_ros_mechanical_energy_battery_consumer.so`, or `libgazebo_race_car_model.so`.

Mechanism:

The local workspace/install state does not reflect the authored overlay. Static inspection cannot establish which version a running container would use unless it is rebuilt from the Dockerfile.

Likely effect on research:

Experiments may be run against different URDF, mass, battery link, and plugin build state than the files being edited.

Proposed remedy:

Make the build pipeline single-source-of-truth. Add a non-destructive audit command that compares overlay, `ws/src`, and installed share files before running experiments.

Acceptance check:

After a clean prepare/build, key files in `ws/src` and installed share must match overlay or intentionally generated output. Expected plugin `.so` files must exist in install paths and load successfully.

Remaining uncertainty:

I did not run the Docker build, so this is a local workspace-state finding, not a claim about a fresh container image.

## Additional Findings

### 8. Vehicle model launch arguments are misleading on the authored Ackermann path

Severity: medium-high.

Evidence:

- EUFS track launch files pass `vehicleModel` and `commandMode` into `load_car.launch.py`, for example `eufs-f1-sim/ws/src/eufs_sim/eufs_tracks/launch/small_track.launch:63` and `:64`.
- The authored `load_car.launch.py` declares those arguments at `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:115` and `:116`.
- The same authored launch file processes xacro with mappings only for `config_file`, `forgez_mode`, `forgez_T_core`, `forgez_E_lap`, and `forgez_R_OT` at `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:47` to `:55`.
- The authored robot includes `racecar.xacro` and `forgez_battery.gazebo` at `eufs-f1-sim/overlay/eufs_racecar/robots/eufs/robot.urdf.xacro:10` to `:11`; it does not include `eufs_plugins.gazebo.xacro`.
- The authored `racecar.gazebo` uses `libgazebo_ros_ackermann_drive.so` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/racecar.gazebo:126`.

Mechanism:

On the authored overlay path, selecting `vehicleModel:=DynamicBicycle` or `commandMode:=acceleration` does not activate the upstream EUFS dynamic-bicycle plugin. Those arguments are accepted by launch plumbing but not consumed by the xacro that is actually spawned.

Likely effect on research:

Experiments can be mislabeled. A run may be described as DynamicBicycle/acceleration-mode while actually using Gazebo ROS Ackermann velocity control.

Proposed remedy:

Make the selected plant explicit. Either remove dead launch arguments from the Ackermann path, or add a separate launch mode that includes the upstream EUFS race-car plugin and a complete compatible config.

Acceptance check:

For every benchmark launch, the generated robot description should contain exactly the expected motion plugin, and a smoke test should confirm the expected command topic and dynamics path.

Remaining uncertainty:

I did not launch Gazebo, so this is a static launch/xacro wiring finding.

### 9. EUFS dynamic vehicle model path is not suitable as-is for energy research

Severity: medium-high.

Evidence:

- Upstream plugin directly sets world pose and velocity at `eufs-f1-sim/ws/src/eufs_sim/eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp:363` to `:365`.
- It updates state with a kinematic/dynamic model at `eufs-f1-sim/ws/src/eufs_sim/eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp:598`.
- `DynamicBicycle::_getFx` uses requested acceleration and drag at `eufs-f1-sim/ws/src/eufs_sim/eufs_models/src/dynamic_bicycle.cpp:73` to `:76`.
- The model has aero drag and downforce terms at `eufs-f1-sim/ws/src/eufs_sim/eufs_models/src/dynamic_bicycle.cpp:83` to `:85`, but no battery power, tyre temperature, tyre wear, or energy accounting.
- `VehicleModel::validateInput` clips acceleration/velocity/steering by YAML ranges at `eufs-f1-sim/ws/src/eufs_sim/eufs_models/src/vehicle_model.cpp:10` to `:22`.

Mechanism:

This path is a model-state setter, not a fully coupled vehicle/energy simulation. It can be useful for fast strategy prototyping, but not for proving energy effects unless extended.

Likely effect on research:

It can produce plausible motion while bypassing contact-rich dynamics and battery causality.

Proposed remedy:

Use it only as a fast benchmark model unless extended with an explicit energy/power-limited longitudinal model. Treat Gazebo contact/URDF path and analytical dynamic-bicycle path as separate simulator tiers.

Acceptance check:

A commanded acceleration above available battery power must be clipped by the same limit in both analytical and Gazebo-contact paths.

Remaining uncertainty:

The authored overlay patch says it skips `gazebo_race_car_model`, but current `ws/src/eufs_sim/eufs_plugins/CMakeLists.txt` still includes it. Runtime build state is unverified.

### 10. Current race car model config is incompatible with upstream EUFS dynamic model expectations

Severity: medium if using the authored Ackermann path; medium-high if using the upstream EUFS dynamic model path.

Evidence:

- `eufs-f1-sim/ws/src/eufs_racecar/robots/eufs/configDry.yaml:1` to `:4` contains only a stub `vehicle.mass` and `vehicle.wheelbase`.
- `eufs-f1-sim/ws/src/eufs_sim/eufs_models/include/eufs_models/vehicle_param.hpp:38` to `:42` expects top-level `inertia`, `kinematics`, `tire`, `aero`, and `input_ranges`.
- `eufs-f1-sim/ws/src/eufs_sim/eufs_launcher/launch/simulation.launch.py:20` to `:31` defaults to `DynamicBicycle` and `configDry.yaml`.

Mechanism:

If the upstream EUFS dynamic model path is launched with this config, YAML parsing should fail or produce invalid state because required nodes are missing. This is not proven active on the authored Ackermann path; it is an integration risk for anyone trying to use the EUFS `DynamicBicycle` plugin.

Likely effect on research:

The dynamic-bicycle path may be unusable until config is restored or replaced. The more immediate authored-path risk is that launch arguments can suggest DynamicBicycle while the generated robot still uses Ackermann control.

Proposed remedy:

Provide a complete F1-oriented model config or route launches explicitly to the Ackermann/URDF path while the dynamic model is disabled.

Acceptance check:

Launching the selected vehicle model should parse config without exceptions and publish vehicle state for a short dry run.

Remaining uncertainty:

I did not run the launch file.

### 11. Upstream `ToQuaternion` implementation writes into a reserved but zero-sized vector

Severity: medium if built/used.

Evidence:

- `eufs-f1-sim/ws/src/eufs_sim/eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp:643` calls `q.reserve(4)`.
- Lines `:645` to `:648` write `q[0]` through `q[3]` without resizing.

Mechanism:

`reserve` changes capacity, not size. Indexing an empty vector is undefined behaviour.

Likely effect on research:

If this plugin is built and used, publishing car state or odometry can crash or corrupt memory.

Proposed remedy:

Use `std::array<double, 4>` or initialize `std::vector<double> q(4)`.

Acceptance check:

Run the plugin under AddressSanitizer or a short launch that publishes state without memory errors.

Remaining uncertainty:

The authored overlay patch may intend not to build this plugin.

### 12. Tyre/grip model is static and lacks tyre state

Severity: medium-high.

Evidence:

- Gazebo URDF sets constant wheel friction coefficients `mu1` and `mu2` to `1.0` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/racecar.gazebo:52` to `:53`, `:70` to `:71`, `:88` to `:89`, and `:106` to `:107`.
- Upstream dynamic model uses Pacejka-like lateral force from static parameters at `eufs-f1-sim/ws/src/eufs_sim/eufs_models/src/dynamic_bicycle.cpp:87` to `:96`.
- There is no inspected tyre temperature or wear state update.

Mechanism:

Grip is fixed or parameterized only by static slip-angle equations. It does not respond to stint age, thermal state, lockups, sliding, following, or energy deployment.

Likely effect on research:

Attack/defend choices cannot trade energy against tyre degradation or retention through corner exit.

Proposed remedy:

Add tyre state variables: compound, wear, carcass/surface temperature, grip multiplier, and uncertainty. Couple them to braking, acceleration, sliding, and cooling.

Acceptance check:

Repeated aggressive attacks should measurably reduce future grip or increase tyre temperature compared with efficient following.

Remaining uncertainty:

No external tyre package was present in the inspected workspace.

### 13. Ground-truth topics are enabled by default and can leak oracle state

Severity: medium for research validity.

Evidence:

- `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:119` declares `pub_ground_truth` default `true`.
- EUFS simulation launch also defaults `pub_ground_truth` true at `eufs-f1-sim/ws/src/eufs_sim/eufs_launcher/launch/simulation.launch.py:55` to `:57`.
- Upstream plugin publishes ground truth car state when enabled at `eufs-f1-sim/ws/src/eufs_sim/eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp:412` to `:414`.
- It publishes ground truth odometry if enabled at `eufs-f1-sim/ws/src/eufs_sim/eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp:522` to `:524`.

Mechanism:

If planners subscribe to ground-truth topics during training or evaluation, results can overstate real advisory performance.

Likely effect on research:

Opponent pose estimation and speed-gap inference may be bypassed accidentally.

Proposed remedy:

Separate oracle, training, and operational topic namespaces. Require operational strategy nodes to consume only estimator outputs with covariance and latency.

Acceptance check:

Run an experiment with all ground-truth topics disabled; strategy should still operate using estimated pose and report higher uncertainty.

Remaining uncertainty:

The current authored Ackermann path publishes `/odom`; the exact operational subscriptions are not present yet.

### 14. Multi-car simulation will collide in ROS namespaces unless refactored

Severity: medium-high for the two-car benchmark.

Evidence:

- Battery plugin namespace is hard-coded to `/eufs` at `eufs-f1-sim/overlay/eufs_racecar/eufs_racecar/urdf/forgez_battery.gazebo:20` and `:47`.
- `load_car.launch.py:27` reads a `namespace` launch argument, but uses it as Gazebo entity name at `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:76`.
- `robot_state_publisher` and `joint_state_publisher` are not placed under that namespace at `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:60` to `:95`.
- Joint states remap to `/eufs/joint_states` at `eufs-f1-sim/overlay/eufs_racecar/launch/load_car.launch.py:94`.

Mechanism:

Spawning two cars can cause shared topics and node names unless launch groups/namespaces are applied consistently.

Likely effect on research:

Two-car COTA benchmark may mix telemetry, battery state, odometry, and joint state across cars.

Proposed remedy:

Make car namespace a first-class launch parameter propagated into every ROS node, plugin namespace, remap, frame prefix, and entity name.

Acceptance check:

Spawn two cars and verify distinct topics such as `/car_a/forgez/battery_state`, `/car_b/forgez/battery_state`, `/car_a/odom`, `/car_b/odom`, with no duplicate node-name warnings.

Remaining uncertainty:

I did not spawn multiple cars.

### 15. No COTA circuit asset is present

Severity: medium for roadmap.

Evidence:

- Searching for `cota`, `austin`, and `circuit` under `eufs-f1-sim` returned no COTA asset.
- Available track assets are EUFS tracks such as `small_track`, `skidpad`, `rectangle`, `comp_2021`, and `hairpins_increasing_difficulty` under `eufs-f1-sim/ws/src/eufs_sim/eufs_tracks`.

Mechanism:

The agreed second milestone depends on COTA-specific layout, straights, braking zones, and 2026 overtaking/activation parameters.

Likely effect on research:

The two-car benchmark cannot represent the target circuit until COTA geometry and rule metadata are imported.

Proposed remedy:

Create a COTA track package with centerline, boundaries, elevation if needed, event graph, braking/recovery zones, and version-pinned rule metadata.

Acceptance check:

The planner can convert pose to COTA Frenet coordinates and identify named events such as Turn 1 braking, Turn 11 exit, back straight, Turn 12 braking, and lap boundary.

Remaining uncertainty:

The COTA asset may exist outside this workspace.

## Suggested Fix Plan

Do not start with the global planner. Fix simulator causality first.

1. Stabilize the build/source-of-truth pipeline.

   Regenerate `ws/src` from overlay, rebuild, and verify installed files and plugin libraries match the intended source. Add a pre-experiment audit script to detect overlay/workspace/install drift.

2. Choose one vehicle simulation path for the first milestone.

   For a credible single-car energy model, either extend the Gazebo Ackermann/contact path with energy-limited force control, or explicitly use an analytical dynamic-bicycle path with energy coupling. Mixing both without a clear boundary will make validation ambiguous.

3. Replace potential-energy battery load with propulsion/recovery accounting.

   Add longitudinal force/power accounting, drag/rolling losses, auxiliary load, and regeneration as signed flows. Keep physical battery energy and regulatory counters separate.

4. Couple battery limits back into motion.

   Low state of charge, thermal derate, and rule deployment limit must reduce available acceleration/top speed. Recovery must be limited by braking demand, tyre grip, battery headroom, thermal state, and rules.

5. Repair battery units and thermal model.

   Define usable battery energy in Wh/MJ, map to plugin units at boundaries, and replace the one-lump timestep-dependent thermal model with at least a timestep-consistent two-node model.

6. Add operational telemetry topics.

   Publish own absolute energy, signed own Delta Energy, deploy power, recovery power, rule counters, thermal state, derate reason, and validity flags. Keep oracle topics separate.

7. Prepare two-car namespace support.

   Namespaces must cover nodes, plugin ROS namespaces, remaps, frames, and Gazebo entities. This is required before opponent pose estimation can be trusted.

8. Import COTA and build the event graph.

   Add centerline, boundaries, event points, braking/recovery zones, and rule config placeholders with pinned source URLs and issue dates.

9. Add estimation-only opponent interface.

   Feed the strategy layer opponent pose with covariance and identity. Derive signed track gap and closing speed from time-aligned pose history. Do not feed rival battery or intent as ground truth.

10. Build the two-car strategic benchmark.

   Only after single-car energy validation passes, create the COTA scenario with at least two passing opportunities and one recovery opportunity.

## Acceptance Tests For The Roadmap

Minimum single-car tests:

- Flat acceleration consumes energy consistent with kinetic energy plus losses.
- Constant-speed straight consumes energy consistent with drag and rolling loss.
- Braking recovery increases battery energy when allowed and below full charge.
- Low battery or thermal derate reduces achievable acceleration.
- Re-running the same thermal power profile at different timesteps gives consistent temperature traces.

Minimum two-car tests:

- Two cars publish isolated telemetry namespaces.
- Opponent pose estimator reports covariance and survives missed detections.
- Signed track gap handles lap wraparound.
- Closing speed is derived from time-aligned track-coordinate history.
- Strategy can decline an immediate pass and choose a later pass because of future energy value.
- Position retention, not just overlap, is part of success.

Minimum research-validity checks:

- Operational planner does not subscribe to ground-truth topics.
- Oracle results are labelled separately.
- Rule values are loaded from pinned 2026 FIA sources.
- No claim of global optimality is made unless the discretised benchmark is exhaustively solved.
