# Quick three-layer overtake prototype

This is the active delivery plan. It replaces the detailed plan's prerequisite sequence for the first running demo. Keep the three architectural responsibilities; implement their simplest useful versions and improve them after the cars run together.

## First deliverable

One launch starts two visible F1 cars on a short COTA back-straight scenario. Ego uses EUFS vehicle dynamics, energy-aware strategy, tactical planning and a real receding-horizon MPC. Russell follows the frozen, adapted FastF1 reference exactly. Ego chooses a feasible passing side, passes and reaches clear separation. Save enough logs to see the decision, planned path, actual path, energy use and solver result.

This is a working prototype, with an easy scenario and approximate models. It is not yet evidence of optimal race strategy, realistic F1 calibration or robustness across race situations.

## What each layer does now

| Layer | First implementation |
| --- | --- |
| 1 — Energy strategy | Compare attack and follow using current gap, a simple pass-time/energy estimate, a configurable energy price and a hard reserve. Issue attack/follow intent, deployment budget and minimum exit energy. Replan around 1 Hz. No dynamic programme or scenario tree yet. |
| 2 — Tactical planning | Generate follow, left-pass and right-pass trajectories from the COTA map. Predict Russell from recent pose-derived speed. Check basic track corridor and vehicle separation; select a feasible side and keep it during the pass. Return to the reference after full-car clearance. Replan around 5 Hz. |
| 3 — MPC | Use a reduced bicycle model and an initial approximately 2-second horizon with around 20 intervals. Optimize steering, acceleration and electrical assistance against the tactical reference and energy budget. Include the native input delay, steering/acceleration limits, corridor and reserve. Warm-start and brake on unusable solver output. Target 10–20 Hz initially; measure achieved rate. |

The MPC must solve an actual optimization problem. A path follower must not be presented as MPC. The strategy's first energy price and future reserve are explicit heuristics; the first implementation does not claim globally optimal energy allocation.

## Minimum supporting work

- **Ego plant:** finish the existing opt-in native hybrid connection so electrical assistance changes delivered propulsion and stored energy changes with delivered power. Use the existing synthetic profile. Do not recalibrate tyres, temperature, FIA rules or the whole vehicle for this milestone.
- **Russell:** finish the existing static/kinematic Gazebo replay plugin. Reuse the F1 visual with the correct transform. Use the already exported Russell 2025 US GP lap 28 reference mapped onto the existing COTA geometry. Exact playback refers to this frozen adaptation, not exact historical position accuracy.
- **State and command path:** reuse the existing geometry, estimator, contracts and command arbiter. Begin with zero added opponent noise/delay; estimate opponent speed from pose history. Only ego publishes vehicle commands. Russell's future reference stays private to playback.
- **Runtime:** thin ROS adapters and one scenario entrypoint. Reuse the isolated container and existing Gazebo view. Start both cars near the back straight, allow a short unscored acceleration phase, and give ego enough permitted pace to make the easy pass possible. Restart the launch between early experiments instead of building elaborate reset/operator workflows.
- **Evidence:** one run bundle with simulation time, ego/Russell poses, own energy, strategy intent/budget, candidate and selected paths, requested/applied commands, MPC status and pass/overlap/track events. No new GUI or evidence browser is required.

## Execution order

1. **Complete the vehicle endpoints.** Compile the native ego extension and the exact replay plugin, start them together, and confirm that only ego has a dynamic vehicle controller. Check motion, reference alignment and basic energy-to-force coupling.
2. **Make short-horizon MPC drive ego.** Get a simple solve working, then track a short straight/gentle path in Gazebo. Tune only what blocks stable motion. Do not require a full lap, comprehensive model parity or a 50 Hz qualification first.
3. **Connect the thin strategy and tactical layers.** Use a basic energy/reserve decision and three path candidates. Feed the selected trajectory into MPC. Decisions depend on observed gap, feasibility and own energy; the scenario does not contain an instruction to pass left or right.
4. **Run the overtake.** Adjust the declared starting gap and pace until this easy situation produces a clear pass. Keep checks for unusable commands, body overlap and leaving the track. Fix observed blockers and rerun the same short scenario.
5. **Deliver the runnable prototype.** Provide one command, the chosen configuration and the recorded outcome. A successful first run unlocks iteration; broader repeated trials and additional situations follow.

Agents write larger coherent batches. The primary agent performs a build/basic execution check at the component boundary, then reviews issues exposed by integration. Do not repeatedly review in-progress edits or expand a working component while another required component is missing.

## Explicitly later

Full nine-state/six-second MPC; detailed tyre/thermal calibration; FIA rule fidelity; global dynamic programming and branching opponent hypotheses; optimal waiting/retention studies; noisy/dropout sweeps; exhaustive fault injection; full-lap and repeated-seed qualification; comprehensive reset/manual-operation workflows; rich GUI, judge layout and searchable replay inspectors.

The original recovery snapshot stays preserved. Existing deeper implementation work remains available for later use. Prototype limitations and any failed run remain visible in the report.
