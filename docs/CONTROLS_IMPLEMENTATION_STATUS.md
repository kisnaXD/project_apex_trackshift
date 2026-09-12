# Controls implementation and review record

**Active delivery plan:** [Quick three-layer overtake prototype](TWO_CAR_QUICK_DEMO_PLAN.md). The user wants the crudest useful implementation of all three layers running together quickly. Energy strategy and real MPC remain in scope. The [detailed architecture and GUI plan](TWO_CAR_CONTROL_ARCHITECTURE_PLAN.md) is the longer-term design, not a list of prerequisites for the first pass. Luna agents write code; the primary agent runs component build/smoke checks and fixes integration blockers through the agents. Existing simulator behavior remains the recovery baseline.

**Review cadence (latest user steering):** roughly halve review frequency. Let agents finish coherent components before review; do not inspect or compile their in-progress edits. Batch review and required tests at completed layer milestones, then at integration/driving gates. Focus detailed review on defects that affect behavior, contracts or recorded evidence. Use Luna with staggered retries and fresh task contexts when needed; the user chose to retain Luna after being offered available alternatives.

The user confirmed the language boundary before resuming implementation: strategy, tactical planning, orchestration, logging and GUI remain Python; MPC uses Python model definitions with a generated numerical solver; the existing C++ EUFS physics receives an optional native hybrid extension. There is no whole-stack C++ migration.

**Current priority:** the user has deferred judge-view and GUI work. Continue only the estimator, deterministic opponent, physical coupling, three control layers, and logging/validation necessary for algorithm operation. Existing GUI edits remain available for later review; further GUI enhancements are paused.

**Opponent decision:** the user explicitly confirmed exact trajectory playback. Only ego uses native vehicle dynamics and the three control layers. Russell is a simulation-clock-driven kinematic replay of the frozen adapted reference, with no vehicle controller, CAN mission or hybrid model. The prior two-native-car readiness run remains baseline evidence only. The operational planner receives synthetic pose observations; private reference data remains with playback and evaluation.

Progress communicated after this clarification: approximately **25% of the headless controls milestone**, an engineering estimate rather than a count of files. No overtake has yet been executed by the new stack.

## Recovery point

- Original branch: `feat/modular-grid-telemetry`.
- Original parent commit: `7c08f726a3880875d8a74091030c3c2b189ae07a`.
- Implementation branch: `feat/two-car-controls`.
- Snapshot directory: `/home/gera/Desktop/EUFS_SNAPSHOTS/pre-controls-20260912T183012Z`.
- Full workspace: `workspace.tar.gz`; all **16,155 regular files** verified against `workspace-manifest.json`. Includes uncommitted files, nested Git repositories and ignored local artifacts.
- Preserved container: `eufs-f1-sim`, stopped at snapshot time.
- Preserved image: `eufs-f1-sim:pre-controls-20260912t183012z`.
- Image ID: `sha256:d1809f825b8f2054d2ce6455e45524826c5cb1f9f6e9fc0462cdc446ac2cb317`.
- Portable image: `runtime-image.tar.gz`, 1,440,687,660 bytes; SHA256 `1954366869c7c8ec0a992ce97a0a52b657fd92b942194b93a27199b5911069f8`.
- Recovery instructions: `RESTORE.md` in the snapshot directory. Recover into a new directory for inspection before replacing any working tree.

## Baseline verification

Existing Python suite: **32 passed**, before new controls tests were added. Host pytest needs the authored overlay and simulator root on `PYTHONPATH`; automatic ROS pytest plugin loading is disabled for these unit tests.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/eufs-f1-sim:$PWD/eufs-f1-sim/overlay/eufs_racecar:$PWD/eufs-f1-sim/scripts:$PYTHONPATH" \
python3 -m pytest -q eufs-f1-sim/tests
```

This establishes existing unit-test behavior; it is not a fresh live driving/physics validation.

## Review gates

| Stage | Status | Evidence / remaining gate |
| --- | --- | --- |
| Snapshot and baseline | Preserved; baseline and two-car readiness pass | Workspace verification, portable image export, 32 existing tests, fresh full COTA lap and both namespace/TF/mission acknowledgements |
| Contracts, geometry, estimator, supervision | Pure-core tests and ROS build pass | 12 focused tests pass, including corrected yaw-wrap/swept collision and positive closing speed across a corner/lap seam. Humble interfaces compile and representative messages round-trip. ROS runtime adapters remain to be integrated |
| Causal recording and Russell reference | 16 focused tests pass; offline reference checks pass | Provisional Russell lap 28 adaptation has continuous seam speed and bounded acceleration. 14,708 sampled body corners are inside COTA. Exact kinematic Gazebo playback, strict loader validation and final provenance remain review gates |
| GUI foundation | Initial tests, rendering and ROS bridge pass; further work deferred | 13 tests passed; actual baseline telemetry rendered offscreen; Humble bridge connects and shuts down cleanly. Dedicated inspectors, render batching and GUI integration await the later presentation milestone |
| Physical coupling | Configured pure model accepted; native integration in progress | Nine hybrid tests pass; seven compiled C++ cases exactly match Python. Two thousand seeded randomized cases respect force/energy/store/lap-counter bounds; actual native force coupling remains the next gate |
| Vehicle MPC | Pending | Useful horizon, latency-aware feasibility and measured deadline checks |
| Tactical planning | Pending | Candidate corridors, constraints, exit/abort feasibility |
| Race strategy | Pending | Continuation value, alternatives, repeatable pass/decline/wait outcomes |
| Headless two-car controls | Pending | Exact Russell replay, ego D1 execution, complete causal logs and one-lap regression |
| Integrated GUI demo | Deferred | Presentation/replay, inspectors and GUI acceptance after the controls milestone |

No stage is considered accepted solely because code was generated. Review findings and measured validation results will replace the entries above as implementation proceeds.

The combined existing/new test suite passed **79 tests in 8.31 s** on the host at this checkpoint. This result covers current unit tests and does not imply that the three planning layers or the judge-facing demo are implemented.

Subsequent focused foundation review passed **12 tests**. A full-buffer COTA estimator probe measured a 14.5 ms median and 43.6 ms maximum over 15 calls after bounding the recent progress fit. This is an initial host measurement, not a qualified end-to-end controller deadline result. Reprojecting the entire history previously took about 703 ms and was removed from the estimate path.

Pure hybrid validation uses the explicitly synthetic profile. Seven sequential/constrained compiled C++ cases (drive, recovery, zero speed, reserve, hot pack, exhausted deployment quota, fully worn tyres) match the Python outputs exactly. A separate 2,000-case probe with seed 7381 checks energy balance, force allocation, force-circle limits, store bounds and electrical lap budgets; maximum energy-balance error was `2.3267e-10 J`. This verifies the allocator under tested inputs, not plant-level motion or calibrated F1 performance. At the 15 m/s pilot pace, default ICE capability masks electrical acceleration benefit; the controls profile therefore needs separately declared propulsion limits for low-speed energy experiments.

## Isolated validation environment

The preserved image runs in a separate developer container, `eufs-controls-dev-20260913`, with ROS domain 42 and Gazebo master `http://127.0.0.1:11365`. Its workspace mount is read-only. The canonical `eufs-f1-sim` container remains stopped and unchanged. Build products and live baseline traces are stored in the developer container or ignored `.tmp` paths.

The preserved native plugin SHA256 is `806b4d14fa991fd86da7bf6d9f8a259bd520864fb1c0efffa08417581babecaf`. A fresh COTA baseline completed 5,513.15 m, stopped at zero velocity, and reported no failure or external command override. The 4,808 control samples have a minimum sampled body clearance of 3.908 m. The run took 485 simulation seconds and 496.81 wall seconds. [Baseline evidence](validation/controls_preserved_baseline.json) records artifact hashes and the exact validation scope.

An isolated two-car launch also published distinct odometry/frame chains and acknowledged both manual missions. Native readiness is `AMI_MANUAL` **or** `AS_DRIVING`; a manual mission correctly retains `AS_OFF`. Both cars had zero competing command publishers during this check. [Two-car readiness evidence](validation/controls_two_car_readiness.json) records the observed namespaces, transforms and CAN states. This baseline launch has now been stopped; the new controls launch will use a dynamic ego and kinematic Russell replay.

The acados solver toolchain was compiled from tag `v0.5.5`, commit `59d93e17d2985fdd73fc58b8a83ed8f83a024171`. Its official nonlinear pendulum example generated C code, compiled, and solved successfully using CasADi 3.7.2 and NumPy 1.26.4 in a workspace-local environment. This verifies the solver toolchain only; the vehicle MPC and its deadline targets are not yet validated.

The latest [provisional Russell reference review](validation/controls_russell_reference_review.json) checks 3,677 samples over 5,513 m at a maximum 15 m/s, with an adapted duration of 488.264 s. Interval acceleration stays within -1.2 to +1.0 m/s², seam speed difference is zero, and the minimum sampled body clearance is 5.118 m. Finite-difference progress agrees with reference speed to `7.1e-8 m/s` at the sampled interval midpoints. This is an offline geometry/pace result, not a Gazebo playback or ego controller result.
