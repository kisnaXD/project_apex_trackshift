# Streamlit 2D F1 Energy And Overtake Simulator Plan

Date: 2026-09-12

Status: implementation plan only. No Streamlit app, data download, solver integration, or historical dataset import is implemented by this document.

## Purpose

Build a custom 2D Python simulator that can validate the race-strategy ideas before they are moved back into ROS/Gazebo. Streamlit is only the UI and experiment control surface. The simulator core must be deterministic, fixed-step, headless, testable, and callable from scripts, CI, notebooks, and later ROS/Gazebo adapters.

The first research benchmark is a single own car with credible energy, tyre, and lap-time effects. The second benchmark is a two-car COTA scenario with opponent pose estimation and speed-gap inference. Later milestones add tactical MPC, stochastic global planning, advisory output, and multi-car traffic.

## Source And Library Baseline

Use a small library set at first:

| Need | Baseline library | Why | Notes |
| --- | --- | --- | --- |
| Array math and deterministic vector operations | NumPy | Standard numerical array base for the engine | Pin version in requirements and CI |
| Interpolation, filtering, optimization utilities | SciPy | Mature scientific routines for interpolation, integration checks, and non-MPC optimization | Use fixed-step simulator integration even if `solve_ivp` is used for offline validation |
| Dataframes and telemetry export | pandas plus PyArrow for Parquet | CSV/Parquet experiment outputs and replay preprocessing | Keep schemas explicit and versioned |
| Interactive plots | Plotly | Streamlit-friendly telemetry, track, outcome-tree, and confidence plots | Plotly is UI only, not simulator state |
| UI/session control | Streamlit | Fast experiment dashboard | Use session state, caching, and fragments carefully; no physics loop inside normal widget reruns |
| QP baseline | OSQP | Convex quadratic programs with linear constraints | Do not claim SOCP support; use only for suitable QP subproblems |
| Nonlinear MPC prototyping | CasADi | Symbolic NLP and optimal-control prototyping | Good for research prototypes, not a hard real-time promise |
| Faster generated MPC later | acados | OCP solver path for faster deployed MPC experiments | Add after the model stabilizes |
| Battery reference modelling | PyBaMM | Offline/reference battery model and calibration support | Do not run full electrochemistry in every real-time Streamlit frame initially |
| Historical F1 timing/telemetry access | FastF1, if licensing and available channels are acceptable | Reproducible timed-lap baseline and session/lap metadata | Treat position and telemetry quality as dataset-limited, not ground truth for all states |

Primary documentation to check and pin when implementation begins:

- Streamlit architecture, fragments, session state, and caching: `https://docs.streamlit.io/develop/concepts/architecture`, `https://docs.streamlit.io/develop/api-reference/execution-flow/st.fragment`, `https://docs.streamlit.io/develop/api-reference/caching-and-state/st.session_state`, `https://docs.streamlit.io/develop/api-reference/caching-and-state/st.cache_data`, `https://docs.streamlit.io/develop/api-reference/caching-and-state/st.cache_resource`
- NumPy docs: `https://numpy.org/doc/stable/`
- SciPy docs: `https://docs.scipy.org/doc/scipy/`
- Plotly Python docs: `https://plotly.com/python/`
- pandas IO docs: `https://pandas.pydata.org/docs/`
- PyArrow Parquet docs: `https://arrow.apache.org/docs/python/parquet.html`
- OSQP docs: `https://osqp.org/docs/`
- CasADi docs: `https://web.casadi.org/docs/`
- acados docs: `https://docs.acados.org/`
- PyBaMM docs: `https://docs.pybamm.org/`
- FastF1 docs: `https://docs.fastf1.dev/`

Pin exact versions in `requirements.txt` or `uv.lock`. Store the docs/date reviewed in a `docs/sources.md` file during implementation.

## Architecture

The simulator has one authoritative engine clock and one authoritative dynamics integration path. Streamlit never owns simulation time.

```text
streamlit_app/
  app.py
  pages/
  components/
f1sim/
  engine/
    world.py
    clock.py
    scenario.py
    replay.py
  dynamics/
    point_mass.py
    bicycle.py
    tyres.py
    aero.py
    fuel.py
    ers.py
    battery.py
  estimation/
    opponent_tracker.py
    sensors.py
    track_projection.py
  planning/
    global_strategy.py
    tactical.py
    mpc.py
    advisory.py
  data/
    schema.py
    export.py
    fastf1_adapter.py
  tests/
```

The Streamlit app sends commands such as start, pause, step, reset, load scenario, change seed, and export run. A background worker or explicit step function advances the engine. UI reruns read immutable snapshots from session state or a bounded run store.

No widget callback should run an infinite simulation loop. Long planner calls must support cancellation, timeout, stale-result rejection, and a result timestamp. If the user changes SOC, tyres, start gap, or scenario seed while a solver is running, the app must discard old results unless their scenario hash still matches.

## Timing Model

Use fixed-step simulation:

| Component | Initial target | Notes |
| --- | ---: | --- |
| Physics engine | 100 Hz | Deterministic fixed `dt=0.01 s` |
| Battery/electrical state | 100 to 1000 Hz internal, 100 Hz engine-facing | Substep only if needed for numerical stability |
| Tyre thermal/wear model | 100 Hz | Same clock as dynamics initially |
| Synthetic perception | 20 to 50 Hz | Add noise, latency, dropout, occlusion |
| Opponent tracker | 20 to 50 Hz | Time-align pose history and speed gap |
| MPC | 20 to 50 Hz | Provisional target, depends on solver |
| Tactical planner | 2 to 10 Hz | Candidate feasibility |
| Global strategy | 0.2 to 1 Hz plus event triggers | Event graph and scenario tree |
| Streamlit UI refresh | 5 to 20 Hz equivalent | UI schedule must not define physics time |

Every snapshot includes `scenario_id`, `run_id`, `seed`, `sim_time_s`, `lap`, and `step_index`.

## Telemetry Schema

The schema mirrors the main design doc. Every field must declare source, unit, frame, sign convention, and whether it is simulator truth, measured own telemetry, or an estimate.

| Field group | Required fields | Notes |
| --- | --- | --- |
| Own pose | `x_m`, `y_m`, `yaw_rad`, `s_m`, `n_m`, `lap`, `heading_error_rad` | `x,y` in map; `s,n` in track frame |
| Own speed | `vx_body_mps`, `vy_body_mps`, `speed_mps`, `progress_speed_mps` | Strategy mostly uses progress speed |
| Opponent estimate | `opp_id`, `opp_x_m`, `opp_y_m`, `opp_yaw_rad`, `opp_s_m`, `opp_n_m`, covariance, identity confidence | Planner sees estimates, not hidden truth |
| Speed gap | `gap_s_m`, `gap_t_s`, `speed_gap_mps`, `closing_speed_mps`, uncertainty | Derived from time-aligned pose histories |
| Battery | SOC, SOH capacity, SOH power, usable energy Wh/MJ, Delta Energy Wh, signed current A, desired current A, delivered current A, voltage V, deploy W, recovery W | Positive Delta Energy means own battery gained energy over the sample |
| Temperatures | pack core, pack surface, max cell/module, min cell/module if modelled | degC, own car; per-cell values may be unavailable in simple model |
| Tyres | compound, surface/core temperatures, friction multiplier, combined-slip limit, wear fraction, degradation rate, remaining life, lap-time loss | Separate reversible temperature from irreversible wear |
| Race/circuit | track event id, flag state, rule config version, detection/activation zone state | 2026 rules stay configurable |
| Planner/advisory | candidate id, action family, horizon, confidence components, expected energy cost, expected position outcome | Used for engineer explanation |

Do not infer actual rival SOC, SOH, internal temperatures, tyre life, or tyre wear from historical or perception channels unless the dataset explicitly provides them. For opponents, these are modelled latent states.

## Circuit And COTA Model

The concrete EUFS/Gazebo asset and provenance workflow is specified in [COTA_EUFS_TRACK_PLAN.md](COTA_EUFS_TRACK_PLAN.md). Keep this Streamlit plan focused on the headless model and planner interface; the linked plan defines the canonical metric geometry, EUFS cone semantics, front-bumper spawn calculation, packaging, and staged Gazebo/RViz checks.

Create a versioned COTA track asset:

- Centerline in metres, with cumulative unwrapped `s`.
- Left/right boundaries or local width envelope.
- Turn/event annotations: start/finish, corners, straights, braking zones, recovery windows, attack windows, pit entry/exit if used.
- Optional elevation profile, labelled as optional until sourced and validated.
- Provenance file with source, transform, scale, date, and known limitations.

Do not invent an official map. If a public map is digitized, store the source and the fitting error. Validate lap length, turn order, start/finish location, and coordinate scale separately. Keep COTA event/rule settings in a config file so 2026 detection, activation, manual override, recovery, and deployment limits can be edited without code changes.

## Dynamics

Start with a point-mass car:

```text
state = [x, y, yaw, vx, vy, yaw_rate, fuel_kg, battery_state, tyre_state]
control = [steer, throttle, brake, ers_deploy_request, regen_request]
```

Then add a bicycle model once the basic energy and planning loops pass deterministic tests.

The vehicle model includes:

- Longitudinal force from ICE plus ERS, limited by tyre grip and power.
- Braking split between friction brakes and regenerative braking.
- Drag, rolling resistance, downforce, and optional wake/draft multipliers.
- Fuel mass and fuel energy if modelled.
- Combined-slip tyre envelope, so braking, cornering, and traction compete.
- No physics-to-battery disconnect: delivered force/torque determines electrical power and heat.

ERS and battery:

- Equivalent-circuit pack model initially, with cells/groups represented by capacity, resistance, voltage curve, current limits, power limits, and thermal nodes.
- SOC from usable energy; SOH as slow capacity and power capability ageing.
- Desired current is requested by propulsion/recovery; delivered current is clipped by voltage, SOC, temperature, SOH, and rules.
- Recovery is bounded by braking demand, tyre grip, SOC headroom, thermal limits, and configured rule limits.
- PyBaMM may be used offline for reference curves and calibration sweeps. Do not put a full unvalidated PyBaMM electrochemistry solve in the real-time UI loop for MVP.

Tyres:

- Surface and core temperature states per tyre.
- Wear fraction or damage state per tyre.
- Grip multiplier from compound, temperature, wear, vertical load, track grip, and slip.
- Degradation lap-time loss relative to a controlled reference, conditional on fuel, traffic, track temperature, and driving style.
- Initial parameters are hypotheses until calibrated against controlled simulation or data.

## Opponent Models

Use a library of opponent policies:

| Mode | Description | Use |
| --- | --- | --- |
| Fixed replay ghost | Follows historical or scripted speed-distance/time profile exactly; nonphysical collision semantics | Visual/reference demo only |
| Fixed nominal replay | Follows recorded timing unless collision treatment forces documented safety behavior | Reproducible benchmark with limitations |
| Recorded-reference closed-loop | Tracks the historical reference but can brake, deviate, or defend around ego | First physical two-car benchmark |
| Scripted reactive | Hold line, defend inside, conserve, deploy, or abort based on ego action | Strategy testing |
| Stochastic policy mixture | Behaviour probabilities updated from observed pose history | Planner uncertainty testing |
| Learned/game model | Later research layer | Not MVP |

The planner must receive opponent pose estimates from sensors/tracking. It must not see future replay samples, hidden opponent truth, or modelled latent energy/tyre state except in a labelled oracle benchmark.

## Historical Timed-Lap Opponent

A historical timed-lap opponent is a good first demo because it is reproducible and easy to explain. It is not the same as a responsive racing opponent.

Dataset choices:

- Prefer FastF1 if the chosen session/lap has sufficient timing, car telemetry, and position channels for the demo.
- Record exact event, session, year, driver, lap number, data source, library version, cache hash, and license/terms note.
- Treat F1 telemetry as dataset-limited: sampling may be irregular, position can be approximate, and public channels do not provide full opponent battery, tyre truth, internal temperatures, or strategy intent.
- Do not download or bundle data until licensing and provenance are reviewed.

COTA selection and checks:

- Choose a dry, valid COTA lap with no pit in/out, safety car, red flag, major traffic obstruction, or invalid lap flag if the dataset exposes those fields.
- Inspect available channels before coding assumptions: timestamp, distance, speed, throttle, brake, gear, RPM, DRS/overtake status if present, X/Y position if present, and sampling intervals.
- Verify coordinate units and whether distance is lap distance, session distance, or telemetry-derived distance.
- Check acceleration implied by resampled speed. Reject or smooth segments that imply impossible jumps unless the source explains a timing gap.

Preprocessing:

- Keep original timestamps. Create a replay table with `t_s`, `lap_distance_m`, `speed_mps`, optional `x_m`, `y_m`, and channel-quality flags.
- Fit the replay onto the metric COTA centerline. If reliable XY is unavailable, use speed-distance along the centerline and label lateral line as modelled.
- Unwrap lap distance and lap count for multi-lap replays.
- Interpolate onto the simulator clock using timestamp-aware interpolation.
- Preserve known gaps and add validity windows instead of silently filling long missing spans.
- Separate one-lap looping from multi-lap race replay. A looped one-lap ghost has a discontinuity at lap end unless speed, acceleration, tyres, fuel, and energy are explicitly blended.

Replay clock:

- The opponent follows its own replay sim time.
- Do not snap the opponent to ego progress.
- Initialize gap by choosing replay start offset, ego start offset, or explicit time offset.
- Define end behavior: stop, loop with discontinuity flag, continue to next historical lap, or hand off to a scripted policy.

Collision semantics:

- Immutable ghost/reference: no physical collision. It is a visual and timing baseline.
- Fixed nominal replay: cannot both follow the dataset exactly and respond physically. If collision avoidance is added, document the deviation from the historical trace.
- Recorded-reference closed-loop: a controller tracks the historical speed/line but may brake or deviate around ego. This becomes a simulated opponent, not the exact historical outcome.

Historical observations:

- Truth replay generates noisy, delayed, possibly missing observations.
- The planner sees only current and past estimated pose history, speed gap, covariance, identity confidence, and stale flags.
- The planner must not access future samples or hidden historical channels.

Demo scenarios:

- Same historical replay, different ego SOC.
- Same replay, different ego tyre temperature/wear.
- Same replay, different start gap.
- Same replay, different global strategy objective or risk envelope.

Metrics:

- Pass-and-retain rate.
- Energy spent and exit reserve.
- Time gained/lost versus follow baseline.
- Regret versus finite benchmark oracle.
- Frequency of unsafe or stale recommendations.
- Difference between immutable replay claims and responsive competitive-racing claims.

Dataset adapter interface:

```text
HistoricalLapReplay:
  metadata: source, event, session, driver, lap, license_note, cache_hash
  samples: t_s, s_m, speed_mps, optional x_m/y_m/yaw_rad, channel_quality
  validity_windows: valid sample intervals and discontinuities
  replay_policy: ghost | fixed_nominal | closed_loop_reference
```

Acceptance tests:

- Loading the same cached replay twice gives byte-identical preprocessed samples.
- Replay sampled at two UI refresh rates gives identical physics-clock opponent states.
- The opponent follows replay time rather than ego `s`.
- Long gaps and lap-loop discontinuities are surfaced in diagnostics.
- The planner cannot read future replay samples.
- If closed-loop collision avoidance is enabled, the trace-deviation metric becomes nonzero and is reported.

## Planning Stack

Global strategy:

- Initial finite benchmark dynamic programme over declared COTA events, energy bins, tyre bins, gap bins, and opponent response modes.
- Later adaptive candidate generator and scenario tree.
- Actions are parameterised families, not an exhaustive real-world list.
- Every score includes horizon, candidate coverage, terminal value, and uncertainty components.

Tactical planner:

- Evaluates approach, overlap, line, abort, pass completion, pass retention, and counterattack exposure.
- Uses continuous feasibility checks around the selected opportunity.
- Returns limiting reason when infeasible.

MPC:

- Tracks trajectory and energy targets.
- Uses the same vehicle, tyre, and battery constraints as the engine.
- Starts with a simple convex or nonlinear prototype; only use OSQP for true QP subproblems.
- CasADi/acados can be added after the model interface stabilizes.

Advisory:

- Converts counterfactual outcomes into engineer/driver messages.
- Includes human delay and imperfect execution.
- Separates state-estimate confidence, opponent-model confidence, outcome probability, and candidate-search coverage.
- Shows ranked evaluated alternatives and rejected infeasible actions.

## Streamlit UI

Core views:

- Track view with ego, opponent estimate, covariance ellipse, racing line, COTA events, and candidate corridors.
- Energy view with SOC, usable energy, Delta Energy, current, deploy, recovery, temperature, SOH, and derate reason.
- Tyre view with temperatures, wear, grip multiplier, degradation rate, remaining life, and lap-time loss.
- Planner view with candidate actions, outcome branches, confidence components, and expected race result.
- Replay view with historical lap metadata, channel quality, preprocessing diagnostics, and replay mode.
- Run controls: start, pause, step, reset, seed, checkpoint, replay, export.

Streamlit state rules:

- Store run configuration and latest immutable snapshot in `st.session_state`.
- Cache static track assets and preprocessed historical replay files with explicit hashes.
- Cache heavy model resources separately from data.
- Use fragments or timed refresh only for rendering snapshots, not for physics ownership.
- Bounded logs only; long runs write telemetry to disk-backed run artifacts.

## Exports And Replays

Each run writes:

- `run_config.yaml`
- `sources.json`
- `telemetry.parquet`
- `planner_decisions.parquet`
- `candidate_outcomes.parquet`
- `events.parquet`
- `diagnostics.json`

CSV export is optional for quick inspection. Parquet is preferred for typed telemetry and large runs.

Deterministic replay requires:

- Fixed seed and random-stream names.
- Fixed solver settings and timeouts.
- Versioned track, car, battery, tyre, opponent, and rule configs.
- Scenario hash recorded in every planner result.

## MVP

MVP 1: headless single-car COTA engine.

- Point-mass dynamics.
- Battery equivalent-circuit energy accounting.
- Simple tyre temperature/wear/grip model.
- COTA centerline and event graph.
- Deterministic fixed-step tests.
- CSV/Parquet export.

MVP 2: Streamlit dashboard.

- Load scenario, step, run, pause, reset.
- Track, energy, tyre, and telemetry plots.
- No planner blocking UI.
- Reproducible replay from saved run artifacts.

MVP 3: two-car benchmark.

- Synthetic opponent policy.
- Noisy/delayed opponent pose observations.
- Speed-gap estimator.
- First finite event-based strategy planner.

MVP 4: historical timed-lap demo.

- Versioned historical lap replay adapter.
- Immutable ghost and fixed nominal replay modes.
- Clear diagnostics for missing channels and replay limitations.

MVP 5: tactical/MPC/advisory.

- Candidate generation.
- Tactical feasibility.
- MPC prototype.
- Advisory action/outcome explorer.

## Acceptance Tests

Engine:

- Same seed and config produce identical telemetry.
- Physics result is independent of Streamlit refresh rate.
- Changing UI widgets during a run does not mutate the active scenario unless reset/restart is requested.
- Solver results with stale scenario hashes are rejected.

Battery:

- Positive delivered propulsion decreases usable energy.
- Braking recovery increases usable energy only when recovery is allowed and headroom exists.
- Full and empty pack limits are respected.
- Thermal update is approximately invariant under smaller fixed `dt`.
- High temperature and low SOC reduce delivered power.

Tyres:

- Tyre temperature changes grip reversibly.
- Wear accumulates monotonically.
- Lap-time loss is reported relative to a controlled reference.

Opponent estimation:

- Speed gap is derived from time-aligned pose history.
- Pose covariance grows with stale or missing observations.
- Planner receives no hidden opponent battery, tyre truth, future replay samples, or response oracle.

Planning:

- Finite DP benchmark is exhaustive only inside its declared abstraction.
- Candidate planner regret is measured against the finite benchmark.
- Advisory output lists evaluated candidates, confidence components, and validity conditions.

Historical replay:

- Dataset provenance and cache hash are recorded.
- Irregular samples are resampled onto the engine clock without losing original timestamps.
- One-lap loop discontinuities are marked.
- Closed-loop replay deviations are measured when collision avoidance is enabled.

Performance:

- 100 Hz headless single-car simulation for at least 10 simulated laps faster than real time on the target dev machine.
- Streamlit can display a running scenario without blocking interaction.
- Planner timeouts are enforced and visible.

## ROS/Gazebo Adapter Path

Keep the Python simulator interfaces close to ROS concepts:

- `CarState` maps to odometry, TF, battery, tyre, and diagnostic topics.
- `OpponentEstimate` maps to tracked object messages later.
- `StrategyDirective` maps to advisory/MPC command topics.
- `RunArtifact` maps to bag-like offline replay data.

The adapter should be a boundary layer. Do not make Streamlit or ROS own the research dynamics. The same headless engine should run in CI, Streamlit, and offline batch experiments.

## Known Limits

This plan does not claim calibrated 2026 F1 performance. It defines a parameterized F1-inspired research simulator. Claims about FIA compliance, historical accuracy, or competitive racing strength must wait for sourced rules/configs, validated maps, calibrated vehicle data, and benchmark results.
