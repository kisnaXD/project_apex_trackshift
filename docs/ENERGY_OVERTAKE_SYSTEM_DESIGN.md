# Energy-Aware Overtake Strategy System Design

This document captures the proposed architecture for a Driver Suggestive Energy and Overtake Intelligence system for an F1 race strategy use case. It incorporates the current steering assumptions:

- Target rule family: configurable 2026 FIA Formula 1 hybrid and overtaking rules.
- "Delta Energy" means signed change in our own usable battery energy over time.
- Our own speed is known from telemetry.
- Opponent pose is estimated. Opponent speed gap is inferred from time-aligned pose history and relative motion, not from rival telemetry.
- Opponent battery state, tyre state, intent, and response model are latent hypotheses with uncertainty.
- The near-term simulator roadmap is credible single-car energy model, then two-car strategic benchmark on COTA, then tactical MPC and advisory behavior, then uncertainty and multi-car scale-up.

The system should not answer only "can we overtake right now?" The decision problem is:

> Does spending energy on this manoeuvre improve our eventual race result after accounting for opponent response, future passing opportunities, energy recovery, tyre consequences, and whether we can retain the position?

That is the core race strategy question.

## System Layers

Use three planning responsibilities, each with a different job.

1. Race strategy planner.

   This layer chooses whom to attack, where to attack, whether to prepare now for a later opportunity, how much energy can be spent, and what continuation must remain possible after the manoeuvre. It reasons over race progress, energy state, tyres, circuit opportunities, traffic, and opponent response hypotheses.

2. Tactical planner.

   This layer converts a strategic intent into feasible manoeuvre candidates. It evaluates approach, overlap, braking, corner entry, exit, abort options, and immediate counterattack exposure. It should tell strategy not only whether an overtake is feasible, but why a request is infeasible.

3. Vehicle MPC.

   This layer tracks the selected trajectory and energy target while respecting vehicle, tyre, thermal, and electrical constraints. The MPC horizon must be measured in useful physical duration or distance, not just "three short solver steps." At 50 Hz, three solver steps cover only 60 ms, which is far too short for an overtake. A starting point is 4 to 8 seconds with denser points near the car and coarser points farther ahead.

A practical execution flow:

1. Telemetry, circuit map, rule config, and perception feed a state estimator.
2. The estimator produces own vehicle state, own energy state, tyre state, opponent pose beliefs, and track-relative gaps.
3. The strategy planner generates candidate actions: hold, close, harvest, prepare attack, attack, defend, or disengage.
4. The tactical planner evaluates candidate manoeuvres against vehicle and traffic feasibility.
5. The strategy planner scores each candidate over future opportunities and chooses the next directive.
6. The MPC executes the directive or the advisory system converts it into driver-facing guidance.
7. The system replans when state, uncertainty, or race context changes.

Suggested rates for early development:

| Layer | Initial rate | Purpose |
| --- | ---: | --- |
| Race strategy | 0.2 to 1 Hz plus event triggers | Choose attack/hold/prepare/defend strategy |
| Tactical evaluation | 2 to 10 Hz | Score feasible manoeuvre candidates |
| Vehicle MPC | 20 to 50 Hz | Track selected trajectory and power target |
| Estimation | Sensor-rate plus prediction | Maintain time-aligned own and opponent state |

## Objective

"Optimal" must be defined before choosing the global planning algorithm. The stated priority is winning first and finishing second. A clean mathematical form is lexicographic:

```text
lexmax_pi [
  P_pi(win),
  P_pi(finish),
  -E_pi(classified finishing position)
]
```

The second objective breaks ties in the first, and the third breaks remaining ties. In practice, a team should also set a risk envelope. A literal win-first rule can accept too much retirement risk for a tiny gain in win probability. The risk envelope can change with laps remaining, championship context, tyre life, and the car's position.

For early simulator work, expected finishing position or expected race time is easier to debug than a full win-probability objective. Keep the simpler objective separate from the later win-priority policy so each design choice can be measured.

Do not reward "number of overtakes" as the main objective. Passing and immediately losing the place should not be profitable. Distance covered is a progress constraint, but the terminal result must depend on completing the race and the classification.

## State And Observability

The minimum state should separate measured telemetry from inferred beliefs.

Measured or directly available from our car:

- Position, orientation, speed, acceleration, and timestamp.
- Own absolute usable battery energy.
- Signed Delta Energy over time, `dE_own/dt` or finite differences of own battery energy.
- Available deploy and recovery limits if the simulator or model supports them.
- Tyre compound, age, temperature, wear estimate, and uncertainty.
- Fuel mass or fuel-energy proxy if modelled.
- Battery temperature and thermal limits if modelled.

Estimated from perception and tracking:

- Opponent pose with covariance and identity.
- Track-coordinate opponent progress `s_opponent`, lateral offset, heading, and covariance.
- Signed track gap `g = s_opponent - s_ego`, with lap wraparound handled.
- Positive closing speed `c = -dg/dt`, estimated from filtered pose history.
- Opponent line choice and behaviour class as a belief distribution.
- Missed-detection probability and identity-switch probability.

Inferred latent hypotheses:

- Opponent resource state.
- Opponent intent: hold line, defend inside, deploy, conserve, pit, or disengage.
- Opponent tyre/grip condition.
- Opponent response probability conditioned on our proposed manoeuvre.

Do not treat rival battery or tyre state as known unless running an explicitly labelled oracle benchmark.

### Telemetry Contract

This contract maps the requested telemetry terms to model state, available observation, units, uncertainty, and planner consumers. The simulator should expose truth state for scoring and debugging, but the operational planner should receive only the observation or estimated field.

| Parameter | Truth state in simulator | Planner observation or estimate | Unit, sign, and frame | State evolution and estimator | Initial rate target | Main consumers |
| --- | --- | --- | --- | --- | --- | --- |
| Battery SOC | Pack charge fraction derived from usable stored energy and present capacity | Own measured SOC plus validity flag | 0 to 1, dimensionless, own car | Integrated from signed pack current and voltage; bounded by min/max usable energy | Battery model 100 to 1000 Hz internally; published 10 to 50 Hz | Strategy energy budget, tactical feasibility, MPC power limits, advisory confidence |
| Absolute usable energy | Pack usable energy above configured reserve | Own measured or simulated energy | Wh or MJ, positive stored energy, own car | Integrated from delivered deployment, recovery, auxiliaries, and losses | 10 to 50 Hz published | Strategy value of energy, MPC terminal energy target, overtake cost |
| Delta Energy | Finite difference of own absolute usable energy | Own measured `E(t)-E(t-dt)` and optionally `dE/dt` | Wh per sample or W for rate; positive means battery gained energy | Computed from time-aligned own energy samples; never used alone without absolute energy | 10 to 50 Hz | Strategy budget tracking, advisory explanation, anomaly detection |
| Battery SOH | Present capacity and power capability relative to nominal new pack | Estimated own SOH with slow uncertainty | 0 to 1 capacity health and 0 to 1 power health | Slow ageing state from cycles, temperature, current, and calibration; not inferred reliably from one race sensor alone | Offline or stint-level | Strategy stint assumptions, long-run simulation, diagnostics |
| Pack temperature | Lumped pack/core/surface truth depending on model fidelity | Own measured pack/core/surface estimate | degC, own car | Thermal ODE from `I^2R`, coolant/ambient, and heat capacities | 10 to 50 Hz published | MPC power derate, recovery limits, advisory cancellation |
| Cell or module temperatures | Per-cell/module temperatures if modelled; otherwise unavailable | Estimated max/min/mean cell temperature with confidence | degC, pack-local | Equivalent-circuit thermal model or offline PyBaMM-calibrated reference; expose max cell temp for constraints | 10 to 50 Hz for max/min; slower for full vector | MPC derate, BMS constraints, risk display |
| Current demand | Desired electrical current requested by propulsion/recovery controller | Own desired current and capability-limited delivered current | A; positive demand means discharge/deploy, negative means charge/recovery | Computed from requested mechanical power and voltage, then clipped by SOC, voltage, temperature, power, and rule limits | Same as propulsion controller | MPC feasibility, battery heat, advisory energy cost |
| Delivered signed current | Actual pack current after clipping | Own measured or simulated signed current | A; use one convention consistently; document BatteryState sign separately if ROS convention differs | Delivered current updates energy, heat, and voltage sag | 50 to 100 Hz preferred | Battery model, telemetry validation, power derate |
| Deploy power | Mechanical or electrical deployment, explicitly labelled | Own delivered deployment power | W or kW; positive deployment | Derived from delivered force/torque and speed, with efficiency | 20 to 100 Hz | Strategy overtake cost, MPC force limits |
| Recovery power | Mechanical braking recovery or electrical charging, explicitly labelled | Own delivered recovery power | W or kW; positive recovery value, separate from signed net power | Bounded by braking demand, grip, SOC headroom, temperature, and rules | 20 to 100 Hz | Energy planning, braking-zone modelling |
| Tyre friction coefficients | Longitudinal/lateral/combined-slip grip state | Estimated grip multiplier and uncertainty | Dimensionless coefficient or multiplier, tyre-local/contact patch | Function of compound, temperature, wear, load, slip, track grip; cannot be directly measured from one scalar | 20 to 100 Hz for estimator; slower strategy summary | MPC tyre constraints, tactical pass feasibility |
| Tyre degradation lap-time effect | Controlled-reference pace loss caused by tyre condition | Estimated lap-time loss relative to reference | s/lap, conditional on fuel, traffic, temperature, and driving style | Learned/calibrated from controlled runs; separate from traffic and energy effects | 0.2 to 1 Hz | Strategy stint value, attack/hold tradeoff |
| Tyre degradation rate | Rate of irreversible tyre wear or damage accumulation | Estimated wear-rate distribution | fraction/lap or damage units/s | Driven by load, slip, temperature, lockups, kerbs, and compound | 1 to 10 Hz summary | Strategy future pace and pass-retention cost |
| Tyre life | Remaining usable tyre life under forecast driving | Estimated remaining life with confidence | laps, seconds, or normalized 0 to 1 | Derived from wear state and degradation-rate model; not a direct sensor truth in real use | 0.2 to 1 Hz | Strategy risk envelope, advisory warnings |
| Tyre temperatures | Surface/core truth per tyre if modelled | Own estimated tyre temps | degC, per tyre, optionally inner/middle/outer | Reversible thermal dynamics from load, slip, speed, cooling, and ambient | 10 to 50 Hz | MPC grip, warmup/cooling strategy, advisory cancellation |
| Own speed | Vehicle body and track-progress speed | Own measured speed | m/s; body-frame longitudinal and track-tangent progress speed should be separate | Sensor fusion from wheel speed, IMU, pose, and track projection | 50 to 100 Hz | All layers |
| Own position | Global and track-relative pose truth | Own estimated pose | `x,y,z` in map; `s,n` in track frame; include lap count | Localization plus track projection and covariance | 20 to 100 Hz | Tactical/MPC collision checks, strategy event position |
| Own heading | Global yaw and heading relative to track tangent | Own estimated heading | rad in map and rad error in track frame | Pose estimator, filtered for latency and wraparound | 20 to 100 Hz | MPC, tactical line feasibility |
| Opponent pose | Opponent truth pose in simulator only | Estimated opponent pose history with covariance and identity confidence | `x,y,yaw` map plus `s,n,heading_error`; timestamped | Detector/tracker or synthetic noisy delayed truth in sim; no future samples | Sensor-rate; planner receives time-aligned state | Strategy gap, opponent response model, tactical collision risk |
| Speed gap | Truth relative speed in evaluation | Derived from time-aligned own and opponent track-progress histories | m/s; define positive as ego closing or define signed `v_ego_s-v_opp_s` and keep consistent | Filtered derivative of wrapped/unwrapped track gap; carry covariance and stale-data flags | 20 to 50 Hz estimate | Candidate generation, attack timing, advisory confidence |

SOC, absolute usable energy, and SOH are different quantities. SOC says where the pack sits between usable bounds now. Absolute usable energy says how much work remains available for racing decisions. SOH says how capacity and power capability have aged relative to the nominal pack. A car can have high SOC but poor power capability because of temperature, voltage sag, ageing, or rules.

Pack, module, and cell temperatures should also be explicit. A first simulator may use a two-node pack model, such as core and surface. Later models can add module or cell extrema. The planner usually needs the limiting maximum temperature and the available power after derating, not the full internal temperature vector.

Tyre warmup and tyre wear must stay separate. Temperature is mostly reversible and changes grip immediately. Wear and damage accumulate and shift future pace. Lap-time degradation is a measured effect relative to a controlled reference, so it must be conditioned on fuel mass, traffic, track temperature, and energy deployment. Do not infer exact tyre life, friction coefficients, or internal tyre state from one noisy speed trace alone.

### Track-Relative Gap

Use Frenet or another track-relative coordinate system for interaction planning. Pose in global `x, y` is not enough for race strategy.

Define:

```text
s_ego       = own progress along the lap
s_opponent  = opponent progress along the lap
g           = signed wrapped gap = s_opponent - s_ego
c           = positive closing speed = -dg/dt
```

If `g > 0`, the opponent is ahead on the same lap. If `c > 0`, we are closing. Wraparound matters at start/finish and when cars are on different laps. The estimator must keep lap count or unwrapped progress, not only modulo track position.

Own scalar speed is not the same as track-progress speed in every situation. In cornering, sideways velocity, line choice, and local track tangent matter. Compute progress speed by projecting velocity onto the track tangent or by differentiating filtered `s`.

### Timestamp And Filtering

All state comparisons must be made at a common decision time.

1. Buffer own telemetry and opponent pose estimates with timestamps.
2. Propagate each state to the planner's decision timestamp.
3. Convert propagated poses to track coordinates.
4. Estimate `g` and `c` from filtered, time-aligned histories.
5. Carry covariance through the Frenet conversion and derivative estimate.
6. Increase uncertainty for missed detections, occlusions, identity switches, and stale measurements.

The planner should branch or penalize uncertainty. A small gap with high covariance is not the same as a small gap with a clean track.

### Staged Observation Development

Use staged observation fidelity while keeping the estimator-to-planner interface unchanged.

1. Truth-backed evaluation mode.

   Use simulator truth only to score experiments and debug upper bounds. Do not feed hidden opponent state, future responses, rival battery state, or rival intent into the operational planner.

2. Synthetic estimated mode.

   Convert simulator truth into timestamped, noisy, delayed, and sometimes missing opponent pose observations. Run the same tracker and estimator interface that the strategy planner will later use. This mode should include pose covariance, identity confidence, stale-measurement flags, missed detections, and identity switches.

3. Detector/tracker mode.

   Replace the synthetic observation generator with the real detector and tracker when available. The downstream estimator and planner contract should not change: they still receive time-aligned opponent pose beliefs, signed track gap, closing speed, covariance, and validity flags.

Validation results should state which observation mode was used. Strategy validation should default to synthetic estimated mode once the basic simulator is stable. Oracle/truth mode is useful for debugging but must not be mixed with estimated-observation results.

## Energy State

Keep three energy quantities distinct.

Own signed energy change:

```text
Delta_E_own(t) = E_own(t) - E_own(t - dt)
```

This is the user's intended Delta Energy. It says how our battery changed over the last interval.

Opponent energy difference belief:

```text
Delta_E_rival_belief = E_own - E_opponent_hat
```

This is not measured rival telemetry. It is an uncertain estimate, if used at all.

Energy plan deviation:

```text
Delta_E_plan = E_own - E_reference(s, lap)
```

This says whether we are ahead or behind our planned battery budget.

The planner needs absolute own usable battery energy, not only Delta Energy. A positive Delta Energy rate means something different at high state of charge, low state of charge, a thermal limit, or near the finish.

A minimal electrical balance for the simulator is:

```text
E[k+1] = E[k] + dt * (eta_regen * P_regen - P_deploy / eta_drive - P_aux)
```

Subject to:

```text
E_min <= E[k] <= E_max
P_deploy <= P_max(v, E, T, rules)
P_regen  <= P_regen_max(v, E, T, tyres, rules)
```

Track physical battery energy separately from rule accounting counters. A regulatory lap counter may reset at a lap boundary. The physical battery does not reset.

## Future Value Of Energy

The most important bridge between global strategy and MPC is the future value of stored energy.

One more unit of battery energy can be valuable or nearly worthless depending on what comes next:

- Before a high-value passing opportunity, preserving energy may be decisive.
- Before a large recovery zone, spending energy can create useful charging headroom.
- After a pass, energy may be needed to resist a counterattack.
- Near the finish, stored energy with no usable deployment opportunity has low value.
- At a thermal or power limit, stored energy may be unavailable even if the battery is not empty.

Let `V(z, E)` be the estimated remaining race cost, where lower is better. The marginal value of energy is:

```text
lambda_E(z, E) = -dV/dE
```

If `V` is measured in seconds, `lambda_E` has units of seconds per MJ. Estimate it numerically:

```text
lambda_E ~= (V(z, E) - V(z, E + epsilon)) / epsilon
```

The strategy layer should pass both energy prices and hard resource targets to MPC. A soft price tells MPC how to trade lap time against energy. A hard target preserves the resources required for a later action.

Example directive:

```text
target: car_22
opportunity: back_straight_after_turn_11
mode: prepare_then_attack
valid_until: turn_11_exit + 1.0 s
deployment_budget: 0.55 MJ
minimum_exit_energy: 2.1 MJ
required_entry_gap: [0.45 s, 0.85 s]
planned_continuation: defend_into_turn_12
abort_if: opponent_gap_covariance_too_high or entry_gap_outside_window
```

## Circuit Model

The global planner should operate on circuit events rather than uniform time alone. For COTA, the event graph should include:

- Start/finish and lap boundary.
- Long straights and braking zones.
- Corner exits where deployment affects the next straight.
- Heavy braking zones where recovery is feasible.
- Detection/activation zones or equivalent 2026-specific overtaking parameters, loaded from a version-pinned rule configuration.
- Pit entry, pit exit, safety-car/flag state, and traffic merge points if modelled.

Do not invent COTA-specific rule numbers. Store them in a rules file with source, issue date, and article references. The system should run if those values change.

Aerodynamics must be context-sensitive. Drafting can reduce drag on a straight, but following can reduce downforce and cooling in corners. Avoid a universal "slipstream is good" bonus.

Tyre temperature and tyre wear should be separate states. Temperature can recover. Wear mostly accumulates. Their effects should appear in corner speed, braking performance, traction, and future stint pace.

## Global Planner

Start with a small stochastic dynamic programme or explicit scenario tree. Discretise:

- Race progress or circuit event.
- Own battery energy.
- Own tyre/grip state.
- Track gap to relevant opponent.
- Opponent behaviour hypothesis.
- Race context such as remaining laps and flags.

Actions should be structured:

```text
FOLLOW_EFFICIENTLY
CLOSE_GAP
PREPARE_ATTACK(opportunity, energy_budget)
ATTACK_NOW(target, line, deployment_budget)
DEFEND(position, energy_budget)
DISENGAGE_AND_RECOVER
```

Each transition should include resource use, probability of success, failure cost, retention probability, and future opportunity value.

Online decision cycle:

1. Estimate current state and uncertainty.
2. Generate a small candidate set.
3. Screen candidates with cheap manoeuvre models.
4. Run tactical feasibility on the strongest candidates.
5. Branch over opponent responses conditioned on our action.
6. Simulate continuation through pass retention and counterattack risk.
7. Score race outcome and resource state.
8. Commit only the next actionable part and replan.

The planner must not know future opponent responses before they are observed. Future branches can adapt after observations, but the current committed action must be valid across still-unobserved outcomes.

For the first benchmark, explicitly represent the next two or three meaningful opportunities and approximate the rest of the race with a continuation value. Without a continuation value, a short-horizon planner can spend energy just beyond its horizon and look falsely optimal.

## Candidate Coverage And Confidence

The candidate list in this design is not exhaustive. Labels such as `FOLLOW_EFFICIENTLY`, `PREPARE_ATTACK`, `ATTACK_NOW`, `DEFEND`, and `DISENGAGE_AND_RECOVER` are manoeuvre families, not a complete enumeration of every action the car could take.

Real racing actions are continuous. Timing, steering, throttle, braking, line choice, deployment level, lateral placement, abort point, and continuation all vary continuously. Opponent responses are also continuous and reactive. With finite compute, noisy state estimates, and partial observability, the system cannot honestly promise that it has mapped every possible action and outcome in a real race.

The architecture is still sound if the planner treats each label as a parameterised option:

```text
option_family: prepare_then_attack
target: opponent_1
location: turn_11_exit_to_back_straight
timing_window: [t0, t1]
path_family: outside_to_inside / inside_direct / stay_in_tow
energy_range: [E_min, E_max]
duration_range: [d_min, d_max]
abort_conditions: gap_outside_window, pose_covariance_high, battery_exit_reserve_low
continuation: defend_turn_12 / concede_and_recover / follow_after_failed_attack
```

The planner should separate three things:

1. Admissible physical controls.

   These are the continuous steering, throttle, braking, deployment, and recovery controls allowed by vehicle physics, energy state, tyre state, thermal state, track limits, and rules.

2. Generated policy candidates.

   These are the manoeuvre families and parameter ranges the planner actually evaluates before the deadline.

3. Sampled or branched outcomes.

   These are the predicted results under opponent responses, state uncertainty, human execution variation, and random events.

Candidate coverage mainly affects strategic quality. A weak candidate generator can miss a valuable pass, defence, or recovery plan. Do not confuse candidate coverage with collision safety. Collision and feasibility require independent constraints, state-estimate checks, model validity checks, and a feasible fallback or abstention action. No architecture can give an unconditional safety guarantee if an opponent violates assumptions, perception loses the car, the vehicle cannot actuate the required response, or a human driver does something outside the model.

### How To Generate Candidates

Start with a transparent finite template library, then expand it adaptively.

Initial template families:

- Follow and save energy.
- Follow and harvest where recovery is feasible.
- Close the gap before a named opportunity.
- Attack left, attack right, or attack on a defined racing corridor.
- Defer to a later opportunity.
- Defend after a pass.
- Disengage and rebuild battery.
- Pit or strategy reset, only when pit decisions are in scope.

Each family should produce parameter ranges, not one hard-coded action. For example, `attack left` should become several timing windows, energy budgets, lateral corridors, and abort points. A continuous optimiser or MPC then searches within those ranges.

Candidate expansion should trigger when:

- The best and second-best candidates are close.
- Uncertainty can reverse the ranking.
- A candidate is infeasible only because of a narrow timing or energy choice.
- The opponent response model has high entropy.
- The current search did not include both sides of a plausible passing corridor.
- A rare but high-consequence outcome is plausible, such as a failed pass exposing us to the car behind.
- The planner is near a strategic boundary, such as low battery, tyre cliff, or final-lap opportunity.

Topology diversity matters. The candidate set should intentionally cover distinct manoeuvre topologies: left, right, follow, defer, defend, recover, and pit where relevant. Sampling many small variants of the same attack line is not enough.

Every candidate must be revalidated before commit. A manoeuvre that was feasible two seconds ago may become infeasible after a small opponent move, battery derate, tyre-temperature change, or delayed pose update.

When there is no feasible candidate with adequate confidence, the correct output is not a forced attack. The system should return a fallback such as hold position, lift and recover, maintain safe gap, or ask for more data. For the advisory interface, this becomes "no reliable attack recommendation in this window" rather than a low-confidence overtake instruction.

### Outcomes And Nonanticipativity

Opponent responses must branch from our proposed action. If we propose an outside attack, the opponent may hold line, defend, deploy, brake earlier, or leave space. If we stay in tow, their response distribution changes.

The planner must also respect nonanticipativity: it cannot choose a different current action for two future scenarios that have not yet been observed. It may plan conditional continuations, but the first committed action must be the same across branches that are still indistinguishable at decision time.

Rare outcomes should not disappear only because they are inconvenient to model. A low-probability failed attack can matter if it causes a large energy loss, tyre damage, position loss, or counterattack exposure. The outcome tree should include pass, failed pass, abort, forced lift, counterattack, retained position, lost position, and neutral follow where those outcomes are plausible.

### What "Exhaustive" Can Mean

The word "exhaustive" is valid only for a declared finite benchmark.

For example:

```text
Track events: 12 fixed events
Energy bins: 20
Gap bins: 15
Tyre bins: 5
Opponent responses: 4
Actions: 8 parameterised templates
Horizon: 3 opportunities plus terminal value
```

In that finite discretised benchmark, exhaustive dynamic programming can evaluate every state/action branch in the declared abstraction. That does not prove global completeness for the real continuous race. It proves optimality only inside that simplified grid, with those actions, those transition models, and that terminal value.

Use this finite oracle to measure the candidate planner:

- Missed opportunity rate.
- Regret versus the discretised oracle.
- Frequency of choosing a dominated action.
- Sensitivity to horizon length.
- Sensitivity to template library size.
- Sensitivity to opponent response coverage.

Only after the candidate planner performs well against richer search and oracle baselines should it be used for stronger autonomy claims.

### Mapping Possibilities For The Engineer

The suggestion recipient should not be told "we considered everything." The system should say what it evaluated, what it omitted, and how reliable the ranking is.

Engineer-facing output should include:

- All evaluated action families within the stated search coverage.
- Ranked near-Pareto alternatives, not only the top action.
- Why dominated or infeasible alternatives were excluded.
- Branch outcomes: pass, abort, failed pass, counterattack, retained position, lost position, and follow.
- Energy use, expected exit reserve, tyre effect, retention probability, and race-outcome estimate.
- Candidate parameter ranges: timing, path corridor, energy budget, and abort window.
- Search coverage: templates used, opponent responses sampled, horizon length, terminal-value model, and compute deadline.
- Data quality: timestamp age, pose covariance, identity confidence, and dropped observations.
- Uncertainty split into state-estimate uncertainty, model uncertainty, opponent-response uncertainty, human-execution uncertainty, and search-coverage quality.

Do not collapse all of this into one magic confidence percentage. A useful recommendation can carry several confidence statements:

```text
State estimate: medium confidence, opponent pose covariance elevated
Opponent model: low confidence, recent defensive behavior inconsistent
Outcome probability: 58% pass-and-retain conditional on model ensemble
Search coverage: medium, left/right/defer covered; pit and safety-car branches omitted
Recommendation confidence: conditional, ranking reverses if opponent deploys more than expected
```

Unevaluated is not the same as low probability. If the planner did not evaluate a late-braking outside attack because the template library omitted it, the system should not report that action as unlikely. It should report it as outside current search coverage.

Human-facing advice should be much shorter because it must be timely:

```text
Prepare, but do not commit yet. Attack only if the gap enters the window at Turn 11 exit.
Cancel if the opponent defends inside or the battery exit reserve drops below target.
```

The race engineer can inspect the richer tree or table. The driver receives limited, timely guidance.

Illustrative example, not measured:

| Candidate | Result estimate | Energy effect | Main risk | Recommendation |
| --- | --- | --- | --- | --- |
| Attack after Turn 11, inside | 58% pass and retain | -0.55 MJ, exit reserve just above minimum | Ranking reverses if opponent deploys | Conditional prepare |
| Stay in tow, attack Turn 12 braking | 46% pass and retain | -0.25 MJ, better exit reserve | Needs smaller braking-gap error | Keep as backup |
| Attack immediately before Turn 10 | 42% pass, 25% lose place back | -0.70 MJ | High counterattack exposure | Do not recommend |
| Hold and recover | No pass this lap | +0.18 MJ | May lose contact if opponent deploys | Fallback if uncertainty rises |

This example is intentionally small. A real engineer view should show the parameter ranges, branch assumptions, and omitted branches behind each row.

## Overtake Evaluation

Evaluate the manoeuvre through retention, not only overlap.

Separate metrics:

- Pass initiated.
- Overlap achieved.
- Car fully clear.
- Corner exit feasible.
- Position retained through the next counterattack zone.
- Net race outcome improved against the best alternative.

A useful explanation equation is:

```text
A ~= p*G - (1-p)*L - lambda_E*Delta_E - C
```

Where:

- `p` is the probability of success and retention.
- `G` is the benefit if successful.
- `L` is the loss if unsuccessful.
- `Delta_E` is energy spent relative to the alternative.
- `lambda_E` is the future value of energy.
- `C` is tyre, thermal, risk, or tactical cost not already included.

This is an explanation aid. The main planner should compare complete alternatives, including "wait and attack later." Avoid double-counting energy or tyre consequences if the rollout already includes them.

## Opponent Model

Opponent behaviour must depend on our proposed action. A fixed predicted opponent trajectory will overestimate overtaking opportunities against a defending driver.

Start with a mixture model:

```text
P(response | observations, our_proposed_manoeuvre)
```

Candidate responses:

- Hold racing line.
- Defend inside.
- Deploy extra energy.
- Harvest or lift.
- Brake earlier.
- Leave space because of traffic or corner geometry.

Update mixture weights from observed behaviour. For simulation, keep an oracle opponent state only as a benchmark. The operational planner should use only information available through our telemetry, trackside data, and perception assumptions.

## Driver Advisory System

The driver advisory system should be generated from counterfactual race-strategy comparisons, not from raw MPC controls.

Recommendation payload:

```text
action: hold / prepare / attack / defend / abort
target: opponent id
location: named circuit window
expected_advantage: value against best alternative
energy_commitment: expected deploy/recover budget
minimum_exit_energy: required continuation reserve
confidence: decision margin vs uncertainty
validity_conditions: cancel or update triggers
expiry: time, distance, or circuit event
```

Example:

```text
Hold through this section. Prepare attack after Turn 11 exit.
Use deployment only if the signed gap enters the passing window.
Cancel if opponent pose covariance remains high at corner exit.
```

Use a simple lifecycle:

```text
MONITOR -> PREPARE -> COMMIT -> COMPLETE
                     -> ABORT
```

Add hysteresis so the advice does not oscillate between attack and hold. Issue strong recommendations only when expected advantage exceeds both a practical benefit threshold and uncertainty.

Human execution must be modelled separately: reaction delay, imperfect energy timing, braking consistency, and partial adherence. Replan from what the driver actually does.

## Rules And Deployment Boundary

The design targets 2026 FIA Formula 1 hybrid and overtaking rules, but the values must be loaded from version-pinned primary sources, not hard-coded from memory.

Primary source locations to pin in the rules config:

- FIA regulations landing page: https://www.fia.com/regulation/category/110
- FIA Formula 1 regulations category: https://www.fia.com/regulation/category/2182
- Versioned design reference snapshot: 2026 FIA Formula 1 Sporting Regulations, Section B, Issue 08, published 05 Aug 2026: https://www.fia.com/system/files/documents/fia_2026_f1_regulations_-_section_b_sporting_-_iss_08_-_2026-08-05_7.pdf
- Versioned design reference snapshot: 2026 FIA Formula 1 Technical Regulations, Section C, Issue 20, published 05 Aug 2026: https://api.fia.com/system/files/documents/fia_2026_f1_regulations_-_section_c_technical_-_iss_20_-_2026-08-05.pdf

Relevant articles for the initial design snapshot:

- Section C, Article C5.2: energy system reference point for physical/rule energy modelling.
- Section C, Article C8.5.3: telemetry and communication boundary to check before any real deployment claim.
- Section B, Article B1.8.1: driver-alone/unaided boundary to check before any real deployment claim.
- Section B, Article B7.2: circuit-specific Overtake/manual-override-style settings and activation configuration.

These references pin the design baseline. They do not provide COTA-specific event settings in this document. COTA detection, activation, deployment, and recovery parameters must still be entered from the applicable event/circuit document or official rule appendix when available.

Use rule keys like:

```yaml
ruleset:
  series: FIA Formula One World Championship
  season: 2026
  sporting_issue: Section B Issue 08
  sporting_published: 2026-08-05
  technical_issue: Section C Issue 20
  technical_published: 2026-08-05
  source_urls:
    - https://www.fia.com/system/files/documents/fia_2026_f1_regulations_-_section_b_sporting_-_iss_08_-_2026-08-05_7.pdf
    - https://api.fia.com/system/files/documents/fia_2026_f1_regulations_-_section_c_technical_-_iss_20_-_2026-08-05.pdf
energy:
  physical_store_limits: from Section C Article C5.2 and project vehicle model
  deploy_power_limit_by_mode: from versioned rule/event config
  recovery_power_limit: from versioned rule/event config
  lap_or_event_counters: from versioned rule/event config
overtaking:
  activation_windows_by_circuit: unprovided for COTA in this document
  detection_points_by_circuit: unprovided for COTA in this document
  manual_override_or_equivalent: Section B Article B7.2 baseline plus event config
communications:
  advisory_deployment_boundary: check Section B Article B1.8.1 and Section C Article C8.5.3
```

Separate simulation/autonomous control from real F1 advisory deployment. Current F1 rules around driver assistance, team-to-car telemetry, and control systems must be checked against the exact 2026 sporting and technical regulation issues before any real deployment claim. The simulator may run an autonomous branch to test strategy, while the real-world product may need to remain a trackside engineer advisory workflow.

## PDF Corrections

The supplied `strategy.pdf` contains useful architectural language but overstates several claims.

| PDF claim or mechanism | Correction |
| --- | --- |
| Fixed `R_OT` thresholds choose harvest, nominal, or attack mode | Compare complete race alternatives and calibrate any simplified index against those outcomes |
| A dimensionless overtake score divided by variance | Use consistent units; variance is not failure probability or loss severity |
| OSQP solves the stated SOCP directly | OSQP solves convex quadratic programmes with linear constraints; use a compatible conic solver or reformulate as a QP |
| Convexification establishes global optimality | A convex subproblem does not prove global optimality of the original racing problem |
| CBF filters guarantee safety regardless of upstream behaviour | Guarantees depend on model validity, sampling, estimation error, actuator authority, and feasibility |
| ADMM gives assured full-grid real-time convergence | Benchmark the actual problem and deadline; do not assume cooperative V2V or rival plan sharing |
| Slack on a regulatory barrier while claiming no violation | A slack variable can relax the constraint unless bounded so it cannot violate the hard limit |
| Three MPC timesteps are enough | Define horizon in seconds/metres and manoeuvre phases, not only solver step count |

## EUFS Simulator Implications

The current EUFS-based simulator can show messages and visual behaviour, but it is not yet a credible validator for energy-aware race strategy.

The simulator must first support:

1. Commanded longitudinal force/power consuming own battery energy.
2. Available battery power constraining achievable acceleration and top speed.
3. Regeneration during braking or lift, with traction and rule limits.
4. Thermal limits reducing available power.
5. Tyre grip, temperature, and wear affecting feasible manoeuvres.
6. Track-coordinate opponent pose estimation without ground-truth leakage.
7. COTA circuit geometry and rule configuration.

## Roadmap

1. Build a credible single-car energy model.

   Validate acceleration energy, aerodynamic drag, recovery, thermal limits, fuel if modelled, and power-limited performance. Start with simple physical tests: constant-speed drag, flat acceleration, braking recovery, and energy conservation bounds.

2. Build a two-car strategic benchmark on COTA.

   Include at least two different passing opportunities and one recovery section. The first convincing milestone is a repeatable race where the system deliberately declines a feasible early pass, prepares a better later pass, achieves a better result, and explains the decision from predicted alternatives.

3. Connect tactical feasibility and MPC.

   The strategy layer should request manoeuvres with energy budgets and exit reserves. The tactical/MPC layer should return feasibility, time/resource cost, exit state, and abort options.

4. Add uncertain and reactive opponents.

   Use estimated opponent pose and inferred speed gap. Test missed detections, identity switches, late defensive moves, and behaviours excluded from planner calibration.

5. Build the advisory interface.

   Convert strategy decisions into driver-facing action, timing, conditions, and expiry. Evaluate with reaction delay and imperfect execution.

6. Scale to traffic and race scope.

   Keep detailed planning for cars that can interact soon. Use coarse forecasts for the rest of the field. Add queues, defending cars, pits, neutralisations, and multi-car counterattack risk progressively.

## Validation Baselines

Compare against:

- Fixed deployment map.
- Always attack when gap threshold is met.
- Gap-threshold plus battery threshold.
- Energy-aware MPC without global strategy.
- Oracle planner with exact opponent state, clearly labelled as oracle.
- Human/scripted baseline for the same scenarios.

Metrics:

- Finishing outcome or expected race time.
- Retained passes, failed attempts, and immediate counterattacks.
- Energy rule violations and physical battery violations.
- Thermal limit violations and power derates.
- Tyre/grip limit violations.
- Missed computation deadlines.
- Prediction calibration for opponent pose, gap, and response.
- Decision stability and advisory churn.

Small cases should be compared against exhaustive search or dynamic programming. Larger cases should be reported as best-known solutions, not proven global optima.

## Useful References

- FIA regulations landing page: https://www.fia.com/regulation/category/110
- FIA Formula 1 regulations category: https://www.fia.com/regulation/category/2182
- OSQP documentation: https://osqp.org/docs/
- Braghin et al., "Competitors-Aware Stochastic Lap Strategy Optimisation": https://arxiv.org/abs/2203.00084
- Fieni et al., "Game Theory in Formula 1": https://arxiv.org/abs/2503.05421
- Breeden et al., "Control Barrier Functions in Sampled-Data Systems": https://arxiv.org/abs/2103.03677
