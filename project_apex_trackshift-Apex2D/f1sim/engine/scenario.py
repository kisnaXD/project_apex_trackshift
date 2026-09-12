import numpy as np
from f1sim.engine.clock import SimulationClock
from f1sim.engine.world import SilverstoneCircuit
from f1sim.engine.replay import FastF1OpponentReplayer
from f1sim.dynamics.point_mass import F1EnergyVehicleModel
from f1sim.dynamics.tyres import PirelliTire
from f1sim.estimation.sensors import SyntheticSensorPipeline
from f1sim.planning.advisory import F1OvertakeAdvisor
from f1sim.planning.tactical import TacticalTrajectoryPlanner
from f1sim.planning.cbf_safety import DeterministicCBFFilter
from f1sim.data.export import RunArtifactExporter

class RaceScenario:
    DEFAULT_NUM_OPPONENTS = 1

    def __init__(self, max_opponents: int = DEFAULT_NUM_OPPONENTS):
        self.max_opponents = max_opponents
        self.clock = SimulationClock(dt_physics=0.01)  # 100 Hz
        self.world = SilverstoneCircuit()
        self.replayer = FastF1OpponentReplayer(
            year=2026, circuit="Silverstone", dt=0.01, track_length=self.world.track_length
        )
        
        self.ego_model = F1EnergyVehicleModel(dry_mass_kg=798.0, initial_fuel_kg=35.0)
        self.tire_model = PirelliTire("C2_MEDIUM")
        self.sensors = SyntheticSensorPipeline(latency_steps=2)
        self.advisor = F1OvertakeAdvisor()
        self.tactical_planner = TacticalTrajectoryPlanner(horizon_steps=15, ds=3.5)
        self.cbf = DeterministicCBFFilter(T_core_max=58.0, E_max_lap=8.5)
        self.exporter = RunArtifactExporter()

        all_cars = self.replayer.step(ego_s=0.0, ego_v_ms=0.0)
        active_codes = list(all_cars.keys())[:self.max_opponents]
        first_step = {c: all_cars[c] for c in active_codes}

        p1_code = active_codes[0] if active_codes else "LEC"
        p1_s = first_step[p1_code]["s"]
        p1_v = first_step[p1_code]["v_s"]

        # Spawn in slipstream corridor with initial lateral split on wide 14m circuit
        self.ego = {
            "s": float((p1_s - 18.0) % self.world.track_length),
            "y": 3.2,  # Start distinctly in clear passing lane
            "v_s": float(p1_v + 3.0)
        }
        self.ego_model.v = float(p1_v + 3.0)
        self.opponents = first_step
        self.latest_estimates = first_step
        self.latest_adv = {"tactical_call": "INIT"}

    def _get_signed_track_gap(self, s_target: float, s_ego: float) -> float:
        L = self.world.track_length
        delta = (s_target - s_ego) % L
        if delta > L / 2.0:
            delta -= L
        return float(delta)

    def step(self, user_override_strat="AUTO (DP Optimal)"):
        tick = self.clock.tick()
        all_opponents = self.replayer.step(ego_s=self.ego["s"], ego_v_ms=self.ego["v_s"])
        active_keys = list(all_opponents.keys())[:self.max_opponents]
        self.opponents = {k: all_opponents[k] for k in active_keys}

        if tick.get("run_perception", True):
            self.latest_estimates = self.sensors.observe(self.opponents)

        signed_gaps = {
            c: self._get_signed_track_gap(est["s"], self.ego["s"])
            for c, est in self.latest_estimates.items()
        }

        # Cars ahead within active lap window
        ahead_cars = {c: g for c, g in signed_gaps.items() if g > 0.0}
        if ahead_cars:
            target_code = min(ahead_cars, key=ahead_cars.get)
            gap_m = ahead_cars[target_code]
            is_leading = False
        else:
            target_code = min(signed_gaps, key=lambda c: abs(signed_gaps[c]))
            gap_m = signed_gaps[target_code]
            is_leading = True

        target_est = self.latest_estimates[target_code]
        opp_y = target_est.get("y", 0.0)
        target_v = target_est.get("v_s", 70.0)

        # ------------------------------------------------------------------
        # DECISIVE LATERAL CORRIDOR SEPARATION (14m Wide Circuit)
        # ------------------------------------------------------------------
        # Maintain high lateral offset: target +/- 3.2m to guarantee 2.5m+ vehicle clearance
        if opp_y >= 0.0:
            desired_pass_lane = -3.2  # Dive to right side
        else:
            desired_pass_lane = 3.2   # Dive to left side

        if not is_leading and gap_m < 50.0:
            target_y = desired_pass_lane
        else:
            target_y = 0.0 if is_leading else desired_pass_lane

        # Fast, assertive steering transition into the clear lane (0.35s lane change)
        lateral_error = target_y - self.ego["y"]
        self.ego["y"] += float(np.clip(lateral_error * 6.0 * self.clock.dt, -0.25, 0.25))

        # Check lateral clearance
        lateral_separation = abs(self.ego["y"] - opp_y)
        is_clear_lane = lateral_separation >= 2.2  # Clear lateral space for parallel pass

        # Corner braking calculation
        next_brake, dist_to_brake = self.world.get_next_braking_zone(self.ego["s"])
        v_entry = next_brake.get("v_entry", 34.0) if next_brake else 34.0
        
        need_braking = False
        if dist_to_brake < 80.0 and self.ego["v_s"] > (v_entry + 7.0):
            req_decel = (self.ego["v_s"]**2 - v_entry**2) / (2.0 * max(6.0, dist_to_brake))
            if req_decel > 22.0:
                need_braking = True

        # DRS slipstream active
        drs_active = (not is_leading) and (0.0 < gap_m < 35.0)

        # ------------------------------------------------------------------
        # POWER DISPATCH & PASS COMMITMENT
        # ------------------------------------------------------------------
        if need_braking:
            cmd_p_kw = -280.0
            throttle_cmd, brake_cmd = 0.0, 0.95
        elif not is_leading and gap_m < 4.0 and not is_clear_lane:
            # ONLY brake if directly behind without lateral offset (prevent bumper contact)
            cmd_p_kw = -80.0
            throttle_cmd, brake_cmd = 0.0, 0.50
        elif not is_leading and (gap_m < 35.0 or is_clear_lane):
            # FULL PASS COMMIT: 350kW MGU-K + full throttle to complete overtake
            cmd_p_kw = 350.0
            throttle_cmd, brake_cmd = 1.0, 0.0
        elif is_leading:
            cmd_p_kw = 220.0 if self.ego["v_s"] < 86.0 else 0.0
            throttle_cmd, brake_cmd = 0.9 if cmd_p_kw > 0 else 0.0, 0.0
        else:
            cmd_p_kw = 320.0
            throttle_cmd, brake_cmd = 1.0, 0.0

        p_safe_kw, cbf_active, cbf_type, _, _ = self.cbf.filter_power(
            cmd_p_kw, self.ego_model.T_battery_core, 36.0, 1.8, dt=self.clock.dt
        )

        # Apply DRS drag reduction during attack
        orig_cd = getattr(self.ego_model, "Cd", 0.95)
        if drs_active or (not is_leading and gap_m < 25.0):
            self.ego_model.Cd = orig_cd * 0.76  # 24% wake and DRS drag reduction

        car_tel = self.ego_model.step(
            throttle=throttle_cmd,
            brake=brake_cmd,
            mguk_target_w=p_safe_kw * 1e3,
            active_aero_mode=drs_active,
            dt=self.clock.dt
        )
        self.ego_model.Cd = orig_cd

        new_v = float(car_tel["v_ms"])
        step_ds = new_v * self.clock.dt

        # Spatial barrier: ONLY active if vehicles share the exact same lane
        if not is_leading and (not is_clear_lane) and (gap_m - step_ds) < 4.0:
            step_ds = max(0.0, gap_m - 4.0)
            new_v = min(new_v, target_v)

        self.ego["v_s"] = new_v
        self.ego["s"] = float((self.ego["s"] + step_ds) % self.world.track_length)

        # Tire dynamics
        tire_tel = self.tire_model.step(
            lateral_g=(self.ego["v_s"]**2) * 0.0012,
            speed_ms=self.ego["v_s"],
            in_wake=(not is_leading and gap_m < 18.0 and not is_clear_lane),
            gap_m=max(0.0, gap_m),
            dt=self.clock.dt
        )

        if is_leading:
            tactical_call = "P1 DEFEND // CLEAN AIR"
        elif is_clear_lane and gap_m < 8.0:
            tactical_call = "EXECUTING PASS ALONGSIDE"
        elif drs_active:
            tactical_call = "DRS ATTACK // SLIPSTREAM"
        else:
            tactical_call = "PURSUING"

        return {
            "time": tick["time"],
            "ego": self.ego,
            "opponents": self.opponents,
            "estimates": self.latest_estimates,
            "target_code": target_code,
            "gap_m": gap_m,
            "is_leading": is_leading,
            "car_tel": car_tel,
            "tire": tire_tel,
            "ers": {"soc_pct": car_tel["soc_pct"], "energy_mj": car_tel["soc_pct"] * 0.04},
            "advisor": {"tactical_call": tactical_call},
            "cbf_active": cbf_active or ((not is_clear_lane) and gap_m < 5.0),
            "cbf_type": "LANE_SEPARATION" if ((not is_clear_lane) and gap_m < 5.0) else cbf_type
        }
