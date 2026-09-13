import sys
import os
import asyncio
import json
import numpy as np
import websockets
import copy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from f1sim.engine.world import SilverstoneCircuit
from f1sim.engine.replay import FastF1OpponentReplayer
from f1sim.dynamics.point_mass import F1EnergyVehicleModel
from f1sim.dynamics.tyres import PirelliTire
from f1sim.planning.cbf_safety import DeterministicCBFFilter

PREDICTION_HORIZON_S = 30.0

world = SilverstoneCircuit()
track_pts = []
for s_val in np.linspace(0.0, world.track_length, 750, endpoint=False):
    cx, cy, psi = world.frenet_to_cartesian(s_val, 0.0)
    track_pts.append({
        "s": float(s_val), "x": float(cx), "y": float(cy), "psi": float(psi)
    })

TEAM_COLORS = {
    "BEA": "#cfd4d5", "LEC": "#e10600", "VER": "#1e41ff", "NOR": "#ff8000",
    "HAM": "#e10600", "RUS": "#00d2be", "ALO": "#229971", "SAI": "#005aff", "PIA": "#ff8000"
}

DRIVER_NAMES = {
    "BEA": "O. Bearman", "LEC": "C. Leclerc", "VER": "M. Verstappen",
    "NOR": "L. Norris", "HAM": "L. Hamilton", "RUS": "G. Russell",
    "ALO": "F. Alonso", "SAI": "C. Sainz", "PIA": "O. Piastri"
}

class SimEngine:
    def __init__(self):
        self.world = world
        self.replayer = FastF1OpponentReplayer(track_length=self.world.track_length, dt=0.016)
        self.ego_model = F1EnergyVehicleModel(dry_mass_kg=798.0, initial_fuel_kg=35.0)
        self.tire_model = PirelliTire("C2_MEDIUM")
        self.cbf = DeterministicCBFFilter(T_core_max=58.0, E_max_lap=8.5)
        
        self.smooth_margin = 15.0
        self.smooth_deploy_kw = 0.0
        self.smooth_mj = 0.0
        self.current_action = "HOLD"
        self.current_title = "HOLD PRESSURE IN SLIPSTREAM"
        self.current_radio = "Hold tow. Preserve battery charge."
        self.passing_side = "OVERLAP INSIDE (LEFT)"
        self.target_lateral_y = -2.6
        self.action_timer = 0.0
        
        self.current_lap = 14
        self.lap_time_s = 38.45
        self.session_time_s = 1324.5
        self.reset()

    def reset(self):
        self.replayer.reset()
        self.sim_time = 0.0
        self.is_running = False
        self.prediction_mode = False
        self.forecast_frames = []
        self.forecast_idx = 0
        self.current_lap = 14
        self.lap_time_s = 38.45
        self.session_time_s = 1324.5
        
        opps = self.replayer.step()
        p1 = list(opps.values())[0]
        self.ego = {
            "s": float((p1["s"] - 14.0) % self.world.track_length),
            "y": -2.4,
            "v_s": float(p1["v_s"] + 1.2)
        }
        self.ego_model.v = self.ego["v_s"]
        self.last_frame = self.step()

    def step(self):
        dt = 0.016
        self.sim_time += dt
        if self.is_running:
            self.session_time_s += dt
            self.lap_time_s += dt

        prev_s = self.ego["s"]
        opps_raw = self.replayer.step(self.ego["s"], self.ego["v_s"])

        L = self.world.track_length
        ahead_cars = {}
        for code, data in opps_raw.items():
            d = (data["s"] - self.ego["s"]) % L
            if d > L / 2.0: d -= L
            if d > 0: ahead_cars[code] = d

        target_code = min(ahead_cars, key=ahead_cars.get) if ahead_cars else "LEC"
        gap_m = float(ahead_cars[target_code]) if ahead_cars else 12.0
        target_car = opps_raw.get(target_code, list(opps_raw.values())[0])
        target_v = target_car["v_s"]
        soc = (self.ego_model.ES_joules / self.ego_model.ES_capacity_J) * 100.0

        in_overtake_zone = self.world.is_in_overtake_zone(self.ego["s"])
        car_is_moving = bool(self.is_running and self.ego["v_s"] > 35.0)

        # Tactical decision and lateral lane selection
        if gap_m < 16.0 and (self.ego["v_s"] - target_v)*3.6 > 2.0 and soc > 35.0:
            raw_action, raw_title = "ATTACK", "ATTACK: 350 kW DEPLOY OVERRIDE"
            raw_radio = f"DRS tow active on {target_code}. Commit inside pass."
            cmd_p_kw, rec_color = 350.0, "#00d2be"
            self.passing_side = "OVERLAP INSIDE (LEFT)"
            self.target_lateral_y = -2.6
        elif soc < 28.0:
            raw_action, raw_title = "HARVEST", "HARVEST: LIFT & COAST RECOVERY"
            raw_radio = f"Energy Store {soc:.1f}%. Lift and coast into brake phase."
            cmd_p_kw, rec_color = -120.0, "#22c55e"
            self.passing_side = "CENTRAL LINE"
            self.target_lateral_y = 0.0
        elif gap_m < 35.0:
            raw_action, raw_title = "HOLD", "HOLD: SLIPSTREAM TOW PRESSURE"
            raw_radio = f"Maintain tow on {target_code}. Heat management priority."
            cmd_p_kw, rec_color = 240.0, "#f59e0b"
            self.passing_side = "OVERLAP OUTSIDE (RIGHT)"
            self.target_lateral_y = 2.6
        else:
            raw_action, raw_title = "BUILD", "BUILD: DELTA STABILIZATION"
            raw_radio = "Clean air. Maintain stint delta target."
            cmd_p_kw, rec_color = 280.0, "#94a3b8"
            self.passing_side = "CENTRAL LINE"
            self.target_lateral_y = 0.0

        # Smooth lateral vehicle offset transition
        self.ego["y"] += (self.target_lateral_y - self.ego["y"]) * 0.08

        if raw_action != self.current_action:
            self.action_timer += dt
            if self.action_timer > 1.5 or raw_action == "ATTACK":
                self.current_action, self.current_title, self.current_radio = raw_action, raw_title, raw_radio
                self.action_timer = 0.0
        else:
            self.action_timer = 0.0

        p_safe_kw, cbf_active, cbf_type, _, _ = self.cbf.filter_power(
            cmd_p_kw, self.ego_model.T_battery_core, 36.0, self.ego_model.lap_deploy_J / 3.6e6, dt=dt
        )

        active_aero = bool(self.current_action == "ATTACK" and in_overtake_zone)
        throttle = min(1.0, max(0.0, p_safe_kw / 350.0))
        brake = min(1.0, max(0.0, -p_safe_kw / 280.0))
        car_tel = self.ego_model.step(throttle, brake, p_safe_kw * 1e3, active_aero_mode=active_aero, dt=dt)
        
        if gap_m > 25.0: self.ego["v_s"] = min(self.ego["v_s"] + 0.4, target_v + 3.0)
        elif gap_m < 4.0: self.ego["v_s"] = max(self.ego["v_s"] - 0.5, target_v - 1.5)
        else: self.ego["v_s"] = float(car_tel["v_ms"])

        new_s = (self.ego["s"] + self.ego["v_s"] * dt) % self.world.track_length
        if new_s < prev_s and self.is_running:
            self.current_lap += 1
            self.lap_time_s = 0.0
            self.ego_model.lap_deploy_J = 0.0
        self.ego["s"] = float(new_s)

        tire_tel = self.tire_model.step(
            lateral_g=(self.ego["v_s"]**2) * 0.0012, speed_ms=self.ego["v_s"],
            in_wake=(gap_m < 20.0), gap_m=abs(gap_m), dt=dt
        )

        ego_x, ego_y, ego_psi = self.world.frenet_to_cartesian(self.ego["s"], self.ego["y"])
        self.smooth_margin = (0.92 * self.smooth_margin) + (0.08 * gap_m)
        self.smooth_deploy_kw = (0.95 * self.smooth_deploy_kw) + (0.05 * p_safe_kw)
        self.smooth_mj = (0.95 * self.smooth_mj) + (0.05 * float(car_tel["lap_deploy_mj"]))

        # Build combined grid list (all opponents + Bearman)
        grid_cars = []
        
        # Add Bearman (Ego)
        grid_cars.append({
            "is_ego": True,
            "code": "BEA",
            "name": "O. Bearman (HAAS)",
            "color": "#cfd4d5",
            "x": float(ego_x),
            "y": float(ego_y),
            "s": float(self.ego["s"]),
            "lateral_y": float(self.ego["y"]),
            "heading": float(ego_psi),
            "speed_kmh": float(self.ego["v_s"] * 3.6 if self.is_running else 0.0),
            "rel_speed_kmh": 0.0,
            "gap_m": 0.0,
            "gap_sec": 0.0,
            "compound": "C2 MEDIUM",
            "tyre_age": 14,
            "drs_eligible": bool(car_is_moving and in_overtake_zone and gap_m <= 32.0),
            "is_clipping": False,
            "overtake_advisory": self.current_action if self.is_running else "ON GRID",
            "pass_probability": float(np.clip(60.0 + (soc - 30.0)*0.5 - gap_m*0.3, 5.0, 98.0)) if self.is_running else 0.0
        })

        for code, opp in opps_raw.items():
            ox, oy, opsi = self.world.frenet_to_cartesian(opp["s"], opp.get("y", 0.0))
            opp_d = (opp["s"] - self.ego["s"]) % L
            if opp_d > L / 2.0: opp_d -= L

            rel_speed_kmh = (self.ego["v_s"] - opp["v_s"]) * 3.6
            gap_sec = opp_d / max(5.0, opp["v_s"])
            drs_window = bool(car_is_moving and in_overtake_zone and (0 < opp_d <= 32.0) and (gap_sec <= 1.0))
            opp_clipping = bool(car_is_moving and opp["v_s"] * 3.6 > 325.0 and in_overtake_zone)

            grid_cars.append({
                "is_ego": False,
                "code": code,
                "name": DRIVER_NAMES.get(code, code),
                "color": TEAM_COLORS.get(code, "#31d7c7"),
                "x": float(ox),
                "y": float(oy),
                "s": float(opp["s"]),
                "lateral_y": float(opp.get("y", 0.0)),
                "heading": float(opsi),
                "speed_kmh": float(opp["v_s"] * 3.6 if self.is_running else 0.0),
                "rel_speed_kmh": float(rel_speed_kmh if self.is_running else 0.0),
                "gap_m": float(opp_d),
                "gap_sec": float(gap_sec),
                "compound": opp.get("compound", "MEDIUM"),
                "tyre_age": int(opp.get("lap", 1) * 3 + 2),
                "drs_eligible": drs_window,
                "is_clipping": opp_clipping,
                "overtake_advisory": "ATTACK (DRS)" if (drs_window and soc > 30.0) else ("HOLD TOW" if 0 < opp_d < 45 else ("DEFEND" if -30 < opp_d <= 0 else "STABLE")),
                "pass_probability": float(np.clip(50.0 + rel_speed_kmh * 2.5 - max(0.0, opp_d) * 1.2, 5.0, 95.0)) if self.is_running else 0.0
            })

        # Sort strictly by true race track position (highest signed distance ahead is P1)
        grid_cars.sort(key=lambda c: c["gap_m"], reverse=True)
        for idx, car in enumerate(grid_cars):
            car["pos"] = idx + 1

        # Real-time MPC candidate tubes with lateral overtaking corridor
        mpc_horizon_m = 70.0
        mpc_steps = 25
        mpc_candidates = []
        for lane_name, lane_offset in [("INSIDE", -2.6), ("CENTER", 0.0), ("OUTSIDE", 2.6)]:
            path_pts = []
            for step_idx in range(mpc_steps):
                ds = (step_idx / float(mpc_steps - 1)) * mpc_horizon_m
                interp_y = self.ego["y"] + (lane_offset - self.ego["y"]) * min(1.0, ds / 22.0)
                px, py, ppsi = self.world.frenet_to_cartesian(self.ego["s"] + ds, interp_y)
                path_pts.append({
                    "x": float(px), "y": float(py), "ds": float(ds), "dy": float(interp_y), "psi": float(ppsi)
                })
            
            is_optimal = bool(
                (lane_name == "INSIDE" and "INSIDE" in self.passing_side) or
                (lane_name == "OUTSIDE" and "OUTSIDE" in self.passing_side) or
                (lane_name == "CENTER" and "CENTRAL" in self.passing_side)
            )
            mpc_candidates.append({
                "lane": lane_name,
                "offset": lane_offset,
                "optimal": is_optimal,
                "points": path_pts
            })

        lap_m, lap_s = divmod(self.lap_time_s, 60)
        ses_m, ses_s = divmod(self.session_time_s, 60)
        lap_str = f"{int(lap_m):02d}:{lap_s:05.2f}"
        ses_str = f"{int(ses_m):02d}:{ses_s:04.1f}"

        return {
            "mode": "LIVE",
            "time": self.sim_time,
            "target": target_code,
            "session_time": ses_str,
            "lap_time": lap_str,
            "lap_num": self.current_lap,
            "recommendation": {
                "action": self.current_action if self.is_running else "STANDSTILL",
                "title": self.current_title if self.is_running else "STANDSTILL / GRID PHASE",
                "radio": self.current_radio if self.is_running else "Car on grid. Standing by.",
                "color": rec_color if self.is_running else "#64748b",
                "p_deploy_kw": float(self.smooth_deploy_kw if self.is_running else 0.0),
                "cbf_active": cbf_active if self.is_running else False,
                "cbf_type": cbf_type if self.is_running else "SAFE",
                "margin_m": float(self.smooth_margin),
                "passing_side": self.passing_side if self.is_running else "CENTRAL LINE",
                "confidence": float(np.clip(60.0 + (soc - 30.0)*0.5 - gap_m*0.3, 5.0, 98.0)) if self.is_running else 0.0
            },
            "ego": {
                "code": "BEA",
                "x": float(ego_x),
                "y": float(ego_y),
                "s": float(self.ego["s"]),
                "lateral_y": float(self.ego["y"]),
                "heading": float(ego_psi),
                "speed_kmh": float(self.ego["v_s"] * 3.6 if self.is_running else 0.0),
                "soc": float(car_tel["soc_pct"]),
                "soh": 99.4,
                "temp_core": float(car_tel["T_battery_core"]),
                "tire_deg": float(tire_tel["wear_pct"]),
                "tire_temp": float(tire_tel["T_surf"]),
                "lap_mj": float(self.smooth_mj),
                "throttle_pct": float(throttle * 100.0 if self.is_running else 0.0),
                "brake_pct": float(brake * 100.0 if self.is_running else 0.0),
                "gear": (7 if (self.ego["v_s"] * 3.6) > 260 else (6 if (self.ego["v_s"] * 3.6) > 210 else 5)) if self.is_running else 1
            },
            "mpc_candidates": mpc_candidates,
            "grid_order": grid_cars,
            "opponents": [c for c in grid_cars if not c["is_ego"]]
        }

    def generate_prediction(self, horizon_s=PREDICTION_HORIZON_S):
        dt = 0.05
        steps = int(horizon_s / dt)
        sim_copy = SimEngine()
        sim_copy.is_running = True
        sim_copy.sim_time = self.sim_time
        sim_copy.replayer.current_time = self.replayer.current_time
        sim_copy.replayer.driver_states = copy.deepcopy(self.replayer.driver_states)
        sim_copy.ego = copy.deepcopy(self.ego)
        sim_copy.ego_model.v = self.ego["v_s"]
        sim_copy.ego_model.ES_joules = self.ego_model.ES_joules
        sim_copy.ego_model.T_battery_core = self.ego_model.T_battery_core

        forecast = []
        path_history = []
        for _ in range(steps):
            f = sim_copy.step()
            f["mode"] = "PREDICT"
            path_history.append({"x": f["ego"]["x"], "y": f["ego"]["y"]})
            forecast.append(f)

        for f in forecast:
            f["predicted_path"] = path_history

        self.forecast_frames = forecast
        self.forecast_idx = 0
        self.prediction_mode = True

sim = SimEngine()

async def handler(websocket):
    try:
        await websocket.send(json.dumps({"type": "TRACK_GEOMETRY", "track": track_pts}))
        async def rx():
            async for msg in websocket:
                cmd = json.loads(msg).get("cmd")
                if cmd == "PLAY": sim.is_running, sim.prediction_mode = True, False
                elif cmd == "PAUSE": sim.is_running = False
                elif cmd == "RESET": sim.reset()
                elif cmd == "PREDICT": sim.generate_prediction()
        asyncio.create_task(rx())

        while True:
            if sim.is_running:
                if sim.prediction_mode:
                    if sim.forecast_idx < len(sim.forecast_frames):
                        frame = sim.forecast_frames[sim.forecast_idx]
                        sim.forecast_idx += 1
                    else:
                        sim.prediction_mode = False
                        frame = sim.last_frame
                else:
                    frame = sim.step()
                    sim.last_frame = frame
            elif sim.prediction_mode:
                if sim.forecast_idx < len(sim.forecast_frames):
                    frame = sim.forecast_frames[sim.forecast_idx]
                    sim.forecast_idx += 1
                else:
                    sim.prediction_mode = False
                    frame = sim.last_frame
            else:
                frame = sim.last_frame

            frame["type"] = "TELEMETRY_TICK"
            frame["prediction_mode"] = sim.prediction_mode
            await websocket.send(json.dumps(frame))
            await asyncio.sleep(0.016)
    except websockets.exceptions.ConnectionClosed:
        pass

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())