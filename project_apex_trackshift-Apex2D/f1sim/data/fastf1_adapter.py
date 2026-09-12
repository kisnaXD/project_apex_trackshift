import fastf1
import pandas as pd
import numpy as np
import time
from f1sim.data.schema import TelemetryFrame, EgoTelemetry, OpponentTelemetry, TireSet

class F1SessionReplayer:
    def __init__(self, year=2023, circuit="Silverstone", session_type="R", ego_driver="NOR", rival_drivers=None):
        if rival_drivers is None:
            rival_drivers = ["VER", "HAM", "LEC", "PIA"]
            
        fastf1.Cache.enable_cache("telemetry_logs/fastf1_cache")
        self.session = fastf1.get_session(year, circuit, session_type)
        self.session.load(telemetry=True, laps=True, weather=False)
        
        self.ego_driver = ego_driver
        self.rival_drivers = rival_drivers
        
        # Extract synchronized telemetry for all 5 cars
        self.drivers = [ego_driver] + rival_drivers
        self.driver_telemetry = {}
        
        # Find common time window across laps (e.g., Lap 10 to Lap 15)
        for d in self.drivers:
            laps = self.session.laps.pick_driver(d)
            # Pick a representative stint
            stint_laps = laps[(laps['LapNumber'] >= 10) & (laps['LapNumber'] <= 15)]
            tel = stint_laps.get_telemetry().add_distance()
            self.driver_telemetry[d] = tel

        # Harmonize on a uniform time delta (10 Hz = 0.1s step)
        self.time_steps = np.arange(0, 300, 0.1) # 5-minute replay window
        self.step_idx = 0

    def get_next_frame(self, dt=0.1) -> TelemetryFrame:
        if self.step_idx >= len(self.time_steps):
            return None

        current_time = self.time_steps[self.step_idx]
        self.step_idx += 1

        # Fetch interpolated row for Ego
        ego_raw = self._get_interpolated_row(self.ego_driver, self.step_idx)
        
        ego_tel = EgoTelemetry(
            session_time=current_time,
            lap_number=int(ego_raw.get("LapNumber", 12)),
            lap_distance_m=float(ego_raw.get("Distance", 0.0)),
            track_status="GREEN",
            s=float(ego_raw.get("Distance", 0.0) % 5891.0),
            y=0.0,
            speed_kmh=float(ego_raw.get("Speed", 0.0)),
            accel_long_g=0.0,
            accel_lat_g=0.0,
            yaw_rate_rads=0.0,
            throttle_pct=float(ego_raw.get("Throttle", 0.0)),
            brake_pressure_bar=100.0 if ego_raw.get("Brake", False) else 0.0,
            steering_angle_deg=0.0,
            gear=int(ego_raw.get("nGear", 7)),
            drs_active=(int(ego_raw.get("DRS", 0)) in [10, 12, 14]),
            ice_rpm=int(ego_raw.get("RPM", 11000)),
            fuel_flow_kgh=95.0,
            fuel_remaining_kg=35.0,
            ers_soc_pct=68.0,
            mguk_deploy_kw=120.0 if ego_raw.get("Throttle", 0.0) > 95 else 0.0,
            mguh_harvest_kw=0.0,
            battery_core_temp_c=48.0,
            is_clipping=False,
            strat_mode=2,
            tires=TireSet(compound="MEDIUM", laps_old=12, wear_pct=28.0)
        )

        # Build 4 Rivals
        opponents_tel = {}
        for r in self.rival_drivers:
            r_raw = self._get_interpolated_row(r, self.step_idx)
            r_dist = float(r_raw.get("Distance", 0.0) % 5891.0)
            gap_m = r_dist - ego_tel.s
            
            opponents_tel[r] = OpponentTelemetry(
                car_id=r,
                driver_code=r,
                s=r_dist,
                y=0.0,
                speed_kmh=float(r_raw.get("Speed", 0.0)),
                gap_to_ego_s=gap_m / max(10.0, float(r_raw.get("Speed", 10.0)) / 3.6),
                interval_ahead_s=0.5,
                drs_active=(int(r_raw.get("DRS", 0)) in [10, 12, 14]),
                inferred_gear=int(r_raw.get("nGear", 7)),
                is_clipping_visually=False,
                compound="HARD" if r == "VER" else "MEDIUM",
                estimated_tire_age_laps=14
            )

        return TelemetryFrame(timestamp=time.time(), ego=ego_tel, opponents=opponents_tel)

    def _get_interpolated_row(self, driver, idx):
        tel = self.driver_telemetry[driver]
        if idx < len(tel):
            return tel.iloc[idx]
        return tel.iloc[-1]