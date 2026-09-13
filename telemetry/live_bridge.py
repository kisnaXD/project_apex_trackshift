import time

from f1sim.data.schema import (
    ERSTelemetry,
    EgoTelemetry,
    KinematicsTelemetry,
    TelemetryFrame,
    TireChannel,
)
from telemetry.hub import F1TelemetryHub


class LiveTelemetryBridge:
    def __init__(self, log_dir: str = "telemetry_logs"):
        self.hub = F1TelemetryHub(log_dir=log_dir)

    def assemble_and_publish(
        self,
        ego_model,
        ego_s,
        ego_y,
        tire_model,
        opponents_dict,
        drs_active,
        sim_time=0.0,
    ):
        v_kmh = float(ego_model.v * 3.6)
        track_len = 5891.0
        s_mod = float(ego_s % track_len)
        lap_num = int(ego_s // track_len) + 1

        kinematics = KinematicsTelemetry(
            s=s_mod,
            y=float(ego_y),
            x_world=0.0,
            y_world=0.0,
            heading_rad=0.0,
            speed_kmh=v_kmh,
            accel_long_g=float(getattr(ego_model, "a_lon", 0.0) / 9.81),
            accel_lat_g=1.8,
            yaw_rate_rads=0.0,
        )

        p_mguk_kw = 120.0 if v_kmh > 150.0 else 40.0
        ers = ERSTelemetry(
            soc_pct=float((ego_model.ES_joules / ego_model.ES_capacity_J) * 100.0),
            soh_pct=99.2,
            pack_temp_c=float(ego_model.T_battery_core),
            max_cell_temp_c=float(ego_model.T_battery_core + 2.1),
            current_demand_a=float((p_mguk_kw * 1000.0) / 750.0),
            voltage_v=750.0,
            mguk_power_kw=float(p_mguk_kw),
            lap_deployed_mj=float(ego_model.lap_deploy_J / 1.0e6),
        )

        tires = TireChannel(
            compound=tire_model.compound_name,
            laps_old=lap_num,
            wear_pct=float(tire_model.wear_pct),
            deg_rate_pct_per_lap=0.45,
            lap_time_deg_penalty_s=float(tire_model.wear_pct * 0.03),
            mu_friction_coeff=1.55,
            surface_temp_c={
                "FL": float(tire_model.T_surf),
                "FR": float(tire_model.T_surf),
                "RL": float(tire_model.T_surf),
                "RR": float(tire_model.T_surf),
            },
            carcass_temp_c={
                "FL": float(tire_model.T_carc),
                "FR": float(tire_model.T_carc),
                "RL": float(tire_model.T_carc),
                "RR": float(tire_model.T_carc),
            },
        )

        ego_tel = EgoTelemetry(
            session_time=float(sim_time),
            lap_number=lap_num,
            track_status="GREEN",
            kinematics=kinematics,
            ers=ers,
            tires=tires,
            throttle_pct=1.0 if ego_model.v > 20.0 else 0.0,
            brake_pressure_bar=0.0,
            gear=7 if v_kmh > 260.0 else (6 if v_kmh > 210.0 else 5),
            drs_active=bool(drs_active),
            fuel_remaining_kg=float(ego_model.fuel_mass_kg),
        )

        frame = TelemetryFrame(
            timestamp=time.time(),
            ego=ego_tel,
            opponents=opponents_dict,
        )

        self.hub.publish_frame(frame)
        return frame
