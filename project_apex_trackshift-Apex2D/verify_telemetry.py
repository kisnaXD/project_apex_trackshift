import os
import json
import glob
from telemetry.schemas import TelemetryFrame
from telemetry.live_bridge import LiveTelemetryBridge
from engine.f1_energy_model import F1EnergyVehicleModel
from engine.pirelli_tire import PirelliTire
from engine.tracks import SilverstoneCircuit
from telemetry.opponent_replay import FastF1OpponentReplayer

def test_telemetry_pipeline():
    print("[1/3] Initializing test models & telemetry bridge...")
    bridge = LiveTelemetryBridge(log_dir="telemetry_logs")
    car = F1EnergyVehicleModel()
    tire = PirelliTire("C2_MEDIUM")
    track = SilverstoneCircuit()
    
    # Mock opponents dictionary for standalone validation
    from telemetry.schemas import OpponentTelemetry, KinematicsTelemetry
    opponents_mock = {
        "VER": OpponentTelemetry(
            car_id="VER",
            driver_code="VER",
            kinematics=KinematicsTelemetry(
                s=710.0, y=0.0, x_world=120.0, y_world=45.0, heading_rad=0.34,
                speed_kmh=295.0, accel_long_g=0.2, accel_lat_g=1.4, yaw_rate_rads=0.02
            ),
            gap_to_ego_m=35.0,
            gap_to_ego_s=0.45,
            speed_gap_kmh=12.5,
            compound="HARD",
            laps_old=14,
            inferred_deg_delta_s=0.25,
            drs_active=True,
            is_clipping=False
        )
    }

    print("[2/3] Simulating step and publishing frame...")
    frame = bridge.assemble_and_publish(
        ego_model=car,
        ego_s=675.0,
        ego_y=-1.2,
        tire_model=tire,
        opponents_dict=opponents_mock,
        drs_active=True,
        sim_time=1.5
    )

    print("[3/3] Auditing published frame against mandatory channels...")
    data = frame.to_dict()
    ego = data["ego"]
    opps = data["opponents"]

    # Critical assertions
    checklist = [
        ("Ego SoC (%)", ego["ers"]["soc_pct"], 0.0, 100.0),
        ("Ego Battery Temp (°C)", ego["ers"]["pack_temp_c"], 20.0, 120.0),
        ("Ego MGU-K Power (kW)", ego["ers"]["mguk_power_kw"], -120.0, 120.0),
        ("Ego FL Tread Temp (°C)", ego["tires"]["surface_temp_c"]["FL"], 20.0, 160.0),
        ("Ego Wear (%)", ego["tires"]["wear_pct"], 0.0, 100.0),
        ("Ego Heading (rad)", ego["kinematics"]["heading_rad"], -3.15, 3.15),
        ("Ego Speed (km/h)", ego["kinematics"]["speed_kmh"], 0.0, 370.0),
        ("Opponent Count", len(opps), 1, 20),
        ("Opponent Speed Gap (km/h)", opps["VER"]["speed_gap_kmh"], -100.0, 100.0),
        ("Opponent DRS", opps["VER"]["drs_active"], None, None),
    ]

    all_passed = True
    print("\n{:<30} | {:<12} | {:<8}".format("Channel", "Value", "Status"))
    print("-" * 56)
    for name, val, low, high in checklist:
        if low is not None and high is not None:
            passed = (low <= val <= high)
        else:
            passed = val is not None
        status = "OK" if passed else "FAIL"
        if not passed:
            all_passed = False
        print(f"{name:<30} | {str(val)[:12]:<12} | {status:<8}")

    print("-" * 56)
    if all_passed:
        print("\nAll telemetry channels validated and publishing cleanly.")
    else:
        print("\nTelemetry audit failed: Some channels are missing or out of bounds.")

if __name__ == "__main__":
    test_telemetry_pipeline()