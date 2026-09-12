import numpy as np
from engine.f1_energy_model import F1EnergyVehicleModel

# Initialize car with 40 kg starting fuel and 75% battery SoC
car = F1EnergyVehicleModel(dry_mass_kg=798.0, initial_fuel_kg=40.0)
dt = 0.05  # 50 ms simulation tick

print(f"{'Time (s)':<10} | {'Speed (km/h)':<14} | {'SoC (%)':<10} | {'Deploy (MJ)':<12} | {'Batt T (°C)':<12} | {'Fuel (kg)':<10}")
print("-" * 75)

# Phase 1: Full acceleration with maximum 120 kW MGU-K deploy (0 to 5 seconds)
for t in np.arange(0.0, 5.0, dt):
    telemetry = car.step(
        throttle=1.0, 
        brake=0.0, 
        mguk_target_w=120.0e3,  # +120 kW deploy
        drs_active=True, 
        dt=dt
    )
    if round(t, 2) % 1.0 == 0:
        print(f"{t:<10.1f} | {telemetry['v_kmh']:<14.1f} | {telemetry['soc_pct']:<10.1f} | {telemetry['lap_deploy_mj']:<12.3f} | {telemetry['T_core']:<12.2f} | {telemetry['fuel_kg']:<10.3f}")

# Phase 2: Coasting down the straight to verify aerodynamic drag deceleration (5 to 7 seconds)
print("\n[COASTING - ZERO THROTTLE / ZERO DEPLOY]")
for t in np.arange(5.0, 7.0, dt):
    telemetry = car.step(
        throttle=0.0, 
        brake=0.0, 
        mguk_target_w=0.0, 
        drs_active=False, 
        dt=dt
    )
    if round(t, 2) % 1.0 == 0:
        print(f"{t:<10.1f} | {telemetry['v_kmh']:<14.1f} | {telemetry['soc_pct']:<10.1f} | {telemetry['lap_deploy_mj']:<12.3f} | {telemetry['T_core']:<12.2f} | {telemetry['fuel_kg']:<10.3f}")

# Phase 3: Heavy braking into corner with maximum MGU-K harvesting (7 to 9 seconds)
print("\n[HEAVY BRAKING - 120 kW MGU-K REGEN]")
for t in np.arange(7.0, 9.0, dt):
    telemetry = car.step(
        throttle=0.0, 
        brake=0.8, 
        mguk_target_w=-120.0e3,  # -120 kW harvesting
        drs_active=False, 
        dt=dt
    )
    if round(t, 2) % 1.0 == 0:
        print(f"{t:<10.1f} | {telemetry['v_kmh']:<14.1f} | {telemetry['soc_pct']:<10.1f} | {telemetry['lap_deploy_mj']:<12.3f} | {telemetry['T_core']:<12.2f} | {telemetry['fuel_kg']:<10.3f}")