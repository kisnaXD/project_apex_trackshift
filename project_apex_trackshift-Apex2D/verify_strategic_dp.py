import numpy as np
from engine.strategic_dp import StrategicEnergyDP

dp = StrategicEnergyDP(n_nodes=40, node_ds=50.0)

print("[SOLVING GLOBAL BELLMAN RECURSION...]")
V_clean, policy_clean = dp.solve(in_traffic=False)
V_traffic, policy_traffic = dp.solve(in_traffic=True)

sim_clean = dp.forward_simulate_optimal_trace(policy_clean, initial_soc=0.75, in_traffic=False)
sim_traffic = dp.forward_simulate_optimal_trace(policy_traffic, initial_soc=0.75, in_traffic=True)

print(f"\n--- GLOBAL BENCHMARK RESULTS ---")
print(f"Clean Air Lap Time:   {sim_clean['total_time_s']:.3f} s")
print(f"Traffic Wake Lap Time: {sim_traffic['total_time_s']:.3f} s (Delta: +{sim_traffic['total_time_s'] - sim_clean['total_time_s']:.3f} s)")

print("\n--- SECTOR-BY-SECTOR ENERGY POLICY ALLOCATION ---")
print(f"{'Distance (m)':<14} | {'Sector Profile':<22} | {'Optimal MGU-K (kW)':<20} | {'SoC (%)':<10}")
print("-" * 74)

for i in range(0, 40, 4):
    dist = sim_clean['s_m'][i]
    p_kw = sim_clean['p_mguk_kw'][i]
    soc = sim_clean['soc_pct'][i]
    
    if dist <= 500:
        sec = "Passing Straight 1 (Wellington)"
    elif dist <= 1250:
        sec = "Technical / Cooling Complex"
    elif dist <= 1750:
        sec = "Passing Straight 2 (Hangar)"
    else:
        sec = "Braking Chicane"
        
    print(f"{dist:<14.0f} | {sec:<22} | {p_kw:<20.1f} | {soc:<10.1f}")