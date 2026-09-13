import numpy as np

class F1Powertrain:
    """
    Simulates an FIA-spec F1 Turbo-Hybrid PU:
    - Internal Combustion Engine (ICE): Max 580 kW (100 kg/h fuel-flow limit)
    - MGU-K: Exactly 120 kW electrical deployment / 4.0 MJ maximum per-lap cap
    - MGU-H: Unlimited turbo-exhaust thermodynamic energy harvesting
    - ERS Energy Store: 4.0 MJ capacity battery with state of charge (SoC)
    """
    def __init__(self):
        self.P_ice_max_kw = 580.0
        self.P_mguk_max_kw = 120.0
        self.battery_capacity_mj = 4.0
        self.soc_joules = 3.2 * 1e6   # Initial 80% SoC
        self.lap_energy_deployed_j = 0.0
        self.T_core = 45.0             # Core temperature (°C)
        self.T_surf = 35.0

    def step(self, strat_mode, throttle, brake, speed_ms, dt=0.05):
        """
        strat_mode:
          - 'STRAT_2_ATTACK': Full ICE + Max 120 kW MGU-K continuous dump
          - 'STRAT_7_PACE': Balanced pace (60 kW deployment, net neutral)
          - 'STRAT_11_HARVEST': ICE only, MGU-K regen under braking (up to 2.0 MJ/lap)
        """
        # 1. ICE Power Output
        p_ice = self.P_ice_max_kw * throttle

        # 2. MGU-K Deployment based on Strat
        p_mguk_deploy = 0.0
        p_mguk_regen = 0.0

        can_deploy = (self.soc_joules > 0.1 * 1e6) and (self.lap_energy_deployed_j < self.battery_capacity_mj * 1e6)

        if throttle > 0.8 and can_deploy:
            if strat_mode == 'STRAT_2_ATTACK':
                p_mguk_deploy = self.P_mguk_max_kw
            elif strat_mode == 'STRAT_7_PACE':
                p_mguk_deploy = 65.0
            elif strat_mode == 'STRAT_11_HARVEST':
                p_mguk_deploy = 0.0
        elif brake > 0.2:
            # Regenerative braking into MGU-K (max 120 kW recovery)
            p_mguk_regen = min(120.0, brake * 140.0)

        # 3. Energy Account & Clipping
        energy_out = p_mguk_deploy * 1000.0 * dt
        energy_in = p_mguk_regen * 1000.0 * dt
        self.soc_joules = np.clip(self.soc_joules - energy_out + energy_in, 0.0, self.battery_capacity_mj * 1e6)
        self.lap_energy_deployed_j += energy_out

        is_clipping = (self.soc_joules <= 0.15 * 1e6) or (self.lap_energy_deployed_j >= self.battery_capacity_mj * 1e6)
        if is_clipping and p_mguk_deploy > 0.0:
            p_mguk_deploy = 0.0  # Electrical power drops off abruptly

        # 4. Thermal Model (I^2 * R heating + conductive air cooling)
        heat_gen = 0.00015 * (p_mguk_deploy**2 + p_mguk_regen**2)
        cooling = 0.035 * (self.T_core - 35.0) + 0.001 * speed_ms
        self.T_core += (heat_gen - cooling) * dt

        total_power_kw = p_ice + p_mguk_deploy
        soc_pct = (self.soc_joules / (self.battery_capacity_mj * 1e6)) * 100.0

        return {
            "total_p_kw": total_power_kw,
            "p_ice_kw": p_ice,
            "p_mguk_kw": p_mguk_deploy,
            "soc_pct": soc_pct,
            "is_clipping": is_clipping,
            "T_core": self.T_core
        }

    def reset_lap_energy(self):
        self.lap_energy_deployed_j = 0.0