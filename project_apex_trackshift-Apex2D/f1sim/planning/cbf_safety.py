import numpy as np

class DeterministicCBFFilter:
    """
    Deterministic Edge Safety Filter (200 Hz / < 5 ms).
    Enforces forward invariance of:
    - Battery Core Thermal Envelope: h_th(x) = T_core_max - T_core >= 0
    - Regulatory Energy Quota: h_energy(x) = E_max_lap - E_lap(s) >= 0
    """
    def __init__(self, T_core_max=58.0, E_max_lap=4.0, alpha_th=0.6, alpha_reg=0.4):
        # Safety limits
        self.T_core_max = T_core_max   # deg C (Thermal limit)
        self.E_max_lap = E_max_lap     # kWh per lap limit
        self.alpha_th = alpha_th       # Extended Class-K gain for thermal barrier
        self.alpha_reg = alpha_reg     # Extended Class-K gain for energy barrier
        
        # Forgez coupled 2-state thermal parameters
        self.C_core = 90.0             # J/K
        self.C_surf = 45.0             # J/K
        self.R_cond = 1.35             # K/W
        self.R_conv = 0.80             # K/W
        self.R_int = 0.016             # Ohms
        self.T_cool = 28.0             # deg C
        self.V_term_nom = 750.0        # Volts

    def step_electro_thermal_plant(self, T_core, T_surf, E_lap, P_kw, dt):
        """
        Integrates Forgez continuous-discrete electro-thermal state equations.
        """
        P_watts = P_kw * 1000.0
        I_batt = P_watts / self.V_term_nom
        Q_joule = (I_batt ** 2) * self.R_int
        
        # Coupled radial differential gradients
        dT_core = (Q_joule - (T_core - T_surf) / self.R_cond) / self.C_core
        dT_surf = (((T_core - T_surf) / self.R_cond) - ((T_surf - self.T_cool) / self.R_conv)) / self.C_surf
        
        T_core_next = T_core + dT_core * dt
        T_surf_next = T_surf + dT_surf * dt
        
        # Cumulative per-lap deployment
        dE_kwh = max(0.0, (P_watts * dt) / 3.6e6)
        E_lap_next = E_lap + dE_kwh
        
        return T_core_next, T_surf_next, E_lap_next

    def filter_power(self, P_dem_kw, T_core, T_surf, E_lap, dt):
        """
        Solves the analytical active-set CBF-QP projection in < 1.5 ms:
        u_safe = argmin 0.5 * (u - u_demand)^2
        s.t.
          L_f h_th + L_g h_th * u + alpha_th * h_th >= 0
          L_f h_reg + L_g h_reg * u + alpha_reg * h_reg >= 0
        """
        # 1. Thermal Barrier: h_th = T_core_max - T_core >= 0
        h_th = self.T_core_max - T_core
        
        # Conductive dissipation rate to surface
        Q_diss = max(0.0, (T_core - T_surf) / self.R_cond)
        
        # Maximum allowable Joule heat rate before breaching forward invariance
        # Joule_max <= Q_diss + alpha_th * C_core * h_th
        headroom_watts = Q_diss + self.alpha_th * self.C_core * max(0.0, h_th)
        
        # Translate to maximum safe discharge power
        # P_thermal_max = V * sqrt(headroom / R_int)
        I_max = np.sqrt(max(0.0, headroom_watts / self.R_int))
        P_th_max_kw = (self.V_term_nom * I_max) / 1000.0

        # 2. Regulatory Energy Barrier: h_energy = E_max_lap - E_lap >= 0
        h_reg = self.E_max_lap - E_lap
        # Safe rate of deployment (kWh per second converted to kW)
        P_reg_max_kw = max(0.0, (self.alpha_reg * max(0.0, h_reg)) * 3600.0)

        # Upper bound intersection of all active safety barriers
        P_upper_bound = min(P_th_max_kw, P_reg_max_kw, 350.0)
        
        # Regen bound: -250 kW
        P_safe_kw = float(np.clip(P_dem_kw, -250.0, P_upper_bound))
        
        # Flag if filter intervened
        cbf_intervention = P_safe_kw < (P_dem_kw - 1.0)
        
        active_constraint = "None"
        if cbf_intervention:
            active_constraint = "Thermal Boundary" if P_th_max_kw <= P_reg_max_kw else "FIA Energy Quota"

        return P_safe_kw, cbf_intervention, active_constraint, P_th_max_kw, P_reg_max_kw