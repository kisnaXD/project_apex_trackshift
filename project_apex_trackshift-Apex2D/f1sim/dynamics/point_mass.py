import numpy as np

class F1EnergyVehicleModel:
    """
    2026 FIA Hybrid Powertrain Vehicle Dynamics Simulation:
      - 350 kW MGU-K Bi-directional Motor Generator Unit (No MGU-H)
      - ~400 kW Internal Combustion Engine (Sustainable Fuel Energy Flow Capped)
      - 8.5 MJ/lap Max Electrical Deployment Allowance
      - 4.0 MJ Usable Energy Store Buffer
      - Active Aerodynamics: Z-Mode (Cornering high drag/downforce) & X-Mode (Straight-line low drag)
      - First-Order Battery Cell Core Thermal Model
    """
    def __init__(self, dry_mass_kg: float = 798.0, initial_fuel_kg: float = 35.0):
        self.dry_mass_kg = dry_mass_kg
        self.fuel_mass_kg = initial_fuel_kg
        
        # 2026 Powertrain Specifications
        self.P_ice_max_W = 400.0e3         # 400 kW ICE
        self.P_mguk_max_deploy_W = 350.0e3 # 350 kW MGU-K Deployment
        self.P_mguk_max_regen_W = 350.0e3  # 350 kW MGU-K Kinetic Regeneration
        
        # Energy Storage System (ESS)
        self.ES_capacity_J = 4.0e6        # 4.0 MJ Physical Cell Capacity
        self.ES_joules = 3.0e6            # Nominal 75% State of Charge
        self.lap_deploy_J = 0.0           # Cumulative lap deployment
        self.lap_deploy_limit_J = 8.5e6   # 2026 Sporting Reg: 8.5 MJ/lap ceiling
        
        # Aerodynamics (Active Aero Dual-State)
        self.rho_air = 1.225
        self.frontal_area = 1.45
        self.cd_z_mode = 0.88             # Standard / Cornering Aero
        self.cd_x_mode = 0.62             # Low-drag active aero / MOM
        self.cr_rolling = 0.015
        
        # Thermal State
        self.T_battery_core = 45.0        # °C
        self.T_ambient = 28.0             # °C
        self.C_th = 3200.0                # Thermal capacity (J/K)
        self.R_th = 0.22                  # Thermal resistance to cooling circuit (K/W)
        self.R_internal = 0.085           # Battery pack internal resistance (Ohms)
        self.V_bus = 750.0                # DC Bus Nominal Voltage
        
        # Kinematics
        self.v = 50.0                     # m/s
        self.a_lon = 0.0                  # m/s^2

    def step(self, throttle: float, brake: float, mguk_target_w: float, active_aero_mode: bool, dt: float = 0.05) -> dict:
        """
        Advances the vehicle state by dt seconds.
        active_aero_mode: True = X-Mode/MOM (low drag), False = Z-Mode (high downforce)
        """
        mass = self.dry_mass_kg + self.fuel_mass_kg
        
        # 1. 2026 Regulatory Energy Floor & Ceiling Enforcements
        actual_mguk_w = 0.0
        if mguk_target_w > 0:
            # Check 8.5 MJ/lap regulatory cap and battery charge level
            allowable_by_cap = max(0.0, self.lap_deploy_limit_J - self.lap_deploy_J) / dt
            allowable_by_soc = max(0.0, self.ES_joules - 0.2e6) / dt
            actual_mguk_w = min(mguk_target_w, self.P_mguk_max_deploy_W, allowable_by_cap, allowable_by_soc)
            self.lap_deploy_J += actual_mguk_w * dt
            self.ES_joules -= actual_mguk_w * dt
        elif mguk_target_w < 0:
            # Regenerative braking
            allowable_regen_soc = max(0.0, self.ES_capacity_J - self.ES_joules) / dt
            actual_mguk_w = -min(abs(mguk_target_w), self.P_mguk_max_regen_W, allowable_regen_soc)
            self.ES_joules -= actual_mguk_w * dt * 0.94 # 94% round-trip efficiency

        # 2. ICE Power & Fuel Flow Consumption
        actual_ice_w = throttle * self.P_ice_max_W
        fuel_flow_g_per_sec = (actual_ice_w / 43.0e6) / 0.38
        self.fuel_mass_kg = max(0.0, self.fuel_mass_kg - fuel_flow_g_per_sec * (dt / 1000.0))

        # 3. Aerodynamic & Rolling Resistance
        cd = self.cd_x_mode if active_aero_mode else self.cd_z_mode
        f_aero = 0.5 * self.rho_air * cd * self.frontal_area * (self.v ** 2)
        f_roll = mass * 9.81 * self.cr_rolling

        # 4. Net Tractive Force & Acceleration
        p_total_w = actual_ice_w + actual_mguk_w
        if self.v > 1.0:
            f_traction = p_total_w / self.v
        else:
            f_traction = p_total_w / 1.0

        f_brake = brake * mass * 9.81 * 1.6 # Mechanical friction brakes
        f_net = f_traction - f_brake - f_aero - f_roll
        self.a_lon = f_net / mass

        # Integrate velocity
        self.v = max(0.0, self.v + self.a_lon * dt)

        # 5. Core Battery Thermal Evolution (Joule Heating + Dissipation)
        i_current = actual_mguk_w / self.V_bus
        q_joule = (i_current ** 2) * self.R_internal
        q_dissipated = (self.T_battery_core - self.T_ambient) / self.R_th
        dT_dt = (q_joule - q_dissipated) / self.C_th
        self.T_battery_core += dT_dt * dt

        return {
            "v_ms": self.v,
            "v_kmh": self.v * 3.6,
            "a_lon": self.a_lon,
            "soc_pct": (self.ES_joules / self.ES_capacity_J) * 100.0,
            "p_mguk_kw": actual_mguk_w / 1e3,
            "p_ice_kw": actual_ice_w / 1e3,
            "lap_deploy_mj": self.lap_deploy_J / 1e6,
            "T_battery_core": self.T_battery_core,
            "fuel_mass_kg": self.fuel_mass_kg,
            "current_a": i_current
        }

    def reset_lap_accounting(self):
        """Resets the 8.5 MJ regulatory deployment ledger upon crossing start/finish."""
        self.lap_deploy_J = 0.0