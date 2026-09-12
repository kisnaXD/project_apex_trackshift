"""
Deterministic Multi-Physics Single-Car Powertrain and Dynamics Engine.
Complies with 2026 FIA MGU-K deployment limits, 1-RC battery electrical dynamics,
two-node core-surface thermal balances, and Pacejka combined-slip tyre wear.
"""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class VehicleParameters:
    # Chassis Mass & Geometry
    mass_empty: float = 768.0        # Minimum vehicle mass without fuel (kg)
    mass_driver: float = 80.0        # Driver mass (kg)
    wheelbase: float = 3.600         # Wheelbase (m)
    weight_dist_front: float = 0.46  # Static front axle mass fraction
    h_cg: float = 0.30               # Center of gravity height (m)

    # Active Aerodynamics
    rho_air: float = 1.225           # Ambient air density (kg/m^3)
    frontal_area: float = 1.40       # Reference frontal area (m^2)
    cd_mode_z: float = 1.10          # Mode Z cornering drag coefficient
    cd_mode_x: float = 0.65          # Mode X straight-line drag coefficient
    cl_mode_z: float = 3.20          # Mode Z high downforce coefficient
    cl_mode_x: float = 1.40          # Mode X low drag lift coefficient
    crr: float = 0.012               # Rolling resistance coefficient

    # Powertrain & 2026 Regulations
    p_ice_max: float = 400000.0      # ICE peak power (400 kW)
    eta_drivetrain: float = 0.96     # Transmission mechanical efficiency
    eta_inverter: float = 0.95       # Bidirectional inverter efficiency
    p_mguk_max: float = 350000.0     # MGU-K peak power (350 kW)
    e_regen_lap_max: float = 8.5e6   # FIA kinetic recovery limit per lap (8.5 MJ)

    # 1-RC Battery Pack
    q_cell_nominal_ah: float = 5.0   # Individual cell capacity (Ah)
    cells_in_series: int = 180       # Series cell count (approx. 750V nominal)
    cells_parallel: int = 6          # Parallel branch count (30 Ah total pack)
    c1_polarization: float = 2500.0  # Polarization capacitance C1 (F)
    r1_polarization: float = 0.015   # Polarization resistance R1 (Ohms)
    v_term_min: float = 550.0        # Discharge cutoff voltage (V)
    v_term_max: float = 850.0        # Regeneration overvoltage cutoff (V)

    # Battery Two-Node Thermal Architecture
    c_core_bat: float = 14500.0      # Core heat capacity (J/K)
    c_surf_bat: float = 4500.0       # Casing surface heat capacity (J/K)
    r_core_surf_bat: float = 0.22    # Core-to-surface conductive resistance (K/W)
    r_surf_cool_bat: float = 0.12    # Casing-to-coolant convective resistance (K/W)
    t_coolant_bat: float = 35.0      # Active coolant loop temperature (degC)
    t_bat_warn: float = 55.0         # Thermal derating threshold (degC)
    t_bat_crit: float = 65.0         # Critical thermal shutdown temperature (degC)
    dvoc_dt: float = -0.00032        # Entropic coefficient dVoc/dT (V/K)

    # Pacejka & Tyre Thermodynamics
    r_effective: float = 0.33        # Loaded tyre rolling radius (m)
    mu_x_nominal: float = 1.75       # Peak longitudinal friction coefficient
    mu_y_nominal: float = 1.80       # Peak lateral friction coefficient
    k_load_sensitivity: float = 0.12 # Friction load sensitivity decay factor
    fz_ref: float = 4000.0           # Nominal reference tyre normal load (N)

    c_tyre_tread: float = 3800.0     # Tread surface heat capacity (J/K)
    c_tyre_carcass: float = 9500.0   # Bulk carcass heat capacity (J/K)
    r_tyre_cond: float = 0.18        # Tread-to-carcass thermal conduction (K/W)
    t_tyre_opt: float = 100.0        # Optimum thermal grip peak (degC)
    tyre_thermal_sigma: float = 22.0 # Thermal operating window bell-curve spread
    kappa_wear_rate: float = 4.2e-8  # Mechanical wear accumulation rate
    gamma_wear_temp: float = 1.35    # Thermal wear acceleration exponent


class SingleCarEnergyModel:
    """
    Deterministic 100 Hz multi-physics simulation kernel modeling vehicle
    kinematics, 2026 MGU-K logic, battery electro-thermal dynamics, and tyre wear.
    """
    def __init__(self, params: VehicleParameters = VehicleParameters(), dt: float = 0.01):
        self.p = params
        self.dt = dt
        self.reset()

    def reset(self, soc_init: float = 0.85, fuel_init_kg: float = 70.0):
        # Kinematic and Spatial Coordinates
        self.vx: float = 15.0
        self.s: float = 0.0
        self.n: float = 0.0
        self.yaw: float = 0.0
        self.heading_error: float = 0.0
        self.x_map: float = 0.0
        self.y_map: float = 0.0
        self.lap_count: int = 1
        self.fuel_kg: float = fuel_init_kg

        # Battery States
        self.soc: float = float(np.clip(soc_init, 0.0, 1.0))
        self.v1: float = 0.0
        self.t_core_bat: float = 40.0
        self.t_surf_bat: float = 38.0
        self.soh_capacity: float = 1.0
        self.soh_resistance: float = 1.0
        self.e_lap_regen: float = 0.0

        # Tyre States
        self.t_tyre_tread: float = 85.0
        self.t_tyre_carcass: float = 80.0
        self.tyre_wear: float = 0.0

        # Telemetry Cache
        self.p_deploy_delivered: float = 0.0
        self.p_regen_delivered: float = 0.0
        self.current_demand: float = 0.0
        self.current_delivered: float = 0.0
        self.voltage_terminal: float = 750.0
        self.delta_energy_wh: float = 0.0
        self.derate_factor: float = 1.0

    def compute_open_circuit_voltage(self, soc: float, t_core: float) -> float:
        soc_c = float(np.clip(soc, 0.0, 1.0))
        v_cell_ref = (
            3.40
            + 0.70 * soc_c
            - 0.35 * (soc_c ** 2)
            + 0.85 * (soc_c ** 3)
            - 0.45 * (soc_c ** 4)
        )
        v_cell = v_cell_ref + self.p.dvoc_dt * (t_core - 25.0)
        return float(v_cell * self.p.cells_in_series)

    def compute_internal_resistance(self, soc: float, t_core: float) -> float:
        t_kelvin = t_core + 273.15
        arrhenius = np.exp(1200.0 * (1.0 / t_kelvin - 1.0 / 298.15))
        soc_factor = 1.0 + 0.35 * ((1.0 - soc) ** 2)
        r_cell = 0.0018 * arrhenius * soc_factor
        r_pack = (r_cell * self.p.cells_in_series / self.p.cells_parallel) * self.soh_resistance
        return float(r_pack)

    def evaluate_2026_mguk_power_limit(self, speed_mps: float, manual_override: bool) -> float:
        speed_kmh = speed_mps * 3.6
        if manual_override:
            if speed_kmh <= 337.0:
                return self.p.p_mguk_max
            elif speed_kmh <= 355.0:
                taper = (355.0 - speed_kmh) / (355.0 - 337.0)
                return float(self.p.p_mguk_max * np.clip(taper, 0.0, 1.0))
            return 0.0
        else:
            if speed_kmh <= 290.0:
                return self.p.p_mguk_max
            elif speed_kmh <= 339.0:
                slope = (350000.0 - 105000.0) / (339.0 - 290.0)
                return float(350000.0 - slope * (speed_kmh - 290.0))
            elif speed_kmh <= 345.0:
                taper = (345.0 - speed_kmh) / (345.0 - 339.0)
                return float(105000.0 * np.clip(taper, 0.0, 1.0))
            return 0.0

    def solve_battery_electrical(
        self, p_mech_request: float, v_oc: float, r0: float
    ) -> tuple[float, float, float]:
        v_eff = v_oc - self.v1

        if p_mech_request >= 0.0:
            p_elec_req = p_mech_request / self.p.eta_inverter
        else:
            p_elec_req = p_mech_request * self.p.eta_inverter

        # Power transfer feasibility limit
        p_transfer_limit = (v_eff ** 2) / (4.0 * r0)
        if p_elec_req > p_transfer_limit:
            p_elec_req = p_transfer_limit
            i_pack = v_eff / (2.0 * r0)
        else:
            discriminant = max(0.0, v_eff ** 2 - 4.0 * r0 * p_elec_req)
            i_pack = (v_eff - np.sqrt(discriminant)) / (2.0 * r0)

        # Voltage boundary clipping
        i_discharge_max = (v_eff - self.p.v_term_min) / r0
        i_charge_max = (v_eff - self.p.v_term_max) / r0
        i_delivered = float(np.clip(i_pack, i_charge_max, i_discharge_max))

        v_terminal = float(v_eff - i_delivered * r0)
        p_elec_delivered = v_terminal * i_delivered

        if p_elec_delivered >= 0.0:
            p_mech_delivered = p_elec_delivered * self.p.eta_inverter
        else:
            p_mech_delivered = p_elec_delivered / self.p.eta_inverter

        return i_delivered, v_terminal, p_mech_delivered

    def step(
        self,
        throttle_cmd: float,
        brake_cmd: float,
        steer_angle_rad: float,
        manual_override: bool,
        mode_x_aero: bool,
        track_curvature: float = 0.0
    ) -> dict[str, float]:
        # 1. Aerodynamic Loads
        cd = self.p.cd_mode_x if mode_x_aero else self.p.cd_mode_z
        cl = self.p.cl_mode_x if mode_x_aero else self.p.cl_mode_z

        f_drag = 0.5 * self.p.rho_air * cd * self.p.frontal_area * (self.vx ** 2)
        f_downforce = 0.5 * self.p.rho_air * cl * self.p.frontal_area * (self.vx ** 2)

        m_total = self.p.mass_empty + self.p.mass_driver + self.fuel_kg
        f_z_total = m_total * 9.81 + f_downforce
        f_rr = self.p.crr * f_z_total

        # 2. Battery Thermal Derate
        if self.t_core_bat > self.p.t_bat_warn:
            derate = 1.0 - (self.t_core_bat - self.p.t_bat_warn) / (self.p.t_bat_crit - self.p.t_bat_warn)
            self.derate_factor = float(np.clip(derate, 0.0, 1.0))
        else:
            self.derate_factor = 1.0

        # 3. Powertrain Arbitration
        p_mguk_reg_limit = self.evaluate_2026_mguk_power_limit(self.vx, manual_override)
        p_mguk_avail = p_mguk_reg_limit * self.derate_factor
        if self.soc <= 0.05:
            p_mguk_avail = 0.0

        p_ice_dem = throttle_cmd * self.p.p_ice_max
        p_mguk_dem = throttle_cmd * p_mguk_avail

        p_regen_mech_req = 0.0
        if brake_cmd > 0.0 and self.soc < 0.98 and self.e_lap_regen < self.p.e_regen_lap_max:
            p_brake_total = brake_cmd * 800000.0
            p_regen_mech_req = -min(p_brake_total * 0.55, self.p.p_mguk_max)

        p_mech_req = p_mguk_dem if throttle_cmd > 0.0 else p_regen_mech_req

        # 4. Electrical Battery Resolution
        v_oc = self.compute_open_circuit_voltage(self.soc, self.t_core_bat)
        r0 = self.compute_internal_resistance(self.soc, self.t_core_bat)

        i_delivered, v_term, p_mguk_delivered = self.solve_battery_electrical(
            p_mech_req, v_oc, r0
        )

        self.current_delivered = i_delivered
        self.voltage_terminal = v_term

        if p_mguk_delivered >= 0.0:
            self.p_deploy_delivered = p_mguk_delivered
            self.p_regen_delivered = 0.0
        else:
            self.p_deploy_delivered = 0.0
            self.p_regen_delivered = -p_mguk_delivered
            self.e_lap_regen += (-p_mguk_delivered * self.dt)

        # 5. Tyre Thermo-Mechanical Adhesion
        mu_thermal = np.exp(
            -((self.t_tyre_tread - self.p.t_tyre_opt) ** 2) / (2.0 * (self.p.tyre_thermal_sigma ** 2))
        )
        load_sens = 1.0 / (1.0 + self.p.k_load_sensitivity * (f_z_total / self.p.fz_ref))
        wear_penalty = max(0.0, 1.0 - 0.35 * self.tyre_wear)

        mu_x_eff = self.p.mu_x_nominal * mu_thermal * load_sens * wear_penalty
        mu_y_eff = self.p.mu_y_nominal * mu_thermal * load_sens * wear_penalty

        p_drive_total = p_ice_dem + self.p_deploy_delivered
        f_prop_raw = (p_drive_total * self.p.eta_drivetrain) / max(self.vx, 1.0)
        f_brake_raw = brake_cmd * 25000.0
        f_net_longitudinal = f_prop_raw - f_brake_raw

        f_x_max = mu_x_eff * f_z_total
        f_y_demand = m_total * (self.vx ** 2) * track_curvature
        f_y_max = mu_y_eff * f_z_total

        ellipse_norm = (f_net_longitudinal / f_x_max) ** 2 + (f_y_demand / f_y_max) ** 2
        if ellipse_norm > 1.0:
            scale = 1.0 / np.sqrt(ellipse_norm)
            f_x_delivered = f_net_longitudinal * scale
            f_y_delivered = f_y_demand * scale
        else:
            f_x_delivered = f_net_longitudinal
            f_y_delivered = f_y_demand

        # 6. Chassis Acceleration and Curvilinear Kinematics
        dv_x = (f_x_delivered - f_drag - f_rr) / m_total
        self.vx = max(0.1, self.vx + dv_x * self.dt)

        denom = max(0.001, 1.0 - self.n * track_curvature)
        ds = (self.vx * np.cos(self.heading_error)) / denom
        dn = self.vx * np.sin(self.heading_error)
        yaw_rate = self.vx * track_curvature
        d_heading_error = yaw_rate - track_curvature * ds

        self.s += ds * self.dt
        self.n += dn * self.dt
        self.yaw += yaw_rate * self.dt
        self.heading_error += d_heading_error * self.dt
        self.x_map += self.vx * np.cos(self.yaw) * self.dt
        self.y_map += self.vx * np.sin(self.yaw) * self.dt

        # 7. Battery State Integration
        q_pack_as = self.p.q_cell_nominal_ah * 3600.0 * self.p.cells_parallel * self.soh_capacity
        self.soc = float(np.clip(self.soc - (i_delivered / q_pack_as) * self.dt, 0.0, 1.0))

        d_v1 = (-self.v1 / (self.p.r1_polarization * self.p.c1_polarization) 
                + i_delivered / self.p.c1_polarization) * self.dt
        self.v1 += d_v1

        # Bernardi Heat Generation
        q_joule = (i_delivered ** 2) * r0
        q_pol = (self.v1 ** 2) / self.p.r1_polarization
        t_core_k = self.t_core_bat + 273.15
        q_entropic = -i_delivered * t_core_k * (self.p.dvoc_dt * self.p.cells_in_series)
        q_gen_total = q_joule + q_pol + q_entropic

        q_core_to_surf = (self.t_core_bat - self.t_surf_bat) / self.p.r_core_surf_bat
        q_surf_to_cool = (self.t_surf_bat - self.p.t_coolant_bat) / self.p.r_surf_cool_bat

        self.t_core_bat += ((q_gen_total - q_core_to_surf) / self.p.c_core_bat) * self.dt
        self.t_surf_bat += ((q_core_to_surf - q_surf_to_cool) / self.p.c_surf_bat) * self.dt

        # Capacity and Resistance Ageing
        arrhenius_soh = np.exp((3500.0 / 8.314) * (1.0 / 298.15 - 1.0 / t_core_k))
        d_soh = -(abs(i_delivered) * self.dt) / (2.0 * 1500.0 * q_pack_as) * arrhenius_soh
        self.soh_capacity = max(0.60, self.soh_capacity + d_soh)
        self.soh_resistance = min(2.0, self.soh_resistance - 1.2 * d_soh)

        # 8. Tyre Thermal and Wear Integration
        p_slip = abs(f_x_delivered * (self.vx * 0.035)) + abs(f_y_delivered * (self.vx * np.sin(abs(steer_angle_rad))))
        q_cond = (self.t_tyre_tread - self.t_tyre_carcass) / self.p.r_tyre_cond
        h_conv = 15.0 + 1.8 * (self.vx ** 0.8)
        q_conv = h_conv * 0.085 * (self.t_tyre_tread - 25.0)

        self.t_tyre_tread += ((p_slip - q_cond - q_conv) / self.p.c_tyre_tread) * self.dt
        self.t_tyre_carcass += ((q_cond - 0.05 * (self.t_tyre_carcass - 25.0)) / self.p.c_tyre_carcass) * self.dt

        temp_ratio = max(1.0, self.t_tyre_tread / self.p.t_tyre_opt)
        d_wear = self.p.kappa_wear_rate * p_slip * (temp_ratio ** self.p.gamma_wear_temp) * self.dt
        self.tyre_wear = float(np.clip(self.tyre_wear + d_wear, 0.0, 1.0))

        # Fuel and Energy Accounting
        self.fuel_kg = max(0.0, self.fuel_kg - p_ice_dem * 0.000055 * self.dt)
        self.delta_energy_wh = (-i_delivered * v_term * self.dt) / 3600.0

        return {
            "speed_mps": self.vx,
            "speed_kmh": self.vx * 3.6,
            "track_s_m": self.s,
            "track_n_m": self.n,
            "heading_error_rad": self.heading_error,
            "pos_x_m": self.x_map,
            "pos_y_m": self.y_map,
            "battery_soc": self.soc,
            "battery_soh_capacity": self.soh_capacity,
            "battery_soh_resistance": self.soh_resistance,
            "battery_t_core_c": self.t_core_bat,
            "battery_t_surf_c": self.t_surf_bat,
            "battery_i_delivered_a": self.current_delivered,
            "battery_v_terminal_v": self.voltage_terminal,
            "battery_power_deploy_kw": self.p_deploy_delivered / 1000.0,
            "battery_power_regen_kw": self.p_regen_delivered / 1000.0,
            "battery_thermal_derate": self.derate_factor,
            "delta_energy_wh": self.delta_energy_wh,
            "tyre_t_tread_c": self.t_tyre_tread,
            "tyre_t_carcass_c": self.t_tyre_carcass,
            "tyre_wear_fraction": self.tyre_wear,
            "tyre_life_remaining": max(0.0, 1.0 - self.tyre_wear),
            "tyre_grip_multiplier": float(mu_thermal * wear_penalty),
            "lap_regen_mj": self.e_lap_regen / 1e6,
            "fuel_mass_kg": self.fuel_kg,
            "lap_time_loss_sec": 2.8 * self.tyre_wear + 1.2 * (self.tyre_wear ** 2)
        }


def extract_relative_speed_gap(
    ego_telemetry: dict[str, float], opponent_telemetry: dict[str, float]
) -> dict[str, float]:
    """
    Computes track-progress spatial offsets and closing dynamics between
    ego telemetry and time-aligned opponent tracking estimates.
    """
    spatial_gap = opponent_telemetry["track_s_m"] - ego_telemetry["track_s_m"]
    speed_gap = ego_telemetry["speed_mps"] - opponent_telemetry["speed_mps"]
    closing_speed = speed_gap * np.cos(ego_telemetry["heading_error_rad"])

    return {
        "spatial_gap_m": spatial_gap,
        "speed_gap_mps": speed_gap,
        "closing_speed_mps": closing_speed,
        "heading_delta_rad": ego_telemetry["heading_error_rad"] - opponent_telemetry["heading_error_rad"]
    }