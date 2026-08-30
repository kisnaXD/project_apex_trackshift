"""Formula-E-inspired pack and stint numbers. Tuned so a 12-lap stint is visible."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PackParams:
    capacity_j: float = 50.0e3 * 3600.0  # 50 kWh
    v_ocv_empty: float = 600.0
    v_ocv_full: float = 800.0
    r_int_ohm: float = 0.085

    c_core_j_per_k: float = 1.2e5
    c_surf_j_per_k: float = 7.0e4
    r_cs_k_per_w: float = 9.0e-4
    r_sa_k_per_w: float = 1.5e-3
    t_amb_c: float = 32.0
    t_core_limit_c: float = 60.0

    p_max_w: float = 350_000.0
    p_attack_w: float = 320_000.0
    p_nominal_w: float = 210_000.0
    p_harvest_w: float = 115_000.0

    e_quota_lap_j: float = 8.5e3 * 3600.0  # 8.5 kWh / lap: nominal fits; late-lap attack is clipped
    lap_time_s: float = 92.0
    n_laps: int = 12

    alpha_thermal: float = 0.35  # 1/s  CBF class-K gain
    alpha_energy: float = 0.12  # 1/s
    alpha_soc: float = 0.20  # 1/s  remaining pack energy barrier
    t_limit_margin_c: float = 0.25

    def ocv_v(self, soc: float) -> float:
        s = min(1.0, max(0.0, soc))
        return self.v_ocv_empty + (self.v_ocv_full - self.v_ocv_empty) * s
