"""Control Barrier Function QP: max safe power s.t. thermal + energy barriers."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter

import numpy as np
import osqp
from scipy import sparse

from apex.params import PackParams
from apex.physics import BatteryState, dq_dp, thermal_rates


@dataclass
class CBFResult:
    p_safe: float
    p_requested: float
    clipped: bool
    solve_time_ms: float
    solver_status: str
    thermal_dual: float
    energy_dual: float
    thermal_bound_w: float
    energy_bound_w: float
    thermal_slack_w: float
    energy_slack_w: float
    active_constraints: list[str]
    iterations: int
    qp: dict = field(default_factory=dict)


def _bounds(state: BatteryState, p_lin: float, params: PackParams) -> tuple[float, float, dict]:
    h_t = (params.t_core_limit_c - params.t_limit_margin_c) - state.t_core_c
    q, dqdP, i = dq_dp(max(p_lin, 1.0), state.soc, params)
    tdot, _ = thermal_rates(state.t_core_c, state.t_surf_c, q, params)
    g_t = dqdP / params.c_core_j_per_k
    # ḣ + α h ≥ 0 with ḣ = -Ṫ(P), Ṫ ≈ Ṫ_lin + g_T (P - P_lin)
    if g_t > 1e-12:
        p_thermal = p_lin + (params.alpha_thermal * h_t - tdot) / g_t
    else:
        p_thermal = params.p_max_w if (params.alpha_thermal * h_t - tdot) >= 0.0 else 0.0
    if p_thermal < 0.0:
        p_thermal = 0.0

    h_e = params.e_quota_lap_j - state.e_used_lap_j
    if h_e < 0.0:
        h_e = 0.0
    p_energy = params.alpha_energy * h_e
    p_soc = params.alpha_soc * max(0.0, state.soc) * params.capacity_j

    meta = {
        "h_thermal_c": h_t,
        "h_energy_j": h_e,
        "h_soc_j": max(0.0, state.soc) * params.capacity_j,
        "t_dot_c_per_s": tdot,
        "g_thermal": g_t,
        "q_gen_w": q,
        "i_a": i,
        "dq_dp": dqdP,
        "p_linearization_w": p_lin,
    }
    return p_thermal, p_energy, p_soc, meta


def kkt_power(state: BatteryState, p_req: float, params: PackParams) -> float:
    """Closed-form 1-D QP solution: project P_req onto the CBF box."""
    p_lin = min(max(p_req, 0.0), params.p_max_w)
    p_th, p_en, p_soc, _ = _bounds(state, p_lin, params)
    upper = min(params.p_max_w, p_th, p_en, p_soc)
    if upper < 0.0:
        upper = 0.0
    return min(max(p_req, 0.0), upper)


def filter_power(state: BatteryState, p_req: float, params: PackParams) -> CBFResult:
    p_req = float(p_req)
    p_lin = min(max(p_req, 0.0), params.p_max_w)
    p_th, p_en, p_soc, meta = _bounds(state, p_lin, params)

    # min ½(P − P_req)²  s.t.  0 ≤ P ≤ P_max,  P ≤ P_thermal,  P ≤ P_energy,  P ≤ P_soc
    p_mat = sparse.csc_matrix([[1.0]])
    q = np.array([-p_req])
    a = sparse.csc_matrix([[1.0], [1.0], [1.0], [1.0]])
    lo = np.array([0.0, -np.inf, -np.inf, -np.inf])
    up = np.array([params.p_max_w, p_th, p_en, p_soc])

    t0 = perf_counter()
    solver = osqp.OSQP()
    solver.setup(
        p_mat,
        q,
        a,
        lo,
        up,
        verbose=False,
        polishing=False,
        eps_abs=1e-6,
        eps_rel=1e-6,
        max_iter=2000,
    )
    result = solver.solve()
    solve_ms = (perf_counter() - t0) * 1000.0

    status = result.info.status
    if status not in ("solved", "solved inaccurate"):
        p_safe = kkt_power(state, p_req, params)
        status = f"fallback_kkt ({status})"
    else:
        p_safe = float(result.x[0])

    if p_safe < 0.0:
        p_safe = 0.0
    if p_safe > params.p_max_w:
        p_safe = params.p_max_w

    duals = result.y if result.y is not None else np.zeros(4)
    thermal_dual = float(duals[1])
    energy_dual = float(duals[2])

    slack_t = p_th - p_safe
    slack_e = p_en - p_safe
    slack_s = p_soc - p_safe
    active: list[str] = []
    slack_tol = 250.0
    if slack_t <= slack_tol:
        active.append("thermal")
    if slack_e <= slack_tol:
        active.append("energy")
    if slack_s <= slack_tol:
        active.append("soc")
    if abs(p_safe - params.p_max_w) <= slack_tol:
        active.append("p_max")

    clipped = p_safe < p_req - 200.0

    qp = {
        "objective": "minimize 0.5 * (P - P_req)^2",
        "P": [[1.0]],
        "q": [-p_req],
        "A": [[1.0], [1.0], [1.0], [1.0]],
        "l": [0.0, None, None, None],
        "u": [params.p_max_w, p_th, p_en, p_soc],
        "constraints": [
            "0 <= P <= P_max",
            "P <= P_thermal  (ḣ_T + α_T h_T >= 0, linearized)",
            "P <= P_energy   (ḣ_E + α_E h_E >= 0)",
            "P <= P_soc      (remaining pack energy)",
        ],
        "soc_bound_w": p_soc,
        **meta,
    }

    return CBFResult(
        p_safe=p_safe,
        p_requested=p_req,
        clipped=clipped,
        solve_time_ms=solve_ms,
        solver_status="solved" if status == "solved" else status,
        thermal_dual=thermal_dual,
        energy_dual=energy_dual,
        thermal_bound_w=p_th,
        energy_bound_w=p_en,
        thermal_slack_w=slack_t,
        energy_slack_w=slack_e,
        active_constraints=active,
        iterations=int(result.info.iter),
        qp=qp,
    )
