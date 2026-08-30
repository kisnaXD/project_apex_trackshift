"""One shared stint: strategy → R_OT → power request → CBF-QP → plant."""

from __future__ import annotations

from dataclasses import asdict

from apex.cbf_qp import filter_power
from apex.params import PackParams
from apex.physics import BatteryState, step
from apex.risk_reward import compute_rot
from apex.strategy import shadow_prices

HIST_CAP = 180


def _initial_state() -> BatteryState:
    return BatteryState(
        soc=0.82,
        t_core_c=38.0,
        t_surf_c=36.5,
        e_used_lap_j=0.0,
        time_s=0.0,
        lap=1,
        tire_wear=0.08,
        gap_m=10.0,
    )


class StintSim:
    def __init__(self) -> None:
        self.params = PackParams()
        self.opponent_aggression = 0.40
        self.push_bias = 0.45
        self.reset()

    def reset(self) -> dict:
        self.state = _initial_state()
        preview = self._evaluate(self.state)
        self.history = {
            "t_s": [0.0],
            "soc": [self.state.soc],
            "t_core_c": [self.state.t_core_c],
            "t_surf_c": [self.state.t_surf_c],
            "r_ot": [preview["rot"]["r_ot"]],
            "p_req": [preview["cbf"]["p_requested"]],
            "p_safe": [preview["cbf"]["p_safe"]],
            "mode": [preview["rot"]["mode"]],
            "gap_m": [self.state.gap_m],
        }
        return self.snapshot()

    def set_inputs(self, opponent_aggression: float, push_bias: float) -> None:
        self.opponent_aggression = min(1.0, max(0.0, float(opponent_aggression)))
        self.push_bias = min(1.0, max(0.0, float(push_bias)))

    def step(self, dt: float) -> dict:
        remaining = float(dt)
        while remaining > 1e-12:
            if remaining >= 4.0:
                tick = 2.0
            elif remaining > 1.0:
                tick = 1.0
            else:
                tick = remaining
            self._tick(tick)
            remaining -= tick
        return self.snapshot()

    def snapshot(self) -> dict:
        ev = self._evaluate(self.state)
        return {
            "state": asdict(self.state),
            "rot": ev["rot"],
            "cbf": ev["cbf"],
            "strategy": ev["strategy"],
            "inputs": {
                "opponent_aggression": self.opponent_aggression,
                "push_bias": self.push_bias,
            },
            "history": self.history,
            "limits": {
                "t_core_limit_c": self.params.t_core_limit_c,
                "e_quota_lap_j": self.params.e_quota_lap_j,
                "p_max_w": self.params.p_max_w,
            },
            "prototype": True,
        }

    def _requested_power(self, mode: str) -> float:
        base = {
            "harvest": self.params.p_harvest_w,
            "nominal": self.params.p_nominal_w,
            "attack": self.params.p_attack_w,
        }[mode]
        return base + self.push_bias * max(0.0, self.params.p_attack_w - base)

    def _evaluate(self, state: BatteryState) -> dict:
        prices = shadow_prices(state, self.params)
        rot = compute_rot(state, self.opponent_aggression, self.params)
        p_req = self._requested_power(rot.mode)
        cbf = filter_power(state, p_req, self.params)
        return {
            "rot": {
                "r_ot": rot.r_ot,
                "mode": rot.mode,
                "clean_air_benefit": rot.clean_air_benefit,
                "energy_cost": rot.energy_cost,
                "thermal_cost": rot.thermal_cost,
                "tire_cost": rot.tire_cost,
                "collision_cost": rot.collision_cost,
                "lambda_e": rot.lambda_e,
                "lambda_t": rot.lambda_t,
            },
            "cbf": {
                "p_safe": cbf.p_safe,
                "p_requested": cbf.p_requested,
                "clipped": cbf.clipped,
                "solve_time_ms": cbf.solve_time_ms,
                "solver_status": cbf.solver_status,
                "thermal_dual": cbf.thermal_dual,
                "energy_dual": cbf.energy_dual,
                "thermal_bound_w": cbf.thermal_bound_w,
                "energy_bound_w": cbf.energy_bound_w,
                "thermal_slack_w": cbf.thermal_slack_w,
                "energy_slack_w": cbf.energy_slack_w,
                "active_constraints": cbf.active_constraints,
                "iterations": cbf.iterations,
                "qp": cbf.qp,
            },
            "strategy": {
                "lambda_e": prices.lambda_e,
                "lambda_t": prices.lambda_t,
                "progress": prices.progress,
                "soc_target": prices.soc_target,
                "note": prices.note,
            },
        }

    def _tick(self, dt: float) -> None:
        ev = self._evaluate(self.state)
        p_safe = ev["cbf"]["p_safe"]
        nxt = step(self.state, p_safe, dt, self.params)
        nxt.gap_m = _update_gap(self.state.gap_m, p_safe, self.opponent_aggression, dt, self.params)
        nxt.tire_wear = _update_tires(self.state.tire_wear, ev["rot"]["mode"], dt)
        nxt = _roll_lap(self.state, nxt, self.params)
        self.state = nxt
        self.history["t_s"].append(nxt.time_s)
        self.history["soc"].append(nxt.soc)
        self.history["t_core_c"].append(nxt.t_core_c)
        self.history["t_surf_c"].append(nxt.t_surf_c)
        self.history["r_ot"].append(ev["rot"]["r_ot"])
        self.history["p_req"].append(ev["cbf"]["p_requested"])
        self.history["p_safe"].append(p_safe)
        self.history["mode"].append(ev["rot"]["mode"])
        self.history["gap_m"].append(nxt.gap_m)
        overflow = len(self.history["t_s"]) - HIST_CAP
        if overflow > 0:
            for series in self.history.values():
                del series[:overflow]


def _update_gap(gap_m: float, p_w: float, aggression: float, dt: float, params: PackParams) -> float:
    p_opp = params.p_nominal_w * (0.86 + 0.28 * aggression)
    closing = 0.018 * (p_w - p_opp) / params.p_nominal_w
    nxt = gap_m - closing * dt * 18.0
    return min(40.0, max(1.8, nxt))


def _update_tires(wear: float, mode: str, dt: float) -> float:
    extra = 0.0022 if mode == "attack" else 0.0007
    return min(1.0, wear + extra * dt)


def _roll_lap(prev: BatteryState, nxt: BatteryState, params: PackParams) -> BatteryState:
    boundary = prev.lap * params.lap_time_s
    if nxt.time_s >= boundary:
        nxt.lap = prev.lap + 1
        nxt.e_used_lap_j = 0.0
    if nxt.lap > params.n_laps:
        nxt.lap = params.n_laps
    return nxt
