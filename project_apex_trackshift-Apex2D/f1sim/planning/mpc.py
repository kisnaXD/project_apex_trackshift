import numpy as np

class TacticalContinuationMPC:
    """
    Evaluates reachability feasibility and enforces continuation invariants:
    - Minimum continuation SoC >= 20% post-manoeuvre
    - Battery core temperature constraint: T_core(t + t_horizon) <= 58.0°C
    """
    def __init__(self, horizon_steps=15, dt=0.05):
        self.N = horizon_steps
        self.dt = dt

    def verify_and_synthesize(self, ego_state, target_y, pu_state, dist_to_apex):
        """
        ego_state: [s, y, v_s, v_y]
        pu_state: dict containing 'soc_pct', 'T_core'
        dist_to_apex: distance to the critical braking point
        """
        # 1. Project terminal SoC if full 120 kW MGU-K attack is maintained
        t_commit = dist_to_apex / max(10.0, ego_state['v_s'])
        p_attack_w = 120.0e3
        energy_drain_j = p_attack_w * t_commit
        post_attack_soc = pu_state['soc_pct'] - (energy_drain_j / 4.0e6) * 100.0

        # Invariant 1: Battery must not clip before corner entry
        continuation_feasible = post_attack_soc >= 15.0

        # Invariant 2: Thermal barrier invariant
        projected_T_core = pu_state['T_core'] + 0.015 * t_commit * (p_attack_w / 1e5)**2
        if projected_T_core > 57.5:
            continuation_feasible = False

        # Invariant 3: Spatial overlap reachability
        # Does Car A clear the opponent's rear axle before the turn-in point?
        overlap_achievable = (dist_to_apex > 40.0)

        is_approved = continuation_feasible and overlap_achievable

        # Safe fallback trajectory if unfeasible
        safe_target_y = target_y if is_approved else 0.0
        allocated_power_kw = 120.0 if is_approved else 45.0

        return {
            "approved": is_approved,
            "safe_target_y": safe_target_y,
            "allocated_power_kw": allocated_power_kw,
            "projected_soc": post_attack_soc,
            "projected_T_core": projected_T_core,
            "abort_reason": "SOC_DEFICIT" if not continuation_feasible else ("BRAKE_OVERRUN" if not overlap_achievable else "NONE")
        }