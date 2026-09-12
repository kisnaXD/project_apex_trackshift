"""Small causal Layer 1 attack/follow decision for the quick demo."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StrategyConfig:
    energy_price_s_per_mj: float = 0.35
    hard_reserve_j: float = 400_000.0
    deploy_budget_j: float = 300_000.0
    minimum_gap_m: float = 8.0
    maximum_pass_time_s: float = 12.0


@dataclass(frozen=True)
class StrategyDecision:
    action: str
    reason: str
    estimated_pass_time_s: float | None
    estimated_energy_cost_j: float
    deploy_budget_j: float
    feasible: bool


class StrategyLayer:
    """Compare follow and attack from observations available at this instant.

    The layer intentionally has no reference trajectory or future opponent
    data.  A pass is feasible only when the observed closing speed, energy
    reserve and per-opportunity budget all support it.
    """

    def __init__(self, config: StrategyConfig = StrategyConfig()):
        self.config = config

    def decide(self, *, gap_m: float | None, ego_speed_mps: float,
               opponent_speed_mps: float | None, stored_energy_j: float | None,
               dt_s: float = 0.1) -> StrategyDecision:
        if gap_m is None or opponent_speed_mps is None or gap_m <= self.config.minimum_gap_m:
            return StrategyDecision("follow", "opponent_unavailable_or_far", None, 0.0, 0.0, True)
        closing = max(0.0, float(ego_speed_mps) - float(opponent_speed_mps))
        if closing <= 0.1:
            return StrategyDecision("follow", "no_observed_closing_speed", None, 0.0, 0.0, True)
        pass_time = max(0.0, gap_m / closing)
        # A modest 1.5 m/s2 attack increment is enough to expose the native
        # MGU-K coupling in the demo and keeps the estimate conservative.
        energy = max(0.0, 788.0 * 1.5 * min(pass_time, self.config.maximum_pass_time_s) * 0.2)
        available = None if stored_energy_j is None else stored_energy_j - self.config.hard_reserve_j
        feasible = (pass_time <= self.config.maximum_pass_time_s and
                    energy <= self.config.deploy_budget_j and
                    (available is None or energy <= max(0.0, available)))
        if not feasible:
            return StrategyDecision("follow", "attack_budget_or_time_infeasible", pass_time, energy, 0.0, False)
        budget = max(0.0, self.config.deploy_budget_j - energy)
        return StrategyDecision("attack", "observed_gap_and_budget_feasible", pass_time, energy, budget, True)
