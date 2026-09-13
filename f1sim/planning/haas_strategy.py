from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from f1sim.data.schema import TelemetryFrame


@dataclass(frozen=True)
class F12026Constraints:
    mgu_k_max_kw: float = 350.0
    race_boost_cap_kw: float = 150.0
    recovery_limit_mj_per_lap: float = 8.5
    override_extra_energy_mj: float = 0.5
    override_speed_limit_kmh: float = 337.0
    leader_deploy_taper_start_kmh: float = 290.0
    deploy_zero_speed_kmh: float = 355.0
    one_second_overtake_gap_s: float = 1.0
    minimum_parallel_clearance_m: float = 3.5
    pitwall_prediction_horizon_s: float = 30.0


@dataclass
class PublicOpponentState:
    driver_code: str
    team_name: str
    team_color: str
    lap_number: int
    gap_to_ego_m: float
    gap_to_ego_s: float
    x_world: float
    y_world: float
    compound: str
    laps_old: int
    active: bool
    estimated_speed_kmh: Optional[float] = None


@dataclass
class StrategyRecommendation:
    action: str
    title: str
    target_code: str
    confidence_pct: float
    safety_margin_m: float
    overtake_confidence_pct: float
    deploy_kw: float
    harvest_kw: float
    expected_gain_m: float
    reason: str
    constraints_used: List[str] = field(default_factory=list)


class HaasPitwallStrategyPlanner:
    """
    Strategy planner that deliberately uses only Haas-visible race information.

    The app may ingest FastF1 frames, but this planner strips opponent channels
    down to public/timing-style data: identity, team, position estimate, lap,
    gap, compound and tyre age. It does not use opponent throttle, brake, RPM,
    gear, ERS state or private powertrain channels.
    """

    def __init__(self, constraints: Optional[F12026Constraints] = None):
        self.constraints = constraints or F12026Constraints()
        self._last_seen: Dict[str, tuple[float, float, float]] = {}
        self._last_public: Dict[str, PublicOpponentState] = {}
        self.recommendation_history: List[StrategyRecommendation] = []

    def observe_public_opponents(self, frame: TelemetryFrame) -> Dict[str, PublicOpponentState]:
        public: Dict[str, PublicOpponentState] = {}
        now = float(frame.replay_time_s)
        ego_speed_ms = max(1.0, frame.ego.kinematics.speed_kmh / 3.6)

        for code, opponent in frame.opponents.items():
            x = opponent.kinematics.x_world
            y = opponent.kinematics.y_world
            previous = self._last_seen.get(code)
            estimated_speed = None
            if previous is not None:
                prev_t, prev_x, prev_y = previous
                dt = max(1e-3, now - prev_t)
                dist_m = float(np.hypot(x - prev_x, y - prev_y))
                estimated_speed = float(np.clip((dist_m / dt) * 3.6, 0.0, 370.0))

            self._last_seen[code] = (now, x, y)

            gap_m = float(opponent.gap_to_ego_m)
            public_state = PublicOpponentState(
                driver_code=code,
                team_name=opponent.team_name,
                team_color=opponent.team_color,
                lap_number=int(max(1, frame.ego.lap_number + int(np.floor(gap_m / 5891.0)))),
                gap_to_ego_m=gap_m,
                gap_to_ego_s=float(gap_m / ego_speed_ms),
                x_world=x,
                y_world=y,
                compound=opponent.compound,
                laps_old=opponent.laps_old,
                active=opponent.active,
                estimated_speed_kmh=estimated_speed,
            )
            public[code] = public_state

        self._last_public = public
        return public

    def recommend(self, frame: TelemetryFrame) -> StrategyRecommendation:
        public = self.observe_public_opponents(frame)
        ego = frame.ego
        constraints = self.constraints

        ahead = [op for op in public.values() if op.active and op.gap_to_ego_m > 0.0]
        if ahead:
            target = min(ahead, key=lambda op: op.gap_to_ego_m)
        elif public:
            target = min(public.values(), key=lambda op: abs(op.gap_to_ego_m))
        else:
            target = None

        if target is None:
            rec = StrategyRecommendation(
                action="DEFEND",
                title="DEFEND CLEAN AIR",
                target_code="NONE",
                confidence_pct=92.0,
                safety_margin_m=99.0,
                overtake_confidence_pct=0.0,
                deploy_kw=80.0,
                harvest_kw=60.0,
                expected_gain_m=0.0,
                reason="No active car is close enough to affect Bearman's current phase.",
                constraints_used=["Haas-visible telemetry only"],
            )
            self.recommendation_history.append(rec)
            return rec

        ego_speed = ego.kinematics.speed_kmh
        ego_speed_ms = max(1.0, ego_speed / 3.6)
        target_gap_s = target.gap_to_ego_m / ego_speed_ms
        soc = ego.ers.soc_pct
        tyre_wear = ego.tires.wear_pct
        safety_margin = max(0.0, abs(target.gap_to_ego_m) - constraints.minimum_parallel_clearance_m)
        target_est_speed = target.estimated_speed_kmh if target.estimated_speed_kmh is not None else ego_speed
        closing_kmh = ego_speed - target_est_speed

        gap_score = np.clip(1.0 - (max(0.0, target.gap_to_ego_m) / 55.0), 0.0, 1.0)
        speed_score = np.clip(0.45 + closing_kmh / 45.0, 0.0, 1.0)
        energy_score = np.clip((soc - 22.0) / 55.0, 0.0, 1.0)
        tyre_score = np.clip(1.0 - tyre_wear / 85.0, 0.0, 1.0)
        override_eligible = 0.0 < target_gap_s <= constraints.one_second_overtake_gap_s
        override_score = 1.0 if override_eligible and ego_speed <= constraints.override_speed_limit_kmh else 0.35
        overtake_conf = float(
            np.clip(
                (0.28 * gap_score + 0.22 * speed_score + 0.23 * energy_score + 0.17 * tyre_score + 0.10 * override_score)
                * 100.0,
                2.0,
                98.0,
            )
        )

        if safety_margin < 1.0 and closing_kmh > 10.0:
            action = "SLOW"
            title = "SLOW / PROTECT FRONT WING"
            deploy_kw = -80.0
            harvest_kw = 180.0
            reason = "Closing rate is too high for the visible safety margin."
        elif soc < 30.0:
            action = "HARVEST"
            title = "HARVEST BEFORE ATTACK"
            deploy_kw = 0.0
            harvest_kw = min(350.0, constraints.recovery_limit_mj_per_lap * 32.0)
            reason = "Battery state is below the attack threshold; recharge before committing."
        elif override_eligible and overtake_conf >= 58.0:
            action = "ATTACK"
            title = "ATTACK WITH OVERRIDE"
            deploy_kw = constraints.mgu_k_max_kw
            harvest_kw = 0.0
            reason = "Within one second with enough energy and tyre margin for a controlled overtake."
        elif target.gap_to_ego_m < 45.0:
            action = "HOLD"
            title = "HOLD PRESSURE"
            deploy_kw = min(250.0, constraints.mgu_k_max_kw)
            harvest_kw = 35.0
            reason = "Stay in the window and force the next opportunity without overspending energy."
        else:
            action = "BUILD"
            title = "BUILD DELTA"
            deploy_kw = 180.0
            harvest_kw = 40.0
            reason = "Target is outside the immediate attack range; build pace while preserving battery."

        expected_gain_m = self._estimate_gain(action, ego_speed, target_est_speed, soc)
        confidence = float(np.clip(0.55 * overtake_conf + 0.45 * (100.0 - min(80.0, tyre_wear)), 0.0, 99.0))

        rec = StrategyRecommendation(
            action=action,
            title=title,
            target_code=target.driver_code,
            confidence_pct=confidence,
            safety_margin_m=safety_margin,
            overtake_confidence_pct=overtake_conf,
            deploy_kw=deploy_kw,
            harvest_kw=harvest_kw,
            expected_gain_m=expected_gain_m,
            reason=reason,
            constraints_used=[
                "Opponent throttle/brake/RPM/gear/ERS redacted",
                "One-second overtake/override window",
                "350 kW MGU-K maximum",
                "+150 kW race boost cap",
                "8.5 MJ/lap recovery envelope",
                "3.5 m minimum tactical safety margin",
            ],
        )
        self.recommendation_history.append(rec)
        return rec

    def _estimate_gain(self, action: str, ego_speed_kmh: float, target_speed_kmh: float, soc_pct: float) -> float:
        base_delta_ms = (ego_speed_kmh - target_speed_kmh) / 3.6
        if action == "ATTACK":
            return float(np.clip((base_delta_ms + 2.8) * 30.0, -15.0, 95.0))
        if action == "HOLD":
            return float(np.clip((base_delta_ms + 0.8) * 30.0, -25.0, 45.0))
        if action == "HARVEST":
            return float(np.clip((base_delta_ms - 1.4) * 30.0, -60.0, 20.0))
        if action == "SLOW":
            return float(np.clip((base_delta_ms - 3.0) * 30.0, -85.0, 5.0))
        return float(np.clip((base_delta_ms + min(1.2, soc_pct / 80.0)) * 30.0, -30.0, 60.0))

    def public_state_for_code(self, code: str) -> Optional[PublicOpponentState]:
        return self._last_public.get(code)
