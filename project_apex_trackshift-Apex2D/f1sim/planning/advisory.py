import numpy as np

class F1OvertakeAdvisor:
    """
    Real-time Decision Helper for Pitwall Race Engineers:
    Calculates out-braking limits, energy balance deltas, and DRS opportunities.
    """
    def evaluate(self, ego_s, ego_v, opp_s, opp_v, gap_m, drs_active, 
                 ego_soc, opp_clipping, tire_grip, next_brake_zone, dist_to_brake):
        
        rel_speed_kmh = (ego_v - opp_v) * 3.6
        gap_sec = gap_m / max(10.0, opp_v)

        # 1. Delta-Braking Distance (Kinematic stopping calculation)
        # s_brake = v^2 / (2 * a_decel)
        a_decel = 5.2 * 9.81 * (tire_grip / 1.5)  # F1 brakes pull ~5.2G at high speed
        v_apex = next_brake_zone['v_apex'] if next_brake_zone else 30.0

        ego_stopping_dist = (ego_v**2 - v_apex**2) / (2 * a_decel)
        opp_stopping_dist = (opp_v**2 - v_apex**2) / (2 * a_decel)
        brake_overlap_margin = dist_to_brake - ego_stopping_dist

        # 2. Probability of Completing Pass
        score = 0.0
        reasons = []

        if drs_active:
            score += 35.0
            reasons.append("DRS ENABLED (+18 km/h)")
        if opp_clipping:
            score += 30.0
            reasons.append("RIVAL ERS CLIPPING (-120 kW)")
        if ego_soc > 40.0:
            score += 15.0
            reasons.append("STRAT 2 READY")
        if rel_speed_kmh > 8.0:
            score += 20.0
            reasons.append(f"CLOSING AT +{rel_speed_kmh:.1f} KM/H")

        # Penalties
        if dist_to_brake < 80.0 and gap_m > 12.0:
            score -= 40.0
            reasons.append("TOO LATE BEFORE BRAKING ZONE")
        if tire_grip < 1.30:
            score -= 25.0
            reasons.append("TIRE THERMALS COMPROMISED")

        success_prob = float(np.clip(score, 5.0, 98.0))

        # 3. Decision Vector & Radio Call
        if success_prob >= 70.0 and dist_to_brake > 90.0:
            tactical_call = "COMMIT OVERTAKE"
            maneuver_type = "INSIDE LINE DIVE" if gap_m < 15.0 else "DRS CRUISE-BY"
            radio_msg = f"Strat 2 available. Deploy Overtake Button. Commit {maneuver_type} into {next_brake_zone['name']}."
            action_code = 3 # ATTACK
        elif success_prob >= 45.0:
            tactical_call = "PREPARE DRAFT"
            maneuver_type = "SWITCHBACK EXIT"
            radio_msg = f"Hold wake. Force defensive line, focus on corner exit drive for {next_brake_zone['name']}."
            action_code = 2 # NOMINAL
        else:
            tactical_call = "HARVEST & COOL"
            maneuver_type = "CLEAN AIR LIFT & COAST"
            radio_msg = "Tires critical in dirty air. Pull out of wake to cool front axle. Recharge ERS."
            action_code = 1 # PRE-COOL

        return {
            "success_prob": success_prob,
            "tactical_call": tactical_call,
            "maneuver_type": maneuver_type,
            "radio_msg": radio_msg,
            "action_code": action_code,
            "brake_margin_m": brake_overlap_margin,
            "reasons": reasons
        }