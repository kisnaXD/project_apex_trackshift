import numpy as np

class PirelliTire:
    """
    Pirelli F1 18-inch Slick Thermal & Degradation Engine:
    - Compounds: C1 (Hard), C2 (Medium), C3 (Soft)
    - Surface (T_surf) vs Carcass (T_carc) dual-layer thermal inertia
    - Dirty air wake heating penalty (downforce loss -> lateral micro-slip)
    """
    COMPOUND_PROPERTIES = {
        "C1_HARD":   {"opt_temp": 120.0, "window": 15.0, "base_grip": 1.45, "wear_rate": 0.0008},
        "C2_MEDIUM": {"opt_temp": 110.0, "window": 12.0, "base_grip": 1.55, "wear_rate": 0.0014},
        "C3_SOFT":   {"opt_temp": 98.0,  "window": 10.0, "base_grip": 1.68, "wear_rate": 0.0028}
    }

    def __init__(self, compound="C2_MEDIUM"):
        self.props = self.COMPOUND_PROPERTIES[compound]
        self.compound_name = compound
        self.T_surf = 95.0     # Tread surface temp (°C)
        self.T_carc = 90.0     # Structural carcass temp (°C)
        self.wear_pct = 0.0
        self._current_grip = self.props["base_grip"]    # 0% new, 100% worn down to canvas

    def step(self, lateral_g, speed_ms, in_wake, gap_m, dt=0.05):
        # 1. Base Frictional Heating
        frictional_heat = 0.08 * (lateral_g**2) * (speed_ms / 30.0)

        # 2. Dirty Air Washout Penalty
        # Loss of aerodynamic front downforce increases slip scrub
        wake_heat_penalty = 0.0
        if in_wake and gap_m < 35.0:
            proximity_factor = max(0.0, 1.0 - (gap_m / 35.0))
            wake_heat_penalty = 2.4 * proximity_factor * (lateral_g / 2.0)

        # 3. Cooling & Thermal Conduction
        ambient_air = 28.0
        h_conv = 0.045 * (1.0 + 0.02 * speed_ms)
        q_cooling = h_conv * (self.T_surf - ambient_air)
        q_conduct = 0.025 * (self.T_surf - self.T_carc)

        self.T_surf += (frictional_heat + wake_heat_penalty - q_cooling - q_conduct) * dt
        self.T_carc += (q_conduct - 0.015 * (self.T_carc - ambient_air)) * dt

        # 4. Grip Factor via Parabolic Thermal Peak
        temp_delta = abs(self.T_surf - self.props["opt_temp"])
        thermal_grip_mult = max(0.65, 1.0 - (temp_delta / self.props["window"])**2 * 0.25)

        # 5. Wear Degradation Rate
        slip_severity = 1.0 + (1.5 if temp_delta > self.props["window"] else 0.0) + (0.8 if in_wake else 0.0)
        self.wear_pct = min(100.0, self.wear_pct + self.props["wear_rate"] * slip_severity * dt * 100.0)
        wear_mult = max(0.70, 1.0 - (self.wear_pct / 100.0)**1.8 * 0.3)

        effective_grip = self.props["base_grip"] * thermal_grip_mult * wear_mult
        self._current_grip = float(effective_grip)

        return {
            "effective_grip": effective_grip,
            "T_surf": self.T_surf,
            "T_carc": self.T_carc,
            "wear_pct": self.wear_pct,
            "is_overheating": self.T_surf > (self.props["opt_temp"] + self.props["window"])
        }
    @property
    def current_grip(self) -> float:
        return getattr(self, "_current_grip", self.props["base_grip"])
