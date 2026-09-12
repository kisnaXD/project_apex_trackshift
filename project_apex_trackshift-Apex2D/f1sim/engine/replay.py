import logging
import numpy as np
from typing import Dict, Any

logger = logging.getLogger(__name__)

DRIVER_MAP = {
    "LEC": "16", "VER": "1", "HAM": "44", "NOR": "4",
    "RUS": "63", "ALO": "14", "SAI": "55", "PIA": "81",
    "GAS": "10", "ALB": "23"
}

class FastF1OpponentReplayer:
    def __init__(
        self,
        year: int = 2026,
        circuit: str = "Silverstone",
        dt: float = 0.01,
        track_length: float = 5891.0,
        session_type: str = "R",
        cache_dir: str = "/tmp/fastf1_cache"
    ):
        self.year = year
        self.circuit = circuit
        self.dt = float(dt)
        self.track_length = float(track_length)
        self.session_type = session_type
        self.cache_dir = cache_dir

        self.drivers = ["LEC", "VER", "NOR", "HAM", "RUS", "ALO", "SAI", "PIA"]
        self.driver_data: Dict[str, Dict[str, Any]] = {}
        self.current_time = 0.0

        # Initialize continuous multi-lap track states for all competitors
        self.driver_states: Dict[str, Dict[str, float]] = {}
        for idx, code in enumerate(self.drivers):
            grid_s = float((160.0 - idx * 18.0) % self.track_length)
            lane_y = 0.0 if code == "LEC" else (0.9 if idx % 2 == 1 else -0.9)
            self.driver_states[code] = {
                "s": grid_s,
                "s_total": grid_s,
                "y": lane_y,
                "base_speed": 66.0 - (idx * 0.25)
            }

    def _get_driver_instant_velocity(self, code: str, s_pos: float, t_now: float) -> float:
        state = self.driver_states[code]
        base_v = state["base_speed"]
        
        # Continuous Silverstone track profile across all laps
        norm_s = (s_pos / self.track_length) * 2.0 * np.pi
        v_track_mod = (
            np.sin(norm_s) * 12.0 +
            np.cos(2.0 * norm_s) * 16.0 +
            np.sin(4.0 * norm_s) * 8.0
        )
        return float(np.clip(base_v + v_track_mod, 28.0, 91.5))

    def step(self, ego_s: float = 0.0, ego_v_ms: float = 0.0) -> Dict[str, Dict[str, Any]]:
        self.current_time += self.dt
        snapshot: Dict[str, Dict[str, Any]] = {}

        for code, state in self.driver_states.items():
            v_now = self._get_driver_instant_velocity(code, state["s"], self.current_time)
            ds = v_now * self.dt
            state["s_total"] += ds
            state["s"] = float(state["s_total"] % self.track_length)
            lap_count = int(state["s_total"] // self.track_length) + 1

            snapshot[code] = {
                "s": state["s"],
                "s_total": state["s_total"],
                "y": state["y"],
                "v_s": v_now,
                "clipping": bool(v_now > 88.0),
                "lap": lap_count,
                "compound": "MEDIUM"
            }

        return snapshot

    def reset(self):
        self.current_time = 0.0
        for idx, (code, state) in enumerate(self.driver_states.items()):
            grid_s = float((160.0 - idx * 18.0) % self.track_length)
            state["s"] = grid_s
            state["s_total"] = grid_s
