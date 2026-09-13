class SimulationClock:
    """
    Deterministic multi-rate simulation clock.
    - Physics / Battery / Tires: 100 Hz (dt = 0.01s)
    - Perception / Tracking: 20 Hz (every 5 steps)
    - Tactical Planning: 10 Hz (every 10 steps)
    - Global Strategy: 1 Hz (every 100 steps)
    """
    def __init__(self, dt_physics: float = 0.01):
        self.dt = dt_physics
        self.sim_time = 0.0
        self.step_idx = 0

    def tick(self):
        self.sim_time = round(self.sim_time + self.dt, 6)
        self.step_idx += 1
        
        is_20hz = (self.step_idx % 5 == 0)
        is_10hz = (self.step_idx % 10 == 0)
        is_1hz  = (self.step_idx % 100 == 0)

        return {
            "time": self.sim_time,
            "step": self.step_idx,
            "run_perception": is_20hz,
            "run_tactical": is_10hz,
            "run_strategy": is_1hz
        }

    def reset(self):
        self.sim_time = 0.0
        self.step_idx = 0