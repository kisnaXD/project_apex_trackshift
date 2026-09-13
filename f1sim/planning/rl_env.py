# f1sim/planning/rl_env.py
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from f1sim.engine.scenario import RaceScenario

class F1TrackShiftEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.scenario = RaceScenario()
        
        # Action: [MGU-K Power scalar (-1 to 1), Active Aero switch (-1 to 1)]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # State: [v_s, gap_m, delta_v, soc_pct, T_core, lap_deploy_mj, dist_to_brake, is_leading]
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float32)
        self.max_steps = 1500
        self.current_step = 0

    def _get_obs(self, step_info):
        ego = step_info["ego"]
        car = step_info["car_tel"]
        gap = step_info["gap_m"]
        dist_b = step_info["dist_to_brake"]
        opp_v = step_info["estimates"][step_info["target_code"]].get("v_s", 70.0)
        
        obs = np.array([
            ego["v_s"] / 90.0,
            np.clip(gap / 60.0, -1.0, 1.0),
            (ego["v_s"] - opp_v) / 25.0,
            car["soc_pct"] / 100.0,
            car["T_battery_core"] / 60.0,
            car["lap_deploy_mj"] / 8.5,
            min(1.0, dist_b / 200.0),
            1.0 if step_info["is_leading"] else 0.0
        ], dtype=np.float32)
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.scenario = RaceScenario()
        self.current_step = 0
        init_step = self.scenario.step(rl_action=None)
        return self._get_obs(init_step), {}

    def step(self, action):
        self.current_step += 1
        
        power_kw = float(action[0]) * 350.0
        active_aero = bool(action[1] > 0.0)

        step_info = self.scenario.step(rl_action={"power_kw": power_kw, "active_aero": active_aero})
        obs = self._get_obs(step_info)

        # Reward formulation
        v_progression = step_info["ego"]["v_s"] * 0.05
        reward = v_progression

        # Reward successful overtakes
        if step_info["is_leading"]:
            reward += 2.0

        # Penalize CBF safety violations (thermal breach or 8.5 MJ regulatory overrun)
        if step_info["cbf_intervention"]:
            reward -= 1.5

        # Penalize battery thermal runaways above 57 deg C
        if step_info["car_tel"]["T_battery_core"] > 57.0:
            reward -= 2.0

        terminated = self.current_step >= self.max_steps
        truncated = False

        return obs, float(reward), terminated, truncated, step_info