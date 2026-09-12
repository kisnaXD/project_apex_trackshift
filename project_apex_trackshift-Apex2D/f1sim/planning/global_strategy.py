import numpy as np

class StrategicEnergyDP:
    """
    Solves for global lap-time optimal SoC schedule via backward Dynamic Programming:
    V(k, soc_i) = min_{u \in U} [ dt(k, soc_i, u) + V(k+1, soc_{i+1}) ]
    """
    def __init__(self, n_nodes=40, node_ds=50.0):
        self.N = n_nodes
        self.ds = node_ds
        self.track_length = self.N * self.ds  # 2000 m circuit

        # Circuit Geometry & Curvature Profile
        # Node 0-10: Sector 1 (Wellington Straight, DRS Zone 1)
        # Node 10-25: Sector 2 (Twisty Technical Complex / High Aero Wake Penalty)
        # Node 25-35: Sector 3 (Hangar Straight, DRS Zone 2)
        # Node 35-40: Sector 4 (Final Chicane & Braking)
        self.curvature = np.zeros(self.N)
        self.curvature[10:25] = 0.028  # Radius ~35m (max speed capped by tire friction)
        self.curvature[35:] = 0.040    # Radius ~25m

        # Discretized State Space
        # Battery SoC: 15% to 95% in 25 discrete bins
        self.soc_grid = np.linspace(0.15, 0.95, 25)
        
        # Discretized Action Space: MGU-K Power in kW (-120 kW Regen to +120 kW Deploy)
        self.actions_kw = np.array([-120.0, -80.0, -40.0, 0.0, 40.0, 80.0, 120.0])

        # Physical Constants
        self.mass = 830.0
        self.capacity_j = 4.0e6
        self.mu_tire = 1.55

    def step_dynamics(self, k, soc, p_kw, in_traffic=False):
        """Calculates transit time dt and next SoC for transition (k -> k+1)."""
        kappa = self.curvature[k]
        v_apex_limit = np.sqrt((self.mu_tire * 9.81) / max(1e-4, kappa))

        # Nominal velocity estimate on straight vs corner
        if kappa > 0.005:
            v_curr = min(v_apex_limit, 42.0)
        else:
            # Power to velocity balance on straights
            p_total_kw = 580.0 + p_kw
            v_curr = np.clip((p_total_kw / 2.5)**(1/3) * 11.5, 45.0, 92.0)

        dt = self.ds / max(10.0, v_curr)

        # Traffic penalty: stuck behind opponent in dirty air technical section
        if in_traffic and (10 <= k <= 25):
            dt *= 1.18  # 18% pace penalty in wake

        # Battery SoC progression
        energy_flow_j = p_kw * 1e3 * dt
        next_soc = soc - (energy_flow_j / self.capacity_j)

        return dt, next_soc

    def solve(self, in_traffic=False):
        """Executes backward Bellman recursion across all nodes and state bins."""
        M = len(self.soc_grid)
        V = np.full((self.N + 1, M), np.inf)
        policy_idx = np.zeros((self.N, M), dtype=int)

        # Terminal boundary condition: Zero cost remaining at start/finish line
        V[self.N, :] = 0.0

        # Backward Induction
        for k in range(self.N - 1, -1, -1):
            for s_idx, soc in enumerate(self.soc_grid):
                best_val = np.inf
                best_a = 3  # Default 0 kW coast

                for a_idx, p_kw in enumerate(self.actions_kw):
                    dt, next_soc = self.step_dynamics(k, soc, p_kw, in_traffic)

                    # Invariant: Prevent depletion below 12% or overcharging past 98%
                    if next_soc < 0.12 or next_soc > 0.98:
                        continue

                    # Interpolate value function at next continuous state
                    next_s_idx = np.argmin(np.abs(self.soc_grid - next_soc))
                    cost = dt + V[k + 1, next_s_idx]

                    if cost < best_val:
                        best_val = cost
                        best_a = a_idx

                V[k, s_idx] = best_val
                policy_idx[k, s_idx] = best_a

        return V, policy_idx

    def forward_simulate_optimal_trace(self, policy_idx, initial_soc=0.75, in_traffic=False):
        """Simulates the race car forward tracking the calculated DP policy."""
        s_trace = []
        soc_trace = [initial_soc]
        p_trace = []
        time_elapsed = 0.0

        curr_soc = initial_soc
        for k in range(self.N):
            s_idx = np.argmin(np.abs(self.soc_grid - curr_soc))
            opt_action_idx = policy_idx[k, s_idx]
            opt_p_kw = self.actions_kw[opt_action_idx]

            dt, next_soc = self.step_dynamics(k, curr_soc, opt_p_kw, in_traffic)
            time_elapsed += dt
            curr_soc = np.clip(next_soc, 0.12, 0.98)

            s_trace.append(k * self.ds)
            p_trace.append(opt_p_kw)
            soc_trace.append(curr_soc)

        return {
            "total_time_s": time_elapsed,
            "s_m": np.array(s_trace),
            "p_mguk_kw": np.array(p_trace),
            "soc_pct": np.array(soc_trace[:-1]) * 100.0
        }