import numpy as np
import cvxpy as cp

class TacticalTrajectoryPlanner:
    """
    Frenet-frame Model Predictive Contour Controller:
    Solves QP minimizing deviation from racing line / attack bias,
    tracking optimal DP energy allocation while respecting the opponent's
    dynamic reachability bubble.
    """
    def __init__(self, horizon_steps=15, ds=3.5):
        self.N = horizon_steps
        self.ds = ds
        self.track_half_width = 5.5  # 11m usable width

    def solve_tactical_plan(self, ego_s, ego_y, target_y, opt_p_kw, pred_s, pred_y, a_safe, b_safe):
        """
        Solves QP for lateral trajectory y[0...N] over longitudinal horizon s[0...N].
        """
        y = cp.Variable(self.N + 1)
        dy = cp.Variable(self.N)
        slack = cp.Variable(self.N + 1, nonneg=True)

        s_nodes = np.array([ego_s + k * self.ds for k in range(self.N + 1)])

        cost = 0.0
        constraints = [y[0] == ego_y]

        for k in range(self.N):
            # Kinematic curvature continuity
            constraints.append(y[k+1] - y[k] == dy[k])
            # Maximum lateral steering rate (grip limit)
            constraints.append(cp.abs(dy[k]) <= 0.45)

        for k in range(self.N + 1):
            # Track limit constraints with soft slack to prevent infeasibility
            constraints.append(y[k] <= self.track_half_width - 1.2 + slack[k])
            constraints.append(y[k] >= -self.track_half_width + 1.2 - slack[k])

            # Reachability exclusion hyperplane from Car B
            if k < len(pred_s):
                ds_k = abs(s_nodes[k] - pred_s[k])
                if ds_k < a_safe:
                    # Required lateral clearance based on ellipsoidal bubble
                    req_dy = b_safe * np.sqrt(max(0.01, 1.0 - (ds_k / a_safe)**2))
                    if target_y < pred_y[k]:  # Passing inside (right)
                        constraints.append(y[k] <= pred_y[k] - req_dy + slack[k])
                    else:                     # Passing outside (left)
                        constraints.append(y[k] >= pred_y[k] + req_dy - slack[k])

            # Cost: track optimal lateral target + minimize control effort + penalize slacks
            weight_tracking = 3.5 if opt_p_kw > 60.0 else 1.5
            cost += weight_tracking * cp.square(y[k] - target_y)
            cost += 450.0 * cp.square(slack[k])
            if k < self.N:
                cost += 8.0 * cp.square(dy[k])

        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            if prob.status in ["optimal", "optimal_inaccurate"]:
                return s_nodes, y.value, True
        except Exception:
            pass

        # Fallback to smoothed straight line if QP fails
        return s_nodes, np.linspace(ego_y, target_y, self.N + 1), False