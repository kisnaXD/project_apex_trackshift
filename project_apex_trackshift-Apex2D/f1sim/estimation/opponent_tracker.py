import numpy as np

class OpponentReachabilityBubble:
    """
    Projects estimated opponent kinematics over a receding horizon.
    Constructs dynamic ellipsoidal exclusion zones (E_safe) parameterized
    by relative velocity and inferred aggression (theta_agg).
    """
    def __init__(self, horizon_steps=20, dt=0.05):
        self.N = horizon_steps
        self.dt = dt
        
        # Base clearance dimensions (meters)
        self.base_length = 5.0   # longitudinal semi-axis (a_safe)
        self.base_width = 2.2    # lateral semi-axis (b_safe)

    def rollout_trajectory(self, est_x_B, theta_agg):
        """
        Projects Car B forward using CTRA (Constant Turn-Rate and Acceleration).
        est_x_B: [s, y, v_s, v_y, a_s]
        theta_agg: aggression metric (0.0 to 1.0)
        
        Returns:
            trajectory_s: array of predicted longitudinal coordinates
            trajectory_y: array of predicted lateral coordinates
            bubble_a: dynamic longitudinal safe radius
            bubble_b: dynamic lateral safe radius
        """
        s_0, y_0, v_s_0, v_y_0, a_s_0 = est_x_B
        
        # Aggression dynamically scales the reaction uncertainty envelope
        bubble_a = self.base_length + (0.08 * v_s_0) + (1.5 * theta_agg)
        bubble_b = self.base_width + (0.8 * theta_agg)
        
        pred_s = np.zeros(self.N)
        pred_y = np.zeros(self.N)
        
        curr_s = s_0
        curr_y = y_0
        curr_vs = v_s_0
        curr_vy = v_y_0
        
        for k in range(self.N):
            # Opponent velocity decay/accel projection
            curr_vs = max(5.0, curr_vs + a_s_0 * self.dt)
            curr_s += curr_vs * self.dt
            
            # Opponent lateral dampening towards road bounds [-2.0, 2.0]
            curr_y = np.clip(curr_y + curr_vy * self.dt, -2.2, 2.2)
            curr_vy *= 0.95  # natural lateral settling
            
            pred_s[k] = curr_s
            pred_y[k] = curr_y

        return pred_s, pred_y, bubble_a, bubble_b

    @staticmethod
    def get_ellipse_contour(center_s, center_y, a_safe, b_safe, num_pts=36):
        """Generates 2D coordinates for Plotly visualization."""
        theta = np.linspace(0, 2 * np.pi, num_pts)
        ellipse_s = center_s + a_safe * np.cos(theta)
        ellipse_y = center_y + b_safe * np.sin(theta)
        return ellipse_s, ellipse_y