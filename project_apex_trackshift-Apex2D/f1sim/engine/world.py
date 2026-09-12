import numpy as np
from scipy.interpolate import CubicSpline

class SilverstoneCircuit:
    def __init__(self):
        self.name = "Silverstone GP"
        self.track_width = 13.5
        
        raw_waypoints = [
            [0.0, 0.0], [280.0, 30.0], [430.0, 180.0], [350.0, 340.0],
            [260.0, 310.0], [310.0, 190.0], [720.0, 180.0], [940.0, 260.0],
            [980.0, 390.0], [890.0, 450.0], [920.0, 780.0], [980.0, 1020.0],
            [790.0, 1260.0], [610.0, 1280.0], [460.0, 1150.0], [120.0, 780.0],
            [-80.0, 480.0], [-120.0, 240.0], [-60.0, 120.0], [0.0, 0.0]
        ]
        
        theta = np.radians(-55.0)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        rot_pts = np.array(raw_waypoints, dtype=np.float64) @ R.T
        center = (rot_pts.min(axis=0) + rot_pts.max(axis=0)) / 2.0
        self.waypoints = rot_pts - center

        diffs = np.diff(self.waypoints, axis=0)
        dists = np.sqrt(np.sum(diffs**2, axis=1))
        self.s_nodes = np.concatenate([[0.0], np.cumsum(dists)])
        self.track_length = float(self.s_nodes[-1])

        self.spline_x = CubicSpline(self.s_nodes, self.waypoints[:, 0], bc_type='periodic')
        self.spline_y = CubicSpline(self.s_nodes, self.waypoints[:, 1], bc_type='periodic')

        self.dense_s = np.linspace(0.0, self.track_length, 800)
        cx = self.spline_x(self.dense_s)
        cy = self.spline_y(self.dense_s)
        dx = self.spline_x(self.dense_s, 1)
        dy = self.spline_y(self.dense_s, 1)
        psi = np.arctan2(dy, dx)

        self.left_x = cx - (self.track_width / 2.0) * np.sin(psi)
        self.left_y = cy + (self.track_width / 2.0) * np.cos(psi)
        self.right_x = cx + (self.track_width / 2.0) * np.sin(psi)
        self.right_y = cy - (self.track_width / 2.0) * np.cos(psi)

        self.braking_zones = [
            {"name": "Turn 3 (Village)", "apex_s": 480.0, "v_apex": 28.0},
            {"name": "Turn 6 (Brooklands)", "apex_s": 940.0, "v_apex": 31.0},
            {"name": "Turn 7 (Luffield)", "apex_s": 1150.0, "v_apex": 26.0},
            {"name": "Turn 9 (Copse)", "apex_s": 2750.0, "v_apex": 68.0},
            {"name": "Turn 15 (Stowe)", "apex_s": 4850.0, "v_apex": 42.0},
            {"name": "Turn 16 (Vale)", "apex_s": 5250.0, "v_apex": 24.0}
        ]

    def frenet_to_cartesian(self, s, y=0.0):
        s_mod = np.mod(s, self.track_length)
        cx = float(self.spline_x(s_mod))
        cy = float(self.spline_y(s_mod))
        dx = float(self.spline_x(s_mod, 1))
        dy = float(self.spline_y(s_mod, 1))
        psi = np.arctan2(dy, dx)
        return cx - y * np.sin(psi), cy + y * np.cos(psi), psi

    def get_next_braking_zone(self, s):
        s_mod = np.mod(s, self.track_length)
        best_zone = None
        min_dist = float("inf")
        for zone in self.braking_zones:
            dist = zone['apex_s'] - s_mod
            if dist < 0:
                dist += self.track_length
            if dist < min_dist:
                min_dist = dist
                best_zone = zone
        return best_zone, float(min_dist)