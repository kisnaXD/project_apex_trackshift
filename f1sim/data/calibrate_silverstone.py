# f1sim/data/calibrate_silverstone.py
import json
import os
import numpy as np
from f1sim.engine.world import SilverstoneCircuit

def calibrate_silverstone_2026(output_path="assets/silverstone_2026_baseline.json"):
    circuit = SilverstoneCircuit()
    s_dense = circuit.dense_s
    
    # 1. Extract continuous curvature from cubic spline derivatives
    dx = circuit.spline_x(s_dense, 1)
    ddx = circuit.spline_x(s_dense, 2)
    dy = circuit.spline_y(s_dense, 1)
    ddy = circuit.spline_y(s_dense, 2)
    curvature = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2)**1.5, 1e-6)

    # 2. 2026 FIA Technical Regulations Specification
    baseline_data = {
        "circuit_name": "Silverstone GP",
        "regulation_year": 2026,
        "track_length_m": circuit.track_length,
        "mass_kg": 768.0,
        "p_mguk_max_kw": 350.0,
        "lap_deploy_limit_mj": 8.5,
        "aero": {
            "cd_z_mode": 0.88,       # High downforce cornering mode
            "cd_x_mode": 0.58,       # Active low-drag straightline mode
            "frontal_area": 1.45
        },
        "braking": {
            "c0": 16.5,              # Mechanical friction decel (m/s^2)
            "c2": 0.0032             # Aero-induced decel factor
        },
        "curvature_profile": {
            "s_m": s_dense.tolist(),
            "kappa": curvature.tolist()
        }
    }

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(baseline_data, f, indent=2)

    print(f"Calibration saved to {output_path}")

if __name__ == "__main__":
    calibrate_silverstone_2026()