import numpy as np

class SyntheticSensorPipeline:
    """
    Models realistic track perception:
    - Adds Gaussian noise to position estimates
    - Simulates sensor latency (ring buffer)
    - Computes covariance matrices
    """
    def __init__(self, latency_steps: int = 2, sigma_s: float = 0.45, sigma_y: float = 0.12):
        self.latency_steps = latency_steps
        self.sigma_s = sigma_s
        self.sigma_y = sigma_y
        self.buffer = []

    def observe(self, true_opponents: dict) -> dict:
        noisy_frame = {}
        for code, opp in true_opponents.items():
            meas_s = opp["s"] + np.random.normal(0.0, self.sigma_s)
            meas_y = opp["y"] + np.random.normal(0.0, self.sigma_y)
            meas_v = opp["v_s"] + np.random.normal(0.0, 0.25)
            
            noisy_frame[code] = {
                "s": float(meas_s),
                "y": float(meas_y),
                "v_s": float(meas_v),
                "cov_s": float(self.sigma_s ** 2),
                "cov_y": float(self.sigma_y ** 2),
                "clipping": bool(opp["clipping"]),
                "compound": opp["compound"]
            }

        self.buffer.append(noisy_frame)
        if len(self.buffer) > self.latency_steps:
            return self.buffer.pop(0)
        return noisy_frame