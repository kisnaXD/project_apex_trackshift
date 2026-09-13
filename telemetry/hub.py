import json
import os
import time
from collections import deque
from typing import Optional

from f1sim.data.schema import TelemetryFrame


class F1TelemetryHub:
    def __init__(self, log_dir: str = "telemetry_logs", memory_buffer_size: int = 1000):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        session_id = time.strftime("%Y%m%d-%H%M%S")
        self.log_file = os.path.join(self.log_dir, f"session_{session_id}.jsonl")
        self.buffer = deque(maxlen=memory_buffer_size)

    def publish_frame(self, frame: TelemetryFrame) -> None:
        self.buffer.append(frame)
        with open(self.log_file, "a", encoding="utf-8") as f:
            f.write(json.dumps(frame.to_dict(), separators=(",", ":")) + "\n")

    def get_latest(self) -> Optional[TelemetryFrame]:
        if not self.buffer:
            return None
        return self.buffer[-1]

    def query_opponent_speed_trace(self, car_id: str, frames: int = 40) -> list:
        history = list(self.buffer)[-frames:]
        speeds = []
        for frame in history:
            opponent = frame.opponents.get(car_id)
            if opponent is not None:
                speeds.append(opponent.kinematics.speed_kmh)
        return speeds
