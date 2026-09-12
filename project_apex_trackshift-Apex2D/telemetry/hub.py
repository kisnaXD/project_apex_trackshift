import json
import os
import time
from collections import deque
from telemetry.schemas import TelemetryFrame

class F1TelemetryHub:
    def __init__(self, log_dir="telemetry_logs", memory_buffer_size=1000):
        """
        memory_buffer_size: Stores the last N frames in memory for real-time querying.
        (1000 frames @ 20Hz = 50 seconds of rolling history)
        """
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create a new log file for each session
        session_id = time.strftime("%Y%m%d-%H%M%S")
        self.log_file = os.path.join(self.log_dir, f"session_{session_id}.jsonl")
        
        # Rolling buffer for real-time decision engine queries
        self.buffer = deque(maxlen=memory_buffer_size)
        
    def publish_frame(self, frame: TelemetryFrame):
        """Appends the frame to memory and streams it to the log file."""
        self.buffer.append(frame)
        
        # Write to disk asynchronously (JSONL format is standard for telemetry)
        with open(self.log_file, "a") as f:
            f.write(json.dumps(frame.to_dict()) + "\n")

    def get_latest(self) -> TelemetryFrame:
        if not self.buffer:
            return None
        return self.buffer[-1]

    def query_opponent_speed_trace(self, car_id: str, frames=40) -> list:
        """Example query: Get the speed trace of an opponent over the last 2 seconds (40 frames)."""
        history = list(self.buffer)[-frames:]
        speeds = []
        for f in history:
            if car_id in f.opponents:
                speeds.append(f.opponents[car_id].speed_kmh)
        return speeds