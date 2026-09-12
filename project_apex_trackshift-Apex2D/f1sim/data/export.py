import pyarrow as pa
import pyarrow.parquet as pq
import os
import json

class RunArtifactExporter:
    def __init__(self, export_dir="telemetry_logs/runs"):
        self.export_dir = export_dir
        os.makedirs(self.export_dir, exist_ok=True)
        self.telemetry_buffer = []

    def record_step(self, record: dict):
        self.telemetry_buffer.append(record)

    def write_artifacts(self, run_id: str, run_config: dict):
        if not self.telemetry_buffer:
            return

        run_path = os.path.join(self.export_dir, run_id)
        os.makedirs(run_path, exist_ok=True)

        # 1. Config Metadata
        with open(os.path.join(run_path, "run_config.json"), "w") as f:
            json.dump(run_config, f, indent=2)

        # 2. Typed Parquet Telemetry
        table = pa.Table.from_pylist(self.telemetry_buffer)
        pq.write_table(table, os.path.join(run_path, "telemetry.parquet"))
        print(f"[Artifacts] Successfully wrote {len(self.telemetry_buffer)} records to {run_path}")
        self.telemetry_buffer.clear()