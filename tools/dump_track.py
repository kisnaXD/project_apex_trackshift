import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from f1sim.data.fastf1_adapter import F1SessionReplayer

# Load using the existing cached session in telemetry_logs
replayer = F1SessionReplayer(year=2024, circuit="Silverstone", session_type="R", ego_driver="VER")
df = replayer.track_map.copy()

# Subsample to 500 clean continuous nodes
idxs = np.linspace(0, len(df) - 1, 500, dtype=int)
sub = df.iloc[idxs]

# Standard FIA TV Orientation: FastF1 X is along track Easting, Y is Northing
# Normalizing & centering
cx = (sub['x'].max() + sub['x'].min()) / 2.0
cy = (sub['y'].max() + sub['y'].min()) / 2.0

pts = []
for _, row in sub.iterrows():
    # Invert Y so screen coordinate systems match canvas (Y down)
    pts.append({
        "x": round(float(row['x'] - cx), 1),
        "y": round(float(-(row['y'] - cy)), 1),
        "s": round(float(row['s']), 1)
    })

os.makedirs("assets", exist_ok=True)
with open("assets/silverstone_track.json", "w") as f:
    json.dump(pts, f)

print(f"Dumped {len(pts)} verified FastF1 track coordinates to assets/silverstone_track.json")