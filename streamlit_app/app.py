import os
import subprocess
import sys
import time
import streamlit as st
from canvas_dashboard import render_haas_live_cockpit
from model_server import start_model_server

st.set_page_config(page_title="Haas Bearman Pit-Wall", layout="wide")

st.markdown("""
    <style>
    /* Remove default Streamlit top & bottom padding */
    .block-container {
        padding-top: 0rem !important;
        padding-bottom: 0rem !important;
        padding-left: 0rem !important;
        padding-right: 0rem !important;
        max-width: 100% !important;
    }
    /* Hide Streamlit footer and header */
    footer { visibility: hidden; display: none !important; }
    header { visibility: hidden; display: none !important; }
    
    /* Force Streamlit app background to match your light theme */
    .stApp {
        background-color: #EAEFF2 !important;
    }
    
    /* Remove iframe border & bottom gap */
    iframe {
        display: block !important;
        border: none !important;
    }
    </style>
""", unsafe_allow_html=True)

# 1. Start Model Server for .glb asset on port 8766
MODEL_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "haas_bearman.glb"))
start_model_server(MODEL_PATH, port=8766)

# 2. Start WebSocket Telemetry Bridge on port 8765
@st.cache_resource
def ensure_telemetry_bridge():
    bridge_script = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools", "telemetry_bridge.py"))
    proc = subprocess.Popen(
        [sys.executable, bridge_script],
        stdout=sys.stdout,
        stderr=sys.stderr,
        env=dict(os.environ, PYTHONPATH="/app"),
    )
    time.sleep(1.0)
    return proc

ensure_telemetry_bridge()

# 3. Mount 60 FPS GTA-View Cockpit
render_haas_live_cockpit(height=980)