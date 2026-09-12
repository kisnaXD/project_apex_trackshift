import os
import time
import numpy as np
import streamlit as st
import plotly.graph_objects as go
from f1sim.engine.scenario import RaceScenario

# =========================================================
# CONFIGURATION: CHANGE THIS ONE LINE TO SET OPPONENT COUNT
# =========================================================
NUM_OPPONENTS = 1  # Set to 1, 3, 5, 10, or 19
# =========================================================

st.set_page_config(
    page_title="APEX // 2026 TACTICAL RADAR",
    page_icon="🏎️",
    layout="wide",
    initial_sidebar_state="collapsed"
)

TEAM_COLORS = {
    "LEC": "#e10600", "SAI": "#e10600", "HAM": "#e10600",
    "VER": "#1e41ff", "PER": "#1e41ff", "TSU": "#1e41ff",
    "NOR": "#ff8000", "PIA": "#ff8000",
    "RUS": "#00d2be", "ANT": "#00d2be",
    "ALO": "#229971", "STR": "#229971",
    "GAS": "#0090ff", "DOO": "#0090ff",
    "ALB": "#005aff", "COL": "#005aff",
    "BEA": "#b6babd", "OCO": "#b6babd",
    "HUL": "#52e252", "BOR": "#52e252"
}

EGO_BLUE = "#0090ff"

st.markdown(f"""
<style>
    @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700;900&family=Titillium+Web:wght@600;700;900&display=swap');

    header, footer, #MainMenu {{visibility: hidden !important; display: none !important;}}
    .block-container {{
        padding: 0.35rem 1.0rem 0rem 1.0rem !important;
        max-width: 100% !important;
    }}
    .stApp {{
        background-color: #000000 !important;
        color: #f1f1f1 !important;
        font-family: 'Titillium Web', -apple-system, sans-serif !important;
    }}
    .pitwall-bar {{
        display: flex;
        align-items: center;
        justify-content: space-between;
        padding: 5px 14px;
        background: #050505;
        border: 1px solid #1c1c1c;
        border-left: 5px solid {EGO_BLUE};
        border-radius: 4px;
        margin-bottom: 6px;
    }}
    .pitwall-title {{
        font-size: 1.22rem;
        font-weight: 900;
        letter-spacing: 0.14em;
        color: #ffffff;
        display: flex;
        align-items: center;
        gap: 10px;
    }}
    .pitwall-title span {{ color: {EGO_BLUE}; }}

    .kpi-tile {{
        background: #070707;
        border: 1px solid #1a1a1a;
        border-radius: 4px;
        padding: 7px 10px;
        position: relative;
    }}
    .kpi-tile::before {{
        content: "";
        position: absolute;
        top: 0; left: 0; right: 0;
        height: 2px;
        background: #202020;
    }}
    .kpi-tile.blue-accent::before {{ background: {EGO_BLUE}; }}
    .kpi-tile.cyan-accent::before {{ background: #00d2be; }}
    .kpi-tile.red-accent::before {{ background: #e10600; }}

    .kpi-tag {{
        font-size: 0.65rem;
        text-transform: uppercase;
        letter-spacing: 0.12em;
        color: #888888;
        font-weight: 700;
    }}
    .kpi-val {{
        font-size: 1.45rem;
        font-weight: 900;
        font-family: 'JetBrains Mono', monospace;
        color: #ffffff;
        line-height: 1.1;
    }}
    .kpi-val span {{
        font-size: 0.72rem;
        color: {EGO_BLUE};
        font-weight: 700;
    }}
    .kpi-sub {{
        font-size: 0.67rem;
        font-family: 'JetBrains Mono', monospace;
        color: #a0aec0;
        margin-top: 2px;
    }}

    div.stButton > button {{
        background-color: #0c0c0c !important;
        color: #ffffff !important;
        border: 1px solid #262626 !important;
        font-family: 'Titillium Web', sans-serif !important;
        font-weight: 800 !important;
        letter-spacing: 0.08em !important;
        border-radius: 4px !important;
        padding: 3px 8px !important;
    }}
    div.stButton > button:hover {{
        border-color: {EGO_BLUE} !important;
        color: {EGO_BLUE} !important;
    }}
</style>
""", unsafe_allow_html=True)

if "sim" not in st.session_state or getattr(st.session_state, "active_opponents", None) != NUM_OPPONENTS:
    st.session_state.sim = RaceScenario(max_opponents=NUM_OPPONENTS)
    st.session_state.active_opponents = NUM_OPPONENTS
    st.session_state.is_running = False
    st.session_state.latest_snapshot = st.session_state.sim.step()

c_head, c_run, c_pause, c_rst = st.columns([5.5, 1.1, 1.1, 1.1])

with c_head:
    st.markdown(f"""
    <div class="pitwall-bar">
        <div class="pitwall-title">🏎️ APEX <span>// TACTICAL COMBAT RADAR</span></div>
        <div style="font-size:0.75rem; font-weight:700; font-family:'JetBrains Mono'; color:#888;">
            CIRCUIT: <span style="color:#fff;">SILVERSTONE GP (14m WIDE)</span> | EGO: <span style="color:{EGO_BLUE}; font-weight:900;">ALPINE RACING BLUE</span> | GRID: <span style="color:#fff;">{NUM_OPPONENTS} CARS</span>
        </div>
    </div>
    """, unsafe_allow_html=True)

with c_run:
    if st.button("▶ ENGAGE", use_container_width=True):
        st.session_state.is_running = True
        st.rerun()

with c_pause:
    if st.button("⏸ HOLD", use_container_width=True):
        st.session_state.is_running = False
        st.rerun()

with c_rst:
    if st.button("↺ RESET", use_container_width=True):
        st.session_state.sim = RaceScenario(max_opponents=NUM_OPPONENTS)
        st.session_state.active_opponents = NUM_OPPONENTS
        st.session_state.is_running = False
        st.session_state.latest_snapshot = st.session_state.sim.step()
        st.rerun()

if st.session_state.is_running:
    for _ in range(10):
        st.session_state.latest_snapshot = st.session_state.sim.step()

snapshot = st.session_state.latest_snapshot
world = st.session_state.sim.world
track_len = getattr(world, "track_length", 5891.0)
ego = snapshot.get("ego", {})
opponents = snapshot.get("opponents", {})
target_code = snapshot.get("target_code", "LEC")
gap_m = snapshot.get("gap_m", 0.0)
is_leading = snapshot.get("is_leading", False)
adv_call = snapshot.get("advisor", {}).get("tactical_call", "PACING")

# ---------------------------------------------------------
# CONFIDENCE METRIC
# ---------------------------------------------------------
ego_v = ego.get("v_s", 70.0)
target_v = opponents.get(target_code, {}).get("v_s", 70.0) if opponents else 70.0
delta_v = (ego_v - target_v) * 3.6
ego_y = ego.get("y", 0.0)
opp_y = opponents.get(target_code, {}).get("y", 0.0) if opponents else 0.0
lateral_clearance = abs(ego_y - opp_y)
soc_pct = snapshot.get("ers", {}).get("soc_pct", 74.0)

if is_leading:
    confidence_score = 99.0
    conf_status = "P1 LEAD SECURED"
    conf_color = "#10b981"
else:
    p_dist = np.clip(1.0 - (max(0.0, gap_m) / 35.0), 0.1, 1.0) if gap_m < 35.0 else max(0.05, 1.0 - (gap_m / 150.0))
    p_vel = np.clip(0.5 + (delta_v / 30.0), 0.1, 1.0)
    p_lane = np.clip(lateral_clearance / 2.5, 0.4, 1.0)
    p_ers = np.clip(soc_pct / 80.0, 0.3, 1.0)

    raw_conf = (0.35 * p_dist + 0.30 * p_vel + 0.20 * p_lane + 0.15 * p_ers) * 100.0
    confidence_score = float(np.clip(raw_conf, 5.0, 99.0))

    if confidence_score >= 70.0:
        conf_status = "OPTIMAL // PASS COMMIT"
        conf_color = "#00d2be"
    elif confidence_score >= 40.0:
        conf_status = "ATTACKING // CLOSING"
        conf_color = "#f59e0b"
    else:
        conf_status = "GAP EXPANDING"
        conf_color = "#ef4444"

# ---------------------------------------------------------
# KPIS
# ---------------------------------------------------------
k1, k2, k3, k4, k5, k6 = st.columns(6)

v_kmh = ego_v * 3.6
gear = 8 if v_kmh > 290 else (7 if v_kmh > 250 else (6 if v_kmh > 210 else (5 if v_kmh > 170 else (4 if v_kmh > 130 else 3))))

with k1:
    st.markdown(f"""
    <div class="kpi-tile blue-accent">
        <div class="kpi-tag">Velocity / Powertrain</div>
        <div class="kpi-val">{v_kmh:.1f} <span>KM/H</span></div>
        <div class="kpi-sub">GEAR {gear} • {int(v_kmh*38)} RPM</div>
    </div>
    """, unsafe_allow_html=True)

with k2:
    lead_status = "P1 // LEADING" if is_leading else f"PURSUING {target_code}"
    gap_color = "#10b981" if is_leading else EGO_BLUE
    disp_gap = f"{gap_m:+.2f} m" if abs(gap_m) < 500 else "P1 CLEAN AIR"
    st.markdown(f"""
    <div class="kpi-tile blue-accent">
        <div class="kpi-tag">Target Gap ({target_code})</div>
        <div class="kpi-val" style="color:{gap_color}; font-size:1.35rem;">{disp_gap}</div>
        <div class="kpi-sub">{lead_status} (Δv {delta_v:+.1f} km/h)</div>
    </div>
    """, unsafe_allow_html=True)

with k3:
    st.markdown(f"""
    <div class="kpi-tile" style="border-top: 2px solid {conf_color};">
        <div class="kpi-tag">Overtake Confidence</div>
        <div class="kpi-val" style="color:{conf_color};">{confidence_score:.1f}%</div>
        <div class="kpi-sub" style="color:{conf_color};">{conf_status}</div>
    </div>
    """, unsafe_allow_html=True)

with k4:
    soc = snapshot.get("ers", {}).get("soc_pct", 74.0)
    mj = snapshot.get("ers", {}).get("energy_mj", 2.96)
    st.markdown(f"""
    <div class="kpi-tile">
        <div class="kpi-tag">Hybrid MGU-K SoC</div>
        <div class="kpi-val">{soc:.1f} <span>%</span></div>
        <div class="kpi-sub">{mj:.2f} / 4.00 MJ CAPACITY</div>
    </div>
    """, unsafe_allow_html=True)

with k5:
    t_surf = snapshot.get("tire", {}).get("T_surf", 98.2)
    wear = snapshot.get("tire", {}).get("wear_pct", 3.2)
    st.markdown(f"""
    <div class="kpi-tile">
        <div class="kpi-tag">Pirelli C2 Medium</div>
        <div class="kpi-val">{t_surf:.1f} <span>°C</span></div>
        <div class="kpi-sub">DEGRADATION: {wear:.1f}%</div>
    </div>
    """, unsafe_allow_html=True)

with k6:
    cbf_margin = max(0.0, gap_m - 3.5) if (0.0 < gap_m < 100.0) else 15.0
    st.markdown(f"""
    <div class="kpi-tile blue-accent">
        <div class="kpi-tag">Safety CBF Margin</div>
        <div class="kpi-val">{cbf_margin:.2f} <span>M</span></div>
        <div class="kpi-sub">BARRIER LIMIT: 3.50 M</div>
    </div>
    """, unsafe_allow_html=True)

st.markdown("<div style='height: 4px;'></div>", unsafe_allow_html=True)

# ---------------------------------------------------------
# HIGH-FIDELITY 2D FORMULA CAR HUD WIREFRAME
# ---------------------------------------------------------
def create_f1_hud_chassis(center_x, center_y, heading_rad=0.0, scale=1.5, color="#ffffff"):
    fw = np.array([[-1.05, 2.2], [-1.05, 2.5], [1.05, 2.5], [1.05, 2.2], [-1.05, 2.2]]) * scale
    chassis = np.array([
        [0.0, 2.45], [0.26, 1.4], [0.65, 0.5], [0.65, -1.3],
        [0.42, -2.1], [-0.42, -2.1], [-0.65, -1.3], [-0.65, 0.5],
        [-0.26, 1.4], [0.0, 2.45]
    ]) * scale
    rw = np.array([[-0.9, -2.25], [-0.9, -2.55], [0.9, -2.55], [0.9, -2.25], [-0.9, -2.25]]) * scale
    halo = np.array([[0.0, 0.8], [0.22, 0.3], [0.22, -0.3], [-0.22, -0.3], [-0.22, 0.3], [0.0, 0.8]]) * scale

    tires = [
        np.array([[-1.18, 1.45], [-0.82, 1.45], [-0.82, 0.82], [-1.18, 0.82], [-1.18, 1.45]]) * scale,
        np.array([[0.82, 1.45], [1.18, 1.45], [1.18, 0.82], [0.82, 0.82], [0.82, 1.45]]) * scale,
        np.array([[-1.18, -1.15], [-0.82, -1.15], [-0.82, -1.78], [-1.18, -1.78], [-1.18, -1.15]]) * scale,
        np.array([[0.82, -1.15], [1.18, -1.15], [1.18, -1.78], [0.82, -1.78], [0.82, -1.15]]) * scale,
    ]

    c, s = np.cos(heading_rad), np.sin(heading_rad)
    R = np.array([[c, -s], [s, c]])

    traces = []
    fill_rgba = 'rgba(0, 144, 255, 0.35)' if color == EGO_BLUE else 'rgba(255, 255, 255, 0.18)'

    for part, width, fill_flag in [(chassis, 2.2, True), (halo, 1.5, False), (fw, 2.2, False), (rw, 2.5, False)]:
        rotated = (part @ R.T) + np.array([center_x, center_y])
        traces.append(go.Scatter(
            x=rotated[:, 0], y=rotated[:, 1],
            mode='lines',
            line=dict(color=color, width=width),
            fill='toself' if fill_flag else 'none',
            fillcolor=fill_rgba,
            hoverinfo='skip',
            showlegend=False
        ))

    for tire in tires:
        rotated = (tire @ R.T) + np.array([center_x, center_y])
        traces.append(go.Scatter(
            x=rotated[:, 0], y=rotated[:, 1],
            mode='lines',
            line=dict(color='#444444', width=1.5),
            fill='toself',
            fillcolor='#121212',
            hoverinfo='skip',
            showlegend=False
        ))

    return traces

# ---------------------------------------------------------
# COORDINATE CONVERSIONS
# ---------------------------------------------------------
ego_s = ego.get("s", 0.0)
ego_x, ego_y_cart, ego_psi = world.frenet_to_cartesian(ego_s, ego_y)

def global_to_local(gx, gy, ref_x, ref_y, ref_psi):
    dx = gx - ref_x
    dy = gy - ref_y
    c = np.cos(ref_psi)
    s = np.sin(ref_psi)
    return c * dx + s * dy, -s * dx + c * dy

col_track, col_radar = st.columns([1.02, 0.98])

with col_track:
    st.markdown("<div style='font-size:0.75rem; font-weight:900; letter-spacing:0.12em; color:#888888; margin-bottom:2px;'>SILVERSTONE CIRCUIT // GLOBAL FIELD GPS</div>", unsafe_allow_html=True)

    track_x = world.spline_x(world.dense_s)
    track_y = world.spline_y(world.dense_s)

    fig_track = go.Figure()

    fig_track.add_trace(go.Scatter(x=world.left_x, y=world.left_y, mode='lines', line=dict(color='#1c1c1c', width=1.5), hoverinfo='skip', showlegend=False))
    fig_track.add_trace(go.Scatter(x=world.right_x, y=world.right_y, mode='lines', line=dict(color='#1c1c1c', width=1.5), hoverinfo='skip', showlegend=False))
    fig_track.add_trace(go.Scatter(x=track_x, y=track_y, mode='lines', line=dict(color='#0d0d0d', width=5), hoverinfo='skip', showlegend=False))

    if opponents:
        oxs, oys, codes, colors = [], [], [], []
        for code, o_data in opponents.items():
            ox, oy, _ = world.frenet_to_cartesian(o_data["s"], o_data["y"])
            oxs.append(ox)
            oys.append(oy)
            codes.append(code)
            colors.append(TEAM_COLORS.get(code, "#94a3b8"))

        fig_track.add_trace(go.Scatter(
            x=oxs, y=oys,
            mode='markers+text',
            marker=dict(color=colors, size=8, line=dict(color='#000000', width=1)),
            text=codes,
            textposition="top center",
            textfont=dict(family="JetBrains Mono", size=9, color="#e2e8f0"),
            name='Field'
        ))

    fig_track.add_trace(go.Scatter(
        x=[ego_x], y=[ego_y_cart],
        mode='markers+text',
        marker=dict(color=EGO_BLUE, size=11, symbol='circle', line=dict(color='#ffffff', width=2)),
        text=["EGO"],
        textposition="bottom center",
        textfont=dict(family="JetBrains Mono", size=10, color=EGO_BLUE),
        name='Ego'
    ))

    fig_track.update_layout(
        template="plotly_dark",
        paper_bgcolor="#000000",
        plot_bgcolor="#000000",
        margin=dict(l=0, r=0, t=0, b=0),
        xaxis=dict(showgrid=False, zeroline=False, visible=False),
        yaxis=dict(showgrid=False, zeroline=False, visible=False, scaleanchor="x", scaleratio=1),
        height=620,
        showlegend=False
    )
    st.plotly_chart(fig_track, use_container_width=True)

# ---------------------------------------------------------
# 14-METER WIDE TACTICAL RADAR HUD
# ---------------------------------------------------------
with col_radar:
    st.markdown(f"<div style='display:flex; justify-content:space-between; align-items:center; margin-bottom:2px;'><span style='font-size:0.75rem; font-weight:900; letter-spacing:0.12em; color:#888888;'>TACTICAL PROXIMITY RADAR // AVIONICS HUD</span><span style='font-size:0.75rem; font-family:\"JetBrains Mono\"; font-weight:800; color:{conf_color};'>OVERTAKE PROB: {confidence_score:.1f}%</span></div>", unsafe_allow_html=True)

    fig_hud = go.Figure()

    rad_x_span = [-30.0, 30.0]
    rad_y_span = [-18.0, 58.0]

    # Concentric Distance Rings
    theta_vals = np.linspace(0, 2*np.pi, 100)
    for r, col, dash_type, label in [
        (10.0, "rgba(0, 144, 255, 0.40)", "dash", "10m CLOSE RANGE"),
        (25.0, "rgba(0, 210, 190, 0.30)", "dot", "25m ATTACK ZONE"),
        (45.0, "rgba(75, 85, 99, 0.35)", "solid", "45m DETECTION")
    ]:
        fig_hud.add_trace(go.Scatter(
            x=r * np.cos(theta_vals),
            y=r * np.sin(theta_vals),
            mode='lines',
            line=dict(color=col, width=1.3, dash=dash_type),
            hoverinfo='skip',
            showlegend=False
        ))
        fig_hud.add_trace(go.Scatter(
            x=[0.0], y=[r + 1.2],
            mode='text',
            text=[label],
            textfont=dict(family="JetBrains Mono", size=8, color="#555555"),
            showlegend=False
        ))

    # Crosshair lines
    fig_hud.add_trace(go.Scatter(
        x=[rad_x_span[0], rad_x_span[1]], y=[0, 0],
        mode='lines', line=dict(color='rgba(55, 65, 81, 0.45)', width=1, dash='dot'), hoverinfo='skip', showlegend=False
    ))
    fig_hud.add_trace(go.Scatter(
        x=[0, 0], y=[rad_y_span[0], rad_y_span[1]],
        mode='lines', line=dict(color='rgba(55, 65, 81, 0.45)', width=1, dash='dot'), hoverinfo='skip', showlegend=False
    ))

    # DRS / Aero Slipstream Cone
    cone_x = [0.0, 48.0 * np.sin(np.radians(14)), -48.0 * np.sin(np.radians(14)), 0.0]
    cone_y = [0.0, 48.0 * np.cos(np.radians(14)), 48.0 * np.cos(np.radians(14)), 0.0]
    fig_hud.add_trace(go.Scatter(
        x=cone_x, y=cone_y,
        fill='toself',
        fillcolor='rgba(0, 144, 255, 0.06)',
        line=dict(color='rgba(0, 144, 255, 0.28)', width=1, dash='dot'),
        name='Aero Slipstream'
    ))

    # REAL 14-METER WIDE TRACK CURBS (+/- 7.0m)
    s_samples = np.linspace(ego_s - 18.0, ego_s + 62.0, 60)
    curb_lx, curb_ly, curb_rx, curb_ry, center_lx, center_ly = [], [], [], [], [], []
    for s_step in s_samples:
        gx_l, gy_l, _ = world.frenet_to_cartesian(s_step, 7.0)
        gx_r, gy_r, _ = world.frenet_to_cartesian(s_step, -7.0)
        gx_c, gy_c, _ = world.frenet_to_cartesian(s_step, 0.0)

        lx, ly = global_to_local(gx_l, gy_l, ego_x, ego_y_cart, ego_psi)
        rx, ry = global_to_local(gx_r, gy_r, ego_x, ego_y_cart, ego_psi)
        cx, cy = global_to_local(gx_c, gy_c, ego_x, ego_y_cart, ego_psi)

        curb_lx.append(ly)
        curb_ly.append(lx)
        curb_rx.append(ry)
        curb_ry.append(rx)
        center_lx.append(cy)
        center_ly.append(cx)

    # Road Surface Shading
    poly_curb_x = curb_lx + curb_rx[::-1]
    poly_curb_y = curb_ly + curb_ry[::-1]
    fig_hud.add_trace(go.Scatter(
        x=poly_curb_x, y=poly_curb_y,
        fill='toself',
        fillcolor='rgba(18, 22, 30, 0.45)',
        line=dict(color='rgba(0,0,0,0)', width=0),
        hoverinfo='skip',
        showlegend=False
    ))

    # Red/White Curb Edges & Dashed Centerline
    fig_hud.add_trace(go.Scatter(x=curb_lx, y=curb_ly, mode='lines', line=dict(color='#e10600', width=2.5), hoverinfo='skip', showlegend=False))
    fig_hud.add_trace(go.Scatter(x=curb_rx, y=curb_ry, mode='lines', line=dict(color='#e10600', width=2.5), hoverinfo='skip', showlegend=False))
    fig_hud.add_trace(go.Scatter(x=center_lx, y=center_ly, mode='lines', line=dict(color='#2a3547', width=1.5, dash='dash'), hoverinfo='skip', showlegend=False))

    # Draw Ego F1 Car Wireframe (Blue) at (0, 0)
    for tr in create_f1_hud_chassis(0.0, 0.0, heading_rad=0.0, scale=1.5, color=EGO_BLUE):
        fig_hud.add_trace(tr)

    # Render Opponents with Dynamic Lock Reticles & Yaw
    for code, o_data in opponents.items():
        delta_s = (o_data["s"] - ego_s) % track_len
        if delta_s > track_len / 2.0:
            delta_s -= track_len

        opp_gx, opp_gy, opp_gpsi = world.frenet_to_cartesian(o_data["s"], o_data["y"])
        loc_x, loc_y = global_to_local(opp_gx, opp_gy, ego_x, ego_y_cart, ego_psi)

        hud_x = loc_y
        hud_y = loc_x
        dist_mag = np.hypot(hud_x, hud_y)
        t_col = TEAM_COLORS.get(code, "#ffffff")

        if dist_mag <= 55.0:
            rel_heading = opp_gpsi - ego_psi

            # Draw Opponent F1 Car Wireframe
            for tr in create_f1_hud_chassis(hud_x, hud_y, heading_rad=rel_heading, scale=1.5, color=t_col):
                fig_hud.add_trace(tr)

            # Target Lock Reticle
            box_r = 3.2
            bx = [hud_x - box_r, hud_x + box_r, hud_x + box_r, hud_x - box_r, hud_x - box_r]
            by = [hud_y - box_r, hud_y - box_r, hud_y + box_r, hud_y + box_r, hud_y - box_r]
            fig_hud.add_trace(go.Scatter(
                x=bx, y=by,
                mode='lines',
                line=dict(color=t_col, width=1.5, dash='dot'),
                hoverinfo='skip',
                showlegend=False
            ))

            fig_hud.add_trace(go.Scatter(
                x=[hud_x], y=[hud_y + 5.2],
                mode='text',
                text=[f"TARGET: {code} ({delta_s:+.1f}m)"],
                textfont=dict(family="JetBrains Mono", size=10, color=t_col),
                showlegend=False
            ))

    fig_hud.add_trace(go.Scatter(
        x=[rad_x_span[0] + 2.0], y=[rad_y_span[1] - 4.0],
        mode='text',
        text=[f"FIA TACTICAL: {conf_status}"],
        textfont=dict(family="JetBrains Mono", size=10, color=conf_color),
        showlegend=False
    ))

    fig_hud.update_layout(
        template="plotly_dark",
        paper_bgcolor="#000000",
        plot_bgcolor="#000000",
        margin=dict(l=5, r=5, t=5, b=5),
        xaxis=dict(
            title="Lateral Offset X (m)",
            range=rad_x_span,
            gridcolor="#0d0d0d",
            zerolinecolor="#1f242d",
            scaleanchor="y",
            scaleratio=1,
            dtick=10
        ),
        yaxis=dict(
            title="Longitudinal Proximity Y (m)",
            range=rad_y_span,
            gridcolor="#0d0d0d",
            zerolinecolor="#1f242d",
            dtick=10
        ),
        height=620,
        showlegend=False
    )
    st.plotly_chart(fig_hud, use_container_width=True)

if st.session_state.is_running:
    time.sleep(0.04)
    st.rerun()
