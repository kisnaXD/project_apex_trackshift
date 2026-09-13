import json
import os
import streamlit.components.v1 as components

def render_haas_live_cockpit(height=860):
    json_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "silverstone_track.json"))
    embedded_track = "[]"
    if os.path.isfile(json_path):
        with open(json_path, "r") as f:
            embedded_track = f.read().strip() or "[]"

    raw_html = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700;800&family=Inter:wght@400;600;700;900&display=swap" rel="stylesheet">
<style>
:root {
    --bg: #060809;
    --panel: #0c1012;
    --panel-sub: #111618;
    --line: #1c2427;
    --muted: #64748b;
    --text: #f1f5f9;
    --cyan: #00d2be;
    --amber: #f59e0b;
    --green: #10b981;
    --red: #ef4444;
    --haas: #cfd4d5;
}
html, body {
    margin: 0; padding: 0; width: 100%; height: 100%;
    background: var(--bg); color: var(--text);
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    overflow: hidden;
    -webkit-font-smoothing: antialiased;
}
#container {
    display: flex;
    flex-direction: column;
    width: 100%;
    height: 100vh;
    padding: 10px 14px;
    box-sizing: border-box;
    gap: 10px;
}
.topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 6px;
    padding: 8px 16px;
}
.brand-group {
    display: flex;
    align-items: center;
    gap: 14px;
}
.team-flag {
    width: 4px;
    height: 28px;
    background: var(--haas);
    border-radius: 2px;
}
.title {
    font-size: 13px;
    font-weight: 900;
    letter-spacing: 0.14em;
    text-transform: uppercase;
}
.subtitle {
    font-size: 10px;
    color: var(--muted);
    font-family: 'JetBrains Mono', monospace;
    margin-top: 2px;
}
.timing-hud {
    display: flex;
    align-items: center;
    gap: 20px;
    font-family: 'JetBrains Mono', monospace;
}
.time-unit {
    display: flex;
    flex-direction: column;
    align-items: flex-end;
}
.time-lbl {
    font-size: 8px;
    font-weight: 700;
    color: var(--muted);
    letter-spacing: 0.08em;
    text-transform: uppercase;
}
.time-val {
    font-size: 13px;
    font-weight: 800;
    color: #e2e8f0;
}
.controls {
    display: flex;
    gap: 6px;
    margin-left: 12px;
}
button {
    background: var(--panel-sub);
    color: var(--text);
    border: 1px solid var(--line);
    border-radius: 4px;
    padding: 6px 14px;
    font-size: 11px;
    font-weight: 700;
    font-family: 'JetBrains Mono', monospace;
    cursor: pointer;
    letter-spacing: 0.06em;
    transition: all 0.12s ease;
}
button:hover { border-color: var(--cyan); color: var(--cyan); }
button.active { background: var(--cyan); color: #000; border-color: var(--cyan); }
button.predict-btn { border-color: #3b82f6; color: #60a5fa; }
button.predict-btn:hover { background: #2563eb; color: #fff; }

.kpi-row {
    display: grid;
    grid-template-columns: repeat(7, 1fr);
    gap: 8px;
}
.kpi-card {
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 6px;
    padding: 8px 12px;
}
.kpi-label {
    font-size: 9px;
    color: var(--muted);
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    font-family: 'JetBrains Mono', monospace;
}
.kpi-val {
    font-size: 17px;
    font-weight: 800;
    font-family: 'JetBrains Mono', monospace;
    margin-top: 4px;
    color: #f8fafc;
}
.kpi-sub {
    font-size: 9px;
    color: var(--muted);
    font-family: 'JetBrains Mono', monospace;
    margin-top: 3px;
}
.decision-bar {
    background: var(--panel);
    border: 1px solid var(--line);
    border-left: 4px solid var(--cyan);
    border-radius: 6px;
    padding: 8px 14px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 11px;
}
.radio-msg {
    font-family: 'JetBrains Mono', monospace;
    color: var(--muted);
    margin-left: 8px;
}
.status-pill {
    font-family: 'JetBrains Mono', monospace;
    font-size: 10px;
    font-weight: 800;
    letter-spacing: 0.06em;
    padding: 3px 8px;
    border-radius: 4px;
    background: rgba(148, 163, 184, 0.1);
    color: var(--muted);
}
.split-view {
    display: grid;
    grid-template-columns: 1fr 1.6fr;
    gap: 10px;
    flex: 1;
    min-height: 0;
}
.viewport {
    position: relative;
    border: 1px solid var(--line);
    border-radius: 6px;
    background: var(--panel);
    overflow: hidden;
    display: flex;
    flex-direction: column;
}
.v-header {
    padding: 7px 12px;
    background: var(--panel-sub);
    border-bottom: 1px solid var(--line);
    font-size: 9px;
    font-weight: 800;
    letter-spacing: 0.1em;
    color: var(--muted);
    font-family: 'JetBrains Mono', monospace;
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-shrink: 0;
}
.canvas-holder {
    position: relative;
    flex: 1;
    width: 100%;
    height: 100%;
    min-height: 0;
    overflow: hidden;
}
#mapCanvas, #rvizCanvas {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    display: block;
}
.right-stack {
    display: grid;
    grid-template-rows: 1.15fr 1.05fr;
    gap: 10px;
    min-height: 0;
}
.table-wrap {
    flex: 1;
    overflow-y: auto;
    padding: 6px 10px;
}
table {
    width: 100%;
    border-collapse: separate;
    border-spacing: 0 3px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
}
th {
    text-align: left;
    color: var(--muted);
    padding: 6px 6px;
    font-size: 9px;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    border-bottom: 1px solid var(--line);
}
tbody tr {
    transition: transform 0.25s ease, background-color 0.2s ease;
    background: rgba(17, 22, 24, 0.4);
}
tbody tr.ego-row {
    background: rgba(0, 210, 190, 0.12) !important;
    border-left: 3px solid var(--cyan);
}
td {
    padding: 5px 6px;
    border-top: 1px solid rgba(28, 36, 39, 0.4);
    border-bottom: 1px solid rgba(28, 36, 39, 0.4);
}
td:first-child { border-left: 1px solid rgba(28, 36, 39, 0.4); border-radius: 4px 0 0 4px; }
td:last-child { border-right: 1px solid rgba(28, 36, 39, 0.4); border-radius: 0 4px 4px 0; }

.badge-tag {
    padding: 2px 6px;
    border-radius: 3px;
    font-weight: 800;
    font-size: 9px;
    letter-spacing: 0.04em;
    display: inline-block;
}
.adv-attack { background: rgba(0, 210, 190, 0.15); color: var(--cyan); border: 1px solid rgba(0, 210, 190, 0.4); }
.adv-hold { background: rgba(245, 158, 11, 0.15); color: var(--amber); border: 1px solid rgba(245, 158, 11, 0.4); }
.adv-defend { background: rgba(239, 68, 68, 0.15); color: var(--red); border: 1px solid rgba(239, 68, 68, 0.4); }
.adv-stable { background: rgba(100, 116, 139, 0.1); color: var(--muted); }
</style>
</head>
<body>
<div id="container">
    <div class="topbar">
        <div class="brand-group">
            <div class="team-flag"></div>
            <div>
                <div class="title">HAAS F1 // PIT-WALL DECISION CONSOLE</div>
                <div class="subtitle">CAR 50 (O. BEARMAN) | 2026 BRITISH GP | CIRCUIT GPS & TACTICAL MPC ENGINE</div>
            </div>
        </div>

        <div class="timing-hud">
            <div class="time-unit">
                <span class="time-lbl">Session Elapsed</span>
                <span class="time-val" id="timeSession">22:04.5</span>
            </div>
            <div class="time-unit">
                <span class="time-lbl">Lap</span>
                <span class="time-val" id="hudLap">L14 / 52</span>
            </div>
            <div class="time-unit">
                <span class="time-lbl">Current Lap</span>
                <span class="time-val" id="timeLap">00:38.45</span>
            </div>
            <div class="controls">
                <button id="btnPlay" onclick="cmd('PLAY')">PLAY</button>
                <button id="btnPause" class="active" onclick="cmd('PAUSE')">HOLD</button>
                <button id="btnReset" onclick="cmd('RESET')">RESET</button>
                <button class="predict-btn" onclick="cmd('PREDICT')">PREDICT (30S)</button>
            </div>
        </div>
    </div>

    <div class="kpi-row">
        <div class="kpi-card" style="border-top: 3px solid var(--cyan);">
            <div class="kpi-label">Speed & Powertrain</div>
            <div class="kpi-val" id="kpiSpeed">0.0 km/h</div>
            <div class="kpi-sub" id="kpiGear">GEAR 1 | THROTTLE 0%</div>
        </div>
        <div class="kpi-card" style="border-top: 3px solid var(--amber);">
            <div class="kpi-label">Rival Interval</div>
            <div class="kpi-val" id="kpiGap">+0.00 m</div>
            <div class="kpi-sub" id="kpiTarget">TARGET: NONE</div>
        </div>
        <div class="kpi-card" style="border-top: 3px solid var(--cyan);">
            <div class="kpi-label">Success Confidence</div>
            <div class="kpi-val" id="kpiConf">0.0%</div>
            <div class="kpi-sub" id="kpiRecTitle">MONITORING</div>
        </div>
        <div class="kpi-card" style="border-top: 3px solid var(--haas);">
            <div class="kpi-label">Energy Store (MGU-K)</div>
            <div class="kpi-val" id="kpiSoc">0.0%</div>
            <div class="kpi-sub" id="kpiDeploy">0.00 MJ / 8.50 MJ</div>
        </div>
        <div class="kpi-card" style="border-top: 3px solid var(--green);">
            <div class="kpi-label">Battery Thermal / SOH</div>
            <div class="kpi-val" id="kpiBattTherm">0.0°C</div>
            <div class="kpi-sub" id="kpiSoh">SOH 99.4% | LIMIT 58.0°C</div>
        </div>
        <div class="kpi-card" style="border-top: 3px solid var(--red);">
            <div class="kpi-label">Pirelli Tyre Matrix</div>
            <div class="kpi-val" id="kpiTire">0.0°C</div>
            <div class="kpi-sub" id="kpiDeg">WEAR 0.0% (C2 MED)</div>
        </div>
        <div class="kpi-card" style="border-top: 3px solid var(--muted);">
            <div class="kpi-label">Safety CBF Invariance</div>
            <div class="kpi-val" id="kpiMargin">15.00 m</div>
            <div class="kpi-sub" id="kpiCbf">BARRIER: SAFE</div>
        </div>
    </div>

    <div class="decision-bar" id="decisionBar">
        <div>
            <span id="decTitle" style="font-weight: 800; letter-spacing: 0.06em;">STANDSTILL / GRID PHASE</span>
            <span class="radio-msg" id="decRadio">Standing by for telemetry link...</span>
        </div>
        <div class="status-pill" id="badgeMode">PAUSED</div>
    </div>

    <div class="split-view">
        <div class="viewport">
            <div class="v-header">
                <span>SILVERSTONE GRAND PRIX CIRCUIT // GPS RADAR</span>
                <span style="color:var(--cyan);">5.891 KM</span>
            </div>
            <div class="canvas-holder">
                <canvas id="mapCanvas"></canvas>
            </div>
        </div>

        <div class="right-stack">
            <div class="viewport">
                <div class="v-header">
                    <span>LIVE TRACK RUNNING ORDER & OVERTAKE MATRIX</span>
                    <span id="targetCallBadge" style="color:var(--cyan);">TARGET: MONITORING</span>
                </div>
                <div class="table-wrap">
                    <table id="telemetryTable">
                        <thead>
                            <tr>
                                <th>Pos</th>
                                <th>Driver</th>
                                <th>Gap to Ego</th>
                                <th>Interval</th>
                                <th>Speed</th>
                                <th>Delta</th>
                                <th>Tyre</th>
                                <th>Override</th>
                                <th>Pass %</th>
                                <th>Tactical Action</th>
                            </tr>
                        </thead>
                        <tbody id="opponentRows">
                            <tr><td colspan="10" style="text-align:center; color:var(--muted); padding: 30px;">Establishing telemetry stream...</td></tr>
                        </tbody>
                    </table>
                </div>
            </div>

            <div class="viewport">
                <div class="v-header">
                    <span>2D RVIZ TACTICAL EGO-FRAME // FORWARD REACHABILITY & OVERLAP LINE</span>
                    <span style="color:var(--cyan);">SCALE ZOOM 1:1</span>
                </div>
                <div class="canvas-holder">
                    <canvas id="rvizCanvas"></canvas>
                </div>
            </div>
        </div>
    </div>
</div>

<script>
let ws = null;
let currentFrame = null;
let isPlaying = false;
let lastUiUpdate = 0;
let trackPoints = __EMBEDDED_TRACK__;

function connectWS() {
    const host = window.location.hostname || "localhost";
    ws = new WebSocket("ws://" + host + ":8765");
    ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === "TRACK_GEOMETRY" && msg.track && msg.track.length > 0) {
            trackPoints = msg.track;
        } else if (msg.type === "TELEMETRY_TICK") {
            currentFrame = msg;
            updateTelemetryUI(msg);
        }
    };
    ws.onclose = () => { setTimeout(connectWS, 1500); };
}
connectWS();

function cmd(action) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ cmd: action }));
    if (action === "PLAY") { isPlaying = true; document.getElementById("btnPlay").classList.add("active"); document.getElementById("btnPause").classList.remove("active"); }
    else if (action === "PAUSE") { isPlaying = false; document.getElementById("btnPause").classList.add("active"); document.getElementById("btnPlay").classList.remove("active"); }
}

function updateAnimatedTable(gridCars, targetCode) {
    const tbody = document.getElementById("opponentRows");
    if (!gridCars || gridCars.length === 0) return;

    const frag = document.createDocumentFragment();
    gridCars.forEach(car => {
        const isEgo = car.is_ego;
        const isTarget = car.code === targetCode;
        const gapSign = car.gap_m > 0 ? "+" : (car.gap_m < 0 ? "" : " ");
        const deltaSign = car.rel_speed_kmh > 0 ? "+" : "";
        const deltaColor = car.rel_speed_kmh > 0 ? "var(--cyan)" : (car.rel_speed_kmh < 0 ? "var(--red)" : "var(--muted)");
        const advClass = car.overtake_advisory.includes("ATTACK") ? "adv-attack" : 
                         (car.overtake_advisory.includes("HOLD") ? "adv-hold" : 
                         (car.overtake_advisory.includes("DEFEND") ? "adv-defend" : "adv-stable"));

        const row = document.createElement("tr");
        row.setAttribute("data-driver", car.code);
        if (isEgo) {
            row.className = "ego-row";
        } else if (isTarget) {
            row.style.background = "rgba(0, 210, 190, 0.08)";
        }

        const gapDisplay = isEgo ? '<span style="color:var(--cyan); font-weight:800;">--- (EGO)</span>' : (gapSign + car.gap_m.toFixed(1) + " m");
        const intDisplay = isEgo ? '<span style="color:var(--cyan); font-weight:800;">0.00s</span>' : (gapSign + car.gap_sec.toFixed(2) + "s");
        const deltaDisplay = isEgo ? '---' : (deltaSign + car.rel_speed_kmh.toFixed(1));

        row.innerHTML = `
            <td style="font-weight:800; color:${isEgo ? "var(--cyan)" : "var(--muted)"};">P${car.pos}</td>
            <td><b style="color:${car.color}">${car.code}</b> <span style="color:${isEgo ? "var(--text)" : "var(--muted)"}; font-size:10px;">${car.name}</span></td>
            <td style="font-weight:700;">${gapDisplay}</td>
            <td>${intDisplay}</td>
            <td>${car.speed_kmh.toFixed(0)} km/h</td>
            <td style="color:${deltaColor}; font-weight:700;">${deltaDisplay}</td>
            <td>${car.compound} <span style="color:var(--muted)">L${car.tyre_age}</span></td>
            <td>${car.drs_eligible ? "<span style='color:var(--green); font-weight:700;'>ACTIVE</span>" : "<span style='color:var(--muted)'>INACTIVE</span>"}</td>
            <td><b style="color:${car.pass_probability > 65 ? "var(--cyan)" : (car.pass_probability > 40 ? "var(--amber)" : "var(--muted)")}">${car.pass_probability.toFixed(0)}%</b></td>
            <td><span class="badge-tag ${advClass}">${car.overtake_advisory}</span></td>
        `;
        frag.appendChild(row);
    });

    tbody.innerHTML = "";
    tbody.appendChild(frag);
}

function updateTelemetryUI(frame) {
    const ego = frame.ego, rec = frame.recommendation;
    
    document.getElementById("decisionBar").style.borderLeftColor = rec.color;
    let modeText = "LIVE"; let modeStyle = "color:var(--cyan); background:rgba(0,210,190,0.1);";
    if (!isPlaying && !frame.prediction_mode) { modeText = "PAUSED"; modeStyle = "color:var(--muted); background:rgba(100,116,139,0.1);"; }
    else if (frame.prediction_mode) { modeText = "PREDICTION (30S)"; modeStyle = "color:var(--amber); background:rgba(245,158,11,0.1);"; }
    const pill = document.getElementById("badgeMode");
    pill.innerText = modeText;
    pill.setAttribute("style", modeStyle);

    const now = performance.now();
    if (now - lastUiUpdate > 80) {
        document.getElementById("timeSession").innerText = frame.session_time;
        document.getElementById("hudLap").innerText = "L" + frame.lap_num + " / 52";
        document.getElementById("timeLap").innerText = frame.lap_time;

        document.getElementById("kpiSpeed").innerText = ego.speed_kmh.toFixed(1) + " km/h";
        document.getElementById("kpiGear").innerText = "G" + ego.gear + " | THR " + ego.throttle_pct.toFixed(0) + "% | BRK " + ego.brake_pct.toFixed(0) + "%";
        document.getElementById("kpiGap").innerText = (rec.margin_m > 0 ? "+" : "") + rec.margin_m.toFixed(2) + " m";
        document.getElementById("kpiTarget").innerText = "TARGET: " + frame.target + " (" + rec.passing_side + ")";
        document.getElementById("kpiConf").innerText = rec.confidence.toFixed(1) + "%";
        document.getElementById("kpiRecTitle").innerText = rec.action;
        document.getElementById("kpiSoc").innerText = ego.soc.toFixed(1) + "%";
        document.getElementById("kpiDeploy").innerText = ego.lap_mj.toFixed(2) + " MJ / 8.50 MJ";
        document.getElementById("kpiBattTherm").innerText = ego.temp_core.toFixed(1) + "°C";
        document.getElementById("kpiSoh").innerText = "SOH " + ego.soh.toFixed(1) + "% | LIMIT 58.0°C";
        document.getElementById("kpiTire").innerText = ego.tire_temp.toFixed(1) + "°C";
        document.getElementById("kpiDeg").innerText = "WEAR " + ego.tire_deg.toFixed(1) + "% (C2 MED)";
        document.getElementById("kpiMargin").innerText = Math.abs(rec.margin_m).toFixed(2) + " m";
        document.getElementById("kpiCbf").innerText = rec.cbf_active ? ("CBF: " + rec.cbf_type) : "BARRIER: SAFE";
        document.getElementById("decTitle").innerText = rec.title + " [" + rec.passing_side + "]";
        document.getElementById("decRadio").innerText = rec.radio;
        document.getElementById("targetCallBadge").innerText = "ACTIVE TARGET: " + frame.target;

        updateAnimatedTable(frame.grid_order, frame.target);
        lastUiUpdate = now;
    }
}

// 1. LEFT VIEWPORT: SILVERSTONE GRAND PRIX (LOCKED ZOOM)
const mapCanvas = document.getElementById("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");

let cachedMapScale = null;
let cachedOffsetX = 0;
let cachedOffsetY = 0;

function drawSilverstoneMap() {
    const holder = mapCanvas.parentElement;
    const w = holder.clientWidth || 350;
    const h = holder.clientHeight || 500;

    if (mapCanvas.width !== w || mapCanvas.height !== h) {
        mapCanvas.width = w;
        mapCanvas.height = h;
        cachedMapScale = null;
    }

    mapCtx.fillStyle = "#070a0c";
    mapCtx.fillRect(0, 0, w, h);

    if (!trackPoints || trackPoints.length < 5) return;

    if (cachedMapScale === null) {
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        trackPoints.forEach(p => {
            if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
        });
        const pad = 36;
        cachedMapScale = Math.min((w - pad * 2) / (maxX - minX || 1), (h - pad * 2) / (maxY - minY || 1));
        cachedOffsetX = w / 2 - ((maxX + minX) / 2) * cachedMapScale;
        cachedOffsetY = h / 2 - ((maxY + minY) / 2) * cachedMapScale;
    }

    const mapScale = cachedMapScale;
    const mapOffsetX = cachedOffsetX;
    const mapOffsetY = cachedOffsetY;

    mapCtx.beginPath();
    trackPoints.forEach((p, idx) => {
        const tx = p.x * mapScale + mapOffsetX;
        const ty = p.y * mapScale + mapOffsetY;
        if (idx === 0) mapCtx.moveTo(tx, ty); else mapCtx.lineTo(tx, ty);
    });
    mapCtx.closePath();
    mapCtx.strokeStyle = "rgba(255,255,255,0.12)"; mapCtx.lineWidth = 12; mapCtx.stroke();
    mapCtx.strokeStyle = "#080c0d"; mapCtx.lineWidth = 7; mapCtx.stroke();
    mapCtx.strokeStyle = "rgba(0, 210, 190, 0.45)"; mapCtx.lineWidth = 1.4; mapCtx.stroke();

    const corners = [
        {name: "Abbey", s: 420.0}, {name: "The Loop", s: 1050.0},
        {name: "Wellington", s: 1550.0}, {name: "Brooklands", s: 2200.0},
        {name: "Copse", s: 3100.0}, {name: "Becketts", s: 3900.0},
        {name: "Hangar", s: 4500.0}, {name: "Stowe", s: 4950.0}, {name: "Club", s: 5500.0}
    ];
    mapCtx.font = "700 8px 'JetBrains Mono', monospace";
    mapCtx.fillStyle = "rgba(100, 116, 139, 0.6)";
    corners.forEach(c => {
        const idx = Math.floor((c.s / 5891.0) * trackPoints.length);
        const pt = trackPoints[idx] || trackPoints[0];
        mapCtx.fillText(c.name, pt.x * mapScale + mapOffsetX + 5, pt.y * mapScale + mapOffsetY + 3);
    });

    if (currentFrame && currentFrame.predicted_path && currentFrame.predicted_path.length > 0) {
        mapCtx.beginPath();
        currentFrame.predicted_path.forEach((pt, idx) => {
            const px = pt.x * mapScale + mapOffsetX;
            const py = pt.y * mapScale + mapOffsetY;
            if (idx === 0) mapCtx.moveTo(px, py); else mapCtx.lineTo(px, py);
        });
        mapCtx.strokeStyle = "rgba(245, 158, 11, 0.85)";
        mapCtx.lineWidth = 2.5;
        mapCtx.stroke();
    }

    if (currentFrame) {
        currentFrame.opponents.forEach(opp => {
            const ox = opp.x * mapScale + mapOffsetX;
            const oy = opp.y * mapScale + mapOffsetY;
            mapCtx.fillStyle = opp.color;
            mapCtx.beginPath();
            mapCtx.arc(ox, oy, 4, 0, Math.PI * 2);
            mapCtx.fill();
            mapCtx.fillStyle = "#cbd5e1";
            mapCtx.font = "700 8px 'JetBrains Mono', monospace";
            mapCtx.fillText(opp.code, ox + 6, oy + 3);
        });

        const ego = currentFrame.ego;
        const ex = ego.x * mapScale + mapOffsetX;
        const ey = ego.y * mapScale + mapOffsetY;
        mapCtx.save();
        mapCtx.fillStyle = "#ffffff";
        mapCtx.shadowColor = "#00d2be";
        mapCtx.shadowBlur = 10;
        mapCtx.beginPath();
        mapCtx.arc(ex, ey, 5.5, 0, Math.PI * 2);
        mapCtx.fill();
        mapCtx.shadowBlur = 0;

        mapCtx.fillStyle = "rgba(6, 8, 9, 0.9)";
        mapCtx.strokeStyle = "#00d2be";
        mapCtx.lineWidth = 1.2;
        mapCtx.fillRect(ex + 8, ey - 12, 28, 14);
        mapCtx.strokeRect(ex + 8, ey - 12, 28, 14);
        mapCtx.fillStyle = "#00d2be";
        mapCtx.font = "800 8px 'JetBrains Mono', monospace";
        mapCtx.textAlign = "center";
        mapCtx.fillText("BEA", ex + 22, ey - 2);
        mapCtx.restore();
    }
}

// 2. LOWER RIGHT: 2D RVIZ TOP-DOWN TACTICAL MPC (FACING RIGHT)
const rvizCanvas = document.getElementById("rvizCanvas");
const rvizCtx = rvizCanvas.getContext("2d");

function drawFormulaCar(ctx, x, y, angle, lengthPx, color, label) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);

    const w = lengthPx * 0.44;
    const l = lengthPx;

    ctx.fillStyle = "#1e293b";
    ctx.fillRect(-w/2, -l/2, w, l * 0.12);
    ctx.fillRect(-w * 0.46, l * 0.38, w * 0.92, l * 0.12);

    const tw = w * 0.22, th = l * 0.24;
    ctx.fillStyle = "#020617";
    ctx.fillRect(-w/2 - tw * 0.6, -l/2 + l * 0.08, tw, th);
    ctx.fillRect( w/2 - tw * 0.4, -l/2 + l * 0.08, tw, th);
    ctx.fillRect(-w/2 - tw * 0.6,  l/2 - l * 0.32, tw, th);
    ctx.fillRect( w/2 - tw * 0.4,  l/2 - l * 0.32, tw, th);

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(0, -l/2 + l * 0.04);
    ctx.lineTo(w * 0.24, -l * 0.15);
    ctx.lineTo(w * 0.36, l * 0.1);
    ctx.lineTo(w * 0.22, l * 0.38);
    ctx.lineTo(-w * 0.22, l * 0.38);
    ctx.lineTo(-w * 0.36, l * 0.1);
    ctx.lineTo(-w * 0.24, -l * 0.15);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,0.4)";
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.fillStyle = "#0f172a";
    ctx.beginPath();
    ctx.ellipse(0, 0, w * 0.12, l * 0.1, 0, 0, Math.PI * 2);
    ctx.fill();

    if (label) {
        ctx.rotate(-angle);
        ctx.fillStyle = "rgba(6, 8, 9, 0.88)";
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.2;
        ctx.fillRect(-18, -l/2 - 18, 36, 14);
        ctx.strokeRect(-18, -l/2 - 18, 36, 14);
        ctx.fillStyle = color;
        ctx.font = "800 9px 'JetBrains Mono', monospace";
        ctx.textAlign = "center";
        ctx.fillText(label, 0, -l/2 - 7);
    }

    ctx.restore();
}

function drawRvizCanvas() {
    const holder = rvizCanvas.parentElement;
    const w = holder.clientWidth || 400;
    const h = holder.clientHeight || 280;

    if (rvizCanvas.width !== w || rvizCanvas.height !== h) {
        rvizCanvas.width = w;
        rvizCanvas.height = h;
    }

    rvizCtx.fillStyle = "#06090a";
    rvizCtx.fillRect(0, 0, w, h);

    if (!currentFrame) return;

    const ego = currentFrame.ego;
    const originX = w * 0.20;
    const originY = h * 0.50;
    const pxPerMeter = 11.0;

    const cosPsi = Math.cos(ego.heading);
    const sinPsi = Math.sin(ego.heading);

    function toEgoFrame(gx, gy) {
        const dx = gx - ego.x;
        const dy = gy - ego.y;
        const dLong = dx * cosPsi + dy * sinPsi;
        const dLat = -dx * sinPsi + dy * cosPsi;
        return {
            screenX: originX + dLong * pxPerMeter,
            screenY: originY + dLat * pxPerMeter
        };
    }

    rvizCtx.strokeStyle = "rgba(28, 36, 39, 0.55)";
    rvizCtx.lineWidth = 1;
    const gridStep = 5 * pxPerMeter;
    for (let x = originX % gridStep; x < w; x += gridStep) {
        rvizCtx.beginPath(); rvizCtx.moveTo(x, 0); rvizCtx.lineTo(x, h); rvizCtx.stroke();
    }
    for (let y = originY % gridStep; y < h; y += gridStep) {
        rvizCtx.beginPath(); rvizCtx.moveTo(0, y); rvizCtx.lineTo(w, y); rvizCtx.stroke();
    }

    rvizCtx.strokeStyle = "rgba(42, 55, 59, 0.4)";
    [10, 20, 30, 40].forEach(r => {
        rvizCtx.beginPath();
        rvizCtx.arc(originX, originY, r * pxPerMeter, -Math.PI / 2, Math.PI / 2);
        rvizCtx.stroke();
        rvizCtx.fillStyle = "rgba(100, 116, 139, 0.6)";
        rvizCtx.font = "800 8px 'JetBrains Mono', monospace";
        rvizCtx.fillText("+" + r + "m", originX + r * pxPerMeter - 12, originY - 6);
    });

    if (trackPoints && trackPoints.length > 5) {
        rvizCtx.beginPath();
        trackPoints.forEach((p, idx) => {
            const sc = toEgoFrame(p.x, p.y);
            if (idx === 0) rvizCtx.moveTo(sc.screenX, sc.screenY);
            else rvizCtx.lineTo(sc.screenX, sc.screenY);
        });
        rvizCtx.strokeStyle = "rgba(255, 255, 255, 0.08)";
        rvizCtx.lineWidth = 13.5 * pxPerMeter;
        rvizCtx.stroke();

        rvizCtx.strokeStyle = "rgba(0, 210, 190, 0.35)";
        rvizCtx.lineWidth = 1.8;
        rvizCtx.stroke();
    }

    if (currentFrame.mpc_candidates && currentFrame.mpc_candidates.length > 0) {
        currentFrame.mpc_candidates.forEach(cand => {
            rvizCtx.beginPath();
            cand.points.forEach((p, idx) => {
                const sc = toEgoFrame(p.x, p.y);
                if (idx === 0) rvizCtx.moveTo(sc.screenX, sc.screenY);
                else rvizCtx.lineTo(sc.screenX, sc.screenY);
            });
            
            if (cand.optimal) {
                rvizCtx.strokeStyle = "rgba(0, 210, 190, 0.95)";
                rvizCtx.lineWidth = 3.6;
                rvizCtx.stroke();

                cand.points.forEach((p, pidx) => {
                    if (pidx % 3 === 0) {
                        const sc = toEgoFrame(p.x, p.y);
                        rvizCtx.fillStyle = "#ffffff";
                        rvizCtx.beginPath();
                        rvizCtx.arc(sc.screenX, sc.screenY, 2.5, 0, Math.PI * 2);
                        rvizCtx.fill();
                    }
                });
            } else {
                rvizCtx.strokeStyle = "rgba(100, 116, 139, 0.3)";
                rvizCtx.lineWidth = 1.5;
                rvizCtx.setLineDash([4, 4]);
                rvizCtx.stroke();
                rvizCtx.setLineDash([]);
            }
        });
    }

    currentFrame.opponents.forEach(opp => {
        const sc = toEgoFrame(opp.x, opp.y);
        if (sc.screenX < -40 || sc.screenX > w + 40) return;

        const relAngle = -(opp.heading - ego.heading) + Math.PI / 2;
        const aSafe = (5.0 + 0.08 * (opp.speed_kmh / 3.6)) * pxPerMeter;
        const bSafe = 2.4 * pxPerMeter;

        rvizCtx.save();
        rvizCtx.translate(sc.screenX, sc.screenY);
        rvizCtx.rotate(relAngle);
        rvizCtx.beginPath();
        rvizCtx.ellipse(0, 0, bSafe, aSafe, 0, 0, Math.PI * 2);
        const isTarget = opp.code === currentFrame.target;
        rvizCtx.strokeStyle = isTarget ? "rgba(245, 158, 11, 0.9)" : "rgba(239, 68, 68, 0.4)";
        rvizCtx.lineWidth = 1.2;
        rvizCtx.fillStyle = isTarget ? "rgba(245, 158, 11, 0.1)" : "rgba(239, 68, 68, 0.05)";
        rvizCtx.fill();
        rvizCtx.stroke();
        rvizCtx.restore();

        const carLenPx = 5.4 * pxPerMeter;
        drawFormulaCar(rvizCtx, sc.screenX, sc.screenY, relAngle, carLenPx, opp.color, opp.code);
    });

    const egoCarLenPx = 5.4 * pxPerMeter;
    drawFormulaCar(rvizCtx, originX, originY, Math.PI / 2, egoCarLenPx, "#cfd4d5", "BEA");
}

function animate() {
    drawSilverstoneMap();
    drawRvizCanvas();
    requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
</script>
</body>
</html>
"""
    final_html = raw_html.replace("__EMBEDDED_TRACK__", embedded_track)
    components.html(final_html, height=height, scrolling=False)