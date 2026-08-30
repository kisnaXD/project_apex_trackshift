const $ = (id) => document.getElementById(id);

const state = {
  sessionId: null,
  playing: false,
  timer: null,
  chart: null,
  inflight: false,
};

const MODE_LABEL = {
  harvest: "HARVEST & COOL",
  nominal: "NOMINAL PACE",
  attack: "ATTACK",
};

const CHART_POINTS = 80;

function fmt(n, d = 1) {
  return Number(n).toFixed(d);
}

function pct(n) {
  return `${(100 * n).toFixed(1)}%`;
}

async function api(path, opts) {
  const res = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...opts,
  });
  if (!res.ok) throw new Error(`${res.status} ${path}`);
  return res.json();
}

function rotToAngle(r) {
  const clamped = Math.min(3, Math.max(0, r));
  return Math.PI * (1 - clamped / 3);
}

function paintGauge(r) {
  const needle = $("needle");
  const a = rotToAngle(r);
  needle.setAttribute("x2", String(120 + 88 * Math.cos(a)));
  needle.setAttribute("y2", String(120 - 88 * Math.sin(a)));
  const frac = Math.min(3, Math.max(0, r)) / 3;
  $("arc-fill").style.strokeDasharray = `${314 * frac} 314`;
}

function factorRow(label, value, max) {
  const width = Math.min(100, (100 * value) / max);
  return `<div class="factor"><span>${label}</span><b>${fmt(value, 2)}</b><div class="track"><div class="fill" style="width:${width}%"></div></div></div>`;
}

function render(snap) {
  const rot = snap.rot;
  const cbf = snap.cbf;
  const st = snap.state;
  const p = snap.strategy;

  $("rot-value").textContent = fmt(rot.r_ot, 2);
  const pill = $("mode-pill");
  pill.textContent = MODE_LABEL[rot.mode];
  pill.className = `mode ${rot.mode}`;
  paintGauge(rot.r_ot);

  $("factors").innerHTML = [
    factorRow("Clean-air time gained", rot.clean_air_benefit, 2.2),
    factorRow("Energy cost", rot.energy_cost, 2.0),
    factorRow("Thermal derate cost", rot.thermal_cost, 2.0),
    factorRow("Tire / off-line cost", rot.tire_cost, 1.2),
    factorRow("Collision / fail risk", rot.collision_cost, 1.4),
  ].join("");

  $("p-req").textContent = `${fmt(cbf.p_requested / 1000, 0)} kW`;
  $("p-safe").textContent = `${fmt(cbf.p_safe / 1000, 0)} kW`;
  $("bar-req").style.width = `${Math.min(100, (100 * cbf.p_requested) / 350000)}%`;
  $("bar-safe").style.width = `${Math.min(100, (100 * cbf.p_safe) / 350000)}%`;

  const badge = $("clip-badge");
  if (cbf.clipped) {
    const why = cbf.active_constraints.filter((c) => c !== "p_max").join(" + ") || "CBF";
    badge.textContent = `CLIPPED · ${why.toUpperCase()}`;
    badge.className = "badge clip";
  } else {
    badge.textContent = "PASSED";
    badge.className = "badge pass";
  }
  $("solver").textContent = `OSQP ${cbf.solver_status} · ${fmt(cbf.solve_time_ms, 2)} ms`;

  $("soc").textContent = pct(st.soc);
  $("tcore").textContent = `${fmt(st.t_core_c, 2)} °C`;
  $("tsurf").textContent = `${fmt(st.t_surf_c, 2)} °C`;
  $("quota").textContent = `${pct(st.e_used_lap_j / snap.limits.e_quota_lap_j)} used`;
  $("gap").textContent = `${fmt(st.gap_m, 1)} m`;
  $("tires").textContent = pct(st.tire_wear);
  $("clock").textContent = `L${st.lap} · ${fmt(st.time_s, 1)}s · ${fmt(st.t_core_c, 1)}°C · ${pct(st.soc)}`;

  $("strategy-note").textContent = p.note;
  const qp = cbf.qp;
  $("qp-box").textContent = [
    `min  ½ (P − ${fmt(cbf.p_requested, 0)})²`,
    `s.t. P ≤ ${fmt(cbf.thermal_bound_w, 0)} thermal`,
    `     P ≤ ${fmt(cbf.energy_bound_w, 0)} energy`,
    `     P ≤ ${fmt(qp.soc_bound_w || 0, 0)} SOC`,
    `P* = ${fmt(cbf.p_safe, 1)} W`,
    `active: ${cbf.active_constraints.join(", ") || "none"}`,
    `λ_E = ${fmt(p.lambda_e, 2)}  λ_T = ${fmt(p.lambda_t, 2)}`,
  ].join("\n");

  updateChart(snap.history);
}

function buildChart() {
  const canvas = $("chart");
  canvas.height = 220;
  state.chart = new Chart(canvas.getContext("2d"), {
    type: "line",
    data: {
      labels: [],
      datasets: [
        { label: "SOC %", data: [], borderColor: "#f3f3f3", yAxisID: "y1", tension: 0, pointRadius: 0, borderWidth: 1.5 },
        { label: "T_core °C", data: [], borderColor: "#e10600", yAxisID: "y2", tension: 0, pointRadius: 0, borderWidth: 1.5 },
        { label: "R_OT", data: [], borderColor: "#8d8d8d", yAxisID: "y3", tension: 0, pointRadius: 0, borderWidth: 1.2, borderDash: [4, 3] },
      ],
    },
    options: {
      animation: false,
      responsive: false,
      maintainAspectRatio: false,
      plugins: {
        legend: { labels: { color: "#bdbdbd", boxWidth: 10, font: { family: "IBM Plex Mono", size: 10 } } },
      },
      scales: {
        x: { ticks: { color: "#6a6a6a", maxTicksLimit: 6, font: { size: 10 } }, grid: { color: "#1c1c1c" } },
        y1: { position: "left", min: 0, max: 100, ticks: { color: "#f3f3f3", maxTicksLimit: 5 }, grid: { color: "#1c1c1c" } },
        y2: { position: "right", min: 30, max: 62, ticks: { color: "#e10600", maxTicksLimit: 5 }, grid: { drawOnChartArea: false } },
        y3: { display: false, min: 0, max: 6 },
      },
    },
  });
}

function downsample(arr, n) {
  if (arr.length <= n) return arr;
  const step = (arr.length - 1) / (n - 1);
  const out = [];
  for (let i = 0; i < n; i += 1) out.push(arr[Math.round(i * step)]);
  return out;
}

function updateChart(hist) {
  if (!state.chart) return;
  const labels = downsample(hist.t_s, CHART_POINTS);
  state.chart.data.labels = labels.map((t) => fmt(t, 0));
  state.chart.data.datasets[0].data = downsample(hist.soc, CHART_POINTS).map((s) => 100 * s);
  state.chart.data.datasets[1].data = downsample(hist.t_core_c, CHART_POINTS);
  state.chart.data.datasets[2].data = downsample(hist.r_ot, CHART_POINTS);
  state.chart.update("none");
}

async function createSession() {
  const snap = await api("/api/session", { method: "POST" });
  state.sessionId = snap.session_id;
  render(snap);
}

async function step(seconds) {
  if (state.inflight || !state.sessionId) return;
  state.inflight = true;
  try {
    const snap = await api(`/api/session/${state.sessionId}/step`, {
      method: "POST",
      body: JSON.stringify({ seconds }),
    });
    render(snap);
    return snap;
  } finally {
    state.inflight = false;
  }
}

async function sendInputs() {
  const agg = Number($("agg").value) / 100;
  const push = Number($("push").value) / 100;
  $("agg-lbl").textContent = fmt(agg, 2);
  $("push-lbl").textContent = fmt(push, 2);
  if (!state.sessionId || state.inflight) return;
  const snap = await api(`/api/session/${state.sessionId}/inputs`, {
    method: "POST",
    body: JSON.stringify({ opponent_aggression: agg, push_bias: push }),
  });
  render(snap);
}

function setPlaying(on) {
  state.playing = on;
  $("btn-play").textContent = on ? "Pause" : "Play";
  if (state.timer) {
    clearInterval(state.timer);
    state.timer = null;
  }
  if (on) {
    state.timer = setInterval(() => {
      if (state.inflight) return;
      const speed = Number($("speed").value);
      step(Math.min(8, Math.max(1, speed / 2)));
    }, 400);
  }
}

function bind() {
  $("btn-play").onclick = () => setPlaying(!state.playing);
  $("btn-step").onclick = () => step(1);
  $("btn-lap").onclick = async () => {
    const snap = await step(92);
    tourNotify("lap", snap);
  };
  $("btn-reset").onclick = async () => {
    setPlaying(false);
    if (!state.sessionId) return;
    const snap = await api(`/api/session/${state.sessionId}/reset`, { method: "POST" });
    render(snap);
  };
  $("speed").oninput = () => {
    $("speed-lbl").textContent = `${$("speed").value}×`;
  };
  let inputTimer = null;
  const onSlide = () => {
    clearTimeout(inputTimer);
    inputTimer = setTimeout(sendInputs, 80);
    $("agg-lbl").textContent = fmt(Number($("agg").value) / 100, 2);
    $("push-lbl").textContent = fmt(Number($("push").value) / 100, 2);
    tourNotify("push", Number($("push").value) / 100);
  };
  $("agg").oninput = onSlide;
  $("push").oninput = onSlide;
}

function sizeChart() {
  const wrap = document.querySelector(".chart-wrap");
  const canvas = $("chart");
  if (!wrap || !canvas || !state.chart) return;
  const w = Math.max(200, Math.floor(wrap.clientWidth));
  const h = Math.max(160, Math.floor(wrap.clientHeight || 220));
  if (canvas.width === w && canvas.height === h) return;
  canvas.width = w;
  canvas.height = h;
  state.chart.resize();
}

buildChart();
sizeChart();
window.addEventListener("resize", () => {
  clearTimeout(state.resizeTimer);
  state.resizeTimer = setTimeout(sizeChart, 150);
});
bind();
createSession()
  .then(() => {
    if (!sessionStorage.getItem("apex-tour-done") || new URLSearchParams(location.search).has("tour")) {
      startTour();
    }
  })
  .catch((err) => {
    $("qp-box").textContent = `Backend unreachable: ${err.message}`;
  });

const TOUR_STEPS = [
  {
    title: "A pretend race, a real decision",
    body: "This is one electric race car. APEX keeps asking: should we overtake, hold pace, or save the battery? A safety lock sits on top so the pack cannot get too hot or too empty, no matter how hard you push.",
    target: null,
    next: "Show me",
  },
  {
    title: "Should we overtake?",
    body: "The big number is a simple score. Over 1.5 means ATTACK — go for it. Under 1 means back off and cool down. It starts high because the car ahead is close and our battery is still fresh.",
    target: "tour-rot",
  },
  {
    title: "Two power numbers",
    body: "Requested is what the driver asked for. Allowed is what APEX actually lets through. If they match, you are fine. If Allowed is smaller, the safety lock just said no.",
    target: "tour-power",
  },
  {
    title: "Run one lap",
    body: "Click the glowing Step lap button. Watch battery % fall and temperature rise. That happens because the car used power — nobody typed those numbers in.",
    target: "btn-lap",
    wait: "lap",
    next: "Waiting…",
  },
  {
    title: "One story, three lines",
    body: "White is how full the battery is. Red is how hot the pack is. They move together: more power means emptier and hotter. That is the whole simulation in one picture.",
    target: "tour-chart",
  },
  {
    title: "Ask for everything",
    body: "Drag Driver push all the way to the right. You are telling the car you want maximum power. The lock still gets the last word.",
    target: "tour-push",
    wait: "push",
    next: "Waiting…",
  },
  {
    title: "Watch for CLIPPED",
    body: "Click Step lap twice more. After a few hard laps the pack gets hot. Look for the word CLIPPED — that means APEX cut the power so the battery cannot pass 60°C. You cannot break it from here. That is the point.",
    target: "btn-lap",
    wait: "laps2",
    next: "Waiting…",
  },
  {
    title: "That’s the idea",
    body: "Play runs the race live. Reset starts over. Open this guide any time with How this works.",
    target: null,
    next: "Got it",
  },
];

const tour = { i: 0, open: false, laps: 0 };

function tourNotify(kind, payload) {
  if (!tour.open) return;
  const step = TOUR_STEPS[tour.i];
  if (!step) return;
  if (kind === "lap" && step.wait === "lap") {
    tourNext();
    return;
  }
  if (kind === "lap" && step.wait === "laps2") {
    tour.laps += 1;
    if (payload && payload.cbf && payload.cbf.clipped) {
      $("tour-body").textContent = `${step.body} There it is — CLIPPED. The lock is working.`;
    }
    if (tour.laps >= 2) tourNext();
    return;
  }
  if (kind === "push" && step.wait === "push" && payload >= 0.85) tourNext();
}

function startTour() {
  tour.i = 0;
  tour.open = true;
  tour.laps = 0;
  $("tour").classList.remove("hidden");
  document.body.classList.add("touring");
  showTourStep();
}

function endTour() {
  tour.open = false;
  $("tour").classList.add("hidden");
  $("tour").classList.remove("has-spot");
  document.body.classList.remove("touring");
  document.querySelectorAll(".tour-hit").forEach((el) => el.classList.remove("tour-hit"));
  $("tour-spot").hidden = true;
  $("tour-spot").classList.remove("pulse");
  sessionStorage.setItem("apex-tour-done", "1");
}

function tourNext() {
  if (tour.i >= TOUR_STEPS.length - 1) {
    endTour();
    return;
  }
  tour.i += 1;
  showTourStep();
}

function showTourStep() {
  const step = TOUR_STEPS[tour.i];
  $("tour-step").textContent = `${tour.i + 1} / ${TOUR_STEPS.length}`;
  $("tour-title").textContent = step.title;
  $("tour-body").textContent = step.body;
  const next = $("tour-next");
  next.textContent = step.next || "Next";
  next.disabled = Boolean(step.wait);
  next.style.visibility = step.wait ? "hidden" : "visible";

  document.querySelectorAll(".tour-hit").forEach((el) => el.classList.remove("tour-hit"));
  const target = step.target ? $(step.target) : null;
  const spot = $("tour-spot");
  if (target) {
    target.classList.add("tour-hit");
    target.scrollIntoView({ block: "nearest", behavior: "auto" });
    const r = target.getBoundingClientRect();
    const pad = 8;
    spot.hidden = false;
    spot.style.top = `${r.top - pad}px`;
    spot.style.left = `${r.left - pad}px`;
    spot.style.width = `${r.width + pad * 2}px`;
    spot.style.height = `${r.height + pad * 2}px`;
    spot.classList.toggle("pulse", Boolean(step.wait));
    $("tour").classList.add("has-spot");
    placeTourCard(r);
  } else {
    spot.hidden = true;
    spot.classList.remove("pulse");
    $("tour").classList.remove("has-spot");
    const card = $("tour-card");
    card.style.top = "50%";
    card.style.left = "50%";
    card.style.right = "auto";
    card.style.bottom = "auto";
    card.style.transform = "translate(-50%, -50%)";
  }
}

function placeTourCard(r) {
  const card = $("tour-card");
  const gap = 14;
  const width = Math.min(360, window.innerWidth - 24);
  let top = r.bottom + gap;
  let left = Math.min(r.left, window.innerWidth - width - 12);
  left = Math.max(12, left);
  card.style.transform = "none";
  card.style.right = "auto";
  card.style.bottom = "auto";
  card.style.left = `${left}px`;
  card.style.top = `${top}px`;
  requestAnimationFrame(() => {
    const cr = card.getBoundingClientRect();
    if (cr.bottom > window.innerHeight - 8) {
      card.style.top = `${Math.max(12, r.top - cr.height - gap)}px`;
    }
  });
}

$("tour-next").onclick = () => tourNext();
$("tour-skip").onclick = () => endTour();
$("btn-tour").onclick = () => startTour();
window.addEventListener("resize", () => {
  if (tour.open) showTourStep();
});
