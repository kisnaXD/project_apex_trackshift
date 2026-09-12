# APEX-AI // Deterministic Multi-Agent Motorsport Decision Engine
### High-Performance Autonomous Racing Telemetry & Supervisory Overtake Arbitration

[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/docker-containerized-2496ED.svg)](https://www.docker.com/)
[![Streamlit](https://img.shields.io/badge/UI-Streamlit-FF4B4B.svg)](https://streamlit.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/mit)

**APEX-AI** is a real-time, multi-agent autonomous decision-making engine built for Formula Student Driverless (FSD) and competitive closed-circuit racing. The system combines high-speed perception, game-theoretic multi-agent negotiation, Lyapunov-stable reinforcement learning co-states, and a deterministic Control Barrier Function Quadratic Program (CBF-QP) to guarantee absolute collision avoidance and electro-thermal battery safety under 200 Hz execution loops.

---

## 🏎️ Key Architecture & Modules

1. **Onboard Perception EKF (`engine/perception.py`)**: 
   * Estimates relative kinematic states ($s, y, v_s, \dot{y}$) from noisy polar range and azimuth sensor measurements.
   * Infers rival defensive behavioral aggression ($\theta_{\text{agg}}$) in real time.
2. **Dynamic Reachability Hyperplane (`engine/reachability.py`)**:
   * Computes forward reachable sets (Ellipsoidal Hamilton-Jacobi reachability bubbles) to bound opponent maneuver uncertainties.
3. **Strategic RL Governor (`engine/rl_governor.py`)**:
   * Solves asynchronous Hamilton-Jacobi-Bellman co-states ($\lambda_E, \lambda_T$) balancing stint energy margins ($\Delta\text{SOC}$) against thermal limits.
4. **Supervisory Overtake Arbitrator ($\mathcal{R}_{OT}$) (`engine/supervisory_rot.py`)**:
   * Evaluates a 5-term real-time trade-off ratio (Clean-Air Delta vs. Energy Cost, Thermal Spike, Tire Scrub, and Pass Risk) to dynamically transition between **Mode 1 (Slipstream Harvest)**, **Mode 2 (Nominal Pace)**, and **Mode 3 (Tactical Attack)**.
5. **Deterministic Active-Set CBF-QP (`engine/cbf_safety.py`)**:
   * Enforces forward invariance on battery cell temperature ($T_{\text{core}} \le 58.0^\circ\text{C}$) and collision avoidance by clamping inverter power demands via Quadratic Programming within sub-5 ms execution windows.
6. **Silverstone GP Circuit (`engine/silverstone_track.py`)**:
   * Closed-loop parametric cubic spline interpolation of the 5.891 km Grand Prix circuit (Copse, Maggotts, Becketts, Hangar Straight, Stowe, Club) with Frenet-to-Cartesian coordinate mapping.

---

## 🛠️ Project Structure

```text
Apex_2D/
├── app.py                   # Main Streamlit pitwall telemetry dashboard & 60 FPS HTML5 Canvas loop
├── Dockerfile               # Container build recipe
├── docker-compose.yml       # Container orchestration configuration
├── requirements.txt         # Python dependencies
└── engine/
    ├── __init__.py
    ├── perception.py        # Extended Kalman Filter (EKF)
    ├── reachability.py      # Reachability bubble forecasting
    ├── tactical_planner.py  # ADMM / Unilateral trajectory generation
    ├── rl_governor.py       # Asynchronous co-state RL governor
    ├── supervisory_rot.py   # Supervisory Overtake Index (R_OT)
    ├── cbf_safety.py        # Active-Set Control Barrier Function (CBF-QP)
    └── silverstone_track.py # 5.891 km Silverstone GP parametric map
```

## 🚀 Quickstart & Installation (Via Docker)

The entire simulation runs inside a fully containerized Docker environment with automated dependency resolution.

### Prerequisites
* [Docker Desktop](https://www.docker.com/products/docker-desktop/) (or Docker Engine + Docker Compose) installed on your machine.

### Step 1: Build and Run with Docker Compose
```bash
docker compose up --build
```
### Step 2: Access the Pitwall Dashboard
```bash
http://localhost:8501
```
### Step 3: Shutting Down
```bash
docker compose down
```
