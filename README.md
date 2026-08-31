# APEX — AI Motorsport Intelligence

Interactive prototype of a real-time EV racing decision engine: a two-state electro-thermal pack model, an Overtake Risk-Reward Index (`R_OT`), and a Control Barrier Function quadratic program that filters power so core temperature and the per-lap energy quota cannot be breached.

This is **not** a claim that the full four-tier architecture is implemented. The CBF-QP, pack physics, and `R_OT` run for real. Shadow prices `λ_E` / `λ_T` are a **rule-based stand-in** for the PPO strategic layer. Multi-agent ADMM is not built.

## Run locally

```bash
python -m pip install -r requirements.txt
python -m uvicorn app:app --reload --port 8000
```

Open http://127.0.0.1:8000

```bash
python -m pytest -q
```

## What is real

- Two-state thermal pack: `C_c Ṫ_c = I²R − (T_c−T_s)/R_cs`, `C_s Ṫ_s = (T_c−T_s)/R_cs − (T_s−T_amb)/R_sa`
- SOC and lap energy from integrated terminal power
- CBF-QP solved by **OSQP** every step: `min ½(P−P_req)²` subject to linearized thermal and energy barrier inequalities
- `R_OT` from five live-state factors (clean-air gain, energy, thermal, tire, collision)

Push the driver slider while the pack is hot: requested power rises, the QP clips, `T_core` stays under 60 °C.

## Deploy (public URL)

### Vercel

This repo includes `vercel.json` so Vercel runs the FastAPI app in `app.py` (not a static export). Connect the GitHub repo as a Vercel project; the FastAPI preset and Python 3.12 are set in-repo.

Root directory: repo root. Python `3.12`. After deploy, the live site should serve `/` and `/api/health`.

### Render / Railway / Fly

1. Push this folder to a GitHub repo.
2. New Web Service → Docker, or native Python with `uvicorn app:app --host 0.0.0.0 --port $PORT`.
3. Health check: `/api/health`.
