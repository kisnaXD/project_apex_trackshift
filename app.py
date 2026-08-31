"""APEX demo API: one stint session per visitor, real CBF-QP on every step."""

from __future__ import annotations

from pathlib import Path
from uuid import uuid4

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from starlette.responses import FileResponse

ROOT = Path(__file__).resolve().parent

from apex.stint import StintSim

SESSIONS: dict[str, StintSim] = {}
MAX_SESSIONS = 250

app = FastAPI(title="APEX", version="1.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class StepBody(BaseModel):
    seconds: float = Field(ge=0.05, le=120.0)


class InputBody(BaseModel):
    opponent_aggression: float = Field(ge=0.0, le=1.0)
    push_bias: float = Field(ge=0.0, le=1.0)


def _put(sim: StintSim) -> str:
    if len(SESSIONS) >= MAX_SESSIONS:
        SESSIONS.pop(next(iter(SESSIONS)))
    sid = uuid4().hex
    SESSIONS[sid] = sim
    return sid


def _sim(sid: str) -> StintSim:
    sim = SESSIONS.get(sid)
    if sim is None:
        raise HTTPException(status_code=404, detail="session expired")
    return sim


@app.get("/api/health")
def health() -> dict:
    return {"ok": True, "engine": "apex-cbf-qp"}


@app.post("/api/session")
def create_session() -> dict:
    sid = _put(StintSim())
    body = SESSIONS[sid].snapshot()
    body["session_id"] = sid
    return body


@app.get("/api/session/{sid}")
def get_session(sid: str) -> dict:
    body = _sim(sid).snapshot()
    body["session_id"] = sid
    return body


@app.post("/api/session/{sid}/reset")
def reset_session(sid: str) -> dict:
    body = _sim(sid).reset()
    body["session_id"] = sid
    return body


@app.post("/api/session/{sid}/step")
def step_session(sid: str, body: StepBody) -> dict:
    snap = _sim(sid).step(body.seconds)
    snap["session_id"] = sid
    return snap


@app.post("/api/session/{sid}/inputs")
def set_inputs(sid: str, body: InputBody) -> dict:
    sim = _sim(sid)
    sim.set_inputs(body.opponent_aggression, body.push_bias)
    snap = sim.snapshot()
    snap["session_id"] = sid
    return snap


@app.get("/")
def index() -> FileResponse:
    return FileResponse(ROOT / "static" / "index.html")


app.mount("/static", StaticFiles(directory=str(ROOT / "static")), name="static")

# Vercel Python looks for this name on some runtimes.
handler = app
