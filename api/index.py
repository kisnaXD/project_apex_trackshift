"""Vercel Python entrypoint. Re-exports the FastAPI app from the repo root."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
root = str(ROOT)
if root not in sys.path:
    sys.path.insert(0, root)

from app import app  # noqa: E402
