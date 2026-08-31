from fastapi.testclient import TestClient

from app import app


def test_index_page_is_served():
    client = TestClient(app)
    page = client.get("/")
    assert page.status_code == 200
    assert "APEX" in page.text


def test_session_step_runs_real_qp_solve():
    client = TestClient(app)
    created = client.post("/api/session")
    assert created.status_code == 200
    sid = created.json()["session_id"]
    stepped = client.post(f"/api/session/{sid}/step", json={"seconds": 2.0})
    body = stepped.json()
    assert body["cbf"]["solver_status"] == "solved"
    assert body["cbf"]["solve_time_ms"] > 0.0
    assert body["state"]["time_s"] == 2.0
    assert "qp" in body["cbf"]
