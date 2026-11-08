import pytest
from fastapi.testclient import TestClient

from app.main import create_app


@pytest.fixture()
def client():
    app = create_app(database_url="sqlite:///:memory:", autostart=False, seed=42, log=False, ml_samples=1500)
    with TestClient(app) as c:
        yield c


def step(client, n=1):
    r = client.post("/simulation/control", json={"action": "step", "steps": n})
    assert r.status_code == 200
    return r.json()
