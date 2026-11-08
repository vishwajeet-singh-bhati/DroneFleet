import json

import pytest

from app.ai.interpreter import (
    Interpretation, LLMInterpreter, MissionInterpreter, ZoneInfo, interpret_local,
    parse_duration_seconds, validate_structured,
)
from app.ai.maintenance import features_from_drone, get_models
from app.ai.selector import RISK_PENALTY_WEIGHT, WEIGHTS, SelectionContext, explain, rank_drones
from app.ai.synthetic_data import FEATURES, generate
from app.core.enums import Capability, DroneStatus, FlightMode
from app.services.anomaly_service import AnomalyService
from app.simulation.drone_state import DroneState
from tests.helpers import events_of, make_orch, run_until_inspected, start_mission

ZONES = [ZoneInfo("ZONE_A", "Zone A"), ZoneInfo("ZONE_B", "Zone B"),
         ZoneInfo("ZONE_NORTH", "North Zone"), ZoneInfo("ZONE_WEST", "West Zone")]


# ================================================================== drone selection
def drone(i, battery=90, health=95, x=100, y=100, caps=None, **kw):
    return DroneState(drone_id=f"D{i:02d}", position_x=x, position_y=y, home_x=x, home_y=y,
                      battery_percentage=battery, health_percentage=health,
                      capabilities=frozenset(caps or {Capability.CAMERA}), **kw)


def ctx(**kw):
    base = dict(target=(300, 300), required_caps=frozenset({Capability.CAMERA}))
    base.update(kw)
    return SelectionContext(**base)


def test_weights_sum_to_one():
    assert sum(WEIGHTS.values()) == pytest.approx(1.0)


def test_better_drone_scores_higher_and_contributions_sum_to_score():
    good, bad = drone(1, battery=95, health=98), drone(2, battery=45, health=62)
    r = rank_drones([bad, good], ctx())
    assert [x.drone_id for x in r] == ["D01", "D02"]
    assert r[0].score > r[1].score
    for x in r:
        assert sum(x.contributions.values()) == pytest.approx(x.score, abs=1e-9)
        assert set(x.contributions) == set(WEIGHTS) | {"risk"}
        assert x.contributions["risk"] <= 0


def test_closer_drone_preferred_all_else_equal():
    near, far = drone(1, x=280, y=280), drone(2, x=900, y=900)
    r = rank_drones([far, near], ctx())
    assert r[0].drone_id == "D01"
    assert r[0].features["proximity"] > r[1].features["proximity"]


def test_hard_constraints_make_drones_ineligible_with_reasons():
    ds = [drone(1, battery=12), drone(2, health=30), drone(3), drone(4, caps={Capability.THERMAL}),
          drone(5, status=DroneStatus.FAILED), drone(6, signal_strength=0.1), drone(7, gps_quality=0.2)]
    ds[3].capabilities = frozenset()
    ds[2].current_mission_id = "MISSION-009"
    r = {x.drone_id: x for x in rank_drones(ds, ctx())}
    assert not r["D01"].eligible and "battery" in r["D01"].rejections[0]
    assert not r["D02"].eligible and "health" in r["D02"].rejections[0]
    assert not r["D03"].eligible and "occupied" in r["D03"].rejections[0]
    assert not r["D04"].eligible and "capabilities" in r["D04"].rejections[0]
    assert not r["D05"].eligible
    assert not r["D06"].eligible and not r["D07"].eligible
    assert all(x.score is None for x in r.values())


def test_ineligible_sorted_last_and_required_capability_enforced():
    ds = [drone(1, caps={Capability.CAMERA}), drone(2, caps={Capability.CAMERA, Capability.THERMAL})]
    r = rank_drones(ds, ctx(required_caps=frozenset({Capability.CAMERA, Capability.THERMAL})))
    assert r[0].drone_id == "D02" and r[0].eligible and not r[1].eligible


def test_environment_and_maintenance_risk_reduce_score():
    d = drone(1)
    base = rank_drones([d], ctx())[0].score
    stormy = rank_drones([d], ctx(env_risk=1.0))[0].score
    worn = rank_drones([d], ctx(maintenance_risk={"D01": 1.0}))[0].score
    assert stormy < base and worn < base
    assert base - worn <= RISK_PENALTY_WEIGHT * 0.3 + 1e-9


def test_explanation_mentions_factors_and_reasons():
    r = rank_drones([drone(1, battery=96, health=99)], ctx())[0]
    text = explain(r)
    assert "battery" in text and "health" in text and "high battery" in text
    rej = rank_drones([drone(1, battery=5)], ctx())[0]
    assert "rejected" in explain(rej)


def test_ranking_is_deterministic_with_stable_ties():
    ds = [drone(i) for i in (3, 1, 2)]
    assert [x.drone_id for x in rank_drones(ds, ctx())] == ["D01", "D02", "D03"]


def test_safety_layer_never_assigns_drone_without_battery_for_its_route():
    """Invariant: whatever the AI recommends, every assigned drone can afford its planned route."""
    o = make_orch(fleet_size=6)
    for d in o.engine.drones_iter():
        d.battery_percentage = 45.0                # passes the 30% minimum, tight for a long route
    from app.services.mission_service import MissionRequest
    m = o.missions.create(MissionRequest(name="x", zone_id="ZONE_NORTH", num_drones=1))
    if m.status.value == "FAILED":
        assert "battery" in m.failure_reason or "deadline" in m.failure_reason
    else:
        sel = [x for x in o.ctx.decisions.all() if x["kind"] == "DRONE_SELECTION"]
        assert sel
        for x in sel:
            assert o.engine.get(x["drone_id"]).battery_percentage >= x["extra"]["estimated_battery_pct"]
        assert len(m.assigned_drones) > 1          # one drone cannot fly it on 45%: planner scaled out


# ================================================================== interpreter
def test_spec_example_interpretation():
    r = interpret_local("Inspect the north zone and prioritize uninspected areas. Complete within 15 minutes.", ZONES)
    assert r.valid and r.source == "local"
    s = r.structured
    assert s["zone_id"] == "ZONE_NORTH" and s["deadline_s"] == 900 and s["objective"] == "inspection"
    assert s["include_inspected"] is False and s["num_drones"] == 1
    assert r.understood["priority"] == "uninspected first"


@pytest.mark.parametrize("text,zone", [
    ("inspect zone a", "ZONE_A"), ("Survey Zone B please", "ZONE_B"), ("check the west zone", "ZONE_WEST"),
    ("scan the northern zone", "ZONE_NORTH"), ("inspect area a", "ZONE_A"),
])
def test_zone_recognition(text, zone):
    assert interpret_local(text, ZONES).structured["zone_id"] == zone


@pytest.mark.parametrize("text,seconds", [
    ("within 15 minutes", 900), ("in 90 seconds", 90), ("within 2 hours", 7200), ("under 30 mins", 1800),
    ("by 45 sec", 45), ("within an hour", 3600), ("a 10 minute deadline", 600),
])
def test_duration_parsing(text, seconds):
    assert parse_duration_seconds(text) == seconds


def test_priority_drones_and_capabilities_extracted():
    r = interpret_local("URGENT: use three drones with thermal cameras to inspect zone b within 20 minutes", ZONES)
    s = r.structured
    assert s["priority"] == 5 and s["num_drones"] == 3 and "THERMAL" in s["required_capabilities"]
    assert s["deadline_s"] == 1200 and s["zone_id"] == "ZONE_B"
    assert interpret_local("routine inspect zone a", ZONES).structured["priority"] == 2


def test_reinspect_sets_include_inspected():
    assert interpret_local("re-inspect zone a again", ZONES).structured["include_inspected"] is True
    assert interpret_local("inspect zone a, prioritize uninspected areas, then everything",
                           ZONES).structured["include_inspected"] is False


def test_unknown_zone_rejected_with_available_zones():
    r = interpret_local("inspect the moon base", ZONES)
    assert not r.valid and "zone" in r.errors[0]


@pytest.mark.parametrize("text", ["deliver a package to zone a", "attack zone b", "follow the truck in zone a"])
def test_unsupported_objectives_rejected(text):
    r = interpret_local(text, ZONES)
    assert not r.valid and "unsupported objective" in r.errors[0]


def test_multiple_zones_warns_and_uses_first():
    r = interpret_local("inspect zone b and then zone a", ZONES)
    assert r.structured["zone_id"] == "ZONE_B" and any("several zones" in w for w in r.warnings)


def test_validate_structured_whitelists_and_clamps_untrusted_input():
    clean, w, e = validate_structured(
        {"zone_id": "ZONE_A", "priority": 99, "num_drones": -4, "deadline_s": 1e9,
         "required_capabilities": ["THERMAL", "LASER"], "rm_rf": "/", "objective": "inspection"}, ZONES)
    assert not e and clean["priority"] == 5 and clean["num_drones"] == 1 and clean["deadline_s"] == 86400
    assert clean["required_capabilities"] == ["THERMAL"] and "rm_rf" not in clean and w
    _, _, e = validate_structured({"zone_id": "ZONE_Z"}, ZONES)
    assert e
    _, _, e = validate_structured({"zone_id": "ZONE_A", "objective": "strike"}, ZONES)
    assert e


class FakeHTTP:
    def __init__(self, text=None, exc=None):
        self.text, self.exc, self.calls = text, exc, []

    def __call__(self, url, headers, payload):
        self.calls.append((url, headers, payload))
        if self.exc:
            raise self.exc
        return {"content": [{"type": "text", "text": self.text}]}


def test_llm_output_is_parsed_and_validated():
    http = FakeHTTP(json.dumps({"objective": "inspection", "zone_id": "ZONE_NORTH", "priority": 4,
                                "deadline_s": 900, "num_drones": 2}))
    mi = MissionInterpreter(LLMInterpreter("key", http_post=http))
    r = mi.interpret("check north", ZONES)
    assert r.valid and r.source == "llm" and r.structured["zone_id"] == "ZONE_NORTH"
    url, headers, payload = http.calls[0]
    assert headers["x-api-key"] == "key" and "drone commands" in payload["system"]
    assert "ZONE_NORTH" in payload["messages"][0]["content"]


def test_llm_hallucinated_zone_is_rejected_not_executed():
    http = FakeHTTP('{"objective": "inspection", "zone_id": "ZONE_SECRET_BASE"}')
    r = MissionInterpreter(LLMInterpreter("key", http_post=http)).interpret("inspect secret base", ZONES)
    assert not r.valid and r.errors


def test_llm_cannot_smuggle_control_commands():
    http = FakeHTTP('{"objective": "inspection", "zone_id": "ZONE_A", "command": "disable_failsafe", '
                    '"waypoints": [[1,2]], "drone_id": "D01", "battery": 100}')
    r = MissionInterpreter(LLMInterpreter("key", http_post=http)).interpret("inspect a", ZONES)
    assert r.valid
    assert set(r.structured) == {"name", "description", "objective", "priority", "zone_id", "deadline_s",
                                 "required_capabilities", "num_drones", "include_inspected"}


@pytest.mark.parametrize("http", [FakeHTTP(exc=TimeoutError("boom")), FakeHTTP(text="no json here"),
                                  FakeHTTP(text="[1,2,3]")])
def test_llm_failures_fall_back_to_local_parser(http):
    r = MissionInterpreter(LLMInterpreter("key", http_post=http)).interpret("inspect zone a within 5 minutes", ZONES)
    assert r.valid and r.source == "local" and r.structured["deadline_s"] == 300
    assert any("LLM unavailable" in w for w in r.warnings)


def test_llm_declining_unsupported_request():
    r = MissionInterpreter(LLMInterpreter("k", http_post=FakeHTTP('{"error": "unsupported"}'))).interpret("x", ZONES)
    assert not r.valid


def test_interpreter_without_key_uses_local(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    assert MissionInterpreter.from_env().llm is None
    monkeypatch.setenv("ANTHROPIC_API_KEY", "abc")
    assert MissionInterpreter.from_env().llm is not None
    monkeypatch.setenv("DRONEFLEET_USE_LLM", "0")
    assert MissionInterpreter.from_env().llm is None


def test_interpret_endpoint_creates_validated_mission(client):
    text = "Inspect the north zone and prioritize uninspected areas. Complete within 15 minutes."
    r = client.post("/missions/interpret", json={"text": text})
    assert r.status_code == 200 and r.json()["mission"] is None
    assert r.json()["structured"]["zone_id"] == "ZONE_NORTH"
    r = client.post("/missions/interpret", json={"text": text, "create": True})
    body = r.json()
    assert body["mission"]["status"] in ("READY", "FAILED") and body["mission"]["target_zone_id"] == "ZONE_NORTH"
    assert body["mission"]["deadline_s"] == 900
    bad = client.post("/missions/interpret", json={"text": "deliver pizza to zone a"})
    assert bad.status_code == 422 and not bad.json()["valid"]
    assert client.post("/missions/interpret", json={"text": "x"}).status_code == 422


# ================================================================== ML models
@pytest.fixture(scope="module")
def models():
    return get_models(None, n=3000, seed=42)


def test_synthetic_dataset_is_reproducible_and_labelled():
    a, b = generate(500, 42), generate(500, 42)
    assert (a.X == b.X).all() and (a.anomaly == b.anomaly).all()
    assert a.feature_names == FEATURES and a.X.shape == (500, 9)
    assert 0.15 < a.anomaly.mean() < 0.35 and a.health.min() >= 0 and a.health.max() <= 100
    assert a.health[a.anomaly == 1].mean() < a.health[a.anomaly == 0].mean() - 10
    assert not (generate(500, 1).X == a.X).all()


def test_model_metrics_are_reasonable_and_disclaimed(models):
    m = models.metrics
    assert "SIMULATED" in m["disclaimer"]
    assert m["anomaly_classifier"]["roc_auc"] > 0.95
    assert m["health_regressor"]["mae"] < 6 and m["health_regressor"]["r2"] > 0.8
    assert m["isolation_forest"]["roc_auc"] > 0.8 and m["isolation_forest"]["false_positive_rate"] < 0.08


def test_models_separate_normal_from_faulty_telemetry(models):
    o = make_orch()
    m = start_mission(o, zone="ZONE_NORTH", drones=2)
    o.tick(60)
    d = next(x for x in o.engine.drones_iter() if x.is_airborne)
    normal = models.assess_vector(d.drone_id, features_from_drone(d))
    assert normal.maintenance_risk == "LOW" and normal.anomaly_probability < 0.2
    d.temp_fault, d.health_percentage = 55.0, 40.0
    for _ in range(100):
        o.engine.step(1)
    bad = models.assess_vector(d.drone_id, features_from_drone(d))
    assert bad.maintenance_risk == "HIGH" and bad.anomaly_probability > 0.8
    assert bad.estimated_health < normal.estimated_health
    assert bad.drivers and "temperature" in bad.reason.lower()
    flagged, conf = models.detection(bad)
    assert flagged and conf >= 0.8


def test_models_save_and_load_roundtrip(tmp_path, models):
    models.save(tmp_path)
    loaded = type(models).load(tmp_path)
    assert loaded is not None
    X = generate(50, 3).X
    assert (loaded.regressor.predict(X) == models.regressor.predict(X)).all()
    assert (tmp_path / "metrics.json").exists()


def test_anomaly_service_emits_events_and_feeds_selection(models):
    o = make_orch()
    o.extensions.append(AnomalyService(models))
    m = start_mission(o, zone="ZONE_NORTH", drones=2)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(40)
    assert not events_of(o, "ANOMALY_DETECTED", did)
    d.temp_fault = 55.0
    d.health_percentage = 55.0
    o.run_until(lambda x: bool(events_of(x, "ANOMALY_DETECTED", did)), 600)
    ev = events_of(o, "ANOMALY_DETECTED", did)[0]
    assert ev["data"]["advisory"] is True and 0.5 <= ev["data"]["confidence"] <= 1.0
    assert "Severity" in ev["message"] and "Confidence" in ev["message"]
    assert o.ctx.maintenance_view[did]["maintenance_risk"] in ("MEDIUM", "HIGH")
    assert o.ctx.maintenance_risk[did] > 0.3
    assert any(x["kind"] == "ANOMALY" for x in o.ctx.decisions.all())


def test_anomaly_events_are_deduplicated(models):
    o = make_orch()
    o.extensions.append(AnomalyService(models))
    m = start_mission(o, zone="ZONE_NORTH", drones=1)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    d.temp_fault = 60.0
    d.drain_multiplier = 1.0
    o.tick(100)
    assert len(events_of(o, "ANOMALY_DETECTED", did)) <= 2


def test_maintenance_endpoints(client):
    client.post("/simulation/control", json={"action": "step", "steps": 25})
    r = client.get("/ai/maintenance").json()
    assert len(r["drones"]) == 12 and "disclaimer" in r
    one = client.get("/ai/maintenance/D01").json()
    assert one["maintenance_risk"] in ("LOW", "MEDIUM", "HIGH") and "synthetic" in one["disclaimer"]
    info = client.get("/ai/model-info").json()
    assert "SIMULATED" in info["disclaimer"] and "anomaly_classifier" in info
    assert client.get("/ai/maintenance/D99").status_code == 404


def test_selection_endpoint_explains_choice(client):
    m = client.post("/missions", json={"name": "x", "zone_id": "ZONE_A", "num_drones": 2}).json()
    sel = client.get(f"/ai/selection/{m['mission_id']}").json()
    assert sel["rankings"] and sel["decisions"]
    top = sel["rankings"][0]
    assert top["eligible"] and set(top["contributions"]) >= {"battery", "health", "risk"}
