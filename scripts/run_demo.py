#!/usr/bin/env python3
"""Run one of the predefined scenarios and narrate it.

  python scripts/run_demo.py --scenario human_takeover
  python scripts/run_demo.py --list
  python scripts/run_demo.py --scenario takeover_rejoin --url http://localhost:8000   # watch it in the UI too
  python scripts/run_demo.py --all                                                    # headless regression run

Without --url (or if the server is unreachable) the scenario runs headless, in-process, with the same
code the server uses. With a running server the scenario is loaded into the *live* simulation, so the
React dashboard shows it while this script narrates the event log.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))

SHOW = {"MISSION_CREATED", "DRONE_ASSIGNED", "MISSION_STARTED", "CONTROL_TAKEOVER", "CONTROL_RELEASED",
        "ROUTE_REJOIN_STARTED", "ROUTE_REJOIN_COMPLETED", "ROUTE_REPLANNED", "FAILURE_DETECTED",
        "MISSION_REASSIGNED", "RETURN_HOME_STARTED", "RETURN_HOME_COMPLETED", "MISSION_COMPLETED",
        "MISSION_FAILED", "INJECTED_EVENT", "MODE_CHANGED", "AI_DECISION", "INFO"}


def line(c: str = "─") -> None:
    print(c * 78)


def print_result(r: dict) -> bool:
    line("═")
    print(f"SCENARIO  {r['title']}   (seed {r['seed']}, {r['ticks']} ticks, sim time {r['sim_time']} s)")
    line("═")
    for e in r["timeline"]:
        print(f"  [{e['clock']:>7}] {e['type']:<22} {e['message']}")
    line()
    for rep in r["mission_reports"]:
        print(f"  REPORT {rep['mission_id']}: {rep['status']}, coverage {rep['coverage_ratio']:.0%} "
              f"({rep['targets_inspected']}/{rep['targets_total']} targets), {rep['duration_s']:.0f} s, "
              f"replans {rep['replans']}, takeovers {rep['takeovers']}, rejoins {rep['rejoins']}, "
              f"reassignments {rep['reassignments']}")
        for d in rep["drones"]:
            print(f"      {d['drone_id']}: {d['distance_m']:.0f} m flown, {d['battery_used_pct']:.1f}% battery, "
                  f"{d['targets_inspected']} targets")
    line()
    print("CHECKS")
    for c in r["checks"]:
        print(f"  {'✔' if c['ok'] else '✘'} {c['name']}" + ("" if c["ok"] else f"   -> {c['detail']}"))
    print()
    ok = r["passed"]
    print("RESULT:", "PASSED" if ok else "FAILED")
    return ok


def run_headless(name: str, seed: int) -> bool:
    from app.scenarios import run_scenario
    return print_result(run_scenario(name, seed))


def run_live(name: str, seed: int, url: str, speed: float) -> bool:
    import httpx
    with httpx.Client(base_url=url, timeout=10) as c:
        c.post("/simulation/control", json={"action": "speed", "speed": speed})
        r = c.post(f"/simulation/scenarios/{name}/load", json={"seed": seed})
        r.raise_for_status()
        info = r.json()
        line("═")
        print(f"LIVE SCENARIO  {info['title']}\n{info['description']}\nOpen the dashboard to watch it.")
        line("═")
        last_id = 0
        while True:
            for e in c.get("/events", params={"limit": 2000}).json():
                if e["id"] > last_id:
                    last_id = e["id"]
                    if e["type"] in SHOW:
                        print(f"  [{e['clock']:>7}] {e['type']:<22} {e['message']}")
            st = c.get("/simulation/scenarios/active/status").json()
            if st.get("finished"):
                line()
                print("CHECKS")
                for ch in st["checks"]:
                    print(f"  {'✔' if ch['ok'] else '✘'} {ch['name']}" + ("" if ch["ok"] else f"   -> {ch['detail']}"))
                print("\nRESULT:", "PASSED" if st["passed"] else "FAILED")
                return bool(st["passed"])
            time.sleep(0.5)


def server_up(url: str) -> bool:
    try:
        import httpx
        return httpx.get(f"{url}/health", timeout=2).status_code == 200
    except Exception:
        return False


def main() -> int:
    from app.scenarios import list_scenarios
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", help="scenario id (see --list)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--all", action="store_true", help="run every scenario headless and summarise")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--url", help="base URL of a running backend, e.g. http://localhost:8000")
    ap.add_argument("--speed", type=float, default=4.0, help="simulation speed when running live")
    a = ap.parse_args()

    if a.list or not (a.scenario or a.all):
        for s in list_scenarios():
            print(f"{s['number']}. {s['id']:<28} {s['title']}\n     {s['description']}")
        return 0
    if a.all:
        from app.scenarios import run_scenario
        bad = 0
        for s in list_scenarios():
            r = run_scenario(s["id"], a.seed)
            print(f"{'PASS' if r['passed'] else 'FAIL'}  {s['id']:<28} {r['ticks']:>5} ticks")
            for c in r["checks"]:
                if not c["ok"]:
                    print(f"      ✘ {c['name']}: {c['detail']}")
            bad += not r["passed"]
        return 1 if bad else 0
    if a.scenario not in {s["id"] for s in list_scenarios()}:
        print(f"unknown scenario '{a.scenario}'. Use --list.", file=sys.stderr)
        return 2
    if a.url and server_up(a.url):
        return 0 if run_live(a.scenario, a.seed, a.url.rstrip("/"), a.speed) else 1
    if a.url:
        print(f"(server at {a.url} not reachable — running headless)\n")
    return 0 if run_headless(a.scenario, a.seed) else 1


if __name__ == "__main__":
    sys.exit(main())
