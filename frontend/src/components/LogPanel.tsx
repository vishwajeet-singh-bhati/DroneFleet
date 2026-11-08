import { useEffect, useRef, useState } from "react";
import type { Decision, FleetState, LogEvent } from "../types";

interface Props { events: LogEvent[]; decisions: Decision[]; state: FleetState | null }
type Tab = "events" | "ai" | "maintenance" | "scenario";

const SEV: Record<string, string> = { WARNING: "warn", CRITICAL: "bad", INFO: "" };

export default function LogPanel({ events, decisions, state }: Props) {
  const [tab, setTab] = useState<Tab>("events");
  const [onlyAlerts, setOnlyAlerts] = useState(false);
  const box = useRef<HTMLDivElement>(null);
  useEffect(() => { box.current?.scrollTo({ top: box.current.scrollHeight }); }, [events.length, decisions.length, tab]);
  const shown = onlyAlerts ? events.filter((e) => e.severity !== "INFO") : events;
  const mx = Object.values(state?.maintenance ?? {}).sort((a, b) => b.anomaly_probability - a.anomaly_probability);
  const sc = state?.scenario;

  return (
    <div className="panel log">
      <div className="tabs">
        {(["events", "ai", "maintenance", "scenario"] as Tab[]).map((t) => (
          <button key={t} className={tab === t ? "active" : ""} onClick={() => setTab(t)}>
            {{ events: "Event log", ai: "AI decisions", maintenance: "Maintenance", scenario: "Scenario" }[t]}
          </button>
        ))}
        {tab === "events" && <label className="chk"><input type="checkbox" checked={onlyAlerts} onChange={(e) => setOnlyAlerts(e.target.checked)} /> alerts only</label>}
      </div>
      <div className="scroll logbody" ref={box}>
        {tab === "events" && shown.map((e) => (
          <div key={e.id} className={`ev ${SEV[e.severity] ?? ""}`}>
            <span className="mono t">{e.clock}</span> <span className="et">{e.type}</span> {e.message}
          </div>
        ))}
        {tab === "ai" && decisions.map((d) => (
          <div key={d.id} className="ev dec">
            <span className="mono t">{d.clock}</span> <span className="et">{d.kind}</span> {d.summary}
            {d.reasons.length > 0 && <div className="muted">because: {d.reasons.join("; ")}</div>}
          </div>
        ))}
        {tab === "maintenance" && (
          <>
            <div className="muted disclaimer">Simulation prototype trained on synthetic data. Advisory only — it never grounds or commands a drone.</div>
            {mx.length === 0 && <p className="muted">Maintenance model not loaded.</p>}
            {mx.map((m) => (
              <div key={m.drone_id} className={`ev mxrow mx-${m.maintenance_risk}`}>
                <b>{m.drone_id}</b> <span className="tag">{m.maintenance_risk}</span> health≈{m.estimated_health.toFixed(0)}% · anomaly p={m.anomaly_probability.toFixed(2)}
                {m.isolation_flag && <span className="tag warn">outlier</span>}
                <div className="muted">{m.reason}{m.drivers.length ? ` (${m.drivers.join(", ")})` : ""}</div>
              </div>
            ))}
          </>
        )}
        {tab === "scenario" && (!sc || !sc.name ? <p className="muted">No scenario loaded. Pick one in the top bar.</p> : (
          <div>
            <b>{sc.title}</b>
            <div className="muted">{sc.description}</div>
            <div>Steps {sc.steps_done}/{sc.steps_total} · {sc.finished ? (sc.passed ? "✔ PASSED" : "✘ FAILED") : "running…"}</div>
            {sc.checks?.map((c, i) => <div key={i} className={c.ok ? "okline" : "warnline"}>{c.ok ? "✔" : "✘"} {c.name}{c.ok ? "" : ` — ${c.detail}`}</div>)}
          </div>
        ))}
      </div>
    </div>
  );
}
