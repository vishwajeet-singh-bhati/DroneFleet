import { useState } from "react";
import { api } from "../services/api";
import type { FleetState, Mission, World } from "../types";

interface Props { state: FleetState | null; world: World | null; act: (p: Promise<unknown>, ok?: string) => void }

const STATUS_CLS: Record<string, string> = {
  ACTIVE: "ok", COMPLETED: "ok", READY: "info", PAUSED: "warn", REPLANNING: "warn", FAILED: "bad", ABORTED: "bad",
};

export default function MissionPanel({ state, world, act }: Props) {
  const zones = world?.zones.filter((z) => z.kind === "INSPECTION") ?? [];
  const [zone, setZone] = useState("ZONE_A");
  const [n, setN] = useState(1);
  const [prio, setPrio] = useState(3);
  const [deadline, setDeadline] = useState("");
  const [nl, setNl] = useState("Inspect the north zone with 2 drones and complete within 15 minutes.");
  const [interp, setInterp] = useState<Record<string, any> | null>(null);
  const [open, setOpen] = useState<string | null>(null);

  const create = () => act(api.createMission({
    name: `Inspect ${zones.find((z) => z.zone_id === zone)?.name ?? zone}`, zone_id: zone, num_drones: n, priority: prio,
    deadline_s: deadline ? Number(deadline) * 60 : null, auto_start: true,
  }), "Mission created");

  const interpret = async (createIt: boolean) => {
    try {
      const r = await api.interpret(nl, createIt);
      setInterp(r);
    } catch (e: any) {
      setInterp({ valid: false, errors: [e.message], warnings: [] });
    }
  };

  return (
    <div className="panel">
      <h3>Missions</h3>
      <div className="form">
        <select value={zone} onChange={(e) => setZone(e.target.value)}>
          {zones.map((z) => <option key={z.zone_id} value={z.zone_id}>{z.name}</option>)}
        </select>
        <label>Drones <input type="number" min={1} max={6} value={n} onChange={(e) => setN(Number(e.target.value))} /></label>
        <label>Priority <input type="number" min={1} max={5} value={prio} onChange={(e) => setPrio(Number(e.target.value))} /></label>
        <label>Deadline (min) <input placeholder="–" value={deadline} onChange={(e) => setDeadline(e.target.value)} /></label>
        <button className="ok" onClick={create}>Create & start</button>
      </div>

      <details className="nl">
        <summary>AI mission interpreter (text → validated request)</summary>
        <textarea rows={2} value={nl} onChange={(e) => setNl(e.target.value)} />
        <div className="btns">
          <button onClick={() => interpret(false)}>Preview</button>
          <button className="ok" onClick={() => { interpret(true); }}>Create mission</button>
        </div>
        {interp && (
          <div className={`interp ${interp.valid ? "" : "bad"}`}>
            <div><b>{interp.valid ? "Valid request" : "Rejected"}</b> · source: {interp.source ?? "–"}</div>
            {interp.understood && <div className="muted">{String(interp.understood)}</div>}
            {interp.structured && <pre>{JSON.stringify(interp.structured, null, 1)}</pre>}
            {[...(interp.warnings ?? []), ...(interp.errors ?? [])].map((w: string, i: number) => <div key={i} className="warnline">⚠ {w}</div>)}
            <div className="muted">The AI only proposes a mission; it never commands drones. Execution is validated by the backend.</div>
          </div>
        )}
      </details>

      <div className="scroll missions">
        {state?.missions.length === 0 && <p className="muted">No missions yet.</p>}
        {[...(state?.missions ?? [])].reverse().map((m) => (
          <MissionRow key={m.mission_id} m={m} open={open === m.mission_id} toggle={() => setOpen(open === m.mission_id ? null : m.mission_id)} act={act} />
        ))}
      </div>
    </div>
  );
}

function MissionRow({ m, open, toggle, act }: { m: Mission; open: boolean; toggle: () => void; act: Props["act"] }) {
  const terminal = ["COMPLETED", "FAILED", "ABORTED"].includes(m.status);
  return (
    <div className="mission">
      <div className="mhead" onClick={toggle}>
        <b>{m.mission_id.replace("MISSION-", "M")}</b> {m.name}
        <span className={`tag ${STATUS_CLS[m.status] ?? ""}`}>{m.status}</span>
      </div>
      <div className="bar"><div style={{ width: `${Math.round(m.progress * 100)}%` }} /></div>
      <div className="muted">{m.assigned_drones.join(", ") || "no drones"} · {Math.round(m.progress * 100)}% covered</div>
      {m.failure_reason && <div className="warnline">⚠ {m.failure_reason}</div>}
      <div className="btns">
        {m.status === "READY" && <button className="ok" onClick={() => act(api.startMission(m.mission_id), "Started")}>Start</button>}
        {m.status === "ACTIVE" && <button onClick={() => act(api.pauseMission(m.mission_id))}>Pause</button>}
        {m.status === "PAUSED" && <button className="ok" onClick={() => act(api.resumeMission(m.mission_id))}>Resume</button>}
        {!terminal && <button className="danger" onClick={() => act(api.abortMission(m.mission_id), "Aborted")}>Abort</button>}
      </div>
      {open && m.plan_info && (
        <div className="why">
          <b>Why these drones? (explainable scoring)</b>
          {m.plan_info.rankings.slice(0, 5).map((r) => (
            <div key={r.drone_id} className="rank">
              <div><b>{r.drone_id}</b> score {r.score.toFixed(2)} {!r.eligible && <span className="tag bad">ineligible</span>}</div>
              <div className="contribs">
                {Object.entries(r.contributions).map(([k, v]) => (
                  <span key={k} className={v < 0 ? "neg" : ""}>{k} {v >= 0 ? "+" : ""}{v.toFixed(2)}</span>
                ))}
              </div>
              <div className="muted">{r.eligible ? r.reasons.join("; ") : r.rejections.join("; ")}</div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
