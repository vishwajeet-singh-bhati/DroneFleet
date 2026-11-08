import { useEffect, useState } from "react";
import { api } from "../services/api";
import type { Conn } from "../hooks/useFleet";
import type { FleetState, ScenarioInfo } from "../types";
import { clockOf } from "../utils/format";

interface Props { conn: Conn; state: FleetState | null; act: (p: Promise<unknown>, ok?: string) => void }

export default function TopBar({ conn, state, act }: Props) {
  const [scenarios, setScenarios] = useState<ScenarioInfo[]>([]);
  const [pick, setPick] = useState("");
  useEffect(() => { api.scenarios().then(setScenarios).catch(() => undefined); }, [conn]);
  const running = state?.running ?? false;
  const speeds = [0.5, 1, 2, 4, 8, 16];

  return (
    <header className="topbar">
      <div className="brand">
        <span className="logo">◈</span> AI DroneFleet Commander
        <span className="sub">software simulation · backend is the source of truth</span>
      </div>
      <div className="grow" />
      <span className={`pill conn-${conn}`}>{conn === "open" ? "LIVE" : conn.toUpperCase()}</span>
      <span className="mono clock" title="simulation time">T+{clockOf(state?.sim_time ?? 0)}</span>
      <button onClick={() => act(api.simControl(running ? "stop" : "start"))}>{running ? "❚❚ Pause" : "▶ Run"}</button>
      <button onClick={() => act(api.simControl("step", { steps: 10 }))} disabled={running} title="advance 5 s">Step</button>
      <select value={state?.speed ?? 1} onChange={(e) => act(api.simControl("speed", { speed: Number(e.target.value) }))}>
        {speeds.map((s) => <option key={s} value={s}>{s}×</option>)}
      </select>
      <select value={pick} onChange={(e) => setPick(e.target.value)} title="predefined scenarios">
        <option value="">Scenario…</option>
        {scenarios.map((s) => <option key={s.id} value={s.id}>{s.number}. {s.title.replace(/^Scenario \d+: /, "")}</option>)}
      </select>
      <button disabled={!pick} onClick={() => act(api.loadScenario(pick), `Scenario loaded: ${pick}`)}>Load</button>
      <button className="ghost" onClick={() => act(api.reset(), "Simulation reset")}>Reset</button>
    </header>
  );
}
