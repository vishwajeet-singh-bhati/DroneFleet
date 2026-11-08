import { useState } from "react";
import { api } from "../services/api";
import type { FleetState } from "../types";

interface Props { state: FleetState | null; selected: string | null; act: (p: Promise<unknown>, ok?: string) => void }

const DRONE_EVENTS = ["LOW_BATTERY", "GPS_DEGRADATION", "SIGNAL_LOSS", "HIGH_TEMPERATURE", "MOTOR_HEALTH_DROP", "DRONE_FAILURE"];
const WORLD_EVENTS = ["OBSTACLE_APPEARED", "WEATHER_RISK"];

export default function FailurePanel({ state, selected, act }: Props) {
  const [target, setTarget] = useState("");
  const id = target || selected || state?.drones.find((d) => d.current_mission_id)?.drone_id || "";
  const dynZones = state ? [] : [];
  void dynZones;
  return (
    <div className="panel">
      <h3>Failure injection <span className="muted">(simulation only)</span></h3>
      <label className="row">Target drone
        <select value={id} onChange={(e) => setTarget(e.target.value)}>
          {state?.drones.map((d) => <option key={d.drone_id} value={d.drone_id}>{d.drone_id} · {d.flight_mode}</option>)}
        </select>
      </label>
      <div className="chips">
        {DRONE_EVENTS.map((t) => (
          <button key={t} className={t === "DRONE_FAILURE" ? "danger" : ""} disabled={!id}
            onClick={() => act(api.inject(t, id), `${t} injected`)}>{t.replace(/_/g, " ")}</button>
        ))}
      </div>
      <div className="muted">World events are placed on the selected drone's remaining route:</div>
      <div className="chips">
        {WORLD_EVENTS.map((t) => (
          <button key={t} disabled={!id} onClick={() => act(api.inject(t, id), `${t} injected`)}>{t.replace(/_/g, " ")}</button>
        ))}
        <button className="ghost" disabled={!id} onClick={() => act(api.clearFaults(id), "Faults cleared")}>Clear drone faults</button>
      </div>
    </div>
  );
}
