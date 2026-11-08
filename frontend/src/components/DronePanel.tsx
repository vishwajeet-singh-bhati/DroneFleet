import { useEffect, useRef, useState } from "react";
import { api, type Axes } from "../services/api";
import type { Drone, FleetState } from "../types";
import { MODE_COLOR } from "../utils/format";

interface Props { drone: Drone | undefined; state: FleetState | null; act: (p: Promise<unknown>, ok?: string) => void }

const KEYS: Record<string, keyof Axes | undefined> = {
  w: "forward", s: "forward", a: "strafe", d: "strafe", q: "yaw", e: "yaw", r: "climb", f: "climb",
};
const SIGN: Record<string, number> = { w: 1, s: -1, d: 1, a: -1, e: 1, q: -1, r: 1, f: -1 };

export default function DronePanel({ drone, state, act }: Props) {
  const manual = drone?.flight_mode === "MANUAL";
  const [axes, setAxes] = useState<Axes>({ forward: 0, strafe: 0, climb: 0, yaw: 0 });
  const held = useRef<Set<string>>(new Set());
  const id = drone?.drone_id;

  // Keyboard manual control: only while the backend says this drone is in MANUAL mode.
  // Input is re-sent at 10 Hz while keys are held (backend dead-man stops the drone ~0.6 s after we stop).
  useEffect(() => {
    if (!manual || !id) { held.current.clear(); return; }
    const compute = (): Axes => {
      const a: Axes = { forward: 0, strafe: 0, climb: 0, yaw: 0 };
      held.current.forEach((k) => { const ax = KEYS[k]; if (ax) a[ax] += SIGN[k]; });
      return a;
    };
    const typing = (t: EventTarget | null) => t instanceof HTMLElement && ["INPUT", "SELECT", "TEXTAREA"].includes(t.tagName);
    const down = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (typing(e.target) || !(k in KEYS)) return;
      e.preventDefault(); held.current.add(k); setAxes(compute());
    };
    const up = (e: KeyboardEvent) => { held.current.delete(e.key.toLowerCase()); setAxes(compute()); };
    const blur = () => { held.current.clear(); setAxes(compute()); };
    window.addEventListener("keydown", down); window.addEventListener("keyup", up); window.addEventListener("blur", blur);
    const timer = window.setInterval(() => {
      const a = compute();
      if (held.current.size) api.manualInput(id, a).catch(() => undefined);
    }, 100);
    return () => {
      window.removeEventListener("keydown", down); window.removeEventListener("keyup", up);
      window.removeEventListener("blur", blur); window.clearInterval(timer);
    };
  }, [manual, id]);

  if (!drone) return <div className="panel"><h3>Drone</h3><p className="muted">Click a drone on the map or in the fleet list.</p></div>;

  const mode = drone.flight_mode;
  const failed = drone.status === "FAILED";
  const mx = state?.maintenance?.[drone.drone_id];
  const session = state?.control?.[drone.drone_id];
  const airborne = drone.altitude > 0.5;

  return (
    <div className="panel">
      <h3>{drone.drone_id} <span className="mode" style={{ color: MODE_COLOR[mode] }}>{failed ? "FAILED" : mode}</span></h3>
      <div className="kv">
        <span>Status</span><b>{drone.status}</b>
        <span>Position</span><b className="mono">{drone.position_x.toFixed(0)}, {drone.position_y.toFixed(0)} · alt {drone.altitude.toFixed(0)} m</b>
        <span>Speed / heading</span><b className="mono">{drone.velocity.toFixed(1)} m/s · {drone.heading.toFixed(0)}°</b>
        <span>Battery</span><b className={drone.battery_percentage < 20 ? "bad" : ""}>{drone.battery_percentage.toFixed(1)}%</b>
        <span>Health / temp</span><b>{drone.health_percentage.toFixed(0)}% · {drone.temperature.toFixed(0)} °C</b>
        <span>GPS / signal</span><b>{(drone.gps_quality * 100).toFixed(0)}% · {(drone.signal_strength * 100).toFixed(0)}%</b>
        <span>Mission</span><b>{drone.current_mission_id ?? "–"} {drone.total_waypoints ? `· wp ${drone.current_waypoint_index}/${drone.total_waypoints}` : ""}</b>
        {session && <><span>Operator</span><b>{session.operator}</b></>}
      </div>
      {mx && (
        <div className={`mx mx-${mx.maintenance_risk}`}>
          <b>Maintenance (simulated ML): {mx.maintenance_risk}</b> · est. health {mx.estimated_health.toFixed(0)}% · anomaly p={mx.anomaly_probability.toFixed(2)}
          <div className="muted">{mx.reason}</div>
        </div>
      )}

      <div className="btns">
        {mode === "AUTONOMOUS" && <button className="warn" onClick={() => act(api.takeControl(drone.drone_id), "Control requested")}>TAKE CONTROL</button>}
        {mode === "MANUAL" && <button className="ok" onClick={() => act(api.releaseControl(drone.drone_id), "Resuming mission via safe rejoin")}>RESUME MISSION</button>}
        {mode === "MANUAL" && <button onClick={() => act(api.returnHome(drone.drone_id))}>Release & RTH</button>}
        {["AUTONOMOUS", "REJOINING", "TAKEOFF", "RETURN_TO_HOME"].includes(mode) && <button onClick={() => act(api.pauseDrone(drone.drone_id))}>PAUSE</button>}
        {mode === "PAUSED" && <button className="ok" onClick={() => act(api.resumeDrone(drone.drone_id), "Resumed")}>RESUME</button>}
        {airborne && !failed && mode !== "RETURN_TO_HOME" && <button onClick={() => act(api.returnHome(drone.drone_id), "Returning home")}>RETURN HOME</button>}
        {airborne && !failed && <button className="danger" onClick={() => act(api.emergency(drone.drone_id), "Emergency landing")}>EMERGENCY</button>}
        {failed && <button onClick={() => act(api.post(`/drones/${drone.drone_id}/reset`), "Drone reset")}>Repair / reset</button>}
      </div>

      {manual && (
        <div className="keys">
          <b>Manual control active</b> — click the page, then hold keys:
          <div className="keyrow">
            {[["W", "forward"], ["S", "back"], ["A", "left"], ["D", "right"], ["Q", "yaw ⟲"], ["E", "yaw ⟳"], ["R", "up"], ["F", "down"]].map(([k, l]) => (
              <span key={k} className={`key ${held.current.has(k.toLowerCase()) ? "on" : ""}`}><kbd>{k}</kbd>{l}</span>
            ))}
          </div>
          <div className="muted mono">axes fwd {axes.forward} · strafe {axes.strafe} · yaw {axes.yaw} · climb {axes.climb}</div>
          <div className="muted">Geofence & dead-man are enforced by the backend. “Resume mission” plans a safe rejoin path (no teleport).</div>
        </div>
      )}
    </div>
  );
}
