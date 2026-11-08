import type { Drone, FleetState } from "../types";
import { MODE_COLOR, droneColor } from "../utils/format";

interface Props { state: FleetState | null; selected: string | null; onSelect: (id: string) => void }

const risk = (s: FleetState | null, d: Drone) => s?.maintenance?.[d.drone_id]?.maintenance_risk;

export default function DroneList({ state, selected, onSelect }: Props) {
  return (
    <div className="panel">
      <h3>Fleet</h3>
      <div className="scroll drone-list">
        <table>
          <thead><tr><th>Drone</th><th>Mode</th><th>Batt</th><th>Health</th><th>Mission</th></tr></thead>
          <tbody>
            {state?.drones.map((d) => (
              <tr key={d.drone_id} className={d.drone_id === selected ? "sel" : ""} onClick={() => onSelect(d.drone_id)}>
                <td><i className="dot" style={{ background: droneColor(d.drone_id) }} /> {d.drone_id}
                  {risk(state, d) === "HIGH" && <span className="tag bad" title="maintenance risk (simulated model)">MX</span>}
                </td>
                <td><span className="mode" style={{ color: MODE_COLOR[d.flight_mode] }}>{d.status === "FAILED" ? "FAILED" : d.flight_mode}</span></td>
                <td className={d.battery_percentage < 20 ? "bad" : ""}>{d.battery_percentage.toFixed(0)}%</td>
                <td>{d.health_percentage.toFixed(0)}%</td>
                <td>{d.current_mission_id?.replace("MISSION-", "M") ?? "–"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
