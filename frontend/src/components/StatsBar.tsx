import type { FleetState } from "../types";

export default function StatsBar({ state }: { state: FleetState | null }) {
  const s = state?.summary;
  const items: [string, string | number, string?][] = [
    ["Fleet", s?.fleet_count ?? "–"],
    ["Active", s?.active_drones ?? "–", "ok"],
    ["Available", s?.available_drones ?? "–"],
    ["Missions", s ? `${s.active_missions} active / ${s.total_missions}` : "–"],
    ["Completed", s?.completed_missions ?? "–"],
    ["Avg battery", s ? `${s.average_battery.toFixed(0)}%` : "–"],
    ["Avg health", s ? `${s.average_health.toFixed(0)}%` : "–"],
    ["Alerts", s?.alerts ?? "–", s && s.alerts > 0 ? "warn" : ""],
  ];
  return (
    <div className="stats">
      {items.map(([k, v, cls]) => (
        <div key={k} className={`stat ${cls ?? ""}`}><div className="v">{v}</div><div className="k">{k}</div></div>
      ))}
    </div>
  );
}
