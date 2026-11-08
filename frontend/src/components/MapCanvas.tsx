import { useEffect, useMemo, useRef } from "react";
import type { Drone, FleetState, Mission, World } from "../types";
import { MODE_COLOR, droneColor } from "../utils/format";

interface Props {
  world: World | null;
  state: FleetState | null;
  selected: string | null;
  onSelect: (id: string | null) => void;
}

const ZONE_STYLE: Record<string, { fill: string; stroke: string; dash?: number[] }> = {
  INSPECTION: { fill: "rgba(61,220,151,0.07)", stroke: "rgba(61,220,151,0.55)", dash: [6, 5] },
  OBSTACLE: { fill: "rgba(255,77,109,0.35)", stroke: "#ff4d6d" },
  RESTRICTED: { fill: "rgba(255,77,109,0.14)", stroke: "#ff4d6d", dash: [3, 3] },
  WEATHER: { fill: "rgba(255,209,102,0.16)", stroke: "#ffd166", dash: [2, 4] },
};
const TRAIL_LEN = 80;

/** Custom 2D canvas map (no map API). World coordinates: x east, y NORTH (up), metres. */
export default function MapCanvas({ world, state, selected, onSelect }: Props) {
  const wrap = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const size = useRef(600);
  const trails = useRef<Record<string, [number, number][]>>({});
  const inspected = useMemo(() => new Set(state?.coverage ?? []), [state?.coverage]);

  // keep canvas square & crisp
  useEffect(() => {
    const el = wrap.current;
    if (!el) return;
    const fit = () => {
      const s = Math.max(320, Math.floor(Math.min(el.clientWidth, window.innerHeight - 140)));
      const c = canvas.current;
      if (!c) return;
      const dpr = window.devicePixelRatio || 1;
      size.current = s;
      c.width = s * dpr; c.height = s * dpr;
      c.style.width = `${s}px`; c.style.height = `${s}px`;
      c.getContext("2d")?.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    };
    const ro = new ResizeObserver(fit);
    ro.observe(el);
    fit();
    return () => ro.disconnect();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // trails
  useEffect(() => {
    if (!state) return;
    for (const d of state.drones) {
      const t = (trails.current[d.drone_id] ??= []);
      const last = t[t.length - 1];
      if (d.altitude > 0.5 && (!last || Math.hypot(last[0] - d.position_x, last[1] - d.position_y) > 4)) {
        t.push([d.position_x, d.position_y]);
        if (t.length > TRAIL_LEN) t.shift();
      }
      if (d.altitude <= 0.5 && t.length) t.length = 0;
    }
    if (state.sim_time < 1) trails.current = {};
  }, [state]);

  useEffect(draw);

  function draw() {
    const c = canvas.current;
    const ctx = c?.getContext("2d");
    if (!c || !ctx || !world) return;
    const S = size.current;
    const k = S / world.width;
    const X = (x: number) => x * k;
    const Y = (y: number) => S - y * k;

    ctx.clearRect(0, 0, S, S);
    ctx.fillStyle = "#0b1220";
    ctx.fillRect(0, 0, S, S);

    // grid
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(255,255,255,0.045)";
    for (let g = 0; g <= world.width; g += 100) {
      ctx.beginPath(); ctx.moveTo(X(g), 0); ctx.lineTo(X(g), S); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, Y(g)); ctx.lineTo(S, Y(g)); ctx.stroke();
    }
    ctx.fillStyle = "rgba(255,255,255,0.25)";
    ctx.font = "10px ui-monospace, monospace";
    for (let g = 100; g < world.width; g += 200) {
      ctx.fillText(`${g}`, X(g) + 2, S - 3);
      ctx.fillText(`${g}`, 3, Y(g) - 2);
    }
    ctx.fillText("N ↑", S - 28, 12);

    // zones
    for (const z of world.zones) {
      if (!z.active) continue;
      const st = ZONE_STYLE[z.kind] ?? ZONE_STYLE.INSPECTION;
      ctx.fillStyle = st.fill;
      ctx.strokeStyle = st.stroke;
      ctx.setLineDash(st.dash ?? []);
      ctx.lineWidth = 1.5;
      ctx.fillRect(X(z.x0), Y(z.y1), (z.x1 - z.x0) * k, (z.y1 - z.y0) * k);
      ctx.strokeRect(X(z.x0), Y(z.y1), (z.x1 - z.x0) * k, (z.y1 - z.y0) * k);
      ctx.setLineDash([]);
      ctx.fillStyle = st.stroke;
      ctx.font = "10px system-ui, sans-serif";
      ctx.fillText(z.name || z.zone_id, X(z.x0) + 4, Y(z.y1) + 12);
    }

    // home base
    ctx.fillStyle = "#4dabf7";
    ctx.beginPath(); ctx.arc(X(world.home.x), Y(world.home.y), 9, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#0b1220"; ctx.font = "bold 10px system-ui"; ctx.fillText("H", X(world.home.x) - 3, Y(world.home.y) + 4);

    if (!state) return;
    const byId = new Map(state.drones.map((d) => [d.drone_id, d] as const));

    // mission routes
    for (const m of state.missions) {
      if (["COMPLETED", "FAILED", "ABORTED"].includes(m.status)) continue;
      drawReplans(ctx, m, state.sim_time, X, Y);
      for (const [did, route] of Object.entries(m.routes)) {
        const d = byId.get(did);
        const col = droneColor(did);
        const cur = d && d.current_mission_id === m.mission_id && d.route_label === "MISSION" ? d.current_waypoint_index : 0;
        const isSel = selected === did;
        ctx.lineWidth = isSel ? 2 : 1.2;
        // done part (dim) then remaining part
        ctx.strokeStyle = col; ctx.globalAlpha = 0.28; ctx.setLineDash([]);
        path(ctx, route.slice(0, cur + 1).map((w) => [X(w.x), Y(w.y)]));
        ctx.globalAlpha = isSel ? 0.95 : 0.6; ctx.setLineDash([5, 4]);
        path(ctx, [d ? [X(d.position_x), Y(d.position_y)] : [X(route[cur]?.x ?? 0), Y(route[cur]?.y ?? 0)],
          ...route.slice(cur).map((w) => [X(w.x), Y(w.y)] as [number, number])]);
        ctx.setLineDash([]); ctx.globalAlpha = 1;
        route.forEach((w, i) => {
          if (w.action === "TRANSIT" || w.action === "LAND") return;
          const done = inspected.has(w.id) || i < cur;
          ctx.beginPath();
          ctx.arc(X(w.x), Y(w.y), 3.2, 0, Math.PI * 2);
          if (done) { ctx.fillStyle = col; ctx.fill(); }
          else { ctx.strokeStyle = col; ctx.lineWidth = 1.2; ctx.stroke(); }
        });
      }
    }

    // trails + drones
    for (const d of state.drones) {
      const col = droneColor(d.drone_id);
      const tr = trails.current[d.drone_id];
      if (tr && tr.length > 1) {
        ctx.strokeStyle = col; ctx.globalAlpha = 0.35; ctx.lineWidth = 2; ctx.setLineDash([]);
        path(ctx, tr.map(([x, y]) => [X(x), Y(y)]));
        ctx.globalAlpha = 1;
      }
      if (d.active_route && d.active_route.length) {              // rejoin path / return-to-home path
        ctx.strokeStyle = MODE_COLOR[d.flight_mode] ?? "#fff"; ctx.lineWidth = 2.2; ctx.setLineDash([2, 4]);
        path(ctx, [[X(d.position_x), Y(d.position_y)], ...d.active_route.map(([x, y]) => [X(x), Y(y)] as [number, number])]);
        ctx.setLineDash([]);
      }
    }
    for (const d of state.drones) drawDrone(ctx, d, X(d.position_x), Y(d.position_y), d.drone_id === selected, state.sim_time);
  }

  const onClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!world || !state) return;
    const r = e.currentTarget.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    const k = size.current / world.width;
    let best: string | null = null, bd = 16;
    for (const d of state.drones) {
      const dist = Math.hypot(d.position_x * k - px, size.current - d.position_y * k - py);
      if (dist < bd) { bd = dist; best = d.drone_id; }
    }
    onSelect(best);
  };

  return (
    <div ref={wrap} className="map-wrap">
      <canvas ref={canvas} onClick={onClick} className="map" />
      <div className="legend">
        <span><i className="lg lg-old" /> old route</span>
        <span><i className="lg lg-new" /> replanned</span>
        <span><i className="lg lg-rejoin" /> rejoin / RTH path</span>
        <span><i className="lg lg-wp" /> inspection point</span>
      </div>
    </div>
  );
}

type P = [number, number];
function path(ctx: CanvasRenderingContext2D, pts: P[]) {
  if (pts.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (const p of pts.slice(1)) ctx.lineTo(p[0], p[1]);
  ctx.stroke();
}

/** Show the most recent replan (old vs new route) for 90 simulated seconds. */
function drawReplans(ctx: CanvasRenderingContext2D, m: Mission, now: number, X: (v: number) => number, Y: (v: number) => number) {
  for (const r of m.route_history) {
    if (now - r.t > 90) continue;
    ctx.lineWidth = 2;
    ctx.strokeStyle = "rgba(255,77,109,0.8)"; ctx.setLineDash([8, 5]);
    path(ctx, r.old.map(([x, y]) => [X(x), Y(y)] as P));
    ctx.strokeStyle = "rgba(61,220,151,0.95)"; ctx.setLineDash([2, 3]);
    path(ctx, r.new.map(([x, y]) => [X(x), Y(y)] as P));
    ctx.setLineDash([]);
  }
}

function drawDrone(ctx: CanvasRenderingContext2D, d: Drone, x: number, y: number, selected: boolean, t: number) {
  const col = MODE_COLOR[d.flight_mode] ?? "#fff";
  const idle = d.flight_mode === "IDLE";
  ctx.save();
  ctx.translate(x, y);
  if (selected) {
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(0, 0, 15, 0, Math.PI * 2); ctx.stroke();
  }
  if (d.flight_mode === "MANUAL" || d.flight_mode === "TAKEOVER_REQUEST") {
    const pulse = 14 + 3 * Math.sin(t * 6);
    ctx.strokeStyle = col; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(0, 0, pulse, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.rotate((d.heading * Math.PI) / 180);
  ctx.globalAlpha = idle ? 0.55 : 1;
  ctx.fillStyle = d.status === "FAILED" ? "#ff4d6d" : col;
  ctx.beginPath(); ctx.moveTo(0, -9); ctx.lineTo(6, 7); ctx.lineTo(0, 3.5); ctx.lineTo(-6, 7); ctx.closePath(); ctx.fill();
  ctx.restore();
  ctx.fillStyle = "#e6edf3"; ctx.font = "10px ui-monospace, monospace";
  ctx.fillText(d.drone_id, x + 10, y - 8);
  if (!idle) {                                         // battery bar
    ctx.fillStyle = "rgba(255,255,255,0.18)"; ctx.fillRect(x - 10, y + 11, 20, 3);
    ctx.fillStyle = d.battery_percentage < 20 ? "#ff4d6d" : d.battery_percentage < 40 ? "#ffd166" : "#3ddc97";
    ctx.fillRect(x - 10, y + 11, 20 * Math.max(0, Math.min(1, d.battery_percentage / 100)), 3);
  }
}
