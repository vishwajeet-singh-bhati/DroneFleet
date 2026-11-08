import { useCallback, useState } from "react";
import { useFleet } from "./hooks/useFleet";
import { ApiError } from "./services/api";
import TopBar from "./components/TopBar";
import StatsBar from "./components/StatsBar";
import MapCanvas from "./components/MapCanvas";
import DroneList from "./components/DroneList";
import DronePanel from "./components/DronePanel";
import MissionPanel from "./components/MissionPanel";
import FailurePanel from "./components/FailurePanel";
import LogPanel from "./components/LogPanel";

interface Toast { id: number; text: string; kind: "ok" | "err" }
let seq = 0;

export default function App() {
  const { conn, state, world, events, decisions } = useFleet();
  const [selected, setSelected] = useState<string | null>(null);
  const [toasts, setToasts] = useState<Toast[]>([]);

  const push = useCallback((text: string, kind: Toast["kind"]) => {
    const id = ++seq;
    setToasts((t) => [...t.slice(-3), { id, text, kind }]);
    window.setTimeout(() => setToasts((t) => t.filter((x) => x.id !== id)), kind === "err" ? 6000 : 2500);
  }, []);

  /** Run a backend command; show the backend's reason if it refuses. The UI never changes state itself. */
  const act = useCallback((p: Promise<unknown>, ok?: string) => {
    p.then(() => ok && push(ok, "ok")).catch((e) => push(e instanceof ApiError ? `${e.code}: ${e.message}` : String(e), "err"));
  }, [push]);

  const drone = state?.drones.find((d) => d.drone_id === selected);

  return (
    <div className="app">
      <TopBar conn={conn} state={state} act={act} />
      <StatsBar state={state} />
      <main className="grid">
        <section className="col-map">
          <MapCanvas world={world} state={state} selected={selected} onSelect={setSelected} />
          <LogPanel events={events} decisions={decisions} state={state} />
        </section>
        <aside className="col-side">
          <DronePanel drone={drone} state={state} act={act} />
          <DroneList state={state} selected={selected} onSelect={setSelected} />
          <MissionPanel state={state} world={world} act={act} />
          <FailurePanel state={state} selected={selected} act={act} />
        </aside>
      </main>
      <div className="toasts">{toasts.map((t) => <div key={t.id} className={`toast ${t.kind}`}>{t.text}</div>)}</div>
    </div>
  );
}
