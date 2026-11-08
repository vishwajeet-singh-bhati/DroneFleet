import { useEffect, useRef, useState } from "react";
import type { Decision, FleetState, LogEvent, World } from "../types";

export type Conn = "connecting" | "open" | "closed";

export interface FleetData {
  conn: Conn;
  state: FleetState | null;
  world: World | null;
  events: LogEvent[];
  decisions: Decision[];
}

const MAX_EVENTS = 400;
const MAX_DECISIONS = 150;

/** Subscribes to /ws/fleet (auto-reconnect). The server pushes everything; we never derive state. */
export function useFleet(): FleetData {
  const [conn, setConn] = useState<Conn>("connecting");
  const [state, setState] = useState<FleetState | null>(null);
  const [world, setWorld] = useState<World | null>(null);
  const [events, setEvents] = useState<LogEvent[]>([]);
  const [decisions, setDecisions] = useState<Decision[]>([]);
  const closed = useRef(false);

  useEffect(() => {
    closed.current = false;
    let ws: WebSocket | null = null;
    let timer: number | undefined;

    const connect = () => {
      setConn("connecting");
      const proto = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${proto}://${location.host}/ws/fleet`);
      ws.onopen = () => setConn("open");
      ws.onmessage = (m) => {
        const msg = JSON.parse(m.data) as { type: string; data: any };
        switch (msg.type) {
          case "hello":
            setWorld(msg.data.world);
            setEvents(msg.data.events);
            setDecisions(msg.data.decisions);
            break;
          case "state":
            setState(msg.data);
            break;
          case "world":
            setWorld(msg.data);
            break;
          case "events":
            setEvents((prev) => {
              const known = new Set(prev.slice(-50).map((e) => e.id));
              const fresh = (msg.data as LogEvent[]).filter((e) => !known.has(e.id));
              return [...prev, ...fresh].slice(-MAX_EVENTS);
            });
            break;
          case "decisions":
            setDecisions((prev) => {
              const known = new Set(prev.slice(-50).map((d) => d.id));
              const fresh = (msg.data as Decision[]).filter((d) => !known.has(d.id));
              return [...prev, ...fresh].slice(-MAX_DECISIONS);
            });
            break;
        }
      };
      ws.onclose = () => {
        setConn("closed");
        if (!closed.current) timer = window.setTimeout(connect, 1500);
      };
      ws.onerror = () => ws?.close();
    };
    connect();
    return () => {
      closed.current = true;
      window.clearTimeout(timer);
      ws?.close();
    };
  }, []);

  // a simulation reset restarts event ids: drop stale logs when the clock goes backwards
  const lastT = useRef(0);
  useEffect(() => {
    if (state && state.sim_time < lastT.current - 1) {
      setEvents([]);
      setDecisions([]);
    }
    if (state) lastT.current = state.sim_time;
  }, [state]);

  return { conn, state, world, events, decisions };
}
