// Thin REST client. Every call returns parsed JSON or throws ApiError with the backend's
// structured {error:{code,message}} body, so the UI can show *why* a command was refused.

export class ApiError extends Error {
  constructor(public status: number, public code: string, message: string) {
    super(message);
  }
}

const BASE = (import.meta.env.VITE_API_URL as string | undefined) ?? "/api";

async function call<T>(method: string, path: string, body?: unknown): Promise<T> {
  const res = await fetch(`${BASE}${path}`, {
    method,
    headers: body === undefined ? undefined : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const text = await res.text();
  const data = text ? JSON.parse(text) : null;
  if (!res.ok) {
    const e = data?.error;
    const detail = Array.isArray(data?.detail) ? data.detail.map((d: { msg: string }) => d.msg).join("; ") : data?.detail;
    throw new ApiError(res.status, e?.code ?? "HTTP_" + res.status, e?.message ?? detail ?? res.statusText);
  }
  return data as T;
}

export const api = {
  get: <T>(p: string) => call<T>("GET", p),
  post: <T>(p: string, b?: unknown) => call<T>("POST", p, b ?? {}),
  del: <T>(p: string) => call<T>("DELETE", p),

  createMission: (b: Record<string, unknown>) => api.post("/missions", b),
  startMission: (id: string) => api.post(`/missions/${id}/start`),
  pauseMission: (id: string) => api.post(`/missions/${id}/pause`),
  resumeMission: (id: string) => api.post(`/missions/${id}/resume`),
  abortMission: (id: string) => api.post(`/missions/${id}/abort`),
  interpret: (text: string, create: boolean) => api.post<Record<string, unknown>>("/missions/interpret", { text, create }),

  takeControl: (id: string) => api.post(`/drones/${id}/take-control`, { operator: "operator" }),
  releaseControl: (id: string) => api.post(`/drones/${id}/release-control`),
  manualInput: (id: string, axes: Axes) => api.post(`/drones/${id}/manual-input`, axes),
  pauseDrone: (id: string) => api.post(`/drones/${id}/pause`),
  resumeDrone: (id: string) => api.post(`/drones/${id}/resume`),
  returnHome: (id: string) => api.post(`/drones/${id}/return-home`),
  emergency: (id: string) => api.post(`/drones/${id}/emergency`),

  inject: (type: string, drone_id?: string, params?: Record<string, unknown>) =>
    api.post("/simulation/events", { type, drone_id, params: params ?? {} }),
  clearZone: (id: string) => api.del(`/simulation/zones/${id}`),
  clearFaults: (id: string) => api.post(`/simulation/drones/${id}/clear-faults`),
  simControl: (action: "start" | "stop" | "step" | "speed", extra: Record<string, unknown> = {}) =>
    api.post("/simulation/control", { action, ...extra }),
  reset: (seed = 42) => api.post("/simulation/reset", { seed }),
  scenarios: () => api.get<import("../types").ScenarioInfo[]>("/simulation/scenarios"),
  loadScenario: (name: string) => api.post(`/simulation/scenarios/${name}/load`),
};

export interface Axes { forward: number; strafe: number; climb: number; yaw: number }
