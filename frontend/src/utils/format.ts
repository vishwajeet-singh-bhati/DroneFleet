export const droneColor = (id: string): string => {
  const n = parseInt(id.replace(/\D/g, ""), 10) || 0;
  return `hsl(${(n * 47) % 360} 75% 62%)`;
};

export const MODE_COLOR: Record<string, string> = {
  IDLE: "#7b8794", TAKEOFF: "#4cc9f0", AUTONOMOUS: "#3ddc97", TAKEOVER_REQUEST: "#ffd166",
  MANUAL: "#ff9f1c", REJOINING: "#b388ff", RETURN_TO_HOME: "#4dabf7", LANDING: "#4cc9f0",
  PAUSED: "#adb5bd", EMERGENCY: "#ff4d6d",
};

export const pct = (v: number, d = 0) => `${v.toFixed(d)}%`;
export const clockOf = (t: number) => {
  const s = Math.floor(t);
  return `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
};
