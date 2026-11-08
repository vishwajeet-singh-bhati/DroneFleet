// Shapes of the backend payloads (REST + /ws/fleet). The backend is the source of truth;
// the UI only displays these and sends commands.

export type FlightMode =
  | "IDLE" | "TAKEOFF" | "AUTONOMOUS" | "TAKEOVER_REQUEST" | "MANUAL" | "REJOINING"
  | "RETURN_TO_HOME" | "PAUSED" | "LANDING" | "EMERGENCY";

export interface Drone {
  drone_id: string;
  position_x: number;
  position_y: number;
  altitude: number;
  velocity: number;
  heading: number;
  battery_percentage: number;
  health_percentage: number;
  temperature: number;
  gps_quality: number;
  signal_strength: number;
  current_mission_id: string | null;
  current_waypoint_index: number;
  total_waypoints: number;
  active_waypoint: { id: string; x: number; y: number; action: string } | null;
  route_label: string;
  flight_mode: FlightMode;
  status: string;
  capabilities: string[];
  reliability: number;
  home: { x: number; y: number };
  active_route?: [number, number][];
}

export interface Waypoint { id: string; x: number; y: number; altitude: number; action: string }

export interface RouteRecord {
  revision: number; drone_id: string; t: number; reason: string;
  old: [number, number][]; new: [number, number][]; dropped: string[];
}

export interface Ranking {
  drone_id: string; eligible: boolean; score: number;
  contributions: Record<string, number>; reasons: string[]; rejections: string[];
}

export interface Mission {
  mission_id: string; name: string; status: string; priority: number; target_zone_id: string;
  num_drones: number; assigned_drones: string[]; released_drones: string[];
  routes: Record<string, Waypoint[]>; route_history: RouteRecord[];
  progress: number; deadline_s: number | null; failure_reason: string | null;
  plan_info?: { targets_total: number; estimated_duration_s: number; rankings: Ranking[] };
  report?: Record<string, unknown> | null;
}

export interface Zone {
  zone_id: string; kind: "INSPECTION" | "RESTRICTED" | "OBSTACLE" | "WEATHER" | string;
  name: string; x0: number; y0: number; x1: number; y1: number; active: boolean;
}

export interface World {
  width: number; height: number; cell_size: number; version: number; zones: Zone[];
  home: { x: number; y: number };
}

export interface Summary {
  fleet_count: number; active_drones: number; available_drones: number; active_missions: number;
  total_missions: number; completed_missions: number; mission_completion: number;
  average_health: number; average_battery: number; alerts: number;
  attention: { drone_id: string; reasons: string[] }[];
}

export interface MaintenanceItem {
  drone_id: string; estimated_health: number; anomaly_probability: number;
  maintenance_risk: "LOW" | "MEDIUM" | "HIGH" | string; reason: string; drivers: string[];
  isolation_flag: boolean;
}

export interface ScenarioStatus {
  name: string | null; title?: string; description?: string; finished?: boolean;
  passed?: boolean | null; steps_done?: number; steps_total?: number;
  checks?: { name: string; ok: boolean; detail: string }[];
}

export interface FleetState {
  sim_time: number; tick: number; seed: number; drones: Drone[]; missions: Mission[];
  summary: Summary; world_version: number;
  control: Record<string, { operator: string; since: number }>;
  coverage: string[];   // ids of inspected targets
  maintenance: Record<string, MaintenanceItem>;
  scenario: ScenarioStatus | null;
  running: boolean; speed: number;
}

export interface LogEvent {
  id: number; type: string; message: string; severity: "INFO" | "WARNING" | "CRITICAL" | string;
  drone_id: string | null; mission_id: string | null; clock: string; sim_time: number;
}

export interface Decision {
  id: number; kind: string; summary: string; clock: string; sim_time: number;
  mission_id: string | null; drone_id: string | null; score: number | null;
  factors: Record<string, number>; reasons: string[]; extra: Record<string, unknown>;
}

export interface ScenarioInfo { id: string; number: number; title: string; description: string }
