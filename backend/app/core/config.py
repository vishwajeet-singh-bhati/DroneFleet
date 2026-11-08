"""Central configuration. All simulation constants live here so tests can override them."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class WorldConfig:
    width: float = 1000.0          # metres (x axis, east)
    height: float = 1000.0         # metres (y axis, north)
    cell_size: float = 20.0        # metres per planning-grid cell
    home: tuple[float, float] = (60.0, 60.0)


@dataclass(frozen=True)
class SimConfig:
    seed: int = 42
    dt: float = 0.5                      # simulated seconds per tick
    cruise_speed: float = 18.0           # m/s
    max_speed: float = 25.0
    manual_speed: float = 14.0           # m/s under manual control
    vertical_speed: float = 4.0          # m/s
    yaw_rate_deg: float = 60.0           # deg/s
    waypoint_threshold: float = 6.0      # arrival radius (m)
    default_altitude: float = 30.0
    min_altitude: float = 5.0
    max_altitude: float = 120.0
    # battery model (percent)
    battery_drain_per_metre: float = 0.012
    battery_idle_drain_per_s: float = 0.002
    battery_hover_drain_per_s: float = 0.02
    battery_warn: float = 30.0
    battery_critical: float = 15.0
    # temperature model (celsius)
    ambient_temp: float = 25.0
    max_safe_temp: float = 75.0
    inspect_dwell_s: float = 2.0         # time spent hovering at INSPECT waypoints


@dataclass(frozen=True)
class Settings:
    world: WorldConfig = field(default_factory=WorldConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    fleet_size: int = 12
    database_url: str = "sqlite:///./data/dronefleet.db"


DEFAULT_SETTINGS = Settings()
