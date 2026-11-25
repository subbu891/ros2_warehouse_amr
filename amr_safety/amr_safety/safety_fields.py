from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

# Safety states, identical to the constants in amr_interfaces/msg/SafetyState.msg
CLEAR, WARNING, PROTECTIVE_STOP, EMERGENCY_STOP = 0, 1, 2, 3
STATE_NAMES = {CLEAR: "CLEAR", WARNING: "WARNING",
               PROTECTIVE_STOP: "PROTECTIVE_STOP", EMERGENCY_STOP: "EMERGENCY_STOP"}


# Field parameters in metres, seconds and m/s^2
@dataclass
class FieldConfig:
    footprint_front: float = 0.30
    footprint_rear: float = 0.30
    footprint_half_width: float = 0.225
    lateral_margin: float = 0.10
    t_response: float = 0.15
    a_brake: float = 1.0
    protective_margin: float = 0.10
    t_warning: float = 1.0
    warning_margin: float = 0.30
    min_speed_scale: float = 0.2
    rotation_margin: float = 0.08
    release_delay: float = 1.0
    sensor_timeout: float = 0.5

    # Docking field set: short protective field, no warning field, limited speed
    docking_margin: float = 0.02
    docking_max_speed: float = 0.15


@dataclass
class Fields:
    protective_length: float
    warning_length: float
    forward: bool


@dataclass
class Evaluation:
    state: int
    speed_scale: float
    closest_distance: float
    fields: Fields
    reason: str = ""


# Distance travelled during reaction time plus braking distance
def stopping_distance(v: float, cfg: FieldConfig) -> float:
    v = abs(v)
    return v * cfg.t_response + v * v / (2.0 * cfg.a_brake)


# Speed-dependent field lengths, switched like a safety laser scanner (ISO 3691-4 concept)
def compute_fields(v: float, cfg: FieldConfig) -> Fields:
    prot = stopping_distance(v, cfg) + cfg.protective_margin
    warn = prot + abs(v) * cfg.t_warning + cfg.warning_margin
    return Fields(protective_length=prot, warning_length=warn, forward=v >= 0.0)


# LaserScan ranges to (N, 2) points in the robot base frame
def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   range_min: float, range_max: float,
                   sensor_x: float = 0.0, sensor_y: float = 0.0, sensor_yaw: float = 0.0) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size) + sensor_yaw
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[valid], a[valid]
    return np.column_stack((sensor_x + r * np.cos(a), sensor_y + r * np.sin(a)))


# Euclidean distance from each point to the rectangular footprint, 0 inside
def distance_to_footprint(points: np.ndarray, cfg: FieldConfig) -> np.ndarray:
    if points.size == 0:
        return np.empty(0)
    dx = np.maximum(np.maximum(points[:, 0] - cfg.footprint_front, -cfg.footprint_rear - points[:, 0]), 0.0)
    dy = np.maximum(np.abs(points[:, 1]) - cfg.footprint_half_width, 0.0)
    return np.hypot(dx, dy)


# Points inside the rectangle of the given length in front of (or behind) the footprint
def _in_band(points: np.ndarray, length: float, forward: bool, cfg: FieldConfig) -> np.ndarray:
    half_w = cfg.footprint_half_width + cfg.lateral_margin
    lateral = np.abs(points[:, 1]) <= half_w
    if forward:
        ahead = points[:, 0] - cfg.footprint_front
    else:
        ahead = -cfg.footprint_rear - points[:, 0]
    return lateral & (ahead >= -0.05) & (ahead <= length)


# Classify one scan against the fields for linear speed v and yaw rate w
def evaluate(points: np.ndarray, v: float, w: float, cfg: FieldConfig) -> Evaluation:
    fields = compute_fields(v, cfg)
    if points.size == 0:
        return Evaluation(CLEAR, 1.0, math.inf, fields)

    dist = distance_to_footprint(points, cfg)
    closest = float(dist.min())

    # Contact with the footprint stops the robot in every motion mode
    if closest <= min(cfg.rotation_margin, cfg.protective_margin) * 0.5:
        return Evaluation(PROTECTIVE_STOP, 0.0, closest, fields, "object at footprint")

    # Turning on the spot uses a ring around the footprint
    if abs(v) < 0.02 and abs(w) > 0.05 and closest <= cfg.rotation_margin:
        return Evaluation(PROTECTIVE_STOP, 0.0, closest, fields, "object in rotation field")

    if np.any(_in_band(points, fields.protective_length, fields.forward, cfg)):
        return Evaluation(PROTECTIVE_STOP, 0.0, closest, fields, "object in protective field")

    # Warning field: scale speed linearly with the distance into the field
    in_warn = _in_band(points, fields.warning_length, fields.forward, cfg)
    if np.any(in_warn):
        pts = points[in_warn]
        ahead = (pts[:, 0] - cfg.footprint_front) if fields.forward else (-cfg.footprint_rear - pts[:, 0])
        d = float(ahead.min())
        span = max(fields.warning_length - fields.protective_length, 1e-6)
        scale = (d - fields.protective_length) / span
        scale = float(np.clip(scale, cfg.min_speed_scale, 1.0))
        return Evaluation(WARNING, scale, closest, fields, "object in warning field")

    return Evaluation(CLEAR, 1.0, closest, fields)


# Stateful monitor: protective stop latching, release delay, sensor watchdog and e-stop
@dataclass
class SafetyMonitor:
    cfg: FieldConfig = field(default_factory=FieldConfig)
    estop: bool = False
    docking: bool = False
    _stopped_since_clear: float | None = None
    _latched: bool = False
    _last_scan_time: float | None = None
    last: Evaluation | None = None

    def press_estop(self) -> None:
        self.estop = True

    # The e-stop never releases on its own, only through this reset
    def reset_estop(self) -> bool:
        was = self.estop
        self.estop = False
        return was

    def scan_received(self, now: float) -> None:
        self._last_scan_time = now

    # Field set switching: docking needs to approach the dock closer than the normal protective field allows
    def active_config(self) -> FieldConfig:
        if not self.docking:
            return self.cfg
        return replace(self.cfg, protective_margin=self.cfg.docking_margin,
                       warning_margin=0.0, t_warning=0.0)

    def max_speed(self) -> float:
        return self.cfg.docking_max_speed if self.docking else math.inf

    def update(self, points: np.ndarray, v: float, w: float, now: float) -> Evaluation:
        cfg = self.active_config()
        fields = compute_fields(v, cfg)
        if self.estop:
            self.last = Evaluation(EMERGENCY_STOP, 0.0, math.inf, fields, "e-stop pressed")
            return self.last

        # Missing lidar data is treated as a protective stop
        if self._last_scan_time is None or now - self._last_scan_time > cfg.sensor_timeout:
            self._latched = True
            self._stopped_since_clear = None
            self.last = Evaluation(PROTECTIVE_STOP, 0.0, math.inf, fields, "scan timeout")
            return self.last

        ev = evaluate(points, v, w, cfg)
        if ev.state == PROTECTIVE_STOP:
            self._latched = True
            self._stopped_since_clear = None
        elif self._latched:
            # Stay stopped until the field has been clear for release_delay seconds
            if self._stopped_since_clear is None:
                self._stopped_since_clear = now
            if now - self._stopped_since_clear < cfg.release_delay:
                ev = Evaluation(PROTECTIVE_STOP, 0.0, ev.closest_distance, ev.fields, "release delay")
            else:
                self._latched = False
                self._stopped_since_clear = None
        self.last = ev
        return ev


# Apply an evaluation and an optional speed cap to a velocity command
def limit_command(v: float, w: float, ev: Evaluation, max_speed: float = math.inf) -> tuple[float, float]:
    if ev.state in (PROTECTIVE_STOP, EMERGENCY_STOP):
        return 0.0, 0.0
    if ev.state == WARNING:
        v = v * ev.speed_scale
    return float(np.clip(v, -max_speed, max_speed)), w
