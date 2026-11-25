from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


# Occupancy grid with a dilated mask of static cells for fast point lookups
class StaticMap:
    def __init__(self, data: np.ndarray, resolution: float, origin_xy: tuple[float, float],
                 dilation_m: float = 0.15, occupied_threshold: int = 50):
        # Occupied and unknown cells both count as static
        occ = (data >= occupied_threshold) | (data < 0)
        k = max(int(math.ceil(dilation_m / resolution)), 0)
        self.mask = _dilate(occ, k)
        self.res = resolution
        self.origin = origin_xy

    def is_static(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.zeros(0, dtype=bool)
        cols = np.floor((points[:, 0] - self.origin[0]) / self.res).astype(int)
        rows = np.floor((points[:, 1] - self.origin[1]) / self.res).astype(int)
        h, w = self.mask.shape
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        # Points outside the map are treated as static
        out = np.ones(len(points), dtype=bool)
        out[inside] = self.mask[rows[inside], cols[inside]]
        return out


# Square binary dilation by k cells without a SciPy dependency
def _dilate(mask: np.ndarray, k: int) -> np.ndarray:
    if k == 0:
        return mask.copy()
    h, w = mask.shape
    padded = np.pad(mask, k)
    out = np.zeros_like(mask)
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            out |= padded[k + dr:k + dr + h, k + dc:k + dc + w]
    return out


@dataclass
class Cluster:
    centroid: np.ndarray
    points: np.ndarray

    @property
    def radius(self) -> float:
        return float(np.max(np.linalg.norm(self.points - self.centroid, axis=1))) if len(self.points) else 0.0


# Split scan-ordered points wherever consecutive points are further apart than gap
def cluster_points(points: np.ndarray, gap: float = 0.3, min_points: int = 3) -> list[Cluster]:
    if len(points) == 0:
        return []
    jumps = np.linalg.norm(np.diff(points, axis=0), axis=1) > gap
    starts = np.concatenate(([0], np.nonzero(jumps)[0] + 1))
    ends = np.concatenate((starts[1:], [len(points)]))
    segments = [points[s:e] for s, e in zip(starts, ends)]
    # The scan wraps around, so the last and first segment may belong together
    if len(segments) > 1 and np.linalg.norm(points[-1] - points[0]) <= gap:
        segments[0] = np.vstack((segments[-1], segments[0]))
        segments.pop()
    return [Cluster(seg.mean(axis=0), seg) for seg in segments if len(seg) >= min_points]


@dataclass
class Track:
    track_id: int
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 0.3
    hits: int = 1
    last_seen: float = 0.0

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))


# Association gate, filter gains, track management and prediction horizon
@dataclass
class TrackerConfig:
    gate: float = 1.0
    alpha: float = 0.5
    beta: float = 0.3
    confirm_hits: int = 3
    max_age: float = 1.0
    moving_speed: float = 0.15
    horizon: float = 2.0
    horizon_step: float = 0.25


# Multi-object tracker: nearest-neighbour association and an alpha-beta constant-velocity filter
class MultiTracker:
    def __init__(self, cfg: TrackerConfig | None = None):
        self.cfg = cfg or TrackerConfig()
        self.tracks: list[Track] = []
        self._next_id = 1
        self._last_time: float | None = None

    def update(self, clusters: list[Cluster], now: float) -> list[Track]:
        dt = 0.0 if self._last_time is None else max(now - self._last_time, 1e-3)
        self._last_time = now

        # Predict every track to the current time
        for t in self.tracks:
            t.position = t.position + t.velocity * dt

        # Greedy association, closest track-cluster pairs first
        pairs = []
        for ti, t in enumerate(self.tracks):
            for ci, c in enumerate(clusters):
                d = float(np.linalg.norm(t.position - c.centroid))
                if d <= self.cfg.gate:
                    pairs.append((d, ti, ci))
        pairs.sort()
        used_t, used_c = set(), set()
        for _, ti, ci in pairs:
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)

            # Alpha-beta correction of position and velocity
            t, c = self.tracks[ti], clusters[ci]
            residual = c.centroid - t.position
            t.position = t.position + self.cfg.alpha * residual
            if dt > 0:
                t.velocity = t.velocity + self.cfg.beta * residual / dt
            t.radius = max(0.5 * t.radius + 0.5 * c.radius, 0.1)
            t.hits += 1
            t.last_seen = now

        # Unmatched clusters start new tracks; old tracks expire
        for ci, c in enumerate(clusters):
            if ci not in used_c:
                self.tracks.append(Track(self._next_id, c.centroid.copy(), radius=max(c.radius, 0.1),
                                         last_seen=now))
                self._next_id += 1
        self.tracks = [t for t in self.tracks if now - t.last_seen <= self.cfg.max_age]
        return self.confirmed()

    def confirmed(self) -> list[Track]:
        return [t for t in self.tracks if t.hits >= self.cfg.confirm_hits]

    def moving(self) -> list[Track]:
        return [t for t in self.confirmed() if t.speed >= self.cfg.moving_speed]


# Ring of points around each predicted future position of every track
def predict_points(tracks: list[Track], horizon: float, step: float, ring_points: int = 8) -> np.ndarray:
    out = []
    angles = np.linspace(0, 2 * math.pi, ring_points, endpoint=False)
    ring = np.column_stack((np.cos(angles), np.sin(angles)))
    for t in tracks:
        for k in np.arange(step, horizon + 1e-9, step):
            centre = t.position + t.velocity * k
            out.append(centre + ring * t.radius)
            out.append(centre[None, :])
    return np.vstack(out) if out else np.empty((0, 2))


def transform_points(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])
