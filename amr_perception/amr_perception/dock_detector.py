from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# Detection window in the lidar frame and the geometry of the docked robot relative to the dock face
@dataclass
class DockConfig:
    window_half_width: float = 0.5
    min_range: float = 0.10
    max_range: float = 2.0
    cluster_gap: float = 0.1
    min_points: int = 8
    min_width: float = 0.3
    max_width: float = 0.9
    docked_gap: float = 0.05
    footprint_front: float = 0.30


@dataclass
class DockDetection:
    x: float
    y: float
    yaw: float
    width: float
    points: int


def ranges_to_points(ranges, angle_min: float, angle_increment: float) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r)
    return np.column_stack((r[valid] * np.cos(a[valid]), r[valid] * np.sin(a[valid])))


# Find the dock face as the closest flat segment in front of the lidar and return the docked base pose
def detect_dock(points: np.ndarray, cfg: DockConfig | None = None) -> DockDetection | None:
    cfg = cfg or DockConfig()
    if points.size == 0:
        return None

    # Candidate points straight ahead of the robot
    window = points[(points[:, 0] >= cfg.min_range) & (points[:, 0] <= cfg.max_range)
                    & (np.abs(points[:, 1]) <= cfg.window_half_width)]
    if len(window) < cfg.min_points:
        return None

    # Split into surfaces where neighbouring points (by bearing) are further apart than cluster_gap
    window = window[np.argsort(np.arctan2(window[:, 1], window[:, 0]))]
    breaks = np.nonzero(np.linalg.norm(np.diff(window, axis=0), axis=1) > cfg.cluster_gap)[0] + 1
    surfaces = [c for c in np.split(window, breaks) if len(c) >= cfg.min_points]
    if not surfaces:
        return None

    # The dock stands in front of the wall, so its face is the nearest surface
    face = min(surfaces, key=lambda c: float(np.median(np.linalg.norm(c, axis=1))))

    # Line fit by principal component analysis
    mean = face.mean(axis=0)
    _, _, vt = np.linalg.svd(face - mean)
    direction = vt[0]
    proj = (face - mean) @ direction
    width = float(proj.max() - proj.min())
    if not cfg.min_width <= width <= cfg.max_width:
        return None
    centre = mean + direction * (proj.max() + proj.min()) / 2

    # Face normal pointing back towards the lidar
    normal = np.array([-direction[1], direction[0]])
    if normal @ (-centre) < 0:
        normal = -normal

    # Docked base pose: in front of the face at gap plus footprint front, facing the dock
    base = centre + normal * (cfg.docked_gap + cfg.footprint_front)
    yaw = math.atan2(-normal[1], -normal[0])
    return DockDetection(float(base[0]), float(base[1]), yaw, width, len(face))
