from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


# Index pairs (i_ref, i_est) of nearest timestamps within max_dt; both arrays must be sorted
def associate(t_ref: np.ndarray, t_est: np.ndarray, max_dt: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    if len(t_ref) == 0 or len(t_est) == 0:
        return np.empty(0, int), np.empty(0, int)
    idx = np.clip(np.searchsorted(t_ref, t_est), 1, len(t_ref) - 1)
    left, right = t_ref[idx - 1], t_ref[idx]
    idx = idx - ((t_est - left) < (right - t_est))
    ok = np.abs(t_ref[idx] - t_est) <= max_dt
    return idx[ok], np.nonzero(ok)[0]


@dataclass
class PoseErrorStats:
    n: int
    rmse_xy: float
    max_xy: float
    mean_xy: float
    rmse_yaw: float
    max_yaw: float


# Position and heading error between ground truth and estimate, both (N, 4) arrays of [t, x, y, yaw]
def pose_error(gt: np.ndarray, est: np.ndarray, max_dt: float = 0.1) -> PoseErrorStats:
    i_gt, i_est = associate(gt[:, 0], est[:, 0], max_dt)
    if len(i_gt) == 0:
        return PoseErrorStats(0, math.nan, math.nan, math.nan, math.nan, math.nan)
    dxy = np.hypot(gt[i_gt, 1] - est[i_est, 1], gt[i_gt, 2] - est[i_est, 2])
    dyaw = np.abs(wrap_angle(gt[i_gt, 3] - est[i_est, 3]))
    return PoseErrorStats(len(i_gt), float(np.sqrt(np.mean(dxy ** 2))), float(dxy.max()),
                          float(dxy.mean()), float(np.sqrt(np.mean(dyaw ** 2))), float(dyaw.max()))


def path_length(xy: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


# Speed per segment of an (N, 3) trajectory [t, x, y]
def speeds(traj: np.ndarray) -> np.ndarray:
    if len(traj) < 2:
        return np.empty(0)
    dt = np.diff(traj[:, 0])
    d = np.linalg.norm(np.diff(traj[:, 1:3], axis=0), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        v = np.where(dt > 0, d / dt, 0.0)
    return v


def in_box(xy: np.ndarray, box_min, box_max, margin: float = 0.0) -> np.ndarray:
    return ((xy[:, 0] >= box_min[0] - margin) & (xy[:, 0] <= box_max[0] + margin) &
            (xy[:, 1] >= box_min[1] - margin) & (xy[:, 1] <= box_max[1] + margin))


# Number of trajectory samples with the robot centre inside a keepout zone (+ radius)
def keepout_violations(xy: np.ndarray, zones: list[dict], robot_radius: float = 0.0) -> int:
    return int(sum(in_box(xy, z["min"], z["max"], robot_radius).sum() for z in zones))


# Maximum moving-average ground-truth speed while the robot centre is inside the zone
def max_speed_in_zone(traj: np.ndarray, zone: dict, smooth: int = 5) -> float:
    v = speeds(traj)
    if len(v) == 0:
        return 0.0
    if smooth > 1 and len(v) >= smooth:
        v = np.convolve(v, np.ones(smooth) / smooth, mode="same")
    mid = (traj[1:, 1:3] + traj[:-1, 1:3]) / 2
    inside = in_box(mid, zone["min"], zone["max"])
    return float(v[inside].max()) if inside.any() else 0.0


# Distance travelled from t_event until the robot is practically stationary
def stopping_distance_after(traj: np.ndarray, t_event: float) -> float:
    after = traj[traj[:, 0] >= t_event]
    if len(after) < 2:
        return 0.0
    v = speeds(after)
    still = np.nonzero(v < 0.02)[0]
    end = still[0] + 1 if len(still) else len(after)
    return path_length(after[:end, 1:3])


# First time after which the error stays below threshold for hold seconds, nan if never
def convergence_time(err_t: np.ndarray, err: np.ndarray, threshold: float, hold: float = 2.0) -> float:
    below = err < threshold
    for i in range(len(err)):
        if below[i]:
            window = (err_t >= err_t[i]) & (err_t <= err_t[i] + hold)
            if below[window].all() and err_t[-1] >= err_t[i] + hold:
                return float(err_t[i] - err_t[0])
    return math.nan


@dataclass
class Check:
    name: str
    value: float
    limit: float
    # Either "<=" or ">="
    comparison: str

    @property
    def passed(self) -> bool:
        if math.isnan(self.value):
            return False
        return self.value <= self.limit if self.comparison == "<=" else self.value >= self.limit

    def __str__(self) -> str:
        mark = "PASS" if self.passed else "FAIL"
        return f"[{mark}] {self.name}: {self.value:.3f} (limit {self.comparison} {self.limit})"


def success_rate(results: list[bool]) -> float:
    return 100.0 * sum(results) / len(results) if results else math.nan


# Wilson 95 % lower confidence bound of a success rate, honest reporting for small samples
def wilson_lower_bound(successes: int, n: int, z: float = 1.96) -> float:
    if n == 0:
        return math.nan
    p = successes / n
    denom = 1 + z * z / n
    centre = p + z * z / (2 * n)
    margin = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return 100.0 * (centre - margin) / denom
