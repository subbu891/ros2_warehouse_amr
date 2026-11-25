from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from amr_validation import metrics as m


@dataclass
class RunData:
    # Ground truth (N, 4): t, x, y, yaw
    gt: np.ndarray
    # AMCL estimate (M, 4): t, x, y, yaw
    est: np.ndarray
    # Safety state (K, 4): t, state, closest distance, speed scale
    safety: np.ndarray
    mission_success: bool
    mission_message: str = ""
    duration: float = 0.0
    # Event timestamps such as estop_t
    events: dict = field(default_factory=dict)
    # Robot pose (x, y, yaw) at the end of each mission step
    step_end_poses: dict = field(default_factory=dict)


def merge_checks(defaults: dict, scenario: dict) -> dict:
    checks = dict(defaults.get("checks", {}))
    checks.update(scenario.get("checks", {}) or {})
    return {k: v for k, v in checks.items() if v is not None}


def evaluate(scenario: dict, defaults: dict, data: RunData, layout: dict) -> list[m.Check]:
    out: list[m.Check] = []
    if scenario.get("expect", "success") == "success":
        out.append(m.Check("mission_success", float(data.mission_success), 1.0, ">="))

    checks = merge_checks(defaults, scenario)
    gt_xy = data.gt[:, 1:3] if len(data.gt) else np.empty((0, 2))

    if "min_clearance" in checks:
        closest = data.safety[:, 2] if len(data.safety) else np.array([math.inf])
        closest = closest[np.isfinite(closest)]
        value = float(closest.min()) if len(closest) else math.inf
        out.append(m.Check("min_clearance_m", value, checks["min_clearance"], ">="))

    if "localization_rmse" in checks or "localization_max" in checks:
        err = m.pose_error(data.gt, data.est)
        if "localization_rmse" in checks:
            out.append(m.Check("localization_rmse_m", err.rmse_xy, checks["localization_rmse"], "<="))
        if "localization_max" in checks:
            out.append(m.Check("localization_max_m", err.max_xy, checks["localization_max"], "<="))

    if "max_speed_in_zone" in checks:
        spec = checks["max_speed_in_zone"]
        zone = next(z for z in layout["speed_zones"] if z["name"] == spec["zone"])
        v = m.max_speed_in_zone(data.gt[:, :3], zone)
        out.append(m.Check(f"max_speed_in_{spec['zone']}_mps", v, spec["limit"], "<="))

    if "keepout_violations" in checks:
        n = m.keepout_violations(gt_xy, layout["keepout_zones"])
        out.append(m.Check("keepout_violations", float(n), float(checks["keepout_violations"]), "<="))

    if "stopping_distance" in checks:
        t = data.events.get("estop_t")
        d = m.stopping_distance_after(data.gt[:, :3], t) if t is not None else math.nan
        out.append(m.Check("estop_stopping_distance_m", d, checks["stopping_distance"], "<="))

    if "convergence_time" in checks:
        spec = checks["convergence_time"]
        i_gt, i_est = m.associate(data.gt[:, 0], data.est[:, 0]) if len(data.gt) else ([], [])
        if len(i_gt):
            err = np.hypot(data.gt[i_gt, 1] - data.est[i_est, 1], data.gt[i_gt, 2] - data.est[i_est, 2])
            ct = m.convergence_time(data.est[i_est, 0], err, spec["threshold"])
        else:
            ct = math.nan
        out.append(m.Check("localization_convergence_s", ct, spec["limit"], "<="))

    if "dock_error" in checks:
        spec = checks["dock_error"]
        dock_idx = next((i for i, s in enumerate(scenario["steps"]) if s.startswith("dock:")), None)
        pose = data.step_end_poses.get(dock_idx)
        if pose is None:
            exy = eyaw = math.nan
        else:
            dx, dy, dyaw = spec["dock_pose"]
            exy = math.hypot(pose[0] - dx, pose[1] - dy)
            eyaw = abs(float(m.wrap_angle(pose[2] - dyaw)))
        out.append(m.Check("dock_error_xy_m", exy, spec["xy"], "<="))
        out.append(m.Check("dock_error_yaw_rad", eyaw, spec["yaw"], "<="))
    return out


# Markdown report from a list of scenario results
def report_markdown(results: list[dict]) -> str:
    n_pass = sum(r["passed"] for r in results)
    lines = ["# Scenario test report", "",
             f"**{n_pass}/{len(results)} scenarios passed**", "",
             "| Scenario | Result | Duration [s] | Notes |", "|---|---|---|---|"]
    for r in results:
        failed = [c.name for c in r["checks"] if not c.passed]
        note = ", ".join(failed) if failed else r.get("message", "")
        lines.append(f"| {r['name']} | {'PASS' if r['passed'] else 'FAIL'} | {r['duration']:.1f} | {note} |")
    for r in results:
        lines += ["", f"## {r['name']}", "", r.get("description", ""), "",
                  "| Check | Value | Limit | Result |", "|---|---|---|---|"]
        for c in r["checks"]:
            lines.append(f"| {c.name} | {c.value:.3f} | {c.comparison} {c.limit} | "
                         f"{'PASS' if c.passed else 'FAIL'} |")
    return "\n".join(lines) + "\n"
