from pathlib import Path

import numpy as np
import yaml

from amr_validation import scenario_checks as sc

REPO = Path(__file__).resolve().parents[2]
LAYOUT = yaml.safe_load((REPO / "amr_gazebo" / "config" / "warehouse_layout.yaml").read_text())
SPEC = yaml.safe_load((REPO / "amr_validation" / "config" / "scenarios.yaml").read_text())
DEFAULTS = SPEC["defaults"]


def scenario(name):
    return next(s for s in SPEC["scenarios"] if s["name"] == name)


def straight_run(x0, x1, y, v, dt=0.1, est_offset=0.02):
    n = int((x1 - x0) / (v * dt)) + 1
    t = np.arange(n) * dt
    gt = np.column_stack((t, x0 + v * t, np.full(n, y), np.zeros(n)))
    est = gt.copy()
    est[:, 2] += est_offset
    saf = np.column_stack((t, np.zeros(n), np.full(n, 1.0), np.ones(n)))
    return gt, est, saf


def by_name(checks):
    return {c.name: c for c in checks}


def test_every_scenario_references_valid_zones_and_steps():
    zone_names = {z["name"] for z in LAYOUT["speed_zones"]}
    for s in SPEC["scenarios"]:
        assert s["steps"], s["name"]
        spec = (s.get("checks") or {}).get("max_speed_in_zone")
        if spec:
            assert spec["zone"] in zone_names


def test_nominal_pass():
    gt, est, saf = straight_run(2, 10, 1.5, 0.5)
    checks = sc.evaluate(scenario("nominal_transport"), DEFAULTS, sc.RunData(gt, est, saf, True), LAYOUT)
    assert all(c.passed for c in checks), [str(c) for c in checks]


def test_mission_failure_and_bad_localization_fail():
    gt, est, saf = straight_run(2, 10, 1.5, 0.5, est_offset=0.5)
    c = by_name(sc.evaluate(scenario("nominal_transport"), DEFAULTS, sc.RunData(gt, est, saf, False), LAYOUT))
    assert not c["mission_success"].passed
    assert not c["localization_rmse_m"].passed


def test_contact_fails_min_clearance():
    gt, est, saf = straight_run(2, 10, 1.5, 0.5)
    saf[10, 2] = 0.0
    c = by_name(sc.evaluate(scenario("sudden_obstacle"), DEFAULTS, sc.RunData(gt, est, saf, False), LAYOUT))
    # The scenario expects any outcome, so mission success is not checked
    assert "mission_success" not in c
    assert not c["min_clearance_m"].passed


def test_speed_zone_check():
    fast = straight_run(3, 12.9, 5.5, 0.8)
    slow = straight_run(3, 12.9, 5.5, 0.4)
    s = scenario("speed_zone")
    assert not by_name(sc.evaluate(s, DEFAULTS, sc.RunData(*fast, True), LAYOUT))["max_speed_in_rack_aisles_mps"].passed
    assert by_name(sc.evaluate(s, DEFAULTS, sc.RunData(*slow, True), LAYOUT))["max_speed_in_rack_aisles_mps"].passed


def test_keepout_check():
    # Trajectory straight through the office walkway
    through = straight_run(15.0, 19.0, 9.3, 0.5)
    c = by_name(sc.evaluate(scenario("keepout_respect"), DEFAULTS, sc.RunData(*through, True), LAYOUT))
    assert not c["keepout_violations"].passed


def test_disabled_default_checks_are_skipped():
    gt, est, saf = straight_run(2, 10, 4.0, 0.5, est_offset=0.4)
    c = by_name(sc.evaluate(scenario("localization_recovery"), DEFAULTS, sc.RunData(gt, est, saf, True), LAYOUT))
    assert "localization_rmse_m" not in c and "localization_convergence_s" in c


def test_dock_error_uses_pose_at_end_of_dock_step():
    gt, est, saf = straight_run(1, 3, 2.0, 0.5)
    s = scenario("docking")
    good = sc.RunData(gt, est, saf, True, step_end_poses={0: (0.76, 2.01, 3.14)})
    bad = sc.RunData(gt, est, saf, True, step_end_poses={0: (0.9, 2.0, 3.14)})
    assert by_name(sc.evaluate(s, DEFAULTS, good, LAYOUT))["dock_error_xy_m"].passed
    assert not by_name(sc.evaluate(s, DEFAULTS, bad, LAYOUT))["dock_error_xy_m"].passed


def test_report_markdown():
    gt, est, saf = straight_run(2, 10, 1.5, 0.5)
    checks = sc.evaluate(scenario("nominal_transport"), DEFAULTS, sc.RunData(gt, est, saf, True), LAYOUT)
    md = sc.report_markdown([{"name": "nominal_transport", "passed": True, "duration": 42.0, "checks": checks}])
    assert "1/1 scenarios passed" in md and "| nominal_transport | PASS |" in md
