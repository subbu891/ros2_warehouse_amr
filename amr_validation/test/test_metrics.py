import math

import numpy as np
import pytest

from amr_validation import metrics as m


def test_associate_nearest_timestamps():
    i_ref, i_est = m.associate(np.array([0.0, 1.0, 2.0]), np.array([0.04, 1.6, 2.02, 5.0]), max_dt=0.1)
    assert list(i_ref) == [0, 2] and list(i_est) == [0, 2]


def test_pose_error_constant_offset():
    t = np.arange(0, 10, 0.1)
    gt = np.column_stack((t, t, np.zeros_like(t), np.zeros_like(t)))
    est = gt.copy()
    est[:, 2] += 0.1
    est[:, 3] += 0.05
    s = m.pose_error(gt, est)
    assert s.n == len(t)
    assert s.rmse_xy == pytest.approx(0.1) and s.max_xy == pytest.approx(0.1)
    assert s.rmse_yaw == pytest.approx(0.05)


def test_yaw_error_wraps():
    gt = np.array([[0.0, 0, 0, math.pi - 0.01]])
    est = np.array([[0.0, 0, 0, -math.pi + 0.01]])
    assert m.pose_error(gt, est).max_yaw == pytest.approx(0.02)


def test_no_overlap_gives_nan():
    s = m.pose_error(np.array([[0.0, 0, 0, 0]]), np.array([[9.0, 0, 0, 0]]))
    assert s.n == 0 and math.isnan(s.rmse_xy)


def test_speeds_and_path_length():
    traj = np.array([[0, 0, 0], [1, 0.5, 0], [2, 1.0, 0], [3, 1.0, 0]], dtype=float)
    np.testing.assert_allclose(m.speeds(traj), [0.5, 0.5, 0.0])
    assert m.path_length(traj[:, 1:]) == pytest.approx(1.0)


def test_keepout_violations():
    xy = np.array([[0.0, 0.0], [5.0, 5.0], [5.5, 5.5]])
    zones = [{"min": [4.9, 4.9], "max": [5.2, 5.2]}]
    assert m.keepout_violations(xy, zones) == 1
    assert m.keepout_violations(xy, zones, robot_radius=0.4) == 2


def test_max_speed_in_zone():
    t = np.arange(0, 20, 0.1)
    # The robot slows down after x = 8
    x = np.where(t < 10, 0.8 * t, 8 + 0.4 * (t - 10))
    traj = np.column_stack((t, x, np.zeros_like(t)))
    zone = {"min": [9.0, -1.0], "max": [11.0, 1.0]}
    assert m.max_speed_in_zone(traj, zone) == pytest.approx(0.4, abs=0.02)


def test_stopping_distance():
    t = np.arange(0, 3, 0.1)
    # Braking at t = 1 s with 1 m/s^2
    v = np.clip(0.8 - 1.0 * np.clip(t - 1.0, 0, None), 0, None)
    x = np.concatenate(([0], np.cumsum(v[:-1] * 0.1)))
    traj = np.column_stack((t, x, np.zeros_like(t)))
    assert m.stopping_distance_after(traj, 1.0) == pytest.approx(0.8 ** 2 / 2, abs=0.06)


def test_convergence_time():
    t = np.arange(0, 10, 0.5)
    err = np.where(t < 4, 0.5, 0.05)
    assert m.convergence_time(t, err, threshold=0.1, hold=2.0) == pytest.approx(4.0)
    assert math.isnan(m.convergence_time(t, np.full_like(t, 0.5), 0.1))


def test_check_and_rates():
    assert m.Check("ate", 0.05, 0.1, "<=").passed
    assert not m.Check("rate", 90.0, 95.0, ">=").passed
    assert not m.Check("x", math.nan, 1.0, "<=").passed
    assert m.success_rate([True, True, False, True]) == 75.0
    assert m.wilson_lower_bound(100, 100) == pytest.approx(96.3, abs=0.1)
    # 20 out of 20 is weak evidence
    assert m.wilson_lower_bound(20, 20) < 85.0
