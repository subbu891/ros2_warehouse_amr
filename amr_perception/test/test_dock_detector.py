import math

import numpy as np

from amr_perception.dock_detector import DockConfig, detect_dock, ranges_to_points


def rotate(points, angle):
    c, s = math.cos(angle), math.sin(angle)
    return points @ np.array([[c, -s], [s, c]]).T


# Dock face 0.6 m wide at distance d and the wall 0.4 m behind it; wall points hidden by the dock are removed
def dock_scene(d=0.83, step=0.015):
    face = np.column_stack((np.full(41, d), np.linspace(-0.3, 0.3, 41)))
    wall_y = np.arange(-1.5, 1.5, step)
    wall = np.column_stack((np.full(len(wall_y), d + 0.4), wall_y))
    visible = np.abs(np.arctan2(wall[:, 1], wall[:, 0])) > math.atan2(0.3, d)
    return np.vstack((face, wall[visible]))


def test_docked_pose_straight_ahead():
    det = detect_dock(dock_scene())
    assert det is not None
    assert abs(det.x - (0.83 - 0.35)) < 0.01
    assert abs(det.y) < 0.01 and abs(det.yaw) < 0.01
    assert abs(det.width - 0.6) < 0.02


def test_docked_pose_follows_a_rotated_dock():
    angle = math.radians(10)
    det = detect_dock(rotate(dock_scene(), angle))
    expected = rotate(np.array([[0.83 - 0.35, 0.0]]), angle)[0]
    assert abs(det.x - expected[0]) < 0.01 and abs(det.y - expected[1]) < 0.01
    assert abs(det.yaw - angle) < 0.02


def test_plain_wall_is_not_a_dock():
    wall = np.column_stack((np.full(80, 1.2), np.linspace(-0.6, 0.6, 80)))
    assert detect_dock(wall) is None


def test_too_few_points_or_nothing_ahead():
    assert detect_dock(np.empty((0, 2))) is None
    assert detect_dock(np.array([[0.8, 0.0], [0.8, 0.1]])) is None
    assert detect_dock(np.array([[-1.0, 0.0]] * 20)) is None


def test_ranges_to_points_drops_invalid_beams():
    pts = ranges_to_points([1.0, math.inf, 2.0], angle_min=0.0, angle_increment=math.pi / 2)
    np.testing.assert_allclose(pts, [[1.0, 0.0], [-2.0, 0.0]], atol=1e-9)


def test_custom_geometry():
    det = detect_dock(dock_scene(), DockConfig(docked_gap=0.10, footprint_front=0.40))
    assert abs(det.x - (0.83 - 0.50)) < 0.01
