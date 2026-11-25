import math

import numpy as np
import pytest

from amr_safety.safety_fields import (CLEAR, EMERGENCY_STOP, PROTECTIVE_STOP, WARNING, FieldConfig,
                                      SafetyMonitor, compute_fields, distance_to_footprint, evaluate,
                                      limit_command, scan_to_points, stopping_distance)

CFG = FieldConfig()


def pts(*xy):
    return np.array(xy, dtype=float).reshape(-1, 2)


def test_stopping_distance_grows_quadratically():
    assert stopping_distance(0.0, CFG) == 0.0
    d1, d2 = stopping_distance(0.4, CFG), stopping_distance(0.8, CFG)
    # Braking term is quadratic in speed
    assert d2 > 2 * d1
    assert stopping_distance(-0.5, CFG) == stopping_distance(0.5, CFG)


def test_fields_are_nested_and_speed_dependent():
    slow, fast = compute_fields(0.2, CFG), compute_fields(0.8, CFG)
    assert slow.warning_length > slow.protective_length
    assert fast.protective_length > slow.protective_length
    assert compute_fields(-0.3, CFG).forward is False


def test_scan_to_points_applies_sensor_offset_and_drops_invalid():
    p = scan_to_points([1.0, math.inf, 0.05, 2.0], angle_min=0.0, angle_increment=math.pi / 2,
                       range_min=0.1, range_max=10.0, sensor_x=0.22)
    assert p.shape == (2, 2)
    np.testing.assert_allclose(p[0], [1.22, 0.0], atol=1e-9)
    np.testing.assert_allclose(p[1], [0.22, -2.0], atol=1e-9)


def test_distance_to_footprint():
    d = distance_to_footprint(pts([0.0, 0.0], [1.30, 0.0], [0.0, 1.225]), CFG)
    np.testing.assert_allclose(d, [0.0, 1.0, 1.0], atol=1e-9)


def test_clear_when_nothing_close():
    ev = evaluate(pts([5.0, 0.0], [-3.0, 2.0]), v=0.8, w=0.0, cfg=CFG)
    assert ev.state == CLEAR and ev.speed_scale == 1.0


def test_object_in_protective_field_stops():
    f = compute_fields(0.8, CFG)
    x = CFG.footprint_front + f.protective_length * 0.5
    ev = evaluate(pts([x, 0.0]), v=0.8, w=0.0, cfg=CFG)
    assert ev.state == PROTECTIVE_STOP
    assert limit_command(0.8, 0.3, ev) == (0.0, 0.0)


def test_object_in_warning_field_scales_speed():
    f = compute_fields(0.8, CFG)
    x = CFG.footprint_front + (f.protective_length + f.warning_length) / 2
    ev = evaluate(pts([x, 0.0]), v=0.8, w=0.0, cfg=CFG)
    assert ev.state == WARNING
    assert CFG.min_speed_scale <= ev.speed_scale < 1.0
    v, w = limit_command(0.8, 0.2, ev)
    assert v == pytest.approx(0.8 * ev.speed_scale) and w == 0.2


def test_object_beside_the_field_is_ignored():
    ev = evaluate(pts([1.0, 1.5]), v=0.8, w=0.0, cfg=CFG)
    assert ev.state == CLEAR


def test_reverse_uses_rear_field():
    f = compute_fields(-0.3, CFG)
    behind = -CFG.footprint_rear - f.protective_length * 0.5
    assert evaluate(pts([behind, 0.0]), v=-0.3, w=0.0, cfg=CFG).state == PROTECTIVE_STOP
    assert evaluate(pts([behind, 0.0]), v=0.3, w=0.0, cfg=CFG).state != PROTECTIVE_STOP


def test_rotation_field_stops_spin_next_to_object():
    p = pts([0.0, CFG.footprint_half_width + 0.05])
    assert evaluate(p, v=0.0, w=1.0, cfg=CFG).state == PROTECTIVE_STOP
    assert evaluate(p, v=0.0, w=0.0, cfg=CFG).state == CLEAR


def test_monitor_latches_and_releases_after_delay():
    m = SafetyMonitor(FieldConfig(release_delay=1.0))
    blocked = pts([CFG.footprint_front + 0.15, 0.0])
    clear = pts([10.0, 0.0])
    m.scan_received(0.0)
    assert m.update(blocked, 0.5, 0.0, 0.0).state == PROTECTIVE_STOP
    m.scan_received(0.5)
    assert m.update(clear, 0.5, 0.0, 0.5).state == PROTECTIVE_STOP
    m.scan_received(1.0)
    assert m.update(clear, 0.5, 0.0, 1.0).state == PROTECTIVE_STOP
    m.scan_received(1.6)
    assert m.update(clear, 0.5, 0.0, 1.6).state == CLEAR


def test_monitor_scan_timeout_is_a_protective_stop():
    m = SafetyMonitor()
    assert m.update(pts([10.0, 0.0]), 0.5, 0.0, 0.0).reason == "scan timeout"
    m.scan_received(1.0)
    # A timeout is a protective stop, so the release delay applies once scans return
    assert m.update(pts([10.0, 0.0]), 0.5, 0.0, 1.1).reason == "release delay"
    m.scan_received(2.2)
    assert m.update(pts([10.0, 0.0]), 0.5, 0.0, 2.2).state == CLEAR
    assert m.update(pts([10.0, 0.0]), 0.5, 0.0, 3.0).state == PROTECTIVE_STOP


def test_estop_needs_explicit_reset():
    m = SafetyMonitor()
    m.scan_received(0.0)
    m.press_estop()
    for t in (0.0, 5.0, 50.0):
        m.scan_received(t)
        assert m.update(pts([10.0, 0.0]), 0.0, 0.0, t).state == EMERGENCY_STOP
    assert m.reset_estop() is True
    assert m.update(pts([10.0, 0.0]), 0.0, 0.0, 50.0).state == CLEAR
    assert m.reset_estop() is False


# Docked, the dock face is 5 cm in front of the footprint
def test_docking_mode_allows_the_final_approach():
    dock_face = pts([CFG.footprint_front + 0.05, 0.0])
    m = SafetyMonitor()
    m.scan_received(0.0)
    assert m.update(dock_face, 0.1, 0.0, 0.0).state == PROTECTIVE_STOP
    m = SafetyMonitor(docking=True)
    m.scan_received(0.0)
    assert m.update(dock_face, 0.1, 0.0, 0.0).state == CLEAR


def test_docking_mode_still_stops_at_contact_and_caps_speed():
    m = SafetyMonitor(docking=True)
    m.scan_received(0.0)
    assert m.update(pts([CFG.footprint_front + 0.005, 0.0]), 0.1, 0.0, 0.0).state == PROTECTIVE_STOP
    m = SafetyMonitor(docking=True)
    m.scan_received(0.0)
    clear = m.update(pts([5.0, 0.0]), 0.8, 0.0, 0.0)
    assert limit_command(0.8, 0.2, clear, m.max_speed()) == (CFG.docking_max_speed, 0.2)
    assert SafetyMonitor().max_speed() == math.inf
