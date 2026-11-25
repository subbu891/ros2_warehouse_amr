import math

import numpy as np

from amr_perception.obstacle_tracker import (Cluster, MultiTracker, StaticMap, TrackerConfig,
                                             cluster_points, predict_points, transform_points)


# Points on the visible west face of a box, in scan order
def box_points(cx, cy, size=0.6, n=12):
    ys = np.linspace(cy - size / 2, cy + size / 2, n)
    return np.column_stack((np.full(n, cx - size / 2), ys))


def test_static_map_filters_wall_points():
    grid = np.zeros((100, 100), dtype=np.int16)
    grid[:, 50] = 100
    smap = StaticMap(grid, 0.1, (0.0, 0.0), dilation_m=0.2)
    pts = np.array([[5.05, 3.0], [5.25, 3.0], [2.0, 3.0], [-1.0, 0.0]])
    np.testing.assert_array_equal(smap.is_static(pts), [True, True, False, True])


def test_unknown_cells_count_as_static():
    grid = np.full((10, 10), -1, dtype=np.int16)
    smap = StaticMap(grid, 0.1, (0.0, 0.0), dilation_m=0.0)
    assert smap.is_static(np.array([[0.5, 0.5]]))[0]


def test_clustering_splits_separate_objects_and_drops_noise():
    pts = np.vstack((box_points(3, 0), box_points(3, 3), [[10.0, 10.0]]))
    clusters = cluster_points(pts, gap=0.3, min_points=3)
    assert len(clusters) == 2
    np.testing.assert_allclose(clusters[0].centroid, [2.7, 0.0], atol=1e-6)


def test_clustering_merges_across_scan_wrap():
    a = np.column_stack((np.full(5, 2.0), np.linspace(-0.2, 0.0, 5)))
    b = np.column_stack((np.full(5, 2.0), np.linspace(0.05, 0.25, 5)))
    pts = np.vstack((b, np.array([[0.0, 5.0], [0.0, 5.1], [0.0, 5.2]]), a))
    assert len(cluster_points(pts, gap=0.3, min_points=3)) == 2


# Object moving +y at 0.5 m/s, observed at 10 Hz
def test_tracker_estimates_constant_velocity():
    tr = MultiTracker(TrackerConfig(confirm_hits=3))
    for k in range(30):
        t = k * 0.1
        tr.update([Cluster(np.array([3.0, 0.5 * t]), np.zeros((0, 2)))], t)
    confirmed = tr.confirmed()
    assert len(confirmed) == 1
    np.testing.assert_allclose(confirmed[0].velocity, [0.0, 0.5], atol=0.05)
    assert tr.moving() == confirmed


def test_static_object_is_not_moving_and_old_tracks_expire():
    tr = MultiTracker(TrackerConfig(max_age=0.5))
    for k in range(10):
        tr.update([Cluster(np.array([1.0, 1.0]), np.zeros((0, 2)))], k * 0.1)
    assert len(tr.confirmed()) == 1 and tr.moving() == []
    tr.update([], 2.0)
    assert tr.tracks == []


def test_two_objects_keep_their_ids():
    tr = MultiTracker()
    for k in range(10):
        t = k * 0.1
        tr.update([Cluster(np.array([0.0, 0.3 * t]), np.zeros((0, 2))),
                   Cluster(np.array([5.0, -0.3 * t]), np.zeros((0, 2)))], t)
    assert sorted(t.track_id for t in tr.confirmed()) == [1, 2]


# After 2 s at 0.5 m/s the prediction must reach about 1 m ahead
def test_prediction_covers_future_positions():
    tr = MultiTracker()
    for k in range(20):
        t = k * 0.1
        tr.update([Cluster(np.array([0.5 * t, 0.0]), np.zeros((0, 2)))], t)
    pts = predict_points(tr.moving(), horizon=2.0, step=0.5)
    assert pts[:, 0].max() > tr.moving()[0].position[0] + 0.9


def test_transform_points():
    out = transform_points(np.array([[1.0, 0.0]]), 2.0, 3.0, math.pi / 2)
    np.testing.assert_allclose(out, [[2.0, 4.0]], atol=1e-9)
