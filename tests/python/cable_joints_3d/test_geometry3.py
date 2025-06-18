import numpy as np
from cable_joints_3d.geometry3 import closest_point_on_segment, line_segment_sphere_intersection

# These tests mirror the JavaScript geometry3 tests under cable_joints_3d/tests/geometry3.js

# --- closest_point_on_segment tests ---

def test_closest_point_projects_inside_segment():
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([10.0, 0.0, 0.0])
    p = np.array([5.0, 5.0, 0.0])
    cp = closest_point_on_segment(p, a, b)
    assert np.allclose(cp, [5.0, 0.0, 0.0])


def test_closest_point_projects_before_start():
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([10.0, 0.0, 0.0])
    p = np.array([-5.0, 5.0, 0.0])
    cp = closest_point_on_segment(p, a, b)
    assert np.allclose(cp, [0.0, 0.0, 0.0])


def test_closest_point_projects_after_end():
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([10.0, 0.0, 0.0])
    p = np.array([15.0, 5.0, 0.0])
    cp = closest_point_on_segment(p, a, b)
    assert np.allclose(cp, [10.0, 0.0, 0.0])

# --- line_segment_sphere_intersection tests ---

CENTER = np.array([0.0, 0.0, 0.0])
RADIUS = 1.0


def test_no_intersection():
    p1 = np.array([2.0, 2.0, 0.0])
    p2 = np.array([3.0, 2.0, 0.0])
    assert not line_segment_sphere_intersection(p1, p2, CENTER, RADIUS)


def test_passes_through():
    p1 = np.array([-2.0, 0.0, 0.0])
    p2 = np.array([2.0, 0.0, 0.0])
    assert line_segment_sphere_intersection(p1, p2, CENTER, RADIUS)


def test_one_endpoint_inside():
    p1 = np.array([0.5, 0.0, 0.0])
    p2 = np.array([2.0, 0.0, 0.0])
    assert line_segment_sphere_intersection(p1, p2, CENTER, RADIUS)


def test_tangent():
    p1 = np.array([-2.0, 1.0, 0.0])
    p2 = np.array([2.0, 1.0, 0.0])
    assert line_segment_sphere_intersection(p1, p2, CENTER, RADIUS)


def test_completely_inside():
    p1 = np.array([-0.5, 0.0, 0.0])
    p2 = np.array([0.5, 0.0, 0.0])
    assert line_segment_sphere_intersection(p1, p2, CENTER, RADIUS)
