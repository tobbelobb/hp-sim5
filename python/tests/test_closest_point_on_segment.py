import numpy as np
from python.geometry import closest_point_on_segment

def test_point_projects_inside_segment():
    """
    Corresponds to: test('point projects inside segment')
    """
    a = np.array([0, 0], dtype=float)
    b = np.array([10, 0], dtype=float)
    p = np.array([5, 5], dtype=float)
    cp = closest_point_on_segment(p, a, b)
    expected = np.array([5, 0], dtype=float)
    assert np.allclose(cp, expected)

def test_point_projects_before_segment_start():
    """
    Corresponds to: test('point projects before segment start')
    """
    a = np.array([0, 0], dtype=float)
    b = np.array([10, 0], dtype=float)
    p = np.array([-5, 5], dtype=float)
    cp = closest_point_on_segment(p, a, b)
    expected = np.array([0, 0], dtype=float)
    assert np.allclose(cp, expected)

def test_point_projects_after_segment_end():
    """
    Corresponds to: test('point projects after segment end')
    """
    a = np.array([0, 0], dtype=float)
    b = np.array([10, 0], dtype=float)
    p = np.array([15, -5], dtype=float)
    cp = closest_point_on_segment(p, a, b)
    expected = np.array([10, 0], dtype=float)
    assert np.allclose(cp, expected)

def test_zero_segment():
    """
    Corresponds to: test('zero segment')
    """
    a = np.array([0, 0], dtype=float)
    b = np.array([0, 0], dtype=float)
    p = np.array([15, -5], dtype=float)
    cp = closest_point_on_segment(p, a, b)
    expected = np.array([0, 0], dtype=float)
    assert np.allclose(cp, expected)

def test_point_projects_on_segment():
    """
    Corresponds to: test('point projects on segment')
    """
    a = np.array([0, 10], dtype=float)
    b = np.array([0, 0], dtype=float)
    p = np.array([0, 5], dtype=float)
    cp = closest_point_on_segment(p, a, b)
    expected = np.array([0, 5], dtype=float)
    assert np.allclose(cp, expected)
