import numpy as np
from python.geometry import line_segment_circle_intersection

# Common center point for all tests, mirroring the JS setup
center = np.array([0.0, 0.0, 0.0])

def test_segment_completely_outside_circle():
    """
    Mirrors JS test: 'segment completely outside circle'
    """
    p1 = np.array([0.0, -1.0000001, 0.0])
    p2 = np.array([2.0, -1.0, 0.0])
    assert not line_segment_circle_intersection(p1, p2, center, 1.0)

def test_segment_endpoint_just_inside_circle_with_true_setting():
    """
    Mirrors JS test: 'segment endpoint just inside circle with true setting'
    """
    p1 = np.array([0.99999999, 0.0, 0.0])
    p2 = np.array([5.0, 0.0, 0.0])
    assert line_segment_circle_intersection(p1, p2, center, 1.0, is_a_pierce_an_intersection=True)

def test_segment_endpoint_just_inside_circle_with_default_false_setting():
    """
    Mirrors JS test: 'segment endpoint just inside circle with default false setting'
    """
    p1 = np.array([0.99999999, 0.0, 0.0])
    p2 = np.array([5.0, 0.0, 0.0])
    assert not line_segment_circle_intersection(p1, p2, center, 1.0)

def test_segment_passes_through_circle():
    """
    Mirrors JS test: 'segment passes through circle'
    """
    p1 = np.array([-2.0, 0.0, 0.0])
    p2 = np.array([2.0, 0.0, 0.0])
    assert line_segment_circle_intersection(p1, p2, center, 1.0)

def test_segment_tangent_to_circle():
    """
    Mirrors JS test: 'segment tangent to circle'
    """
    p1 = np.array([1.0, -1.0, 0.0])
    p2 = np.array([1.0, 1.0, 0.0])
    assert line_segment_circle_intersection(p1, p2, center, 1.0)

def test_zero_length_segment_inside_circle():
    """
    Mirrors JS test: 'zero-length segment inside circle'
    """
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([0.0, 0.0, 0.0])
    assert not line_segment_circle_intersection(p1, p2, center, 1.0)

def test_zero_length_segment_outside_circle():
    """
    Mirrors JS test: 'zero-length segment outside circle'
    """
    p1 = np.array([2.0, 0.0, 0.0])
    p2 = np.array([2.0, 0.0, 0.0])
    assert not line_segment_circle_intersection(p1, p2, center, 1.0)

def test_going_through_zero_radius_circle():
    """
    Mirrors JS test: 'going through zero-radius circle'
    """
    p1 = np.array([-2.0, -2.0, 0.0])
    p2 = np.array([2.0, 2.0, 0.0])
    assert line_segment_circle_intersection(p1, p2, center, 0.0)

def test_segment_through_circle_negative_radius():
    """
    Mirrors JS test: 'segment through circle, negative radius'
    """
    p1 = np.array([-2.0, 0.0, 0.0])
    p2 = np.array([2.0, 0.0, 0.0])
    assert not line_segment_circle_intersection(p1, p2, center, -1.0)

def test_segment_tangent_to_circle_negative_radius():
    """
    Mirrors JS test: 'segment tangent to circle, negative radius'
    """
    p1 = np.array([1.0, -1.0, 0.0])
    p2 = np.array([1.0, 1.0, 0.0])
    assert not line_segment_circle_intersection(p1, p2, center, -1.0)
