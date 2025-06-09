import numpy as np
from python.geometry import right_of_line

def test_point_to_the_right_of_horizontal_line():
    """
    Mirrors JS test: 'point to the right of horizontal line'
    """
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([1.0, 0.0, 0.0])
    x = np.array([0.0, -1.0, 0.0])
    assert right_of_line(x, p0, p1) is True

def test_point_to_the_left_of_horizontal_line():
    """
    Mirrors JS test: 'point to the left of horizontal line'
    """
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([1.0, 0.0, 0.0])
    x = np.array([0.0, 1.0, 0.0])
    assert right_of_line(x, p0, p1) is False

def test_point_collinear_on_line_returns_false():
    """
    Mirrors JS test: 'point collinear on line returns false'
    """
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([1.0, 0.0, 0.0])
    x = np.array([0.5, 0.0, 0.0])
    assert right_of_line(x, p0, p1) is False

def test_point_to_the_right_of_slanted_line():
    """
    Mirrors JS test: 'point to the right of slanted line'
    """
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([1.0, 1.0, 0.0])
    x = np.array([1.0, 0.0, 0.0])
    assert right_of_line(x, p0, p1) is True
