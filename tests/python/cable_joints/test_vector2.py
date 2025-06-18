import numpy as np
from cable_joints.vector2 import angle_to, normalize_inplace, rotate_inplace

# These tests mirror portions of tests/vector2.test.js


def test_angle_to_various_cases():
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    v3 = np.array([-1.0, 0.0, 0.0])
    v4 = np.array([1.0, 1.0, 0.0])
    v5 = np.array([0.0, 0.0, 0.0])
    v6 = np.array([2.0, 0.0, 0.0])
    v7 = np.array([-1.0, 1.0, 0.0])

    assert angle_to(v1, v5) == 0.0
    assert angle_to(v5, v1) == 0.0
    assert np.isclose(angle_to(v1, v1), 0.0)
    assert np.isclose(angle_to(v1, v6), 0.0)
    assert np.isclose(angle_to(v1, v3), np.pi)
    assert np.isclose(angle_to(v1, v2), np.pi/2)
    assert np.isclose(angle_to(v2, v1), np.pi/2)
    assert np.isclose(angle_to(v1, v4), np.pi/4)
    assert np.isclose(angle_to(v1, v7), 3*np.pi/4)


def test_rotate_inplace_clockwise_and_counterclockwise():
    v = np.array([1.0, 0.0, 0.0])
    center = np.array([0.0, 0.0, 0.0])

    rotate_inplace(v, np.pi/2, center, cw=False)
    assert np.allclose(v[:2], [0.0, -1.0])  # CW rotation

    v2 = np.array([1.0, 0.0, 0.0])
    rotate_inplace(v2, np.pi/2, center, cw=True)
    assert np.allclose(v2[:2], [0.0, 1.0])  # CCW rotation


def test_normalize_inplace():
    v = np.array([3.0, 4.0, 0.0])
    normalize_inplace(v)
    assert np.allclose(v, [0.6, 0.8, 0.0])
