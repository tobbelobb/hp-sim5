import numpy as np
import pytest
from cable_joints.geometry import signed_arc_length_on_wheel, tangent_from_point_to_circle, tangent_from_circle_to_point

# Common center point for all tests, mirroring the JS setup
center = np.array([0.0, 0.0, 0.0])

def test_happy_path_quarter_circle_without_force_positive():
    """
    Mirrors JS test: 'happy path: quarter circle without force_positive (default)'
    """
    prev_point = np.array([1.0, 0.0, 0.0])
    curr_point = np.array([0.0, 1.0, 0.0])
    radius = 1.0
    # angle from 0 to π/2 = π/2
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, False)
    assert np.isclose(length, np.pi / 2)

def test_force_positive_true_with_negative_angle_yields_full_positive_arc():
    """
    Mirrors JS test: 'force_positive true with negative angle yields full positive arc'
    """
    prev_point = np.array([1.0, 0.0, 0.0])
    curr_point = np.array([0.0, -1.0, 0.0])
    radius = 1.0
    # angle from 0 to -π/2 = -π/2, force_positive adds 2π -> 3π/2
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, False, force_positive=True)
    assert np.isclose(length, (3 * np.pi) / 2)

def test_clockwise_preference_flips_sign_of_arc_length():
    """
    Mirrors JS test: 'clockwise preference flips sign of arc length'
    """
    prev_point = np.array([1.0, 0.0, 0.0])
    curr_point = np.array([0.0, 1.0, 0.0])
    radius = 2.0
    # angle π/2, with clockwisePreference true => -π/2 * 2 = -π
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, True)
    assert np.isclose(length, -np.pi)

def test_radius_zero_always_returns_zero():
    """
    Mirrors JS test: 'radius zero always returns zero'
    """
    prev_point = np.array([1.0, 0.0, 0.0])
    curr_point = np.array([0.0, 1.0, 0.0])
    radius = 0.0
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, False)
    assert np.isclose(length, 0.0)

def test_negative_radius_inverts_the_sign_of_arc_length():
    """
    Mirrors JS test: 'negative radius inverts the sign of arc length'
    """
    prev_point = np.array([1.0, 0.0, 0.0])
    curr_point = np.array([0.0, 1.0, 0.0])
    radius = -2.0
    # angle π/2 * -2 = -π
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, False)
    assert np.isclose(length, -np.pi)

def test_prev_point_not_on_circle_still_computes_based_on_angles():
    """
    Mirrors JS test: 'prevPoint not on circle still computes based on angles'
    """
    prev_point = np.array([2.0, 0.0, 0.0]) # distance 2 instead of radius
    curr_point = np.array([0.0, 1.0, 0.0])
    radius = 1.0
    # still angle from 0 to π/2 = π/2
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, False)
    assert np.isclose(length, np.pi / 2)

def test_curr_point_not_on_circle_still_computes_based_on_angles():
    """
    Mirrors JS test: 'currPoint not on circle still computes based on angles'
    """
    prev_point = np.array([1.0, 0.0, 0.0])
    curr_point = np.array([0.0, 2.0, 0.0]) # distance 2 instead of radius
    radius = 1.0
    # angle π/2 * 1 = π/2
    length = signed_arc_length_on_wheel(prev_point, curr_point, center, radius, False)
    assert np.isclose(length, np.pi / 2)

def test_large_initial_wrap_around_obstacle():
    """
    Mirrors JS test: 'large initial wrap around obstacle'
    """
    cw = True
    obs_radius = 0.1
    pos_obs = np.array([1.0, 0.3, 0.0])
    p_ball1 = np.array([1.025, -0.5, 0.0])
    p_ball2 = np.array([1.2, -0.7, 0.0])
    # initial tangent points
    ip1 = tangent_from_point_to_circle(p_ball1, pos_obs, obs_radius, cw)
    ip2 = tangent_from_circle_to_point(p_ball2, pos_obs, obs_radius, cw)
    wrap_length = signed_arc_length_on_wheel(ip1['a_circle'], ip2['a_circle'], pos_obs, obs_radius, cw, force_positive=True)
    assert wrap_length > 0
    assert wrap_length > np.pi * obs_radius
    assert wrap_length < np.pi * 1.1 * obs_radius
