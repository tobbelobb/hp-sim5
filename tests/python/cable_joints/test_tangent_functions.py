import math
import pytest
import numpy as np

from cable_joints.geometry import (
    _tangent_point_circle,
    tangent_from_point_to_circle,
    tangent_from_circle_to_point,
    tangent_from_circle_to_circle
)

# Tests translated from tests/tangentFunctions.test.js

def test_tangent_point_circle_point_outside():
    """
    Tests _tangent_point_circle with a point outside the circle.
    Corresponds to: describe('_tangentPointCircle', ...)
    """
    p_attach = np.array([0, 0, 0.0], dtype=float)
    p_circle = np.array([2, 0, 0.0], dtype=float)
    r_circle = 1
    result = _tangent_point_circle(p_attach, p_circle, r_circle, True, True)

    expected_a_attach = np.array([0.0, 0.0, 0.0])
    expected_a_circle = np.array([1.5, math.sqrt(3) / 2, 0.0])

    assert np.allclose(result['a_attach'], expected_a_attach)
    assert np.allclose(result['a_circle'], expected_a_circle)

def test_tangent_from_point_to_circle_wrapper():
    """
    Tests the tangentFromPointToCircle wrapper function.
    Corresponds to: describe('tangentFromPointToCircle', ...)
    """
    p_attach = np.array([1, 1, 0.0], dtype=float)
    p_circle = np.array([4, 1, 0.0], dtype=float)
    r_circle = 2
    cw = False
    result = tangent_from_point_to_circle(p_attach, p_circle, r_circle, cw)
    expected = _tangent_point_circle(p_attach, p_circle, r_circle, cw, True)

    assert np.allclose(result['a_attach'], expected['a_attach'])
    assert np.allclose(result['a_circle'], expected['a_circle'])

def test_tangent_from_circle_to_point_wrapper():
    """
    Tests the tangentFromCircleToPoint wrapper function.
    Corresponds to: describe('tangentFromCircleToPoint', ...)
    """
    p_attach = np.array([1, 1, 0.0], dtype=float)
    p_circle = np.array([4, 1, 0.0], dtype=float)
    r_circle = 2
    cw = True
    result = tangent_from_circle_to_point(p_attach, p_circle, r_circle, cw)
    expected = _tangent_point_circle(p_attach, p_circle, r_circle, cw, False)

    assert np.allclose(result['a_attach'], expected['a_attach'])
    assert np.allclose(result['a_circle'], expected['a_circle'])

def test_tangent_from_circle_to_circle_identical_circles():
    """
    Tests tangentFromCircleToCircle with two identical circles.
    Corresponds to: describe('tangentFromCircleToCircle', ...) in tangentFunctions.test.js
    """
    posA = np.array([0, 0, 0.0], dtype=float)
    radiusA = 1
    cwA = True
    posB = np.array([4, 0, 0.0], dtype=float)
    radiusB = 1
    cwB = True
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    expected_a = np.array([0.0, 1.0, 0.0])
    expected_b = np.array([4.0, 1.0, 0.0])

    assert np.allclose(result['a_circle'], expected_a)
    assert np.allclose(result['b_circle'], expected_b)

# Tests translated from tests/tangentCircleCircleHTML.test.js

def test_tangent_from_circle_to_circle_TT_outer():
    """
    Corresponds to: test('TT: cwA=true, cwB=true (outer tangent)')
    """
    posA = np.array([0, 0, 0.0], dtype=float)
    radiusA = 1.0
    cwA = True
    posB = np.array([3, 0, 0.0], dtype=float)
    radiusB = 0.5
    cwB = True
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    expectedA = np.array([1/6, math.sqrt(35)/6, 0.0])
    expectedB = np.array([37/12, math.sqrt(35)/12, 0.0])

    assert np.allclose(result['a_circle'], expectedA)
    assert np.allclose(result['b_circle'], expectedB)

def test_tangent_from_circle_to_circle_TF_internal():
    """
    Corresponds to: test('TF: cwA=true, cwB=false (internal tangent)')
    """
    posA = np.array([0, 0, 0.0], dtype=float)
    radiusA = 1.0
    cwA = True
    posB = np.array([3, 0, 0.0], dtype=float)
    radiusB = 1.0
    cwB = False
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    phi = math.asin(2/3)
    expectedA = np.array([math.sin(phi), math.cos(phi), 0.0])
    expectedB = np.array([3 - math.sin(phi), -math.cos(phi), 0.0])

    assert np.allclose(result['a_circle'], expectedA)
    assert np.allclose(result['b_circle'], expectedB)

def test_tangent_from_circle_to_circle_FT_internal():
    """
    Corresponds to: test('FT: cwA=false, cwB=true (internal tangent)')
    """
    posA = np.array([0, 0, 0.0], dtype=float)
    radiusA = 1.0
    cwA = False
    posB = np.array([3, 0, 0.0], dtype=float)
    radiusB = 1.0
    cwB = True
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    phi = math.asin(2/3)
    expectedA = np.array([math.sin(phi), -math.cos(phi), 0.0])
    expectedB = np.array([3 - math.sin(phi), math.cos(phi), 0.0])

    assert np.allclose(result['a_circle'], expectedA)
    assert np.allclose(result['b_circle'], expectedB)

def test_tangent_from_circle_to_circle_FF_outer():
    """
    Corresponds to: test('FF: cwA=false, cwB=false (outer tangent)')
    """
    posA = np.array([0, 0, 0.0], dtype=float)
    radiusA = 1.0
    cwA = False
    posB = np.array([3, 0, 0.0], dtype=float)
    radiusB = 0.5
    cwB = False
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    expectedA = np.array([1/6, -math.sqrt(35)/6, 0.0])
    expectedB = np.array([37/12, -math.sqrt(35)/12, 0.0])

    assert np.allclose(result['a_circle'], expectedA)
    assert np.allclose(result['b_circle'], expectedB)
