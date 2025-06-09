import math
import pytest

from python.geometry import (
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
    p_attach = Vector2(0, 0)
    p_circle = Vector2(2, 0)
    r_circle = 1
    result = _tangent_point_circle(p_attach, p_circle, r_circle, True, True)

    assert result['a_attach'].x == pytest.approx(0)
    assert result['a_attach'].y == pytest.approx(0)
    assert result['a_circle'].x == pytest.approx(1.5)
    assert result['a_circle'].y == pytest.approx(math.sqrt(3) / 2)

def test_tangent_from_point_to_circle_wrapper():
    """
    Tests the tangentFromPointToCircle wrapper function.
    Corresponds to: describe('tangentFromPointToCircle', ...)
    """
    p_attach = Vector2(1, 1)
    p_circle = Vector2(4, 1)
    r_circle = 2
    cw = False
    result = tangent_from_point_to_circle(p_attach, p_circle, r_circle, cw)
    expected = _tangent_point_circle(p_attach, p_circle, r_circle, cw, True)

    assert result['a_attach'] == expected['a_attach']
    assert result['a_circle'] == expected['a_circle']

def test_tangent_from_circle_to_point_wrapper():
    """
    Tests the tangentFromCircleToPoint wrapper function.
    Corresponds to: describe('tangentFromCircleToPoint', ...)
    """
    p_attach = Vector2(1, 1)
    p_circle = Vector2(4, 1)
    r_circle = 2
    cw = True
    result = tangent_from_circle_to_point(p_attach, p_circle, r_circle, cw)
    expected = _tangent_point_circle(p_attach, p_circle, r_circle, cw, False)

    assert result['a_attach'] == expected['a_attach']
    assert result['a_circle'] == expected['a_circle']

def test_tangent_from_circle_to_circle_identical_circles():
    """
    Tests tangentFromCircleToCircle with two identical circles.
    Corresponds to: describe('tangentFromCircleToCircle', ...) in tangentFunctions.test.js
    """
    posA = Vector2(0, 0)
    radiusA = 1
    cwA = True
    posB = Vector2(4, 0)
    radiusB = 1
    cwB = True
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    assert result['a_circle'].x == pytest.approx(0)
    assert result['a_circle'].y == pytest.approx(1)
    assert result['b_circle'].x == pytest.approx(4)
    assert result['b_circle'].y == pytest.approx(1)

# Tests translated from tests/tangentCircleCircleHTML.test.js

def test_tangent_from_circle_to_circle_TT_outer():
    """
    Corresponds to: test('TT: cwA=true, cwB=true (outer tangent)')
    """
    posA = Vector2(0, 0)
    radiusA = 1.0
    cwA = True
    posB = Vector2(3, 0)
    radiusB = 0.5
    cwB = True
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    expectedA = Vector2(1/6, math.sqrt(35)/6)
    expectedB = Vector2(37/12, math.sqrt(35)/12)

    assert result['a_circle'].x == pytest.approx(expectedA.x)
    assert result['a_circle'].y == pytest.approx(expectedA.y)
    assert result['b_circle'].x == pytest.approx(expectedB.x)
    assert result['b_circle'].y == pytest.approx(expectedB.y)

def test_tangent_from_circle_to_circle_TF_internal():
    """
    Corresponds to: test('TF: cwA=true, cwB=false (internal tangent)')
    """
    posA = Vector2(0, 0)
    radiusA = 1.0
    cwA = True
    posB = Vector2(3, 0)
    radiusB = 1.0
    cwB = False
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    phi = math.asin(2/3)
    expectedA = Vector2(math.sin(phi), math.cos(phi))
    expectedB = Vector2(3 - math.sin(phi), -math.cos(phi))

    assert result['a_circle'].x == pytest.approx(expectedA.x)
    assert result['a_circle'].y == pytest.approx(expectedA.y)
    assert result['b_circle'].x == pytest.approx(expectedB.x)
    assert result['b_circle'].y == pytest.approx(expectedB.y)

def test_tangent_from_circle_to_circle_FT_internal():
    """
    Corresponds to: test('FT: cwA=false, cwB=true (internal tangent)')
    """
    posA = Vector2(0, 0)
    radiusA = 1.0
    cwA = False
    posB = Vector2(3, 0)
    radiusB = 1.0
    cwB = True
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    phi = math.asin(2/3)
    expectedA = Vector2(math.sin(phi), -math.cos(phi))
    expectedB = Vector2(3 - math.sin(phi), math.cos(phi))

    assert result['a_circle'].x == pytest.approx(expectedA.x)
    assert result['a_circle'].y == pytest.approx(expectedA.y)
    assert result['b_circle'].x == pytest.approx(expectedB.x)
    assert result['b_circle'].y == pytest.approx(expectedB.y)

def test_tangent_from_circle_to_circle_FF_outer():
    """
    Corresponds to: test('FF: cwA=false, cwB=false (outer tangent)')
    """
    posA = Vector2(0, 0)
    radiusA = 1.0
    cwA = False
    posB = Vector2(3, 0)
    radiusB = 0.5
    cwB = False
    result = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)

    expectedA = Vector2(1/6, -math.sqrt(35)/6)
    expectedB = Vector2(37/12, -math.sqrt(35)/12)

    assert result['a_circle'].x == pytest.approx(expectedA.x)
    assert result['a_circle'].y == pytest.approx(expectedA.y)
    assert result['b_circle'].x == pytest.approx(expectedB.x)
    assert result['b_circle'].y == pytest.approx(expectedB.y)
