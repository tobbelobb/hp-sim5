from functools import lru_cache
import numpy as np
from .vector3 import cross, length as norm3
from cable_joints.geometry import (
    _tangent_point_circle, tangent_from_circle_to_circle,
    signed_arc_length_on_wheel as _signed_arc_length_2d, right_of_line,
)

EPSILON = 1e-9


def build_plane_basis(plane_normal=None):
    values = (0., 0., 1.) if plane_normal is None else tuple(plane_normal)
    # Exact value keys also handle changing live member frames. Return owned
    # arrays so callers cannot mutate the cached basis.
    return tuple(vector.copy() for vector in _plane_basis(*values))


@lru_cache(maxsize=512)
def _plane_basis(x, y, z):
    normal = np.array([x, y, z], dtype=float)
    if np.dot(normal, normal) <= EPSILON:
        return np.array([0., 0., 1.]), np.array([1., 0., 0.]), np.array([0., 1., 0.])
    normal /= norm3(normal)
    reference = np.array([1., 0., 0.]) if abs(normal[0]) < .9 else np.array([0., 1., 0.])
    u = reference - normal * np.dot(normal, reference)
    if np.dot(u, u) <= EPSILON:
        reference = np.array([0., 0., 1.])
        u = reference - normal * np.dot(normal, reference)
    u /= norm3(u)
    return normal, u, cross(normal, u)


def _project(point, origin, basis):
    relative = point - origin
    return np.array([np.dot(relative, basis[1]), np.dot(relative, basis[2]), 0.])


def _lift(point, origin, basis):
    return origin + basis[1] * point[0] + basis[2] * point[1]

def closest_point_on_segment(p, a, b):
    ab = b - a
    ap = p - a
    t = np.dot(ap, ab)
    if t <= 0.0:
        return a.copy()
    denom = np.dot(ab, ab)
    if t >= denom:
        return b.copy()
    t = t / denom
    return a + ab * t

def line_segment_sphere_intersection(p1, p2, center, radius, is_a_pierce_an_intersection=False):
    if norm3(p1 - center) <= radius or norm3(p2 - center) <= radius:
        return is_a_pierce_an_intersection
    d = p2 - p1
    lc = center - p1
    d_len_sq = np.dot(d, d)
    t = np.dot(lc, d)
    if d_len_sq > 1e-9:
        t /= d_len_sq
    if t < 0.0 or t > 1.0:
        return False
    closest = p1 + d * t
    return norm3(closest - center) <= radius


def _tangent_point_sphere(attachment, center, radius, plane_normal, cw, point_is_first):
    basis = build_plane_basis(plane_normal)
    result = _tangent_point_circle(
        _project(attachment, center, basis), np.zeros(3), radius, cw, point_is_first
    )
    return {'a_attach': attachment.copy(), 'a_sphere': _lift(result['a_circle'], center, basis)}


def tangent_from_point_to_sphere(attachment, center, radius, plane_normal, cw):
    return _tangent_point_sphere(attachment, center, radius, plane_normal, cw, True)


def tangent_from_sphere_to_point(attachment, center, radius, plane_normal, cw):
    return _tangent_point_sphere(attachment, center, radius, plane_normal, cw, False)


def tangent_from_sphere_to_sphere(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b, plane_normal):
    basis = build_plane_basis(plane_normal)
    result = tangent_from_circle_to_circle(
        np.zeros(3), radius_a, cw_a, _project(pos_b, pos_a, basis), radius_b, cw_b
    )
    return {
        'a_sphere': _lift(result['a_circle'], pos_a, basis),
        'b_sphere': _lift(result['b_circle'], pos_a, basis) + basis[0] * np.dot(pos_b - pos_a, basis[0]),
    }


def signed_arc_length_on_wheel(previous, current, center, radius, clockwise, plane_normal=None, force_positive=False):
    basis = build_plane_basis(plane_normal)
    return _signed_arc_length_2d(
        _project(previous, center, basis), _project(current, center, basis), np.zeros(3),
        radius, clockwise, force_positive,
    )


def right_of_plane(point, start, end, plane_normal=None):
    basis = build_plane_basis(plane_normal)
    return right_of_line(_project(point, start, basis), np.zeros(3), _project(end, start, basis))
