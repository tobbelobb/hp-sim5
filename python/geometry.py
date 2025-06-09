import numpy as np

def closest_point_on_segment(p, a, b):
    """
    Finds the closest point on a line segment (a, b) to a point p.
    """
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

def _tangent_point_circle(p_attach, p_circle, r_circle, cw, point_is_first):
    """
    Helper to calculate tangent points between a point and a circle.
    This is a port of Algorithm 3 from the Cable Joints paper.
    """
    d_vec = p_circle - p_attach
    d_sq = np.dot(d_vec, d_vec)

    if d_sq <= r_circle * r_circle + 1e-9:
        # Attachment point is inside or on the rolling circle.
        # Return a point on the circumference in the opposite direction of the attachment point.
        direction = d_vec / np.linalg.norm(d_vec) if np.linalg.norm(d_vec) > 1e-9 else np.array([1.0, 0.0, 0.0])
        return {
            'a_attach': p_attach.copy(),
            'a_circle': p_circle - direction * r_circle
        }

    d = np.sqrt(d_sq)
    alpha = np.arctan2(d_vec[1], d_vec[0])
    phi = np.arcsin(r_circle / d)

    if (cw and point_is_first) or (not cw and not point_is_first):
        tangent_point_angle_on_circle = alpha + phi + np.pi / 2
    else:
        tangent_point_angle_on_circle = alpha - phi - np.pi / 2

    a_circle = np.array([
        p_circle[0] + r_circle * np.cos(tangent_point_angle_on_circle),
        p_circle[1] + r_circle * np.sin(tangent_point_angle_on_circle),
        p_circle[2] # Preserve Z coordinate
    ])

    return {
        'a_attach': p_attach.copy(),
        'a_circle': a_circle
    }

def tangent_from_point_to_circle(p_attach, p_circle, r_circle, cw):
    """Calculates tangent from a point to a circle."""
    return _tangent_point_circle(p_attach, p_circle, r_circle, cw, True)

def tangent_from_circle_to_point(p_attach, p_circle, r_circle, cw):
    """Calculates tangent from a circle to a point."""
    return _tangent_point_circle(p_attach, p_circle, r_circle, cw, False)

def tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b):
    """Calculates the common tangent between two circles."""
    d_vec = pos_b - pos_a
    d = np.linalg.norm(d_vec)

    r = (radius_b - radius_a) if (cw_a == cw_b) else (radius_a + radius_b)
    alpha = np.arctan2(d_vec[1], d_vec[0])

    # Clamp argument to asin to handle floating point inaccuracies
    asin_arg = np.clip(r / d, -1.0, 1.0)
    phi = np.arcsin(asin_arg)

    angle_a, angle_b = 0.0, 0.0
    if (not cw_a) == cw_b:
        if not cw_a:
            angle_a = alpha - np.pi / 2 + phi
            angle_b = alpha + np.pi / 2 + phi
        else:
            angle_a = alpha + np.pi / 2 - phi
            angle_b = alpha - np.pi / 2 - phi
    else:
        if not cw_a:
            angle_a = alpha - np.pi / 2 - phi
            angle_b = alpha - np.pi / 2 - phi
        else:
            angle_a = alpha + np.pi / 2 + phi
            angle_b = alpha + np.pi / 2 + phi

    tangent_a = np.array([
        pos_a[0] + radius_a * np.cos(angle_a),
        pos_a[1] + radius_a * np.sin(angle_a),
        pos_a[2]
    ])
    tangent_b = np.array([
        pos_b[0] + radius_b * np.cos(angle_b),
        pos_b[1] + radius_b * np.sin(angle_b),
        pos_b[2]
    ])

    return {'a_circle': tangent_a, 'b_circle': tangent_b}

def signed_arc_length_on_wheel(prev_point, curr_point, center, radius, clockwise_preference, force_positive=False):
    """
    Calculates signed arc length between two world-space points on the circumference of a wheel.
    """
    to_prev = prev_point - center
    to_curr = curr_point - center

    angle = np.arctan2(to_curr[1], to_curr[0]) - np.arctan2(to_prev[1], to_prev[0])

    # Normalize angle to range [-pi, pi]
    if angle > np.pi:
        angle -= 2 * np.pi
    if angle < -np.pi:
        angle += 2 * np.pi

    if clockwise_preference:
        angle *= -1

    if force_positive:
        while angle < 0.0:
            angle += 2 * np.pi

    return radius * angle

def line_segment_circle_intersection(p1, p2, center, radius, is_a_pierce_an_intersection=False):
    """
    Checks if a line segment intersects a circle.
    """
    # 1. Check if either endpoint is inside the circle
    if np.linalg.norm(p1 - center) <= radius or np.linalg.norm(p2 - center) <= radius:
        return is_a_pierce_an_intersection

    # 2. Check if the projection of the center onto the line lies within the segment
    d = p2 - p1
    lc = center - p1
    d_length_sq = np.dot(d, d)

    t = np.dot(lc, d)
    if d_length_sq > 1e-9:
        t /= d_length_sq

    if t < 0.0 or t > 1.0:
        return False # Closest point is an endpoint, which we already checked

    closest_point_on_line = p1 + d * t

    # 3. Check if the closest point on the segment is within the circle's radius
    return np.linalg.norm(closest_point_on_line - center) <= radius

def right_of_line(x, p0, p1):
    """
    Determines if point x is to the right of the directed line segment from p0 to p1.
    Uses the 2D cross product on the XY plane.
    """
    v = p1 - p0
    w = x - p0
    cross_product = v[0] * w[1] - v[1] * w[0]
    return cross_product < 0.0
