import numpy as np

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

def line_segment_sphere_intersection(p1, p2, center, radius):
    if np.linalg.norm(p1 - center) <= radius or np.linalg.norm(p2 - center) <= radius:
        return True
    d = p2 - p1
    lc = center - p1
    d_len_sq = np.dot(d, d)
    t = np.dot(lc, d)
    if d_len_sq > 1e-9:
        t /= d_len_sq
    if t < 0.0 or t > 1.0:
        return False
    closest = p1 + d * t
    return np.linalg.norm(closest - center) <= radius
