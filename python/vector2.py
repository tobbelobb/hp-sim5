# This file provides a guide for translating the JavaScript Vector2 class
# from `cable_joints/vector2.js` to Python using the NumPy library.
#
# Using raw NumPy arrays for 2D vectors is generally more idiomatic and
# performant in Python for this kind of numerical work than creating a
# custom Vector2 class.
#
# CONVENTION:
# A 2D vector `v` is represented as a NumPy array of shape (2,).
# Example: `v = np.array([1.0, 2.0])`

import numpy as np

# --- Guide to Vector Operations ---
#
# This section maps the methods from the JavaScript Vector2 class to their
# equivalent operations in NumPy.
#
# In the examples below:
# v, v1, v2, a, b, center are NumPy arrays: np.array([x, y])
# s, ang are scalars (floats)
# cw is a boolean

# JS: constructor(x = 0.0, y = 0.0)
# PY: v = np.array([x, y], dtype=float)

# JS: set(v)
# PY: v1[:] = v2  (in-place assignment)

# JS: clone()
# PY: v.copy()

# JS: add(v, s = 1.0)  (in-place)
# PY: v1 += v2 * s

# JS: addVectors(a, b) (returns new vector)
# PY: a + b

# JS: subtract(v, s = 1.0) (in-place)
# PY: v1 -= v2 * s

# JS: subtractVectors(a, b) (returns new vector)
# PY: a - b

# JS: distanceTo(b)
# PY: np.linalg.norm(a - b)

# JS: distanceToSq(b)
# PY: np.sum((a - b)**2)  # or np.dot(a - b, a - b)

# JS: length()
# PY: np.linalg.norm(v)

# JS: lengthSq()
# PY: np.dot(v, v) # or np.sum(v**2)

# JS: scale(s) (in-place)
# PY: v *= s

# JS: dot(v)
# PY: np.dot(v1, v2)

# JS: perp() (returns new vector, perpendicular)
# PY: np.array([-v[1], v[0]])

# --- Functions for Complex Operations ---

def angle_to(v1, v2):
    """
    Calculates the angle between two vectors.
    Equivalent to JS Vector2.angleTo(v).
    """
    dot = np.dot(v1, v2)
    norm_product = np.linalg.norm(v1) * np.linalg.norm(v2)
    if norm_product == 0:
        return 0.0
    # Clamp the value to handle potential floating point inaccuracies
    cos_angle = np.clip(dot / norm_product, -1.0, 1.0)
    return np.arccos(cos_angle)

def normalize_inplace(v):
    """
    Normalizes a vector in-place.
    Equivalent to JS Vector2.normalize().
    """
    norm = np.linalg.norm(v)
    if norm > 1e-9: # Avoid division by zero
        v /= norm
    return v

def rotate_inplace(v, ang, center, cw):
    """
    Rotates a vector in-place around a center point.
    Equivalent to JS Vector2.rotate(ang, center, cw).

    Note on the 'cw' parameter: The naming from the JS source is kept for
    consistency. The behavior in the original JS is that `cw=true` results
    in a counter-clockwise rotation, and `cw=false` results in a clockwise
    rotation. This implementation matches that behavior.
    """
    # The JS logic is `if (!cw) ang = -ang;` which means `cw=false` negates the angle.
    # A standard CCW rotation matrix is used, so negating the angle results in a CW rotation.
    if not cw:
        ang = -ang

    cos_a, sin_a = np.cos(ang), np.sin(ang)
    rotation_matrix = np.array([[cos_a, -sin_a],
                                [sin_a,  cos_a]])

    # Translate point to origin, rotate, and translate back
    v_centered = v - center
    v_rotated = rotation_matrix @ v_centered
    v[:] = v_rotated + center
    return v
