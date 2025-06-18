# This file provides a guide for translating the JavaScript Vector2 class
# from `cable_joints/vector2.js` to Python using the NumPy library.
#
# Using raw NumPy arrays for 3D vectors is generally more idiomatic and
# performant in Python for this kind of numerical work than creating a
# custom Vector2/Vector3 class.
#
# CONVENTION:
# A 3D vector `v` is represented as a NumPy array of shape (3,).
# For 2D calculations, the z-component is often set to 0.
# Example: `v = np.array([1.0, 2.0, 0.0])`

import numpy as np

# --- Guide to Vector Operations ---
#
# This section maps the methods from the JavaScript Vector2 class to their
# equivalent operations in NumPy.
#
# In the examples below:
# v, v1, v2, a, b, center are NumPy arrays: np.array([x, y, z])
# s, ang are scalars (floats)
# cw is a boolean

# JS: constructor(x = 0.0, y = 0.0)
# PY: v = np.array([x, y, 0.0], dtype=float)

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

# JS: perp() (returns new vector, perpendicular in XY plane)
# PY: np.array([-v[1], v[0], v[2]]) # Preserves Z

# --- Functions for Complex Operations ---

def angle_to(v1, v2):
    """
    Calculates the angle between the XY components of two vectors.
    Equivalent to JS Vector2.angleTo(v).
    """
    v1_2d = v1[:2]
    v2_2d = v2[:2]
    dot = np.dot(v1_2d, v2_2d)
    norm_product = np.linalg.norm(v1_2d) * np.linalg.norm(v2_2d)
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
    Rotates a 3D vector in-place around a center point in the XY plane.
    The Z component is not affected.
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

    # Extract XY components for rotation
    vx, vy = v[0], v[1]
    cx, cy = center[0], center[1]

    # Translate point to origin
    temp_x = vx - cx
    temp_y = vy - cy

    # Rotate in the XY plane
    rotated_x = temp_x * cos_a - temp_y * sin_a
    rotated_y = temp_x * sin_a + temp_y * cos_a

    # Translate back and update the original vector in-place
    v[0] = rotated_x + cx
    v[1] = rotated_y + cy
    # v[2] remains unchanged

    return v
