"""Simple 3D vector helpers using NumPy."""
import numpy as np
from math import sqrt

def vec3(x=0.0, y=0.0, z=0.0):
    return np.array([x, y, z], dtype=float)

def length(v):
    # Keep NumPy's dot-product reduction order without general norm dispatch.
    return sqrt(np.dot(v, v))

def normalize(v):
    n = length(v)
    if n > 0:
        return v / n
    return v

def cross(a, b):
    ax, ay, az = map(float, a)
    bx, by, bz = map(float, b)
    return np.array([ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx])

def dot(a, b):
    return float(np.dot(a, b))
