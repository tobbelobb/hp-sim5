"""Simple 3D vector helpers using NumPy."""
import numpy as np

def vec3(x=0.0, y=0.0, z=0.0):
    return np.array([x, y, z], dtype=float)

def length(v):
    return np.linalg.norm(v)

def normalize(v):
    n = length(v)
    if n > 0:
        return v / n
    return v

def cross(a, b):
    return np.cross(a, b)

def dot(a, b):
    return float(np.dot(a, b))
