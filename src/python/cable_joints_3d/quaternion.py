"""Small quaternion type matching the JavaScript engine."""
from dataclasses import dataclass
import math
import numpy as np
from .vector3 import length as norm3


def rotation_vector_between(previous, current):
    if previous is None or current is None:
        return np.zeros(3)
    delta = current.copy().multiply(previous.copy().conjugate().normalize()).normalize()
    if delta.w < 0:
        delta.x, delta.y, delta.z, delta.w = -delta.x, -delta.y, -delta.z, -delta.w
    w = delta.w
    w = 1. if w > 1. else (-1. if w < -1. else w)
    angle = 2 * math.acos(w)
    sin_half = math.sqrt(max(0., 1 - w * w))
    if angle <= 1e-9 or sin_half <= 1e-9:
        return np.zeros(3)
    return delta.as_xyzw()[:3] / sin_half * angle

@dataclass
class Quaternion:
    x: float = 0.; y: float = 0.; z: float = 0.; w: float = 1.
    def copy(self): return Quaternion(self.x, self.y, self.z, self.w)
    clone = copy
    def set(self, q):
        self.x, self.y, self.z, self.w = q.x, q.y, q.z, q.w; return self
    def normalize(self):
        x, y, z, w = float(self.x), float(self.y), float(self.z), float(self.w)
        length = math.sqrt(x*x + y*y + z*z + w*w)
        if length == 0: self.x = self.y = self.z = 0.; self.w = 1.
        else:
            inverse = 1 / length
            self.x, self.y, self.z, self.w = x*inverse, y*inverse, z*inverse, w*inverse
        return self
    def conjugate(self):
        self.x, self.y, self.z = -self.x, -self.y, -self.z; return self
    def set_from_axis_angle(self, axis, angle):
        axis = np.asarray(axis, dtype=float); norm = norm3(axis)
        if norm == 0: return self.set(Quaternion())
        self.x, self.y, self.z = map(float, axis * (math.sin(angle / 2) / norm))
        self.w = math.cos(angle / 2); return self
    def multiply(self, q):
        return self.multiply_quaternions(self.copy(), q)

    def multiply_quaternions(self, a, b):
        ax, ay, az, aw = a.x, a.y, a.z, a.w
        bx, by, bz, bw = b.x, b.y, b.z, b.w
        self.x = ax*bw + aw*bx + ay*bz - az*by
        self.y = ay*bw + aw*by + az*bx - ax*bz
        self.z = az*bw + aw*bz + ax*by - ay*bx
        self.w = aw*bw - ax*bx - ay*by - az*bz
        return self

    def premultiply(self, q):
        return self.multiply_quaternions(q, self.copy())
    def transform_vector(self, vector):
        # Callers normalize their frames explicitly. Normalizing here changes
        # raw component semantics and adds another rounding step to live frames.
        x, y, z = map(float, vector)
        qx, qy, qz, qw = self.x, self.y, self.z, self.w
        ix = qw*x + qy*z - qz*y
        iy = qw*y + qz*x - qx*z
        iz = qw*z + qx*y - qy*x
        iw = -qx*x - qy*y - qz*z
        return np.array([ix*qw + iw*-qx + iy*-qz - iz*-qy,
                         iy*qw + iw*-qy + iz*-qx - ix*-qz,
                         iz*qw + iw*-qz + ix*-qy - iy*-qx], dtype=float)
    def as_xyzw(self): return np.array([self.x, self.y, self.z, self.w])
