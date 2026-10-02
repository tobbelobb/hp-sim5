"""Small quaternion type matching the JavaScript engine."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass
class Quaternion:
    x: float = 0.; y: float = 0.; z: float = 0.; w: float = 1.
    def copy(self): return Quaternion(self.x, self.y, self.z, self.w)
    clone = copy
    def set(self, q):
        self.x, self.y, self.z, self.w = q.x, q.y, q.z, q.w; return self
    def normalize(self):
        length = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if length == 0: self.x = self.y = self.z = 0.; self.w = 1.
        else: self.x /= length; self.y /= length; self.z /= length; self.w /= length
        return self
    def conjugate(self):
        self.x, self.y, self.z = -self.x, -self.y, -self.z; return self
    def set_from_axis_angle(self, axis, angle):
        axis = np.asarray(axis, dtype=float); norm = np.linalg.norm(axis)
        if norm == 0: return self.set(Quaternion())
        self.x, self.y, self.z = axis * (math.sin(angle / 2) / norm)
        self.w = math.cos(angle / 2); return self
    def multiply(self, q):
        ax, ay, az, aw = self.x, self.y, self.z, self.w
        bx, by, bz, bw = q.x, q.y, q.z, q.w
        self.x = ax*bw + aw*bx + ay*bz - az*by
        self.y = ay*bw + aw*by + az*bx - ax*bz
        self.z = az*bw + aw*bz + ax*by - ay*bx
        self.w = aw*bw - ax*bx - ay*by - az*bz
        return self
    def transform_vector(self, vector):
        q = self.copy().normalize(); v = Quaternion(*np.asarray(vector, dtype=float), 0.)
        result = q.copy().multiply(v).multiply(q.copy().conjugate())
        return np.array([result.x, result.y, result.z])
    def as_xyzw(self): return np.array([self.x, self.y, self.z, self.w])
