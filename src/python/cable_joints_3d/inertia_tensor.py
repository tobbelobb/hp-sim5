"""Three-dimensional inertia helpers equivalent to ``inertia_tensor.js``."""
from dataclasses import dataclass, field
import numpy as np
from .quaternion import Quaternion

EPSILON = 1e-12
DEFAULT_AXIS = np.array([0., 0., 1.])

def normalize_inertia_tensor(value):
    if np.isscalar(value):
        inertia = float(value) if np.isfinite(value) and value > 0 else 0.
        return np.eye(3) * inertia
    tensor = getattr(value, "inertia_tensor", getattr(value, "tensor", value))
    array = np.asarray(tensor, dtype=float)
    return np.nan_to_num(array.reshape(3, 3)) if array.size == 9 else np.zeros((3, 3))

def invert_matrix3(matrix):
    matrix = normalize_inertia_tensor(matrix)
    try:
        # An absolute determinant threshold is scale-dependent: valid tensors
        # in hp-sim are commonly around 1e-6 kg m^2 and have determinants far
        # below 1e-12. LAPACK's solve detects singularity at the matrix's scale.
        return np.linalg.inv(matrix)
    except np.linalg.LinAlgError:
        if np.array_equal(matrix, np.diag(np.diag(matrix))):
            diagonal = np.diag(matrix)
            return np.diag(
                np.divide(1., diagonal, out=np.zeros(3), where=diagonal > 0.)
            )
        return np.zeros((3, 3))

def rotation_matrix_from_quaternion(quaternion):
    if quaternion is None: return np.eye(3)
    x, y, z, w = quaternion.copy().normalize().as_xyzw()
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                     [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                     [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])

def transform_inertia_tensor_to_world(tensor, orientation):
    rotation = rotation_matrix_from_quaternion(orientation)
    return rotation @ normalize_inertia_tensor(tensor) @ rotation.T

def parallel_axis_tensor(mass, offset):
    if not np.isfinite(mass) or mass <= 0: return np.zeros((3, 3))
    offset = np.asarray(offset, dtype=float)
    return mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))

def _axis(axis):
    axis = np.asarray(axis, dtype=float); norm = np.linalg.norm(axis)
    return axis / norm if norm > EPSILON else DEFAULT_AXIS.copy()

def effective_inertia_about_local_axis(moment, axis_local=DEFAULT_AXIS):
    if moment is None: return 0.
    axis = _axis(axis_local); value = float(axis @ moment.inertia_tensor @ axis)
    return value if np.isfinite(value) and value > EPSILON else 0.

def effective_inertia_about_world_axis(moment, orientation, axis_world=DEFAULT_AXIS):
    local = orientation.copy().conjugate().normalize().transform_vector(_axis(axis_world)) if orientation else _axis(axis_world)
    return effective_inertia_about_local_axis(moment, local)

def constrained_inv_inertia_about_local_axis(moment, axis_local=DEFAULT_AXIS):
    inertia = effective_inertia_about_local_axis(moment, axis_local)
    return 1. / inertia if inertia > EPSILON else 0.

def constrained_inv_inertia_about_world_axis(moment, orientation, axis_world=DEFAULT_AXIS):
    inertia = effective_inertia_about_world_axis(moment, orientation, axis_world)
    return 1. / inertia if inertia > EPSILON else 0.

def apply_world_inverse_inertia(moment, orientation, vector_world):
    if moment is None: return np.zeros(3)
    rotation = rotation_matrix_from_quaternion(orientation)
    return rotation @ moment.inv_inertia_tensor @ rotation.T @ np.asarray(vector_world, dtype=float)

def inverse_inertia_quadratic_form(moment, orientation, vector_world):
    vector = np.asarray(vector_world, dtype=float)
    value = float(vector @ apply_world_inverse_inertia(moment, orientation, vector))
    return value if np.isfinite(value) and value > EPSILON else 0.

def has_any_inverse_inertia(moment):
    return moment is not None and bool(np.any(np.abs(moment.inv_inertia_tensor) > EPSILON))

@dataclass
class MomentOfInertiaComponent:
    inertia: object = 1.
    axis_local: np.ndarray = field(default_factory=lambda: DEFAULT_AXIS.copy())
    inertia_tensor: np.ndarray = field(init=False)
    inv_inertia_tensor: np.ndarray = field(init=False)
    inv_inertia: float = field(init=False)
    def __post_init__(self):
        self.inertia_tensor = normalize_inertia_tensor(self.inertia); self.tensor = self.inertia_tensor
        self.inv_inertia_tensor = invert_matrix3(self.inertia_tensor); self.inverse_inertia_tensor = self.inv_inertia_tensor
        self.inertia = effective_inertia_about_local_axis(self, self.axis_local)
        self.inv_inertia = 1. / self.inertia if self.inertia > EPSILON else 0.
