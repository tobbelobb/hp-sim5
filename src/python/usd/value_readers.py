"""Read authored USD opinions using the Hangprinter scene's frame conventions."""
import math
import re

import numpy as np

from cable_joints_3d.quaternion import Quaternion
from cable_joints_3d.spools import normalize_spool_axis_local


def attribute(prim, name):
    if not prim:
        return None
    attr = prim.GetAttribute(name)
    # Schema fallbacks are not authored machine data. The JS reader sees only
    # declarations, so a missing attribute must remain missing here too.
    return attr.Get() if attr and attr.HasAuthoredValueOpinion() else None


def relationship(prim, name):
    return [str(path) for path in prim.GetRelationship(name).GetTargets()] if prim else []


def vector_attribute(prim, name):
    value = attribute(prim, name)
    if value is None or len(value) < 2:
        return None
    result = np.array([value[0], value[1], value[2] if len(value) > 2 else 0.], dtype=float)
    return result if np.isfinite(result).all() else None


def numeric_attribute(prim, name):
    value = attribute(prim, name)
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _rotation_op(prim, op):
    op = op.removeprefix('!invert!')
    value = attribute(prim, op)
    if value is None:
        return None
    if re.fullmatch(r'xformOp:orient(?::.+)?', op):
        quaternion = Quaternion(*value.GetImaginary(), value.GetReal())
        return quaternion.normalize() if np.isfinite(quaternion.as_xyzw()).all() else None
    match = re.fullmatch(r'xformOp:rotate(XYZ|XZY|YXZ|YZX|ZXY|ZYX)(?::.+)?', op)
    if match is None or len(value) < 3 or not np.isfinite(value).all():
        return None
    result = Quaternion()
    for axis in match[1]:
        index = 'XYZ'.index(axis)
        result.premultiply(Quaternion().set_from_axis_angle(np.eye(3)[index], math.radians(value[index]))).normalize()
    return result


def orientation_attribute(prim):
    # Match the specialized scene reader's rotation ordering, rather than
    # replacing it with UsdGeom's complete transform/scale stack.
    result, found = Quaternion(), False
    for op in attribute(prim, 'xformOpOrder') or []:
        rotation = _rotation_op(prim, op)
        if rotation is not None:
            result.premultiply(rotation).normalize()
            found = True
    if found:
        return result
    for op in ['xformOp:orient'] + ['xformOp:rotate' + order for order in ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX']]:
        rotation = _rotation_op(prim, op)
        if rotation is not None:
            return rotation
    return Quaternion()


def axis_attribute(prim):
    for name in ['spool:axisLocal', 'machine:axisLocal', 'physics:rotationAxis']:
        axis = vector_attribute(prim, name)
        if axis is not None:
            return normalize_spool_axis_local(axis)
    return np.array([0., 0., 1.])


def material_properties(stage, prim):
    paths = relationship(prim, 'material:binding')
    material = stage.GetPrimAtPath(paths[0]) if paths else None
    if not material:
        return None, None, None
    shader = material.GetChild('Shader')
    color = attribute(shader, 'inputs:diffuseColor')
    color = '#' + ''.join(f'{math.floor(value * 255 + .5):02x}' for value in color) if color is not None else None
    return color, attribute(material, 'physics:staticFriction'), attribute(material, 'physics:restitution')
