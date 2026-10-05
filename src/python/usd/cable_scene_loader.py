"""Resolve authored cable initialization natively on a pxr.Usd stage.

Reuses the native tangent, arc and layer helpers. Like the JS load-time baker,
this is separate from timestep attachment rebuilding and respects manual values.
"""
import math
from types import SimpleNamespace

import numpy as np
from pxr import Sdf, Tf, Usd

from cable_joints_3d import geometry3 as geometry
from cable_joints_3d.cable_layering import KNOT_SPAN, stored_to_radius_and_theta
from cable_joints_3d.quaternion import Quaternion
from .value_readers import attribute, orientation_attribute, relationship, vector_attribute

EPSILON = 1e-9


def open_stage(path):
    source = isinstance(path, str) and path.lstrip().startswith('#usda')
    label = 'in-memory USDA' if source else str(path)
    try:
        if source:
            layer = Sdf.Layer.CreateAnonymous('.usda')
            layer.ImportFromString(path)
            stage = Usd.Stage.Open(layer)
        else:
            stage = Usd.Stage.Open(str(path))
    except Tf.ErrorException:
        raise ValueError(f'Unable to parse USD scene: {label}') from None
    if stage is None:
        raise ValueError(f'Unable to open USD scene: {label}')
    return stage


def open_cable_scene(path, **options):
    stage = open_stage(path)
    bake_cable_stage(stage, **options)
    return stage


def _world_transform(stage, path, cache):
    if path in cache:
        return cache[path]
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return np.zeros(3), Quaternion()
    position = vector_attribute(prim, 'xformOp:translate')
    position = np.zeros(3) if position is None else position
    orientation = orientation_attribute(prim)
    parent = prim.GetParent()
    if parent and not parent.IsPseudoRoot():
        parent_position, parent_orientation = _world_transform(stage, str(parent.GetPath()), cache)
        position = parent_position + parent_orientation.transform_vector(position)
        orientation = parent_orientation.copy().multiply(orientation).normalize()
    cache[path] = position, orientation
    return position, orientation


def _body(stage, path, transforms, bodies):
    if path in bodies:
        return bodies[path]
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise ValueError(f'Missing body prim at {path}.')
    position, orientation = _world_transform(stage, path, transforms)
    axis = None
    for name in ['spool:axisLocal', 'machine:axisLocal', 'physics:rotationAxis']:
        axis = vector_attribute(prim, name)
        if axis is not None:
            break
    axis = np.array([0., 0., 1.]) if axis is None or np.dot(axis, axis) <= EPSILON else axis / np.linalg.norm(axis)
    radius = attribute(prim, 'radius')
    radius = float(radius) if radius is not None and math.isfinite(radius) else 0.
    body = SimpleNamespace(path=path, position=position, orientation=orientation,
                           radius=radius, normal=orientation.transform_vector(axis))
    bodies[path] = body
    return body


def _rolling_radius(body, kind, index, kinds, stored, half_width):
    if kind not in ('rolling', 'hybrid') or half_width <= EPSILON:
        return body.radius
    radius = body.radius + half_width
    if kind == 'hybrid' and index in (0, len(kinds) - 1) and stored[index] is not None and stored[index] > EPSILON:
        radius = max(radius, stored_to_radius_and_theta(stored[index], body.radius, half_width, body.radius * KNOT_SPAN).radius)
    return radius


def _joint_points(a, b, kinds, clockwise, index, stored, half_width):
    rolling_a, rolling_b = [kind in ('rolling', 'hybrid') for kind in kinds[index:index + 2]]
    ra = _rolling_radius(a, kinds[index], index, kinds, stored, half_width)
    rb = _rolling_radius(b, kinds[index + 1], index + 1, kinds, stored, half_width)
    cw_a, cw_b = (not clockwise[index] if index == 0 else clockwise[index]), clockwise[index + 1]
    if (rolling_a and ra <= EPSILON) or (rolling_b and rb <= EPSILON):
        raise ValueError(f'Cannot derive rolling tangent without radius on {a.path} or {b.path}.')
    if rolling_a and rolling_b and abs(abs(np.dot(a.normal, b.normal)) - 1) <= 1e-6:
        tangents = geometry.tangent_from_sphere_to_sphere(a.position, ra, cw_a, b.position, rb, cw_b, a.normal)
        return tangents['a_sphere'], tangents['b_sphere']
    point_a, point_b = a.position.copy(), b.position.copy()
    if rolling_a:
        projected = b.position - a.normal * np.dot(b.position - a.position, a.normal)
        point_a = geometry.tangent_from_sphere_to_point(projected, a.position, ra, a.normal, cw_a)['a_sphere']
    if rolling_b:
        projected = a.position - b.normal * np.dot(a.position - b.position, b.normal)
        point_b = geometry.tangent_from_point_to_sphere(projected, b.position, rb, b.normal, cw_b)['a_sphere']
    return point_a, point_b


def _values(prim, name, count):
    values = attribute(prim, name)
    if values is None:
        return [None] * count
    if len(values) != count:
        raise ValueError(f'{prim.GetPath()}: {name} must have {count} entries when authored.')
    return list(values)


def _author(prim, name, value, type_name):
    prim.CreateAttribute(name, type_name, custom=True).Set(value)


def bake_cable_stage(stage, *, derive_all=False, cable_path_half_width_override=None):
    """Bake into the in-memory stage; never save or change its source file."""
    transforms, bodies, seen, results = {}, {}, set(), []
    paths = sorted((prim for prim in stage.Traverse() if relationship(prim, 'cablePath:joints')),
                   key=lambda prim: str(prim.GetPath()).casefold())
    for prim in paths:
        joint_paths = relationship(prim, 'cablePath:joints')
        kinds = list(attribute(prim, 'cablePath:linkTypes') or [])
        clockwise = [bool(value) for value in attribute(prim, 'cablePath:clockwise') or []]
        count = len(kinds)
        if len(joint_paths) + 1 != count or len(clockwise) != count:
            raise ValueError(f'{prim.GetPath()}: linkTypes and clockwise must have joint count + 1 entries.')
        policy = 'deriveAll' if derive_all else attribute(prim, 'cablePath:initPolicy')
        policy = policy if policy in ('manual', 'deriveAll', 'deriveMissing') else 'deriveMissing'
        stored = _values(prim, 'cablePath:stored', count)
        modes = _values(prim, 'cablePath:storedMode', count)
        if any(mode not in (None, 'manual', 'auto') for mode in modes):
            raise ValueError(f'{prim.GetPath()}: unsupported cablePath:storedMode.')
        half_width = cable_path_half_width_override if cable_path_half_width_override is not None else attribute(prim, 'cablePath:halfWidth')
        half_width = max(0., half_width) if half_width is not None and math.isfinite(half_width) else 0.
        chain, joints = [], []
        for index, path in enumerate(joint_paths):
            joint = stage.GetPrimAtPath(path)
            if not joint or path in seen:
                raise ValueError(f'{path}: missing or referenced by multiple CablePath prims.')
            seen.add(path)
            endpoints = [relationship(joint, 'physics:body' + str(side)) for side in (0, 1)]
            if any(len(paths) != 1 for paths in endpoints):
                raise ValueError(f'{path}: CableJoint must have one body0 and body1 target.')
            a_path, b_path = endpoints[0][0], endpoints[1][0]
            if index == 0:
                chain.append(a_path)
            elif chain[-1] != a_path:
                raise ValueError(f'{prim.GetPath()}: joints do not form a continuous chain at {path}.')
            chain.append(b_path)
            a, b = [_body(stage, name, transforms, bodies) for name in (a_path, b_path)]
            derived = _joint_points(a, b, kinds, clockwise, index, stored, half_width)
            local, world = [], []
            for side, body in enumerate((a, b)):
                point = vector_attribute(joint, 'localPos' + str(side)) if policy != 'deriveAll' else None
                if point is None:
                    if policy == 'manual':
                        raise ValueError(f'{path}: localPos{side} is required for manual initialization.')
                    point = body.orientation.copy().conjugate().normalize().transform_vector(derived[side] - body.position)
                local.append(point)
                world.append(body.position + body.orientation.transform_vector(point))
                _author(joint, 'localPos' + str(side), tuple(point), Sdf.ValueTypeNames.Point3d)
            length = attribute(joint, 'restLength') if policy != 'deriveAll' else None
            if length is None or not math.isfinite(length):
                if policy == 'manual':
                    raise ValueError(f'{path}: restLength is required for manual initialization.')
                length = float(np.linalg.norm(world[1] - world[0]))
            _author(joint, 'restLength', length, Sdf.ValueTypeNames.Double)
            joints.append({'jointPath': path, 'body0Path': a_path, 'body1Path': b_path,
                           'world0': world[0].tolist(), 'world1': world[1].tolist(),
                           'local0': local[0].tolist(), 'local1': local[1].tolist(), 'restLength': length})
        resolved = []
        for index, kind in enumerate(kinds):
            auto = None
            if kind == 'rolling':
                auto = 0.
                if 0 < index < len(chain) - 1:
                    body = bodies[chain[index]]
                    radians = geometry.signed_arc_length_on_wheel(np.array(joints[index - 1]['world1']), np.array(joints[index]['world0']), body.position, 1., clockwise[index], body.normal, True)
                    auto = max(0., body.radius + half_width) * radians
            authored = stored[index] is not None and math.isfinite(stored[index])
            if policy == 'manual' or (modes[index] == 'manual' and not (policy == 'deriveAll' and auto is not None)):
                if not authored:
                    raise ValueError(f'{prim.GetPath()}: stored[{index}] is required for manual initialization.')
                value = stored[index]
            elif auto is not None and (policy == 'deriveAll' or modes[index] == 'auto' or not authored):
                value = auto
            else:
                value = stored[index] if authored else 0.
            resolved.append(value)
        _author(prim, 'cablePath:stored', resolved, Sdf.ValueTypeNames.DoubleArray)
        _author(prim, 'cablePath:storedMode', ['manual'] * count, Sdf.ValueTypeNames.TokenArray)
        _author(prim, 'cablePath:initPolicy', 'manual', Sdf.ValueTypeNames.Token)
        if cable_path_half_width_override is not None and math.isfinite(cable_path_half_width_override):
            _author(prim, 'cablePath:halfWidth', half_width, Sdf.ValueTypeNames.Double)
        results.append({'pathPath': str(prim.GetPath()), 'resolvedStored': resolved, 'jointResults': joints})
    return results
