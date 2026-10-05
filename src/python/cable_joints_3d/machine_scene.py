"""Build the specialized Hangprinter ECS from authored pxr.Usd machine data."""
import json
import math
from types import SimpleNamespace
import warnings

import numpy as np

from usd.value_readers import (attribute, axis_attribute, material_properties,
    numeric_attribute, orientation_attribute, relationship, vector_attribute)
from . import ecs
from .cable_joints_components import CableJointComponent, CableLinkComponent, create_cable_path_component
from .extruder import ExtruderComponent
from .inertia_tensor import parallel_axis_tensor, transform_inertia_tensor_to_world
from .quaternion import Quaternion
from .rigid_bodies import initialize_rigid_body_sync_state
from .spools import SpoolStateComponent, SpoolTagComponent
from .stepper_motor import StepperMotorComponent


def _add(world, entity, *components):
    for component in components:
        world.add_component(entity, component)


def _dynamic_gravity(world, entity, mass):
    if mass is not None and math.isfinite(mass) and mass > 0:
        world.add_component(entity, ecs.GravityAffectedComponent())


def _orientation(world, entity, orientation):
    _add(world, entity, ecs.OrientationComponent(*orientation.as_xyzw()),
         ecs.PrevFinalOrientationComponent(*orientation.as_xyzw()))


def _color(palette, *keys, fallback):
    return next((palette[key] for key in keys if palette.get(key) is not None), fallback)


def _body(world, stage, prim, machine, palette):
    tags = list(attribute(prim, 'ecs:tags') or [])
    position = vector_attribute(prim, 'xformOp:translate')
    if not tags or position is None:
        return None
    kind = next((tag for tag in ['Spool', 'Wheel', 'Anchor', 'Pinhole', 'Eyelet', 'Attachment'] if tag in tags), None)
    if kind is None:
        return None
    color, friction, restitution = material_properties(stage, prim)
    radius, mass, tensor = [attribute(prim, name) for name in ['radius', 'physics:mass', 'physics:inertiaTensor']]
    angular = vector_attribute(prim, 'physics:angularVelocity')
    if kind in ('Spool', 'Wheel') and (radius is None or mass is None or tensor is None or angular is None):
        warnings.warn(f'Skipping {kind} prim {prim.GetPath()} due to missing attributes.', stacklevel=2)
        return None
    entity = world.create_entity()
    world.add_component(entity, ecs.MachineTagComponent(machine))
    orientation = orientation_attribute(prim)
    axis = axis_attribute(prim)
    if kind == 'Spool':
        motor = StepperMotorComponent()
        for name, field in [('holdingTorque', 'holding_torque'), ('numPolePairs', 'num_pole_pairs'),
                            ('dampingCoeff', 'damping_coeff'), ('maxSpeedRad', 'max_speed_rad')]:
            value = numeric_attribute(prim, 'stepper:' + name)
            if value is not None:
                lower = math.floor(value)
                setattr(motor, field, lower + int(value - lower >= .5) if name == 'numPolePairs' else value)
        _add(world, entity, SpoolTagComponent(), SpoolStateComponent(prim.GetName()[-1].upper(), axis, orientation), motor)
    elif kind == 'Wheel':
        world.add_component(entity, SpoolStateComponent(None, axis, orientation))
    elif kind != 'Anchor':
        world.add_component(entity, SpoolTagComponent())
    if kind == 'Anchor':
        _add(world, entity, ecs.PositionComponent(*position), ecs.RadiusComponent(.01), ecs.MassComponent(-1.),
             ecs.RenderableComponent('circle', _color(palette, 'anchor', fallback=color or '#aaaaaa')))
        if attribute(prim, 'cable:linkable'):
            world.add_component(entity, CableLinkComponent(*position))
    else:
        velocity = vector_attribute(prim, 'physics:velocity')
        velocity = np.zeros(3) if velocity is None else velocity
        _add(world, entity, ecs.PositionComponent(*position), ecs.VelocityComponent(*velocity),
             ecs.MassComponent(mass), ecs.PrevFinalPosComponent(*position), ecs.RadiusComponent(radius if radius is not None else .002))
        _dynamic_gravity(world, entity, mass)
        style = 'spool' if kind == 'Spool' else 'wheel' if kind == 'Wheel' else 'pinhole'
        selected_color = _color(palette, *(['wheel', 'spool'] if style == 'wheel' else [style]),
                                fallback=color or ('#a0a0a0' if style in ('spool', 'wheel') else '#cccccc'))
        height = numeric_attribute(prim, 'height') if kind == 'Wheel' else None
        render = ecs.RenderableComponent('cylinder' if kind in ('Spool', 'Wheel') else 'circle', selected_color, height)
        world.add_component(entity, render)
        if angular is not None:
            _orientation(world, entity, orientation)
            world.add_component(entity, ecs.AngularVelocityComponent(*angular))
        if kind in ('Spool', 'Wheel'):
            world.add_component(entity, ecs.EncoderComponent())
        if tensor is not None:
            world.add_component(entity, ecs.MomentOfInertiaComponent(np.array(tensor), axis if kind in ('Spool', 'Wheel') else np.array([0., 0., 1.])))
        if restitution is not None:
            world.add_component(entity, ecs.RestitutionComponent(restitution))
        if friction is not None:
            world.add_component(entity, ecs.CoefficientOfFrictionComponent(friction))
        if kind == 'Wheel' or attribute(prim, 'cable:linkable'):
            world.add_component(entity, CableLinkComponent(*position, orientation,
                None if kind in ('Spool', 'Wheel') else np.array([0., 0., 1.]),
                axis if kind in ('Spool', 'Wheel') else None))
    world.add_component(entity, ecs.SceneEntityInfoComponent(prim.GetName(), tags))
    return entity


def _render_segments(value):
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            return None
    if value is None:
        return None
    value = list(value)
    sequences = value if any(isinstance(item, (list, tuple)) for item in value) else [value]
    result = []
    for sequence in sequences:
        indices = [int(index) for index in sequence if float(index).is_integer()]
        if len(indices) >= 2:
            result.append(indices)
    return result or None


def _rigid_body(world, members, prim, machine, palette):
    masses = [world.get_component(entity, ecs.MassComponent).mass for entity in members]
    mass = sum(value for value in masses if value is not None and value > 0)
    position, velocity = np.zeros(3), np.zeros(3)
    for entity, value in zip(members, masses):
        if value is not None and value > 0:
            position += world.get_component(entity, ecs.PositionComponent).pos * value
            velocity += world.get_component(entity, ecs.VelocityComponent).vel * value
    if mass > 0:
        position /= mass
        velocity /= mass
    tensor = np.zeros((3, 3))
    for entity, value in zip(members, masses):
        if value is None or value <= 0:
            continue
        member_position = world.get_component(entity, ecs.PositionComponent).pos
        moment = world.get_component(entity, ecs.MomentOfInertiaComponent)
        orientation = world.get_component(entity, ecs.OrientationComponent)
        if moment is not None:
            tensor += transform_inertia_tensor_to_world(moment.inertia_tensor, orientation.quaternion if orientation else Quaternion())
        tensor += parallel_axis_tensor(value, member_position - position)
    if np.trace(tensor) <= 0:
        tensor = np.eye(3) * mass
    body = world.create_entity()
    segments = attribute(prim, 'rigidBody:renderIndices')
    if segments is None:
        segments = attribute(prim, 'rigidGroup:renderIndices')
    _add(world, body, ecs.MachineTagComponent(machine), ecs.SceneEntityInfoComponent(prim.GetName(), ['RigidBody']),
         ecs.PositionComponent(*position), ecs.VelocityComponent(*velocity), ecs.MassComponent(mass),
         ecs.RenderableComponent('line', _color(palette, 'rigidBody', 'rigidGroup', 'distanceConstraint', fallback='#55ff88')),
         ecs.OrientationComponent(), ecs.PrevFinalOrientationComponent(), ecs.AngularVelocityComponent(),
         ecs.MomentOfInertiaComponent(tensor), ecs.PrevFinalPosComponent(*position), ecs.RigidBodyComponent(members, _render_segments(segments)))
    _dynamic_gravity(world, body, mass)
    initialize_rigid_body_sync_state(world, body)
    for entity, value in zip(members, masses):
        member_position = world.get_component(entity, ecs.PositionComponent).pos
        orientation = world.get_component(entity, ecs.OrientationComponent)
        world.add_component(entity, ecs.RigidBodyMemberComponent(body, member_position - position,
            orientation.quaternion if orientation else Quaternion(), value))
        world.remove_component(entity, ecs.GravityAffectedComponent)
        world.get_component(entity, ecs.MassComponent).mass = 0.
        world.get_component(entity, ecs.VelocityComponent).vel[:] = 0.
    return body


def populate_machine_scene(world, stage, scene_prim_path='/World/SlideprinterScene', *,
                           namespace=None, append=False, palette=None, tint_color=None, extrusion_color=None):
    """Construct state in JS builder order; system registration is separate."""
    root = stage.GetPrimAtPath(scene_prim_path.rstrip('/'))
    if not root:
        raise ValueError(f'Unable to find machine scene root at {scene_prim_path}.')
    machine, palette = namespace or 'default', palette or {}
    if not append:
        world.clear()
        # Entity IDs are reused after clear; loads belong to the discarded scene.
        for key in ['torqueModeCableLoadTorques', 'torqueModeCableLoadStiffnesses', 'torqueModeCableLoadDampings']:
            world.set_resource(key, {})
        world.set_resource('sceneGeneration', (world.get_resource('sceneGeneration') or 0) + 1)
        physics = stage.GetPrimAtPath('/World/PhysicsScene')
        direction = vector_attribute(physics, 'physics:gravityDirection')
        magnitude = numeric_attribute(physics, 'physics:gravityMagnitude')
        if direction is None or magnitude is None:
            raise ValueError('Machine scene requires authored physics gravity direction and magnitude.')
        world.set_resource('gravity', direction * magnitude)
        world.set_resource('dt', 1 / stage.GetTimeCodesPerSecond())
        world.set_resource('defaultPlaneNormal', np.array([0., 0., 1.]))
        pause = world.get_resource('pauseState')
        world.set_resource('pauseState', SimpleNamespace(paused=bool(getattr(pause, 'paused', False))))
        world.set_resource('errorState', SimpleNamespace(has_error=False))
        world.set_resource('grabbedBall', None)
        world.set_resource('debugRenderPoints', {})
    colors = world.get_resource('machineColors')
    colors = colors if append and isinstance(colors, dict) else {}
    colors[machine] = {'tintColor': tint_color, 'extrusionColor': extrusion_color}
    world.set_resource('machineColors', colors)
    named, joint_entities, body_members = {}, {}, {}
    children = root.GetChildren()
    extruder_prim = None
    for prim in children:
        tags = list(attribute(prim, 'ecs:tags') or [])
        if not tags or attribute(prim, 'xformOp:translate') is None:
            continue
        if 'Extruder' in tags or relationship(prim, 'machine:centerSources'):
            extruder_prim = prim
            continue
        entity = _body(world, stage, prim, machine, palette)
        if entity is not None:
            named[str(prim.GetPath())] = entity
    for prim in children:
        paths = relationship(prim, 'rigidBody:members') or relationship(prim, 'rigidGroup:members')
        members = [named[path] for path in paths if path in named]
        if len(members) >= 2:
            body = _rigid_body(world, members, prim, machine, palette)
            body_members.update({entity: body for entity in members})
    for prim in children:
        if prim.GetTypeName() != 'DistancePhysicsJoint':
            continue
        endpoints = [relationship(prim, 'physics:body' + str(index)) for index in (0, 1)]
        if any(not paths or paths[0] not in named for paths in endpoints):
            continue
        a, b = [named[paths[0]] for paths in endpoints]
        minimum, maximum = attribute(prim, 'physics:minDistance'), attribute(prim, 'physics:maxDistance')
        if minimum is None or maximum is None or abs(minimum - maximum) >= 1e-6:
            continue
        if a in body_members and body_members[a] == body_members.get(b):
            continue
        entity = world.create_entity()
        _add(world, entity, ecs.MachineTagComponent(machine), ecs.DistanceConstraintComponent(a, b, (minimum + maximum) / 2, 0.),
             ecs.RenderableComponent('line', _color(palette, 'distanceConstraint', fallback='green')))
    for prim in children:
        if prim.GetTypeName() != 'CableJoint':
            continue
        endpoints = [relationship(prim, 'physics:body' + str(index)) for index in (0, 1)]
        if any(not paths or paths[0] not in named for paths in endpoints):
            raise ValueError(f'{prim.GetPath()}: unresolved cable body relationship.')
        a, b = [named[paths[0]] for paths in endpoints]
        length = attribute(prim, 'restLength')
        points = [vector_attribute(prim, 'localPos' + str(index)) for index in (0, 1)]
        if length is None or any(point is None for point in points):
            raise ValueError(f'{prim.GetPath()}: cable joint requires baked length and local attachments.')
        entity = world.create_entity()
        _add(world, entity, ecs.MachineTagComponent(machine), ecs.SceneEntityInfoComponent(prim.GetName(), ['CableJoint']),
             CableJointComponent.from_local(world, a, b, length, *points), ecs.RenderableComponent('line', _color(palette, 'cable', fallback='#FFFF00')))
        joint_entities[str(prim.GetPath())] = entity
    for prim in children:
        schemas = prim.GetMetadata('apiSchemas')
        if schemas is None or 'CablePathAPI' not in schemas.GetAddedOrExplicitItems():
            continue
        joints = [joint_entities[path] for path in relationship(prim, 'cablePath:joints') if path in joint_entities]
        stiffness = attribute(prim, 'cablePath:stiffness')
        path = create_cable_path_component(world, joints, list(attribute(prim, 'cablePath:linkTypes') or []),
            list(attribute(prim, 'cablePath:clockwise') or []), stiffness if stiffness is not None else math.inf,
            list(attribute(prim, 'cablePath:stored') or []), attribute(prim, 'cablePath:halfWidth') or 0.,
            numeric_attribute(prim, 'cablePath:damping') or 0., numeric_attribute(prim, 'cablePath:solverIterations') or 1)
        entity = world.create_entity()
        _add(world, entity, path, ecs.MachineTagComponent(machine), ecs.SceneEntityInfoComponent(prim.GetName(), ['CablePath']))
    extruders = world.query([ExtruderComponent])
    if not extruders:
        entity = world.create_entity()
        world.add_component(entity, ExtruderComponent())
        extruders = [entity]
    extruder = world.get_component(extruders[0], ExtruderComponent)
    paths = relationship(extruder_prim, 'machine:centerSources')
    sources = [named[path] for path in paths if path in named]
    positions = [world.get_component(entity, ecs.PositionComponent).pos for entity in sources]
    for field in ['center_sources', 'center_source_offsets', 'center_offsets', 'tip_offsets', 'cold_end_offsets']:
        getattr(extruder, field).pop(machine, None)
    if sources:
        extruder.center_sources[machine] = sources[:]
        center = sum(positions, np.zeros(3)) / len(positions)
        extruder.center_source_offsets[machine] = [(point - center).copy() for point in positions]
    if extruder_prim:
        extruder.center_offsets[machine] = vector_attribute(extruder_prim, 'xformOp:translate')
        tip = extruder_prim.GetChild('Tip') or extruder_prim.GetChild('HotEnd')
        cold = extruder_prim.GetChild('ColdEnd') or extruder_prim.GetChild('Bottom')
        offset = vector_attribute(tip, 'xformOp:translate')
        extruder.tip_offsets[machine] = np.zeros(3) if offset is None else offset
        offset = vector_attribute(cold, 'xformOp:translate')
        if offset is not None:
            extruder.cold_end_offsets[machine] = offset
    for system in world.systems:
        if hasattr(system, 'reset_axis_mapping'):
            system.reset_axis_mapping()
    return named
