"""Carrier / relative-rotor inverse mass, including unloaded mounted rotors."""
import numpy as np
from .ecs import OrientationComponent, MomentOfInertiaComponent, RigidBodyMemberComponent
from .inertia_tensor import transform_inertia_tensor_to_world, invert_matrix3
from .quaternion import Quaternion
from .rigid_bodies import get_entity_world_orientation


def cable_angular_response(world, ends, inverses, get_rotor):
    rotors = {}
    for entity in world.query([RigidBodyMemberComponent]):
        if get_rotor(entity) is not None:
            body = world.get_component(entity, RigidBodyMemberComponent).body_entity
            rotors.setdefault(body, []).append(entity)
    groups, standalone = {}, []
    for end, inverse in zip(ends, inverses):
        member = world.get_component(end.entity, RigidBodyMemberComponent)
        body = member.body_entity if member else end.entity
        if body not in rotors:
            standalone.append((end, inverse))
            continue
        gradient, spins = groups.setdefault(body, (np.zeros(3), {}))
        if member is None:
            gradient += end.angular_gradient
        if end.spin:
            previous = spins.get(end.spin.entity, (0., inverse))[0]
            spins[end.spin.entity] = (previous + end.solve_spin_gradient, inverse)
    denominator, corrections = 0., []
    for body, (gradient, spins) in groups.items():
        orientation = world.get_component(body, OrientationComponent)
        moment = world.get_component(body, MomentOfInertiaComponent)
        if orientation is None:
            continue
        orientation = orientation.quaternion.copy()
        tensor = transform_inertia_tensor_to_world(moment.inertia_tensor if moment else np.zeros((3, 3)), orientation)
        rhs, members = gradient.copy(), []
        for entity in rotors[body]:
            rotor = get_rotor(entity)
            grad, inverse = spins.get(entity, (0., rotor.inv_inertia))
            q = get_entity_world_orientation(world, entity)
            inertia = world.get_component(entity, MomentOfInertiaComponent).inertia_tensor
            coupling = transform_inertia_tensor_to_world(inertia, q) @ rotor.axis
            tensor -= inverse * np.outer(coupling, coupling)
            rhs -= inverse * coupling * grad
            members.append((entity, rotor.axis, coupling, inverse, grad))
        angular = invert_matrix3(tensor) @ rhs
        denominator += np.dot(gradient, angular)
        angles = []
        for entity, axis, coupling, inverse, grad in members:
            angle = inverse * (grad - np.dot(coupling, angular))
            denominator += grad * angle
            angles.append((entity, axis, angle))
        corrections.append((body, orientation, angular, angles))
    for end, inverse in standalone:
        denominator += end.angular_denominator + inverse * end.solve_spin_gradient**2
    return denominator, corrections, standalone


def apply_cable_angular_response(world, corrections, multiplier, record_encoder):
    for body, orientation, angular, members in corrections:
        delta = -multiplier * angular
        angle = np.linalg.norm(delta)
        if angle > 1e-12:
            q = world.get_component(body, OrientationComponent).quaternion
            q.premultiply(Quaternion().set_from_axis_angle(delta / angle, angle)).normalize()
        for entity, axis, coefficient in members:
            angle = -multiplier * coefficient
            if abs(angle) <= 1e-12:
                continue
            member = world.get_component(entity, RigidBodyMemberComponent)
            old_local = member.local_orientation.copy()
            local_axis = orientation.copy().conjugate().normalize().transform_vector(axis)
            member.local_orientation.premultiply(Quaternion().set_from_axis_angle(local_axis, angle)).normalize()
            raw = world.get_component(entity, OrientationComponent).quaternion
            raw.multiply(old_local.conjugate().normalize()).multiply(member.local_orientation).normalize()
            record_encoder(entity, angle)
