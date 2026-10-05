"""The specialized JS cable solve: tensor bodies, one-axis spools and motor loads."""
from dataclasses import dataclass
import math

import numpy as np
from cable_joints.util import effective_cw, is_hybrid

from .cable_attachment_update_system import effective_rolling_radius
from .cable_frames import delta_angle_for_entity
from .cable_joints_components import CableJointComponent, CableLinkComponent, CablePathComponent
from .ecs import (
    EncoderComponent, MassComponent, MomentOfInertiaComponent, OrientationComponent,
    PositionComponent, PrevFinalOrientationComponent, PrevFinalPosComponent, RigidBodyMemberComponent,
)
from .inertia_tensor import (
    apply_world_inverse_inertia, constrained_inv_inertia_about_world_axis,
    effective_inertia_about_world_axis, has_any_inverse_inertia, inverse_inertia_quadratic_form,
)
from .quaternion import rotation_vector_between
from .rigid_bodies import (
    apply_world_angular_correction, compute_local_attachment, compute_world_attachment,
    get_entity_world_position, resolve_rigid_body_solver_endpoint,
)
from .spools import SpoolStateComponent
from .stepper_motor import (
    StepperMotorComponent, effective_motorized_spin_inv_inertia, finite_number,
    open_loop_stepper_holding_torque_limit,
)

EPSILON = 1e-9


def has_axis_only_cable_spin_dof(world, entity):
    link = world.get_component(entity, CableLinkComponent)
    return world.get_component(entity, SpoolStateComponent) is not None and link is not None and link.cable_plane_normal_local is not None


@dataclass
class SpinSolveInfo:
    entity: int
    center: np.ndarray
    free_inv_inertia: float
    inv_inertia: float
    holding_torque_limit: float
    torque_mode: bool
    axis: np.ndarray
    point: np.ndarray
    gradient: np.ndarray
    stored_gradient: float = 0.
    solve_uses_stored_gradient: bool = False
    use_stored_only_for_load: bool = False
    record_implicit_coefficients: bool = False
    defer_to_pinhole_neighbor: bool = False
    transferred_joint: int | None = None


def _build_spin_info(world, entity, point, gradient, dt):
    link = world.get_component(entity, CableLinkComponent)
    orientation = world.get_component(entity, OrientationComponent)
    moment = world.get_component(entity, MomentOfInertiaComponent)
    center = get_entity_world_position(world, entity)
    if link is None or link.cable_plane_normal_local is None or orientation is None or center is None or not has_any_inverse_inertia(moment):
        return None
    axis = orientation.quaternion.transform_vector(link.cable_plane_normal_local)
    if np.dot(axis, axis) <= EPSILON:
        return None
    axis /= np.linalg.norm(axis)
    free_inverse = constrained_inv_inertia_about_world_axis(moment, orientation.quaternion, axis)
    if free_inverse <= EPSILON:
        return None
    inertia = effective_inertia_about_world_axis(moment, orientation.quaternion, axis)
    stepper = world.get_component(entity, StepperMotorComponent)
    return SpinSolveInfo(
        entity, center, free_inverse, effective_motorized_spin_inv_inertia(world, entity, free_inverse, inertia, dt),
        open_loop_stepper_holding_torque_limit(world, stepper), bool(stepper and stepper.torque_mode),
        axis, point.copy(), gradient.copy(),
    )


def _hybrid_stored_gradient(world, path, index, first, entity):
    if not is_hybrid(path.link_types[index]):
        return 0.
    from .ecs import RadiusComponent

    component = world.get_component(entity, RadiusComponent)
    base_radius = component.radius if component else None
    radius = effective_rolling_radius(world, path, index, base_radius)[0]
    radius = radius if finite_number(radius) else (base_radius if finite_number(base_radius) else 0.)
    if radius <= EPSILON:
        return 0.
    gradient = (1 if effective_cw(path, index, first) else -1) * radius
    return gradient if first else -gradient


def _spin_info(world, path, index, first, entity, mapped, point, gradient, joint_locals, dt):
    link_index = index if first else index + 1
    if has_axis_only_cable_spin_dof(world, entity) and link_index < len(path.link_types):
        info = _build_spin_info(world, entity, point, gradient, dt)
        if info is not None:
            info.stored_gradient = _hybrid_stored_gradient(world, path, link_index, first, entity)
            if abs(info.stored_gradient) > EPSILON:
                info.solve_uses_stored_gradient = info.use_stored_only_for_load = info.record_implicit_coefficients = True
                info.defer_to_pinhole_neighbor = (
                    (first and path.link_types[index + 1] == 'pinhole' and index + 1 < len(path.joint_entities))
                    or (not first and path.link_types[index] == 'pinhole' and index > 0))
            return info
    if mapped.internal_to_body or len(path.joint_entities) < 2 or path.link_types[link_index] != 'pinhole':
        return None
    if first and link_index > 0 and (path.link_types[link_index - 1] == 'rolling' or is_hybrid(path.link_types[link_index - 1])):
        internal_index, spin_first = index - 1, True
    elif not first and link_index < len(path.link_types) - 1 and (path.link_types[link_index + 1] == 'rolling' or is_hybrid(path.link_types[link_index + 1])):
        internal_index, spin_first = index + 1, False
    else:
        return None
    if not 0 <= internal_index < len(path.joint_entities):
        return None
    internal_id = path.joint_entities[internal_index]
    internal = world.get_component(internal_id, CableJointComponent)
    local_points = joint_locals.get(internal_id)
    if internal is None or local_points is None:
        return None
    spin_entity = internal.entity_a if spin_first else internal.entity_b
    spin_point = compute_world_attachment(world, spin_entity, local_points[0 if spin_first else 1])
    if spin_point is None:
        return None
    coupled_gradient = point - spin_point
    if np.dot(coupled_gradient, coupled_gradient) <= EPSILON:
        return None
    coupled_gradient /= np.linalg.norm(coupled_gradient)
    info = _build_spin_info(world, spin_entity, spin_point, coupled_gradient, dt)
    if info is not None:
        info.transferred_joint = internal_id
        spin_index = internal_index if spin_first else internal_index + 1
        info.stored_gradient = _hybrid_stored_gradient(world, path, spin_index, spin_first, spin_entity)
        info.use_stored_only_for_load = abs(info.stored_gradient) > EPSILON
        info.record_implicit_coefficients = info.use_stored_only_for_load
    return info


def _quaternion(world, entity, previous=False):
    component = world.get_component(entity, PrevFinalOrientationComponent if previous else OrientationComponent)
    return component.quaternion if component else None


def _local_quaternion(world, entity, previous=False):
    member = world.get_component(entity, RigidBodyMemberComponent)
    if member is None:
        return _quaternion(world, entity, previous)
    if not previous:
        return member.local_orientation
    body = _quaternion(world, member.body_entity, True)
    part = _quaternion(world, entity, True)
    return body.copy().conjugate().normalize().multiply(part).normalize() if body is not None and part is not None else None


@dataclass
class SolverEnd:
    entity: int
    position: object
    gradient: np.ndarray
    inv_mass: float
    moment: object
    orientation: object
    angular_gradient: np.ndarray
    angular_denominator: float
    spin: SpinSolveInfo | None
    solve_spin_gradient: float
    load_spin_gradient: float
    displacement: float


def _solver_end(world, path, index, first, entity, other, point, gradient, joint_locals, dt):
    mapped = resolve_rigid_body_solver_endpoint(world, entity, other, point)
    spin = _spin_info(world, path, index, first, entity, mapped, point, gradient, joint_locals, dt)
    solver_entity = mapped.entity_id
    position = world.get_component(solver_entity, PositionComponent)
    if position is None:
        return None
    mass = world.get_component(solver_entity, MassComponent)
    inv_mass = 1 / mass.mass if mass is not None and mass.mass > 0 else 0.
    moment = world.get_component(solver_entity, MomentOfInertiaComponent)
    orientation = world.get_component(solver_entity, OrientationComponent)
    angular_gradient = np.cross(point - position.pos, gradient)
    angular_denominator = inverse_inertia_quadratic_form(moment, orientation.quaternion, angular_gradient) if orientation is not None and not has_axis_only_cable_spin_dof(world, solver_entity) else 0.
    previous_position = world.get_component(solver_entity, PrevFinalPosComponent)
    displacement = float(np.dot(gradient, position.pos - previous_position.pos)) if previous_position is not None else 0.
    displacement += np.dot(angular_gradient, rotation_vector_between(_quaternion(world, solver_entity, True), _quaternion(world, solver_entity)))
    solve_spin_gradient = load_spin_gradient = 0.
    if spin is not None:
        lever_gradient = np.dot(np.cross(spin.point - spin.center, spin.gradient), spin.axis)
        solve_spin_gradient = spin.stored_gradient if spin.solve_uses_stored_gradient and spin.torque_mode else lever_gradient
        load_spin_gradient = spin.stored_gradient if spin.use_stored_only_for_load else solve_spin_gradient
        link = world.get_component(spin.entity, CableLinkComponent)
        if link is not None and link.cable_plane_normal_local is not None:
            previous, current = _local_quaternion(world, spin.entity, True), _local_quaternion(world, spin.entity)
            displacement += solve_spin_gradient * delta_angle_for_entity(world, spin.entity, previous, current, previous, current)
    return SolverEnd(solver_entity, position, gradient, inv_mass, moment, orientation, angular_gradient,
                     angular_denominator, spin, solve_spin_gradient, load_spin_gradient, displacement)


def _mechanical_denominator(ends, spin_inverses):
    denominator = 0.
    for end, inverse in zip(ends, spin_inverses):
        denominator += end.inv_mass * np.dot(end.gradient, end.gradient)
        denominator += end.angular_denominator
        if inverse > 0 and abs(end.solve_spin_gradient) > EPSILON:
            denominator += inverse * end.solve_spin_gradient * end.solve_spin_gradient
    return denominator


def _record_transferred_force(world, joint_id, magnitude):
    if joint_id is None or not finite_number(magnitude) or magnitude <= 0:
        return
    joint = world.get_component(joint_id, CableJointComponent)
    if joint is not None:
        joint.transferred_constraint_force_magnitude = max(joint.transferred_constraint_force_magnitude, magnitude)


def _record_load(path, end, multiplier, inv_dt_squared, loads):
    spin = end.spin
    gradient = end.load_spin_gradient
    if spin is None or not spin.torque_mode or spin.defer_to_pinhole_neighbor or inv_dt_squared <= 0 or abs(gradient) <= EPSILON:
        return
    torque = multiplier * inv_dt_squared * gradient
    if not finite_number(torque) or abs(torque) <= EPSILON:
        return
    torques, stiffnesses, dampings = loads
    torques[spin.entity] = torques.get(spin.entity, 0.) + torque
    if spin.record_implicit_coefficients:
        stiffness = max(0., path.spring_constant) if finite_number(path.spring_constant) else 0.
        damping = max(0., path.damping) if finite_number(path.damping) else 0.
        if stiffness > 0:
            stiffnesses[spin.entity] = stiffnesses.get(spin.entity, 0.) + stiffness * gradient * gradient
        if damping > 0:
            dampings[spin.entity] = dampings.get(spin.entity, 0.) + damping * gradient * gradient


def _apply_end_correction(world, end, inverse, multiplier):
    if end.inv_mass > 0:
        end.position.pos += end.gradient * (-end.inv_mass * multiplier)
    if end.angular_denominator > 0 and end.orientation is not None:
        delta = apply_world_inverse_inertia(end.moment, end.orientation.quaternion, end.angular_gradient) * -multiplier
        apply_world_angular_correction(world, end.entity, delta)
    if inverse > 0 and end.spin is not None and abs(end.solve_spin_gradient) > EPSILON:
        delta_angle = -inverse * multiplier * end.solve_spin_gradient
        if abs(delta_angle) > EPSILON and world.get_component(end.spin.entity, OrientationComponent) is not None:
            apply_world_angular_correction(world, end.spin.entity, end.spin.axis * delta_angle)
            encoder = world.get_component(end.spin.entity, EncoderComponent)
            if encoder is not None and finite_number(encoder.angle):
                encoder.angle += delta_angle


def _solve_joint(world, path, index, joint, points, error, iteration, joint_locals, dt, loads):
    direction = (points[1] - points[0]) / np.linalg.norm(points[1] - points[0])
    first = _solver_end(world, path, index, True, joint.entity_a, joint.entity_b, points[0], direction, joint_locals, dt)
    second = _solver_end(world, path, index, False, joint.entity_b, joint.entity_a, points[1], -direction, joint_locals, dt)
    if first is None or second is None:
        return
    ends = first, second
    inverses = [end.spin.inv_inertia if end.spin else 0. for end in ends]
    valid_dt = finite_number(dt) and dt > EPSILON
    zero_stiffness = path.compliance == math.inf
    alpha = path.compliance / (dt * dt) if valid_dt and not zero_stiffness else 0.
    gamma = max(0., path.compliance * path.damping / dt) if valid_dt and not zero_stiffness else 0.
    displacement = first.displacement + second.displacement

    def solve_multiplier():
        mechanical = _mechanical_denominator(ends, inverses)
        if zero_stiffness:
            damping_step = max(0., path.damping) * dt if valid_dt else 0.
            return damping_step * displacement / (1 + damping_step * mechanical)
        denominator = (1 + gamma) * mechanical + alpha
        return (-error + gamma * displacement) / denominator if denominator > EPSILON else None

    multiplier = solve_multiplier()
    if multiplier is None:
        return
    inv_dt_squared = 1 / (dt * dt) if valid_dt else 0.
    released = False
    for i, end in enumerate(ends):
        limit = end.spin.holding_torque_limit if end.spin else math.inf
        if finite_number(limit) and limit >= 0 and inv_dt_squared > 0 and abs(end.solve_spin_gradient) > EPSILON:
            if abs(multiplier) * inv_dt_squared * abs(end.solve_spin_gradient) > limit + EPSILON:
                inverses[i] = end.spin.free_inv_inertia
                released = True
    if released:
        multiplier = solve_multiplier()
        if multiplier is None:
            return
    for end in ends:
        _record_load(path, end, multiplier, inv_dt_squared, loads)
    if _mechanical_denominator(ends, inverses) <= EPSILON:
        return
    if iteration == 0:
        joint.constraint_lambda = multiplier
        joint.constraint_force[:] = direction * (multiplier * inv_dt_squared)
        joint.constraint_force_magnitude = abs(multiplier) * inv_dt_squared
        magnitude = joint.constraint_force_magnitude
        if path.link_types[index] == 'pinhole' and index > 0:
            _record_transferred_force(world, path.joint_entities[index - 1], magnitude)
        if path.link_types[index + 1] == 'pinhole' and index + 1 < len(path.joint_entities):
            _record_transferred_force(world, path.joint_entities[index + 1], magnitude)
        for end in ends:
            _record_transferred_force(world, end.spin.transferred_joint if end.spin else None, magnitude)
    # Corrections are sequential; B's inertia may observe A's change on a shared body.
    for end, inverse in zip(ends, inverses):
        _apply_end_correction(world, end, inverse, multiplier)


class PBDCableConstraintSolver:
    def update(self, world, dt_unused):
        paths = world.query([CablePathComponent])
        dt = world.get_resource('dt')
        loads = ({}, {}, {})
        for key, value in zip(('torqueModeCableLoadTorques', 'torqueModeCableLoadStiffnesses', 'torqueModeCableLoadDampings'), loads):
            world.set_resource(key, value)
        joint_locals = {}
        for path_id in paths:
            path = world.get_component(path_id, CablePathComponent)
            for joint_id in path.joint_entities:
                joint = world.get_component(joint_id, CableJointComponent)
                joint.constraint_lambda = joint.constraint_force_magnitude = joint.transferred_constraint_force_magnitude = 0.
                joint.constraint_force[:] = 0.
                joint_locals[joint_id] = (
                    compute_local_attachment(world, joint.entity_a, joint.attachment_point_a_world),
                    compute_local_attachment(world, joint.entity_b, joint.attachment_point_b_world))
        iterations = max((world.get_component(p, CablePathComponent).solver_iterations for p in paths), default=1)
        for iteration in range(iterations):
            for path_id in paths if iteration % 2 == 0 else reversed(paths):
                path = world.get_component(path_id, CablePathComponent)
                if iteration >= path.solver_iterations:
                    continue
                indices = range(len(path.joint_entities)) if iteration % 2 == 0 else reversed(range(len(path.joint_entities)))
                for index in indices:
                    joint_id = path.joint_entities[index]
                    joint = world.get_component(joint_id, CableJointComponent)
                    points = (compute_world_attachment(world, joint.entity_a, joint_locals[joint_id][0]),
                              compute_world_attachment(world, joint.entity_b, joint_locals[joint_id][1]))
                    length = np.linalg.norm(points[1] - points[0])
                    if length <= EPSILON or length - joint.rest_length <= EPSILON:
                        continue
                    _solve_joint(world, path, index, joint, points, length - joint.rest_length, iteration, joint_locals, dt, loads)
