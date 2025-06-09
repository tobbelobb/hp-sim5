import numpy as np

from .ecs import (
    MassComponent, MomentOfInertiaComponent, PositionComponent,
    VelocityComponent, OrientationComponent, AngularVelocityComponent
)
from .cable_joints_core import CablePathComponent, CableJointComponent

class PBDCableConstraintSolver:
    """
    A Python port of the PBDCableConstraintSolver from JavaScript.

    This system solves distance constraints for cable segments using Position
    Based Dynamics (PBD). It iterates through each cable joint, calculates the
    constraint error, and applies positional and rotational corrections to the
    connected entities to enforce the cable's rest length.
    """
    run_in_pause = False

    def update(self, world, _dt_unused):
        """
        Updates the cable constraints for one PBD iteration.
        """
        # Assuming world.query can take a single component type
        path_entities = world.query(CablePathComponent)
        epsilon = 1e-9  # Small value to avoid division by zero
        dt = world.get_resource('dt')

        for path_id in path_entities:
            path = world.get_component(path_id, CablePathComponent)
            if not path or len(path.joint_entities) < 1:
                continue

            for joint_id in path.joint_entities:
                joint = world.get_component(joint_id, CableJointComponent)
                if not joint:
                    continue

                entity_a = joint.entity_a
                entity_b = joint.entity_b

                p_a = joint.attachment_point_a_world
                p_b = joint.attachment_point_b_world

                current_segment_length = np.linalg.norm(p_a - p_b)
                constraint_error = current_segment_length - joint.rest_length

                # Apply correction only if the segment is over-extended
                if constraint_error > epsilon:
                    mass_a_comp = world.get_component(entity_a, MassComponent)
                    inv_mass_a = 1.0 / mass_a_comp.mass if mass_a_comp and mass_a_comp.mass > 0 else 0.0
                    moi_a_comp = world.get_component(entity_a, MomentOfInertiaComponent)
                    inv_inertia_a = moi_a_comp.inv_inertia if moi_a_comp else 0.0

                    mass_b_comp = world.get_component(entity_b, MassComponent)
                    inv_mass_b = 1.0 / mass_b_comp.mass if mass_b_comp and mass_b_comp.mass > 0 else 0.0
                    moi_b_comp = world.get_component(entity_b, MomentOfInertiaComponent)
                    inv_inertia_b = moi_b_comp.inv_inertia if moi_b_comp else 0.0

                    # If both entities are effectively immovable, skip
                    if inv_mass_a + inv_mass_b + inv_inertia_a + inv_inertia_b <= epsilon:
                        continue

                    diff = p_b - p_a
                    length = np.linalg.norm(diff)
                    if length <= epsilon:
                        continue
                    direction = diff / length  # Normalized direction from A to B

                    # Gradients are defined based on the JS implementation
                    grad_pos_a = direction.copy()
                    grad_pos_b = -direction.copy()

                    pos_a_comp = world.get_component(entity_a, PositionComponent)
                    r_a = p_a - pos_a_comp.pos  # Vector from CoM of A to attachment point A
                    grad_ang_a = np.cross(r_a, direction) # 2D cross product

                    pos_b_comp = world.get_component(entity_b, PositionComponent)
                    r_b = p_b - pos_b_comp.pos  # Vector from CoM of B to attachment point B
                    grad_ang_b = np.cross(r_b, -direction) # 2D cross product

                    denom = (inv_mass_a * np.dot(grad_pos_a, grad_pos_a) +
                             inv_inertia_a * grad_ang_a**2 +
                             inv_mass_b * np.dot(grad_pos_b, grad_pos_b) +
                             inv_inertia_b * grad_ang_b**2 +
                             path.compliance / (dt**2))

                    if denom <= epsilon:
                        continue

                    lambda_ = -constraint_error / denom

                    # Apply corrections to Entity A
                    if inv_mass_a > 0.0:
                        delta_pos_a = grad_pos_a * (-inv_mass_a * lambda_)
                        pos_a_comp.pos += delta_pos_a
                        vel_a_comp = world.get_component(entity_a, VelocityComponent)
                        if vel_a_comp and dt > epsilon:
                            vel_a_comp.vel += delta_pos_a / dt
                            v_a = np.linalg.norm(vel_a_comp.vel)
                            max_speed = 0.03 / (2.0 * dt)
                            if v_a > max_speed:
                                vel_a_comp.vel *= (max_speed / v_a)

                    if inv_inertia_a > 0.0:
                        delta_ang_a = -inv_inertia_a * lambda_ * grad_ang_a
                        orientation_a_comp = world.get_component(entity_a, OrientationComponent)
                        if orientation_a_comp:
                            orientation_a_comp.angle += delta_ang_a
                        ang_vel_a_comp = world.get_component(entity_a, AngularVelocityComponent)
                        if ang_vel_a_comp and dt > epsilon:
                            ang_vel_a_comp.angular_velocity += delta_ang_a / dt

                    # Apply corrections to Entity B
                    if inv_mass_b > 0.0:
                        delta_pos_b = grad_pos_b * (-inv_mass_b * lambda_)
                        pos_b_comp.pos += delta_pos_b
                        vel_b_comp = world.get_component(entity_b, VelocityComponent)
                        if vel_b_comp and dt > epsilon:
                            vel_b_comp.vel += delta_pos_b / dt
                            v_b = np.linalg.norm(vel_b_comp.vel)
                            max_speed = 0.03 / (2.0 * dt)
                            if v_b > max_speed:
                                vel_b_comp.vel *= (max_speed / v_b)

                    if inv_inertia_b > 0.0:
                        delta_ang_b = -inv_inertia_b * lambda_ * grad_ang_b
                        orientation_b_comp = world.get_component(entity_b, OrientationComponent)
                        if orientation_b_comp:
                            orientation_b_comp.angle += delta_ang_b
                        ang_vel_b_comp = world.get_component(entity_b, AngularVelocityComponent)
                        if ang_vel_b_comp and dt > epsilon:
                            ang_vel_b_comp.angular_velocity += delta_ang_b / dt
