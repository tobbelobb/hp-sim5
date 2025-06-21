import numpy as np

from cable_joints.cable_joints_components import (
    CablePathComponent,
    CableJointComponent,
)
from cable_joints.ecs import (
    PositionComponent,
    MassComponent,
    MomentOfInertiaComponent,
    OrientationComponent,
)


class PBDCableConstraintSolver:
    """
    This system solves distance constraints for cable segments using Position
    Based Dynamics (PBD). It iterates through each cable joint, calculates the
    constraint error, and applies positional and rotational corrections to the
    connected entities to enforce the cable's rest length.
    """
    run_in_pause = False

    def update(self, world, _dt_unused):
        path_entities = world.query(CablePathComponent)
        epsilon = 1e-9  # Small value to avoid division by zero
        dt = world.get_resource('dt')

        for path_id in path_entities:
            path = world.get_component(path_id, CablePathComponent)
            if not path.joint_entities:
                continue

            for joint_id in path.joint_entities:
                joint = world.get_component(joint_id, CableJointComponent)

                entity_a = joint.entity_a
                entity_b = joint.entity_b

                p_a = joint.attachment_point_a_world
                p_b = joint.attachment_point_b_world

                current_segment_length = np.linalg.norm(p_a - p_b)
                constraint_error = current_segment_length - joint.rest_length

                # Apply correction only if the segment is longer than its rest length
                if constraint_error <= epsilon:
                    continue

                mass_a_comp = world.get_component(entity_a, MassComponent)
                inv_mass_a = 1.0 / mass_a_comp.mass if mass_a_comp and mass_a_comp.mass > 0 and np.isfinite(mass_a_comp.mass) else 0.0
                moi_a_comp = world.get_component(entity_a, MomentOfInertiaComponent)
                inv_inertia_a = moi_a_comp.inv_inertia if moi_a_comp else 0.0

                mass_b_comp = world.get_component(entity_b, MassComponent)
                inv_mass_b = 1.0 / mass_b_comp.mass if mass_b_comp and mass_b_comp.mass > 0 and np.isfinite(mass_b_comp.mass) else 0.0
                moi_b_comp = world.get_component(entity_b, MomentOfInertiaComponent)
                inv_inertia_b = moi_b_comp.inv_inertia if moi_b_comp else 0.0

                if inv_mass_a + inv_mass_b + inv_inertia_a + inv_inertia_b <= epsilon:
                    continue

                diff = p_b - p_a
                length = np.linalg.norm(diff)
                if length <= epsilon:
                    continue
                direction = diff / length

                grad_pos_a = -direction
                grad_pos_b = direction

                pos_a_comp = world.get_component(entity_a, PositionComponent)
                r_a = p_a - pos_a_comp.pos

                r_a_3d = np.array([r_a[0], r_a[1], 0.0])
                direction_3d = np.array([direction[0], direction[1], 0.0])
                grad_ang_a = np.cross(r_a_3d, direction_3d)[2]

                pos_b_comp = world.get_component(entity_b, PositionComponent)
                r_b = p_b - pos_b_comp.pos

                r_b_3d = np.array([r_b[0], r_b[1], 0.0])
                neg_direction_3d = np.array([-direction[0], -direction[1], 0.0])
                grad_ang_b = np.cross(r_b_3d, neg_direction_3d)[2]

                denom = 0.0
                denom += inv_mass_a # * np.dot(grad_pos_a, grad_pos_a) # The dot product of two unit vectors is 1
                denom += inv_inertia_a * grad_ang_a * grad_ang_a
                denom += inv_mass_b # * np.dot(grad_pos_b, grad_pos_b) # The dot product of two unit vectors is 1
                denom += inv_inertia_b * grad_ang_b * grad_ang_b

                if dt is not None and dt > 0:
                    denom += path.compliance / (dt * dt)

                if denom <= epsilon:
                    continue

                lambda_ = -constraint_error / denom

                # Apply corrections to Entity A
                if inv_mass_a > 0.0:
                    delta_pos_a = grad_pos_a * (inv_mass_a * lambda_)
                    pos_a_comp.pos += delta_pos_a

                if inv_inertia_a > 0.0:
                    delta_ang_a = -inv_inertia_a * lambda_ * grad_ang_a
                    orientation_a_comp = world.get_component(entity_a, OrientationComponent)
                    if orientation_a_comp:
                        orientation_a_comp.angle += delta_ang_a

                # Apply corrections to Entity B
                if inv_mass_b > 0.0:
                    delta_pos_b = grad_pos_b * (inv_mass_b * lambda_)
                    pos_b_comp.pos += delta_pos_b

                if inv_inertia_b > 0.0:
                    delta_ang_b = -inv_inertia_b * lambda_ * grad_ang_b
                    orientation_b_comp = world.get_component(entity_b, OrientationComponent)
                    if orientation_b_comp:
                        orientation_b_comp.angle += delta_ang_b
