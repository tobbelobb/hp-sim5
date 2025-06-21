import numpy as np
from collections import defaultdict

from .ecs import World
from .cable_joints_components import (
    CableJointComponent,
    CablePathComponent,
)
from .ecs import (
    PositionComponent,
    MassComponent,
    MomentOfInertiaComponent,
    OrientationComponent,
)
from .update_attachment_points import calculate_attachment_points


class PBDResolveCableOverCorrections:
    """
    This system runs after the main PBD cable solver to correct for
    over-contraction artifacts. When multiple taut cables pull on the same
    object, the main solver can cause joints that should remain taut to become
    slack. This pass identifies such joints and pushes them back towards
    their rest length.

    It uses a Jacobi-style approach, calculating all corrections based on the
    state after the main solver, and then applying an averaged correction for
    each body. This prevents the instability of a sequential solve and better
    handles bodies affected by multiple over-corrected joints.
    """
    run_in_pause = False

    def update(self, world: World, _dt_unused):
        path_entities = world.query(CablePathComponent)
        if not path_entities:
            return

        joint_to_path_and_index = {}
        all_joint_ids = set()
        for path_id in path_entities:
            path = world.get_component(path_id, CablePathComponent)
            for i, joint_id in enumerate(path.joint_entities):
                all_joint_ids.add(joint_id)
                joint_to_path_and_index[joint_id] = (path, i)

        # 1. Find all joints that were taut but are now slack
        over_corrected_joints = []
        for joint_id in all_joint_ids:
            joint = world.get_component(joint_id, CableJointComponent)
            p_a = joint.attachment_point_a_world
            p_b = joint.attachment_point_b_world
            pre_cable_solve_length = np.linalg.norm(p_a - p_b)
            if pre_cable_solve_length >= joint.rest_length: # If was stretched
                path, i = joint_to_path_and_index[joint_id]
                p_a, p_b = calculate_attachment_points(world, joint, path, i)
                post_cable_solve_length = np.linalg.norm(p_a - p_b)
                if post_cable_solve_length < joint.rest_length: # If now slack
                    over_corrected_joints.append(joint_id)

        if not over_corrected_joints:
            return

        # Jacobi-style solve: calculate all corrections first, then apply averaged corrections.
        position_corrections = defaultdict(list)
        angle_corrections = defaultdict(list)

        # 2. Calculate corrections for each over-corrected joint without applying them
        for joint_id in over_corrected_joints:
            self.calculate_joint_correction(
                world, joint_id, joint_to_path_and_index, position_corrections, angle_corrections
            )

        # 3. Apply the averaged corrections to each affected entity
        for entity_id, pos_deltas in position_corrections.items():
            if pos_deltas:
                avg_delta = np.mean(pos_deltas, axis=0)
                pos_comp = world.get_component(entity_id, PositionComponent)
                if pos_comp:
                    pos_comp.pos += avg_delta

        for entity_id, ang_deltas in angle_corrections.items():
            if ang_deltas:
                avg_delta = np.mean(ang_deltas)
                orientation_comp = world.get_component(entity_id, OrientationComponent)
                if orientation_comp:
                    orientation_comp.angle += avg_delta

    def calculate_joint_correction(self, world, joint_id, joint_to_path_and_index, position_corrections, angle_corrections):
        """
        Calculates the PBD correction for a single joint that has become slack
        and stores the delta values in the provided dictionaries instead of
        applying them directly to the components.
        """
        joint = world.get_component(joint_id, CableJointComponent)
        path_data = joint_to_path_and_index.get(joint_id)
        if not path_data:
            return
        path, i = path_data

        p_a, p_b = calculate_attachment_points(world, joint, path, i)
        if p_a is None or p_b is None:
            return

        current_segment_length = np.linalg.norm(p_a - p_b)
        constraint_error = current_segment_length - joint.rest_length

        epsilon = 1e-9
        # This function should only be called for slack joints, where error is negative.
        if constraint_error >= -epsilon:
            return

        entity_a = joint.entity_a
        entity_b = joint.entity_b

        mass_a_comp = world.get_component(entity_a, MassComponent)
        inv_mass_a = 1.0 / mass_a_comp.mass if mass_a_comp and mass_a_comp.mass > 0 and np.isfinite(mass_a_comp.mass) else 0.0
        moi_a_comp = world.get_component(entity_a, MomentOfInertiaComponent)
        inv_inertia_a = moi_a_comp.inv_inertia if moi_a_comp else 0.0

        mass_b_comp = world.get_component(entity_b, MassComponent)
        inv_mass_b = 1.0 / mass_b_comp.mass if mass_b_comp and mass_b_comp.mass > 0 and np.isfinite(mass_b_comp.mass) else 0.0
        moi_b_comp = world.get_component(entity_b, MomentOfInertiaComponent)
        inv_inertia_b = moi_b_comp.inv_inertia if moi_b_comp else 0.0

        if inv_mass_a + inv_mass_b + inv_inertia_a + inv_inertia_b <= epsilon:
            return

        diff = p_b - p_a
        length = np.linalg.norm(diff)
        if length <= epsilon:
            return
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
        denom += inv_mass_a * np.dot(grad_pos_a, grad_pos_a)
        denom += inv_inertia_a * grad_ang_a * grad_ang_a
        denom += inv_mass_b * np.dot(grad_pos_b, grad_pos_b)
        denom += inv_inertia_b * grad_ang_b * grad_ang_b

        dt = world.get_resource('dt')
        if dt is not None and dt > 0:
            denom += path.compliance / (dt * dt)

        if denom <= epsilon:
            return

        lambda_ = -constraint_error / denom

        # Store corrections instead of applying them directly
        if inv_mass_a > 0.0:
            delta_pos_a = grad_pos_a * (inv_mass_a * lambda_)
            position_corrections[entity_a].append(delta_pos_a)

        if inv_inertia_a > 0.0:
            delta_ang_a = -inv_inertia_a * lambda_ * grad_ang_a
            angle_corrections[entity_a].append(delta_ang_a)

        if inv_mass_b > 0.0:
            delta_pos_b = grad_pos_b * (inv_mass_b * lambda_)
            position_corrections[entity_b].append(delta_pos_b)

        if inv_inertia_b > 0.0:
            delta_ang_b = -inv_inertia_b * lambda_ * grad_ang_b
            angle_corrections[entity_b].append(delta_ang_b)
