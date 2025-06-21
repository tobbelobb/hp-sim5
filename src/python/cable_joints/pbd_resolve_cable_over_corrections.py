import numpy as np

from .ecs import World
from .cable_joints_components import (
    CableJointComponent,
    CablePathComponent,
    PBDCableSolverCache,
)
from .ecs import (
    PositionComponent,
    MassComponent,
    MomentOfInertiaComponent,
    OrientationComponent,
)

class PBDResolveCableOverCorrections:
    """
    This system runs after the main PBD cable solver to correct for
    over-contraction artifacts. When multiple taut cables pull on the same
    object, the solver can cause joints that should remain taut to become
    slack. This pass identifies such joints and pushes them back towards
    their rest length.
    """
    run_in_pause = False

    def __init__(self, iterations=5):
        self.iterations = iterations

    def update(self, world: World, _dt_unused):
        cache = world.get_resource("pbd_cable_solver_cache")
        if not cache or not cache.was_taut:
            return

        path_entities = world.query(CablePathComponent)
        if not path_entities:
            return

        joint_to_path = {}
        all_joint_ids = set()
        for path_id in path_entities:
            path = world.get_component(path_id, CablePathComponent)
            for joint_id in path.joint_entities:
                all_joint_ids.add(joint_id)
                joint_to_path[joint_id] = path

        # Find all joints that were taut but are now slack
        over_corrected_joints = []
        for joint_id in all_joint_ids:
            if cache.was_taut.get(joint_id, False):
                joint = world.get_component(joint_id, CableJointComponent)
                p_a = joint.attachment_point_a_world
                p_b = joint.attachment_point_b_world
                current_length = np.linalg.norm(p_a - p_b)
                if current_length < joint.rest_length:
                    over_corrected_joints.append(joint_id)

        if not over_corrected_joints:
            return

        # Iteratively resolve these over-corrections
        for _ in range(self.iterations):
            for joint_id in over_corrected_joints:
                self.solve_joint(world, joint_id, joint_to_path)

    def solve_joint(self, world, joint_id, joint_to_path):
        joint = world.get_component(joint_id, CableJointComponent)
        path = joint_to_path.get(joint_id)
        if not path:
            return

        p_a = joint.attachment_point_a_world
        p_b = joint.attachment_point_b_world

        current_segment_length = np.linalg.norm(p_a - p_b)
        constraint_error = current_segment_length - joint.rest_length

        epsilon = 1e-9
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
