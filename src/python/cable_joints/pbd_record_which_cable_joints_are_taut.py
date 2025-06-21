import numpy as np

from .ecs import World
from .cable_joints_components import (
    CableJointComponent,
    CablePathComponent,
    PBDCableSolverCache
)

class PBDRecordWhichCableJointsAreTaut:
    """
    Records which cable joints are taut before the main PBD cable constraint
    solver runs. This information is used by subsequent correction passes to
    identify over-correction artifacts.
    """
    run_in_pause = False

    def update(self, world: World, _dt_unused):
        cache = world.get_resource("pbd_cable_solver_cache")
        if not cache:
            cache = PBDCableSolverCache()
            world.set_resource("pbd_cable_solver_cache", cache)

        cache.was_taut.clear()

        path_entities = world.query(CablePathComponent)
        all_joint_ids = set()
        for path_id in path_entities:
            path = world.get_component(path_id, CablePathComponent)
            for joint_id in path.joint_entities:
                all_joint_ids.add(joint_id)

        for joint_id in all_joint_ids:
            joint = world.get_component(joint_id, CableJointComponent)
            if not joint:
                continue

            p_a = joint.attachment_point_a_world
            p_b = joint.attachment_point_b_world
            dist = np.linalg.norm(p_a - p_b)

            # A joint is taut if it's stretched beyond its rest length.
            cache.was_taut[joint_id] = dist > joint.rest_length
