import numpy as np

from .cable_joints_components import CablePathComponent, CableJointComponent

def _slip_slack(world):
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if len(path.joint_entities) < 2:
            continue
        for i in range(len(path.joint_entities) - 1):
            if path.link_types[i + 1] == 'attachment':
                continue

            j0 = world.get_component(path.joint_entities[i], CableJointComponent)
            j1 = world.get_component(path.joint_entities[i + 1], CableJointComponent)

            d0 = np.linalg.norm(j0.attachment_point_a_world - j0.attachment_point_b_world)
            d1 = np.linalg.norm(j1.attachment_point_a_world - j1.attachment_point_b_world)

            l0 = j0.rest_length
            l1 = j1.rest_length

            slack0 = l0 - d0
            slack1 = l1 - d1

            if slack0 > 0 and slack1 < 0:
                slip = min(slack0, -slack1)
                j0.rest_length -= slip
                j1.rest_length += slip
            elif slack1 > 0 and slack0 < 0:
                slip = min(slack1, -slack0)
                j1.rest_length -= slip
                j0.rest_length += slip

class CableSlackSystem:
    """
    Handles the transfer of cable rest length between joints to simulate
    slack slipping from loose segments to taut ones. This system should run
    before the main PBD constraint solver.
    """
    def __init__(self):
        self.run_in_pause = False

    def update(self, world, dt):
        # Iterate a few times to allow slack to propagate along the entire cable path.
        for _ in range(4):
            _slip_slack(world)
