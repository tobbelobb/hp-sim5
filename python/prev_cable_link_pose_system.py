from .ecs import (
    PositionComponent, OrientationComponent, CableLinkComponent
)

def _store_cable_link_poses(world):
    link_entities = world.query([CableLinkComponent, PositionComponent])
    for link_id in link_entities:
        pos_comp = world.get_component(link_id, PositionComponent)
        orientation_comp = world.get_component(link_id, OrientationComponent)
        link_comp = world.get_component(link_id, CableLinkComponent)
        link_comp.prev_cable_attachment_time_pos = pos_comp.pos.copy()
        if orientation_comp:
            link_comp.prev_cable_attachment_time_angle = orientation_comp.angle

class PrevCableLinkPoseSystem:
    """
    Caches the position and orientation of entities with a CableLinkComponent
    at the beginning of a substep. This is analogous to PrevFinalPosSystem
    and is used by the CableAttachmentUpdateSystem to calculate how much
    a cable has wound or unwound from a rotating body.
    """
    def __init__(self):
        self.run_in_pause = False

    def update(self, world, dt):
        _store_cable_link_poses(world)
