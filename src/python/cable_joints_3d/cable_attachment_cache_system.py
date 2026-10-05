"""Cache live world and member-local cable poses after attachment rebuilding."""
from .cable_joints_components import CableLinkComponent
from .ecs import PositionComponent, RigidBodyMemberComponent
from .rigid_bodies import get_entity_world_orientation, get_entity_world_position


class CableAttachmentCacheSystem:
    def update(self, world, dt):
        for entity in world.query([CableLinkComponent, PositionComponent]):
            position = get_entity_world_position(world, entity)
            orientation = get_entity_world_orientation(world, entity)
            member = world.get_component(entity, RigidBodyMemberComponent)
            link = world.get_component(entity, CableLinkComponent)
            if position is not None:
                link.prev_cable_attachment_time_pos[:] = position
            if orientation is not None:
                link.prev_cable_attachment_time_orientation.set(orientation)
            if member is not None:
                link.prev_cable_attachment_time_local_orientation.set(member.local_orientation)
            elif orientation is not None:
                link.prev_cable_attachment_time_local_orientation.set(orientation)
