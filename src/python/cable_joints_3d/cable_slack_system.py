"""Optional single ordered passes for pinhole tension and loose-span transfer."""
import numpy as np

from .cable_joints_components import CableJointComponent, CablePathComponent


def _adjacent_joints(world):
    for entity in world.query([CablePathComponent]):
        path = world.get_component(entity, CablePathComponent)
        for index in range(len(path.joint_entities) - 1):
            yield path.link_types[index + 1], *(world.get_component(joint, CableJointComponent)
                for joint in path.joint_entities[index:index + 2])


class CableSlackSystem:
    def update(self, world, dt):
        for link_type, first, second in _adjacent_joints(world):
            if link_type != 'pinhole':
                continue
            distance_a = np.linalg.norm(first.attachment_point_a_world - first.attachment_point_b_world)
            distance_b = np.linalg.norm(second.attachment_point_a_world - second.attachment_point_b_world)
            available, distance = first.rest_length + second.rest_length, distance_a + distance_b
            if available <= 1e-9 or distance <= 1e-9:
                continue
            first.rest_length = available * distance_a / distance
            second.rest_length = available - first.rest_length


class SlideLooseCableSystem:
    def update(self, world, dt):
        for link_type, first, second in _adjacent_joints(world):
            if link_type == 'attachment':
                continue
            slack_a = first.rest_length - np.linalg.norm(first.attachment_point_a_world - first.attachment_point_b_world)
            slack_b = second.rest_length - np.linalg.norm(second.attachment_point_a_world - second.attachment_point_b_world)
            if slack_a > 0 and slack_b < 0:
                slip = min(slack_a, -slack_b)
                first.rest_length -= slip
                second.rest_length += slip
            elif slack_b > 0 and slack_a < 0:
                slip = min(slack_b, -slack_a)
                second.rest_length -= slip
                first.rest_length += slip
