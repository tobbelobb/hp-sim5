"""Build cable paths, separating spans at intermediate fixed attachments."""
import warnings

from .cable_joints_components import create_cable_path_component


def create_cable_paths(world, joint_entities=None, link_types=None, cw=None,
                       spring_constant=1e6, user_stored=None, cable_half_width=0.):
    joints = joint_entities if joint_entities is not None else []
    links = link_types if link_types is not None else []
    clockwise = cw if cw is not None else []
    if len(links) != len(joints) + 1 or len(clockwise) != len(links) or (user_stored is not None and len(user_stored) != len(links)):
        warnings.warn('Cable path link, winding and stored lengths must match the joint count.', stacklevel=2)
        return []
    created = []
    start = 0
    # The attachment at each cut belongs to both neighboring paths.
    ends = [index for index in range(1, len(links) - 1) if links[index] == 'attachment']
    ends.append(len(joints))
    for end in ends:
        entity = world.create_entity()
        stored = user_stored[start:end + 1] if user_stored is not None else None
        component = create_cable_path_component(
            world, joints[start:end], links[start:end + 1], clockwise[start:end + 1],
            spring_constant, stored, cable_half_width,
        )
        world.add_component(entity, component)
        created.append(entity)
        start = end
    return created
