import warnings
from typing import List, Optional, cast

from .ecs import World
from .cable_joints_components import create_cable_path_component

def create_cable_paths(
    world: World,
    joint_entities: Optional[List[int]] = None,
    link_types: Optional[List[str]] = None,
    cw: Optional[List[bool]] = None,
    spring_constant: float = 1e6,
    user_stored: Optional[List[Optional[float]]] = None,
    cable_half_width: float = 0.0
) -> List[int]:
    """
    Creates one or more CablePathComponent entities from a definition of joints and links.

    This function takes a series of joints and link properties and constructs
    CablePathComponent entities. It splits the series into multiple paths if it
    encounters an 'attachment' link type that is not at the very end of the path.
    This is a Python port of the createCablePaths.js function.

    Args:
        world: The ECS world object.
        joint_entities: A list of entity IDs for the joints in the path.
        link_types: A list of strings describing the type of each link.
                    len(link_types) must be len(joint_entities) + 1.
        cw: A list of booleans indicating clockwise preference for rolling links.
            len(cw) must be len(link_types).
        spring_constant: The spring constant for the cable paths.
        user_stored: Optional list of pre-calculated stored lengths for links.
                     If provided, len(user_stored) must be len(link_types).

    Returns:
        A list of entity IDs for the created CablePathComponent entities.
    """
    if joint_entities is None:
        joint_entities = []
    if link_types is None:
        link_types = []
    if cw is None:
        cw = []

    created_path_entity_ids: List[int] = []

    # Validate input array lengths
    if len(link_types) != len(joint_entities) + 1:
        warnings.warn("create_cable_paths: len(link_types) must be len(joint_entities) + 1. Aborting.", UserWarning)
        return created_path_entity_ids
    if len(cw) != len(link_types):
        warnings.warn("create_cable_paths: len(cw) must be len(link_types). Aborting.", UserWarning)
        return created_path_entity_ids
    if user_stored is not None and len(user_stored) != len(link_types):
        warnings.warn("create_cable_paths: len(user_stored) must be len(link_types) if provided. Aborting.", UserWarning)
        return created_path_entity_ids

    if not joint_entities:
        # This means link_types has length 1. A single link, no joints.
        path_entity_id = world.create_entity()
        path_component = create_cable_path_component(
            world,
            [],
            link_types,
            cw,
            spring_constant,
            user_stored,
            cable_half_width
        )
        world.add_component(path_entity_id, path_component)
        created_path_entity_ids.append(path_entity_id)
        return created_path_entity_ids

    current_path_joints: List[int] = []
    current_path_link_types: List[str] = []
    current_path_cw: List[bool] = []
    current_path_stored: Optional[List[Optional[float]]] = [] if user_stored is not None else None

    # The first link type always starts the first path segment
    current_path_link_types.append(link_types[0])
    current_path_cw.append(cw[0])
    if user_stored is not None:
        cast(list, current_path_stored).append(user_stored[0])

    for i, joint_id in enumerate(joint_entities):
        link_type_after_joint = link_types[i + 1]
        cw_after_joint = cw[i + 1]
        stored_after_joint = user_stored[i + 1] if user_stored is not None else None

        current_path_joints.append(joint_id)
        current_path_link_types.append(link_type_after_joint)
        current_path_cw.append(cw_after_joint)
        if user_stored is not None:
            cast(list, current_path_stored).append(stored_after_joint)

        # Check if the link *after* the current joint is an 'attachment'
        # and if it's not the very last link in the overall path definition.
        if link_type_after_joint == 'attachment' and (i + 1) < (len(link_types) - 1):
            # Finalize current path
            path_entity_id = world.create_entity()
            path_component = create_cable_path_component(
                world,
                current_path_joints,
                current_path_link_types,
                current_path_cw,
                spring_constant,
                current_path_stored,
                cable_half_width
            )
            world.add_component(path_entity_id, path_component)
            created_path_entity_ids.append(path_entity_id)

            # Start a new path
            current_path_joints = []
            # The 'attachment' link that caused the split starts the new path segment.
            current_path_link_types = [link_type_after_joint]
            current_path_cw = [cw_after_joint]
            if user_stored is not None:
                current_path_stored = [stored_after_joint]

    # Add the last (or only remaining) path segment
    if current_path_joints or (not created_path_entity_ids and link_types):
        path_entity_id = world.create_entity()
        path_component = create_cable_path_component(
            world,
            current_path_joints,
            current_path_link_types,
            current_path_cw,
            spring_constant,
            current_path_stored,
            cable_half_width
        )
        world.add_component(path_entity_id, path_component)
        created_path_entity_ids.append(path_entity_id)

    return created_path_entity_ids
