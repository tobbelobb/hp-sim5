import numpy as np
from dataclasses import dataclass, field
from typing import List

from .ecs import PositionComponent, RadiusComponent
from .geometry import signed_arc_length_on_wheel

@dataclass
class CableJointComponent:
    """
    Represents a single segment constraint between two entities.
    Corresponds to CableJointComponent in JavaScript.
    """
    entity_a: int
    entity_b: int
    rest_length: float
    # These are numpy arrays. The caller is responsible for passing copies
    # if the original arrays should not be modified.
    attachment_point_a_world: np.ndarray
    attachment_point_b_world: np.ndarray

@dataclass
class CablePathComponent:
    """
    Connects individual cable joints into a cable path.
    Corresponds to CablePathComponent in JavaScript.

    NOTE: The original JavaScript constructor contained complex logic that
    depended on the `world` object to calculate initial `total_rest_length`
    and `stored` values. In a Python ECS, this logic is better placed in a
    factory function or a system that runs once upon creation, rather than
    in the component's `__init__`. This class is defined as a pure data
    container. The user is responsible for porting and running the
    initialization logic from the original JS constructor.
    """
    joint_entities: List[int] = field(default_factory=list)
    link_types: List[str] = field(default_factory=list)
    cw: List[bool] = field(default_factory=list)
    spring_constant: float = 1e6
    stored: List[float] = field(default_factory=list)
    total_rest_length: float = 0.0
    compliance: float = field(init=False)

    def __post_init__(self):
        self.compliance = 1.0 / self.spring_constant

def create_cable_path_component(world, joint_entities, link_types, cw, spring_constant=1e6, stored=None):
    """
    Factory function to create and initialize a CablePathComponent.
    This ports the logic from the original JavaScript constructor.
    """
    path_comp = CablePathComponent(
        joint_entities=joint_entities,
        link_types=link_types,
        cw=cw,
        spring_constant=spring_constant,
        stored=[0.0] * len(cw)
    )

    total_rest_length = 0.0
    for joint_id in joint_entities:
        joint = world.get_component(joint_id, CableJointComponent)
        total_rest_length += joint.rest_length

    for i in range(len(joint_entities) - 1):
        joint_i = world.get_component(joint_entities[i], CableJointComponent)
        joint_i_plus_1 = world.get_component(joint_entities[i+1], CableJointComponent)
        link_id = joint_i.entity_b
        
        link_id2 = joint_i_plus_1.entity_a
        if link_id != link_id2:
            print("Warning: CablePathComponent constructor: Links don't match up.")

        if link_types[i + 1] == 'rolling':
            center_comp = world.get_component(link_id, PositionComponent)
            radius_comp = world.get_component(link_id, RadiusComponent)
            if center_comp and radius_comp:
                center = center_comp.pos
                radius = radius_comp.radius
                is_cw = cw[i + 1]
                
                initial_stored_length = signed_arc_length_on_wheel(
                    joint_i.attachment_point_b_world,
                    joint_i_plus_1.attachment_point_a_world,
                    center, radius, is_cw, True
                )
                path_comp.stored[i + 1] = initial_stored_length
                total_rest_length += initial_stored_length

    path_comp.total_rest_length = total_rest_length

    if stored is not None:
        for i in range(len(path_comp.stored)):
            if i < len(stored) and stored[i] is not None:
                path_comp.total_rest_length -= path_comp.stored[i]
                path_comp.total_rest_length += stored[i]
                path_comp.stored[i] = stored[i]

    return path_comp
