import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict

from .ecs import PositionComponent, RadiusComponent, OrientationComponent
from .geometry import signed_arc_length_on_wheel

@dataclass
class CableLinkComponent:
    """
    Indicates an entity can be part of a cable.
    Corresponds to CableLinkComponent in JavaScript.
    """
    # These fields are from the JS version, for tracking previous state.
    prev_cable_attachment_time_pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    prev_cable_attachment_time_angle: float = 0.0

def _as_vec3(value) -> np.ndarray:
    arr = np.zeros(3, dtype=float)
    src = np.array(value, dtype=float).reshape(-1)
    limit = min(src.size, 3)
    arr[:limit] = src[:limit]
    return arr


def _compute_world_attachment(world, entity_id, local_point: np.ndarray) -> np.ndarray:
    if local_point is None:
        return None

    local_vec = _as_vec3(local_point)
    pos_comp = world.get_component(entity_id, PositionComponent) if world else None
    if pos_comp is None or pos_comp.pos is None:
        return local_vec

    angle = 0.0
    orientation_comp = world.get_component(entity_id, OrientationComponent) if world else None
    if orientation_comp is not None:
        angle = float(orientation_comp.angle)

    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    rotated = np.array([
        local_vec[0] * cos_a - local_vec[1] * sin_a,
        local_vec[0] * sin_a + local_vec[1] * cos_a,
        local_vec[2],
    ], dtype=float)

    return pos_comp.pos + rotated


@dataclass
class CableJointComponent:
    """
    Represents a single segment constraint between two entities.
    Corresponds to CableJointComponent in JavaScript.
    """
    entity_a: int
    entity_b: int
    rest_length: float
    attachment_point_a_world: np.ndarray
    attachment_point_b_world: np.ndarray

    def __post_init__(self):
        self.attachment_point_a_world = _as_vec3(self.attachment_point_a_world)
        self.attachment_point_b_world = _as_vec3(self.attachment_point_b_world)

    @classmethod
    def from_world(
        cls,
        entity_a: int,
        entity_b: int,
        rest_length: float,
        attachment_point_a_world,
        attachment_point_b_world,
    ) -> "CableJointComponent":
        return cls(
            entity_a=entity_a,
            entity_b=entity_b,
            rest_length=rest_length,
            attachment_point_a_world=attachment_point_a_world,
            attachment_point_b_world=attachment_point_b_world,
        )

    @classmethod
    def from_local(
        cls,
        world,
        entity_a: int,
        entity_b: int,
        rest_length: float,
        attachment_point_a_local,
        attachment_point_b_local,
    ) -> "CableJointComponent":
        local_a = _as_vec3(attachment_point_a_local)
        local_b = _as_vec3(attachment_point_b_local)
        world_a = _compute_world_attachment(world, entity_a, local_a)
        world_b = _compute_world_attachment(world, entity_b, local_b)
        return cls(
            entity_a=entity_a,
            entity_b=entity_b,
            rest_length=rest_length,
            attachment_point_a_world=world_a,
            attachment_point_b_world=world_b,
        )

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
    cable_half_width: float = 0.0
    compliance: float = field(init=False)

    def __post_init__(self):
        self.compliance = 1.0 / self.spring_constant

def create_cable_path_component(
    world,
    joint_entities,
    link_types,
    cw,
    spring_constant=1e6,
    stored=None,
    cable_half_width=0.0
):
    """
    Factory function to create and initialize a CablePathComponent.
    This ports the logic from the original JavaScript constructor.
    """
    path_comp = CablePathComponent(
        joint_entities=joint_entities,
        link_types=link_types,
        cw=cw,
        spring_constant=spring_constant,
        cable_half_width=max(0.0, float(cable_half_width)),
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
                radius = radius_comp.radius + path_comp.cable_half_width
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
