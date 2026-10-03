"""Native 3D cable state; construction uses live rigid-member frames."""
from dataclasses import dataclass, field
import math
import warnings

import numpy as np

from .ecs import RadiusComponent, layering_enabled
from .geometry3 import signed_arc_length_on_wheel
from .quaternion import Quaternion
from .rigid_bodies import compute_world_attachment, get_entity_world_position


class CableLinkComponent:
    def __init__(self, x=0., y=0., z=0., orientation=None, plane_normal=None, plane_normal_local=None):
        self.prev_cable_attachment_time_pos = np.array([x, y, z], dtype=float)
        self.prev_cable_attachment_time_orientation = orientation.copy() if orientation is not None else Quaternion()
        self.prev_cable_attachment_time_local_orientation = orientation.copy() if orientation is not None else Quaternion()
        self.cable_plane_normal = np.asarray(plane_normal, dtype=float).copy() if plane_normal is not None else np.array([0., 0., 1.])
        self.cable_plane_normal_local = None
        if plane_normal_local is not None:
            axis = np.asarray(plane_normal_local, dtype=float).copy()
            norm = np.linalg.norm(axis)
            self.cable_plane_normal_local = axis / norm if norm > 0 else axis


@dataclass
class CableJointComponent:
    entity_a: int
    entity_b: int
    rest_length: float
    attachment_point_a_world: np.ndarray
    attachment_point_b_world: np.ndarray
    constraint_lambda: float = 0.
    constraint_force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    constraint_force_magnitude: float = 0.
    transferred_constraint_force_magnitude: float = 0.

    def __post_init__(self):
        self.attachment_point_a_world = np.asarray(self.attachment_point_a_world, dtype=float).copy()
        self.attachment_point_b_world = np.asarray(self.attachment_point_b_world, dtype=float).copy()

    @classmethod
    def from_world(cls, entity_a, entity_b, rest_length, point_a, point_b):
        return cls(entity_a, entity_b, rest_length, point_a, point_b)

    @classmethod
    def from_local(cls, world, entity_a, entity_b, rest_length, point_a, point_b):
        return cls(entity_a, entity_b, rest_length,
                   compute_world_attachment(world, entity_a, point_a),
                   compute_world_attachment(world, entity_b, point_b))


@dataclass
class CablePathComponent:
    joint_entities: list[int] = field(default_factory=list)
    link_types: list[str] = field(default_factory=list)
    cw: list[bool] = field(default_factory=list)
    spring_constant: float = 1e6
    stored: list[float] = field(default_factory=list)
    cable_half_width: float = 0.
    damping: float = 0.
    solver_iterations: int = 1
    total_rest_length: float = 0.
    compliance: float = field(init=False)

    def __post_init__(self):
        self.compliance = 1. / self.spring_constant if self.spring_constant != 0 else math.inf
        self.cable_half_width = max(0., self.cable_half_width) if np.isfinite(self.cable_half_width) else 0.
        self.damping = max(0., self.damping) if np.isfinite(self.damping) else 0.
        self.solver_iterations = max(1, math.floor(self.solver_iterations)) if np.isfinite(self.solver_iterations) else 1


def create_cable_path_component(world, joint_entities=None, link_types=None, cw=None,
                                spring_constant=1e6, stored=None, cable_half_width=0.,
                                damping=0., solver_iterations=1):
    # Keep the existing Python factory convention: data components do not own
    # a World. Import frame helpers here to avoid a component/helper cycle.
    from .cable_frames import get_plane_normal, ensure_hybrid_knot_angle_for_endpoint

    joints = joint_entities if joint_entities is not None else []
    links = link_types if link_types is not None else []
    clockwise = cw if cw is not None else []
    path = CablePathComponent(joints, links, clockwise, spring_constant,
                              [0.] * len(clockwise), cable_half_width, damping, solver_iterations)
    path.total_rest_length = sum(world.get_component(j, CableJointComponent).rest_length for j in joints)
    for index in range(len(joints) - 1):
        first = world.get_component(joints[index], CableJointComponent)
        second = world.get_component(joints[index + 1], CableJointComponent)
        if first.entity_b != second.entity_a:
            warnings.warn('Cable path joints do not share their intermediate link.', stacklevel=2)
            return path
        if links[index + 1] != 'rolling':
            continue
        center = get_entity_world_position(world, first.entity_b)
        radius = world.get_component(first.entity_b, RadiusComponent).radius
        radius += path.cable_half_width if layering_enabled(world) else 0.
        wrap = signed_arc_length_on_wheel(
            first.attachment_point_b_world, second.attachment_point_a_world,
            center, radius, clockwise[index + 1], get_plane_normal(world, first.entity_b), True,
        )
        path.stored[index + 1] = wrap
        path.total_rest_length += wrap
    if stored is not None:
        for index, length in enumerate(stored):
            if length is not None:
                path.total_rest_length += length - path.stored[index]
                path.stored[index] = length
    if links:
        ensure_hybrid_knot_angle_for_endpoint(world, path, 0)
        if len(links) > 1:
            ensure_hybrid_knot_angle_for_endpoint(world, path, len(links) - 1)
    return path
