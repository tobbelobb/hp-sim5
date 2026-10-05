"""Authored effector frames and headless extrusion state."""
from dataclasses import dataclass, field
import itertools
import math

import numpy as np

from .ecs import MachineTagComponent, PositionComponent
from .machine_runtime import object_keys
from .quaternion import Quaternion
from .rigid_bodies import get_entity_world_position
from .spools import SpoolTagComponent


def _finite_vector(value):
    return isinstance(value, np.ndarray) and value.shape == (3,) and np.isfinite(value).all()


def _frame(a, b, c):
    x = b - a
    if np.dot(x, x) <= 1e-12:
        return None
    x = x / np.linalg.norm(x)
    z = np.cross(x, c - a)
    if np.dot(z, z) <= 1e-12:
        return None
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)
    y = y / np.linalg.norm(y)
    return np.column_stack((x, y, z))


def _quaternion_from_rotation_matrix(m):
    trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return Quaternion((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, .25 * s).normalize()
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return Quaternion(.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s).normalize()
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return Quaternion((m[0, 1] + m[1, 0]) / s, .25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s).normalize()
    s = math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return Quaternion((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, .25 * s, (m[1, 0] - m[0, 1]) / s).normalize()


def estimate_effector_rotation(reference_offsets, center, entities, world):
    if not isinstance(reference_offsets, list) or len(reference_offsets) < 3 or not _finite_vector(center):
        return Quaternion()
    current = [get_entity_world_position(world, entity) for entity in entities]
    if len(current) != len(reference_offsets) or any(not _finite_vector(point) for point in current):
        return Quaternion()
    current = [point - center for point in current]
    # The reference uses the first nondegenerate authored triangle, not a fit
    # across all points. Preserve that order for deforming or redundant sources.
    for i, j, k in itertools.combinations(range(len(reference_offsets)), 3):
        a, b, c = [reference_offsets[index] for index in (i, j, k)]
        if not all(_finite_vector(point) for point in (a, b, c)):
            continue
        edge = b - a
        cross = np.cross(edge, c - a)
        if np.dot(edge, edge) <= 1e-12 or np.dot(cross, cross) <= 1e-12:
            continue
        rest_frame = _frame(a, b, c)
        current_frame = _frame(current[i], current[j], current[k])
        if rest_frame is None or current_frame is None:
            return Quaternion()
        return _quaternion_from_rotation_matrix(current_frame @ rest_frame.T)
    return Quaternion()


@dataclass
class ExtruderComponent:
    extrusions: list = field(default_factory=list)
    effector_center_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    center_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tip_pos: np.ndarray | None = None
    cold_end_pos: np.ndarray | None = None
    machine_effector_centers: dict = field(default_factory=dict)
    machine_centers: dict = field(default_factory=dict)
    machine_tips: dict = field(default_factory=dict)
    machine_cold_ends: dict = field(default_factory=dict)
    center_sources: dict = field(default_factory=dict)
    center_source_offsets: dict = field(default_factory=dict)
    center_offsets: dict = field(default_factory=dict)
    tip_offsets: dict = field(default_factory=dict)
    cold_end_offsets: dict = field(default_factory=dict)


class ExtruderSystem:
    def update(self, world, dt_unused):
        extruders = world.query([ExtruderComponent])
        if not extruders:
            return
        extruder = world.get_component(extruders[0], ExtruderComponent)
        sums, counts = {}, {}
        for entity in world.query([SpoolTagComponent, PositionComponent]):
            position = get_entity_world_position(world, entity)
            if position is None:
                continue
            tag = world.get_component(entity, MachineTagComponent)
            machine = tag.id if tag is not None and tag.id else 'default'
            sums.setdefault(machine, np.zeros(3))[:] += position
            counts[machine] = counts.get(machine, 0) + 1
        sources = extruder.center_sources if isinstance(extruder.center_sources, dict) else {}
        source_order = object_keys(sources)
        centers, roots, tips, cold_ends = {}, {}, {}, {}
        # Authored source order selects the default machine, independently of
        # spool query order. Missing sources may fall back to a spool average.
        for machine in source_order + [key for key in object_keys(sums) if key not in sources]:
            entities = sources.get(machine)
            points = [get_entity_world_position(world, entity) for entity in entities] if isinstance(entities, list) else []
            points = [point for point in points if point is not None]
            center = sum(points, np.zeros(3)) / len(points) if points else (sums[machine] / counts[machine] if machine in sums else None)
            if center is None:
                continue
            rotation = estimate_effector_rotation(extruder.center_source_offsets.get(machine), center, entities, world) if machine in sources else Quaternion()
            centers[machine] = center.copy()
            root = center.copy()
            offset = extruder.center_offsets.get(machine)
            if _finite_vector(offset):
                root += rotation.transform_vector(offset)
            roots[machine] = root
            tip = root.copy()
            offset = extruder.tip_offsets.get(machine)
            if _finite_vector(offset):
                tip += rotation.transform_vector(offset)
            tips[machine] = tip
            offset = extruder.cold_end_offsets.get(machine)
            if _finite_vector(offset):
                cold_ends[machine] = root + rotation.transform_vector(offset)
        if not roots:
            return  # Keep the previous state when no machine is resolvable.
        chosen = next((key for key in source_order if key in roots), object_keys(roots)[0])
        extruder.machine_effector_centers = centers
        extruder.machine_centers = roots
        extruder.machine_tips = tips
        extruder.machine_cold_ends = cold_ends
        extruder.effector_center_pos = centers[chosen].copy()
        extruder.center_pos = roots[chosen].copy()
        extruder.tip_pos = tips[chosen].copy()
        extruder.cold_end_pos = cold_ends[chosen].copy() if chosen in cold_ends else None
