import numpy as np
import math

from .ecs import (
    PositionComponent, RadiusComponent, OrientationComponent, CableLinkComponent,
    CoefficientOfFrictionComponent
)
from .cable_joints_components import CablePathComponent, CableJointComponent
from .geometry import (
    tangent_from_point_to_circle, tangent_from_circle_to_point,
    tangent_from_circle_to_circle, signed_arc_length_on_wheel
)
from .vector2 import rotate_inplace, normalize_inplace
from .update_hybrid_link_states import update_hybrid_link_states
from .split_joints import split_joints
from cable_joints.update_attachment_points import update_attachment_points
from cable_joints.merge_joints import merge_joints

def _clear_debug_points(world):
    debug_points = world.get_resource('debugRenderPoints')
    if debug_points is not None:
        debug_points.clear()

class CableAttachmentUpdateSystem:
    """
    Updates the state of the cable system based on the predicted positions
    of the linked entities. This includes updating attachment points on rollers,
    handling topological changes (merging/splitting joints), and updating
    the state of hybrid links. This system should run after the prediction
    step and before the main PBD constraint solver.
    """
    def __init__(self):
        self.run_in_pause = False

    def update(self, world, dt):
        _clear_debug_points(world)

        # Update attachment points based on predicted geometry
        update_attachment_points(world)

        # Handle topological and state changes
        merge_joints(world)
        split_joints(world)
        update_hybrid_link_states(world)
