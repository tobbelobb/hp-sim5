"""Ideal two-port gate geometry; an experimental solver building block.

No pulley inertia, friction, elastic state, or time integration is implemented.
Port coordinates and internal cable length are fixed in the carrier frame.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PinholeGate:
    entry_local: np.ndarray
    exit_local: np.ndarray
    internal_length_m: float

    def length_and_jacobian(self, position, rotation, anchors):
        """Length and [translation, world rotation] gradient of both legs.

        The caller adds the other spans and spool payout to its path constraint.
        A tension T exerts generalized force -T * jacobian on the carrier.
        """
        arms = np.array([self.entry_local, self.exit_local]) @ rotation.T
        directions = position + arms - anchors
        lengths = np.linalg.norm(directions, axis=1)
        if np.any(lengths <= 0):
            raise ValueError("A cable anchor coincides with a gate port")
        directions /= lengths[:, None]
        jacobian = np.r_[directions.sum(axis=0), np.cross(arms, directions).sum(axis=0)]
        return float(lengths.sum() + self.internal_length_m), jacobian
