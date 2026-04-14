"""Pendulum simulation using the Python PBD framework.

This example mirrors the simple Warp-based demo but runs on the
custom Position Based Dynamics (PBD) engine used by the rest of the
project.  The simulation is built on the ECS framework from
:mod:`cable_joints.ecs` and uses a single :class:`CableJointComponent` to
constrain a mass to an anchor point.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from cable_joints.ecs import (
    World, PositionComponent, VelocityComponent, MassComponent,
    OrientationComponent, AngularVelocityComponent, MomentOfInertiaComponent,
    PrevFinalPosComponent, PrevFinalOrientationComponent,
    GravityAffectedComponent, CableLinkComponent, RadiusComponent,
)
from cable_joints.cable_joints_components import (
    CableJointComponent, create_cable_path_component, CablePathComponent,
)
from cable_joints.cable_attachment_update_system import CableAttachmentUpdateSystem
from cable_joints.pbd_cable_constraint_solver import PBDCableConstraintSolver
from cable_joints.common_systems import (
    PrevFinalPosSystem, PrevFinalOrientationSystem, MovementSystem,
    AngularMovementSystem, GravitySystem, PBDVelocityUpdateSystem,
    PBDAngularVelocityUpdateSystem,
)


@dataclass
class PendulumConfig:
    """Configuration values for :class:`PendulumSimulation`."""

    length: float = 1.0
    mass: float = 1.0
    dt: float = 1.0 / 60.0


class PendulumSimulation:
    """Simple pendulum driven by the PBD engine."""

    def __init__(self, config: PendulumConfig | None = None) -> None:
        if config is None:
            config = PendulumConfig()
        self.cfg = config

        self.world = World()
        self.world.set_resource("dt", config.dt)
        self.world.set_resource("gravity", np.array([0.0, -9.8, 0.0]))

        self._register_systems()
        self.anchor, self.mass = self._create_entities()
        self._create_cable_joint()

    # ------------------------------------------------------------------
    def _register_systems(self) -> None:
        self.world.register_system(PrevFinalPosSystem())
        self.world.register_system(PrevFinalOrientationSystem())
        self.world.register_system(GravitySystem())
        self.world.register_system(MovementSystem())
        self.world.register_system(AngularMovementSystem())
        self.world.register_system(CableAttachmentUpdateSystem())
        self.world.register_system(PBDCableConstraintSolver())
        self.world.register_system(PBDVelocityUpdateSystem())
        self.world.register_system(PBDAngularVelocityUpdateSystem())

    # ------------------------------------------------------------------
    def _create_entities(self) -> tuple[int, int]:
        """Create the anchor and bob entities."""
        length = self.cfg.length
        mass = self.cfg.mass

        anchor = self.world.create_entity()
        self.world.add_component(anchor, PositionComponent(np.zeros(3)))
        self.world.add_component(anchor, VelocityComponent(np.zeros(3)))
        self.world.add_component(anchor, MassComponent(np.inf))
        self.world.add_component(anchor, MomentOfInertiaComponent(np.inf))
        self.world.add_component(anchor, OrientationComponent())
        self.world.add_component(anchor, AngularVelocityComponent())
        self.world.add_component(anchor, RadiusComponent(0.0))
        self.world.add_component(anchor, CableLinkComponent())
        self.world.add_component(anchor, PrevFinalPosComponent(np.zeros(3)))
        self.world.add_component(anchor, PrevFinalOrientationComponent())

        bob = self.world.create_entity()
        self.world.add_component(bob, PositionComponent(np.array([0.0, -length, 0.0])))
        self.world.add_component(bob, VelocityComponent(np.zeros(3)))
        self.world.add_component(bob, MassComponent(mass))
        self.world.add_component(bob, MomentOfInertiaComponent(1.0))
        self.world.add_component(bob, OrientationComponent())
        self.world.add_component(bob, AngularVelocityComponent())
        self.world.add_component(bob, GravityAffectedComponent())
        self.world.add_component(bob, RadiusComponent(0.05))
        self.world.add_component(bob, CableLinkComponent())
        self.world.add_component(bob, PrevFinalPosComponent(np.array([0.0, -length, 0.0])))
        self.world.add_component(bob, PrevFinalOrientationComponent())

        return anchor, bob

    # ------------------------------------------------------------------
    def _create_cable_joint(self) -> None:
        length = self.cfg.length

        joint = self.world.create_entity()
        self.world.add_component(
            joint,
            CableJointComponent(
                self.anchor,
                self.mass,
                length,
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, -length, 0.0]),
            ),
        )

        path_ent = self.world.create_entity()
        path_comp = create_cable_path_component(
            self.world,
            joint_entities=[joint],
            link_types=["attachment", "attachment"],
            cw=[True, True],
        )
        self.world.add_component(path_ent, path_comp)

    # ------------------------------------------------------------------
    def step(self, steps: int = 1) -> None:
        dt = self.cfg.dt
        for _ in range(steps):
            self.world.update(dt)

    # ------------------------------------------------------------------
    def bob_position(self) -> np.ndarray:
        return self.world.get_component(self.mass, PositionComponent).pos


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--frames", type=int, default=240, help="Number of frames")
    args = parser.parse_args()

    sim = PendulumSimulation()
    for _ in range(args.frames):
        sim.step()
        pos = sim.bob_position()
        print(f"time: {_ * sim.cfg.dt:.3f} pos: {pos[0]:.3f}, {pos[1]:.3f}")
