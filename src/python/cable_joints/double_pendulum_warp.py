import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import warp as wp
import warp.sim
import warp.sim.render

from .ecs import (
    World,
    PositionComponent,
    VelocityComponent,
    MassComponent,
    OrientationComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent,
    PrevFinalPosComponent,
    PrevFinalOrientationComponent,
    GravityAffectedComponent,
    CableLinkComponent,
    RadiusComponent,
    BallTagComponent,
)
from .cable_joints_components import (
    CableJointComponent,
    create_cable_path_component,
)
from .cable_attachment_update_system import CableAttachmentUpdateSystem
from .common_systems import (
    PrevFinalPosSystem,
    PrevFinalOrientationSystem,
    GravitySystem,
    MovementSystem,
    AngularMovementSystem,
    PBDVelocityUpdateSystem,
    PBDAngularVelocityUpdateSystem,
    PBDBallBallCollisions,
)
from python_warp.cable_solver_warp import WarpCableConstraintSolver


@dataclass
class PendulumCfg:
    length: float = 1.0
    mass: float = 1.0
    radius: float = 0.05


class DoublePendulumExample:
    """Two cable joint pendulums rendered with Warp."""

    def __init__(self, stage_path: str = "double_pendulum.usd", device=None) -> None:
        self.device = device or wp.get_preferred_device()
        self.dt = 1.0 / 240.0
        self.frame_dt = 1.0 / 60.0
        self.substeps = max(1, round(self.frame_dt / self.dt))
        self.time = 0.0

        self.world = World()
        self.world.set_resource("dt", self.dt)
        self.world.set_resource("gravity", np.array([0.0, -9.8, 0.0]))

        self._register_systems()
        cfg = PendulumCfg()
        self.anchors, self.bobs = self._create_pendulums(cfg)
        self._build_renderer(stage_path)

    # --------------------------------------------------------------
    def _register_systems(self) -> None:
        self.world.register_system(PrevFinalPosSystem())
        self.world.register_system(PrevFinalOrientationSystem())
        self.world.register_system(GravitySystem())
        self.world.register_system(MovementSystem())
        self.world.register_system(AngularMovementSystem())
        self.world.register_system(CableAttachmentUpdateSystem())
        self.world.register_system(WarpCableConstraintSolver(self.device))
        self.world.register_system(PBDVelocityUpdateSystem())
        self.world.register_system(PBDAngularVelocityUpdateSystem())
        self.world.register_system(PBDBallBallCollisions())

    # --------------------------------------------------------------
    def _create_pendulums(self, cfg: PendulumCfg) -> Tuple[list[int], list[int]]:
        anchors: list[int] = []
        bobs: list[int] = []
        offset = 0.6
        for x in (-offset, offset):
            anchor_pos = np.array([x, 0.0, 0.0])
            bob_pos = anchor_pos + np.array([0.0, -cfg.length, 0.0])

            anchor = self.world.create_entity()
            self.world.add_component(anchor, PositionComponent(anchor_pos.copy()))
            self.world.add_component(anchor, VelocityComponent(np.zeros(3)))
            self.world.add_component(anchor, MassComponent(np.inf))
            self.world.add_component(anchor, MomentOfInertiaComponent(np.inf))
            self.world.add_component(anchor, OrientationComponent())
            self.world.add_component(anchor, AngularVelocityComponent())
            self.world.add_component(anchor, RadiusComponent(0.0))
            self.world.add_component(anchor, CableLinkComponent())
            self.world.add_component(anchor, PrevFinalPosComponent(anchor_pos.copy()))
            self.world.add_component(anchor, PrevFinalOrientationComponent())

            bob = self.world.create_entity()
            self.world.add_component(bob, PositionComponent(bob_pos.copy()))
            self.world.add_component(bob, VelocityComponent(np.zeros(3)))
            self.world.add_component(bob, MassComponent(cfg.mass))
            self.world.add_component(bob, MomentOfInertiaComponent(1.0))
            self.world.add_component(bob, OrientationComponent())
            self.world.add_component(bob, AngularVelocityComponent())
            self.world.add_component(bob, GravityAffectedComponent())
            self.world.add_component(bob, RadiusComponent(cfg.radius))
            self.world.add_component(bob, CableLinkComponent())
            self.world.add_component(bob, BallTagComponent())
            self.world.add_component(bob, PrevFinalPosComponent(bob_pos.copy()))
            self.world.add_component(bob, PrevFinalOrientationComponent())

            joint = self.world.create_entity()
            self.world.add_component(
                joint,
                CableJointComponent(
                    anchor,
                    bob,
                    cfg.length,
                    anchor_pos.copy(),
                    bob_pos.copy(),
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

            anchors.append(anchor)
            bobs.append(bob)

        return anchors, bobs

    # --------------------------------------------------------------
    def _build_renderer(self, stage_path: str) -> None:
        builder = wp.sim.ModelBuilder()
        for aid in self.anchors:
            pos = self.world.get_component(aid, PositionComponent).pos
            builder.add_particle(wp.vec3(*pos), wp.vec3(0.0, 0.0, 0.0), 0.0)

        radius = self.world.get_component(self.bobs[0], RadiusComponent).radius
        mass = self.world.get_component(self.bobs[0], MassComponent).mass
        for bid in self.bobs:
            pos = self.world.get_component(bid, PositionComponent).pos
            vel = self.world.get_component(bid, VelocityComponent).vel
            builder.add_particle(wp.vec3(*pos), wp.vec3(*vel), mass, radius=radius)

        builder.add_spring(0, 2, ke=0.0, kd=0.0, control=0.0)
        builder.add_spring(1, 3, ke=0.0, kd=0.0, control=0.0)

        self.model = builder.finalize()
        self.model.ground = False
        self.state = self.model.state()
        self.renderer = wp.sim.render.SimRenderer(self.model, stage_path, scaling=5.0)

    # --------------------------------------------------------------
    def step(self) -> None:
        for _ in range(self.substeps):
            self.world.update(self.dt)
        for idx, aid in enumerate(self.anchors):
            pos = self.world.get_component(aid, PositionComponent).pos
            self.state.particle_q[idx] = wp.vec3(*pos)
        base = len(self.anchors)
        for i, bid in enumerate(self.bobs):
            pos = self.world.get_component(bid, PositionComponent).pos
            vel = self.world.get_component(bid, VelocityComponent).vel
            self.state.particle_q[base + i] = wp.vec3(*pos)
            self.state.particle_qd[base + i] = wp.vec3(*vel)
        self.time += self.frame_dt

    # --------------------------------------------------------------
    def render(self) -> None:
        self.renderer.begin_frame(self.time)
        self.renderer.render(self.state)
        self.renderer.end_frame()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--stage_path", type=str, default="double_pendulum.usd", help="USD output path")
    parser.add_argument("--num_frames", type=int, default=240, help="Number of frames to simulate")
    parser.add_argument("--device", type=str, default=None, help="Override Warp device")
    args = parser.parse_args()

    with wp.ScopedDevice(args.device or "cuda:0"):
        sim = DoublePendulumExample(stage_path=args.stage_path, device=args.device)
        for _ in range(args.num_frames):
            sim.step()
            sim.render()
        sim.renderer.save()
