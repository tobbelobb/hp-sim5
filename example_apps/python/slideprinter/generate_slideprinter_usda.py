import math
from typing import List

import sys
from pathlib import Path

import numpy as np
import warp as wp
import warp.sim
import warp.sim.render

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "example_apps" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from cable_joints.ecs import (
    World,
    PositionComponent,
    VelocityComponent,
    RadiusComponent,
    MassComponent,
    OrientationComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent,
    RenderableComponent,
    PrevFinalPosComponent,
    PrevFinalOrientationComponent,
    RestitutionComponent,
    DistanceConstraintComponent,
)
from cable_joints.cable_joints_components import (
    CableLinkComponent,
    CableJointComponent,
    CablePathComponent,
    create_cable_path_component,
)
from cable_joints.geometry import tangent_from_point_to_circle
from cable_joints.common_systems import (
    PrevFinalPosSystem,
    PrevFinalOrientationSystem,
    MovementSystem,
    AngularMovementSystem,
    XPBDDistanceConstraintSystem,
    PBDVelocityUpdateSystem,
    PBDAngularVelocityUpdateSystem,
)
from cable_joints.cable_attachment_update_system import CableAttachmentUpdateSystem
from cable_joints.pbd_cable_constraint_solver import PBDCableConstraintSolver
from flipper.flipper_common import (
    PauseStateComponent,
    PBDBallBallCollisions,
    BallTagComponent,
)


def create_spool_entity(
    world: World,
    pos,
    vel,
    ang_vel,
    radius,
    mass,
    inertia,
    restitution,
    color="#a0a0a0",
):
    spool = world.create_entity()
    world.add_component(spool, BallTagComponent())
    world.add_component(spool, PositionComponent(np.array([pos[0], pos[1], 0.0])))
    world.add_component(spool, VelocityComponent(np.array([vel[0], vel[1], 0.0])))
    world.add_component(spool, RadiusComponent(radius))
    world.add_component(spool, MassComponent(mass))
    world.add_component(spool, RenderableComponent("circle", color))
    world.add_component(spool, OrientationComponent(0.0))
    world.add_component(spool, AngularVelocityComponent(ang_vel))
    world.add_component(spool, MomentOfInertiaComponent(inertia))
    world.add_component(spool, PrevFinalPosComponent(np.array([pos[0], pos[1], 0.0])))
    world.add_component(spool, PrevFinalOrientationComponent(0.0))
    world.add_component(spool, CableLinkComponent())
    world.add_component(spool, RestitutionComponent(restitution))
    return spool


def create_anchor_entity(world: World, pos, radius=0.01, color="#aaaaaa"):
    anchor = world.create_entity()
    world.add_component(anchor, BallTagComponent())
    world.add_component(anchor, PositionComponent(np.array([pos[0], pos[1], 0.0])))
    world.add_component(anchor, VelocityComponent(np.zeros(3)))
    world.add_component(anchor, RadiusComponent(radius))
    world.add_component(anchor, MassComponent(-1.0))
    world.add_component(anchor, RenderableComponent("circle", color))
    world.add_component(anchor, CableLinkComponent())
    return anchor


def create_cable_and_joint(
    world: World,
    anchor_e,
    spool_e,
    spool_radius,
    initial_stored,
    color="orange",
    stiffness=20000.0,
):
    anchor_pos = world.get_component(anchor_e, PositionComponent).pos
    spool_pos = world.get_component(spool_e, PositionComponent).pos

    joint = world.create_entity()
    tang = tangent_from_point_to_circle(anchor_pos, spool_pos, spool_radius, True)
    rest_len = np.linalg.norm(tang["a_attach"] - tang["a_circle"])
    world.add_component(
        joint,
        CableJointComponent(anchor_e, spool_e, rest_len, tang["a_attach"], tang["a_circle"]),
    )
    world.add_component(joint, RenderableComponent("line", color))

    path_e = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        [joint],
        ["attachment", "hybrid"],
        [True, True],
        stiffness,
        [0.0, initial_stored],
    )
    world.add_component(path_e, path_comp)
    return joint, path_e


def create_distance_constraint(world: World, eA, eB, compliance=0.0):
    constraint = world.create_entity()
    pos_a = world.get_component(eA, PositionComponent).pos
    pos_b = world.get_component(eB, PositionComponent).pos
    rest = np.linalg.norm(pos_a - pos_b)
    world.add_component(constraint, DistanceConstraintComponent(eA, eB, rest, compliance))
    world.add_component(constraint, RenderableComponent("line", "purple"))
    return constraint


def setup_world() -> (World, List[int], List[int]):
    """Create a world identical to :func:`slideprinter_server.setup_scene`."""
    world = World()

    sim_height = 1.7
    world.set_resource("gravity", np.array([0.0, 0.0, 0.0]))
    world.set_resource("dt", 1.0 / 200.0)
    world.set_resource("simWidth", 1.0)
    world.set_resource("simHeight", sim_height)
    world.set_resource("pauseState", PauseStateComponent(True))
    world.set_resource("debugRenderPoints", {})
    world.set_resource("grabbedBall", None)

    spool_radius = 0.03
    spool_mass = 0.005
    spool_inertia = 30 * 0.5 * spool_mass * spool_radius * spool_radius
    ball_restitution = 0.5
    anchor_radius = 0.01
    turns = 5.0
    initial_stored = turns * spool_radius * math.pi * 2.0
    cable_stiffness = 20000.0
    dist = 0.1

    configs = [
        {
            "spoolPos": (0.0, -dist),
            "spoolVel": (1.0, 0.0),
            "spoolAng": 5.0,
            "anchorPos": (0.0, -dist - 2.0),
        },
        {
            "spoolPos": (dist * math.cos(math.pi / 6), dist * math.sin(math.pi / 6)),
            "spoolVel": (-1.0 / math.sqrt(2), 1.0 / math.sqrt(2)),
            "spoolAng": 5.0,
            "anchorPos": (2.05 * math.cos(math.pi / 6), 2.05 * math.sin(math.pi / 6)),
        },
        {
            "spoolPos": (dist * math.cos(5 * math.pi / 6), dist * math.sin(5 * math.pi / 6)),
            "spoolVel": (0.0, 0.0),
            "spoolAng": 5.0,
            "anchorPos": (2.05 * math.cos(5 * math.pi / 6), 2.05 * math.sin(5 * math.pi / 6)),
        },
    ]

    spools = []
    anchors = []
    for cfg in configs:
        s = create_spool_entity(
            world,
            cfg["spoolPos"],
            cfg["spoolVel"],
            cfg["spoolAng"],
            spool_radius,
            spool_mass,
            spool_inertia,
            ball_restitution,
        )
        a = create_anchor_entity(world, cfg["anchorPos"], anchor_radius)
        create_cable_and_joint(world, a, s, spool_radius, initial_stored, stiffness=cable_stiffness)
        spools.append(s)
        anchors.append(a)

    create_distance_constraint(world, spools[0], spools[1])
    create_distance_constraint(world, spools[1], spools[2])
    create_distance_constraint(world, spools[2], spools[0])

    if not world.systems:
        world.register_system(PrevFinalPosSystem())
        world.register_system(PrevFinalOrientationSystem())
        world.register_system(MovementSystem())
        world.register_system(AngularMovementSystem())
        world.register_system(XPBDDistanceConstraintSystem())
        world.register_system(CableAttachmentUpdateSystem())
        world.register_system(PBDCableConstraintSolver())
        world.register_system(PBDVelocityUpdateSystem())
        world.register_system(PBDAngularVelocityUpdateSystem())
        world.register_system(PBDBallBallCollisions())

    return world, spools, anchors


class SlideprinterExample:
    def __init__(self, stage_path: str = "slideprinter.usd"):
        # --- build ECS world ---
        self.world, self.spool_entities, self.anchor_entities = setup_world()
        self.dt = self.world.get_resource("dt")
        # run a few substeps per rendered frame similar to slideprinter.html
        self.frame_dt = 1.0 / 60.0
        self.substeps = max(1, round(self.frame_dt / self.dt))
        self.time = 0.0

        # --- build Warp model purely for rendering ---
        builder = wp.sim.ModelBuilder()

        # anchors first
        for aid in self.anchor_entities:
            pos = self.world.get_component(aid, PositionComponent).pos
            builder.add_particle(wp.vec3(*pos), wp.vec3(0.0, 0.0, 0.0), 0.0)

        # then spools
        spool_radius = self.world.get_component(self.spool_entities[0], RadiusComponent).radius
        spool_mass = self.world.get_component(self.spool_entities[0], MassComponent).mass
        for sid in self.spool_entities:
            pos = self.world.get_component(sid, PositionComponent).pos
            vel = self.world.get_component(sid, VelocityComponent).vel
            builder.add_particle(wp.vec3(*pos), wp.vec3(*vel), spool_mass, radius=spool_radius)

        # springs for visualising cables
        for i in range(3):
            builder.add_spring(i, len(self.anchor_entities) + i, ke=0.0, kd=0.0, control=0.0)
        builder.add_spring(len(self.anchor_entities) + 0, len(self.anchor_entities) + 1, ke=0.0, kd=0.0, control=0.0)
        builder.add_spring(len(self.anchor_entities) + 1, len(self.anchor_entities) + 2, ke=0.0, kd=0.0, control=0.0)
        builder.add_spring(len(self.anchor_entities) + 2, len(self.anchor_entities) + 0, ke=0.0, kd=0.0, control=0.0)

        self.model = builder.finalize()
        self.model.ground = False

        self.state = self.model.state()

        self.renderer = wp.sim.render.SimRenderer(self.model, stage_path, scaling=10.0)

    def step(self):
        for _ in range(self.substeps):
            self.world.update(self.dt)
        # copy positions back to Warp state
        for idx, aid in enumerate(self.anchor_entities):
            pos = self.world.get_component(aid, PositionComponent).pos
            self.state.particle_q[idx] = wp.vec3(*pos) # This doesn't work
        base = len(self.anchor_entities)
        for i, sid in enumerate(self.spool_entities):
            pos = self.world.get_component(sid, PositionComponent).pos
            vel = self.world.get_component(sid, VelocityComponent).vel
            self.state.particle_q[base + i] = wp.vec3(*pos)
            self.state.particle_qd[base + i] = wp.vec3(*vel)
        self.time += self.frame_dt

    def render(self):
        self.renderer.begin_frame(self.time)
        self.renderer.render(self.state)
        self.renderer.end_frame()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--stage_path", type=str, default="public/usd_scenes/generated_slideprinter.usd", help="USD output path"
    )
    parser.add_argument(
        "--num_frames", type=int, default=240, help="Number of frames to simulate"
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Override Warp device (defaults to cuda:0)",
    )
    args = parser.parse_args()

    # Default to CUDA device 0 for best performance
    with wp.ScopedDevice(args.device or "cuda:0"):
        example = SlideprinterExample(stage_path=args.stage_path)
        for _ in range(args.num_frames):
            example.step()
            example.render()
        example.renderer.save()
