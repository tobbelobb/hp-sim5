import asyncio
import json
import os
import sys
from pathlib import Path
from dataclasses import dataclass

import numpy as np
from pxr import Usd, UsdGeom, UsdShade


root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from cable_joints.ecs import (
    World, PositionComponent, VelocityComponent,
    RadiusComponent, MassComponent, OrientationComponent, AngularVelocityComponent,
    MomentOfInertiaComponent, RenderableComponent, PrevFinalPosComponent,
    PrevFinalOrientationComponent,
    DistanceConstraintComponent,
    RestitutionComponent,
    CoefficientOfFrictionComponent
)
from cable_joints.cable_joints_components import (
    CableLinkComponent, CableJointComponent, CablePathComponent, create_cable_path_component
)
from cable_joints.common_systems import (
    PrevFinalPosSystem, PrevFinalOrientationSystem, MovementSystem,
    AngularMovementSystem, XPBDDistanceConstraintSystem,
    PBDVelocityUpdateSystem, PBDAngularVelocityUpdateSystem
)

from cable_joints.cable_attachment_update_system import CableAttachmentUpdateSystem
from cable_joints.cable_attachment_cache_system import CableAttachmentCacheSystem
from cable_joints.cable_slack_system import CableSlackSystem
from cable_joints.pbd_cable_constraint_solver import PBDCableConstraintSolver
from cable_joints.pbd_resolve_cable_over_corrections import PBDResolveCableOverCorrections
from cable_joints.cable_friction_system import CableFrictionSystem

from flipper.flipper_common import (
    PauseStateComponent, BallTagComponent,
)

@dataclass
class SpoolTagComponent:
    """A tag component for entities that are spools."""
    pass


# Files that trigger a server restart when modified
python_dir = src_python_path / "cable_joints"
WATCHED_FILES = [
    Path(__file__),
    root_dir / "examples" / "usd_scenes" / "slideprinter.usda",
    python_dir / "ecs.py",
    python_dir / "common_systems.py",
    python_dir / "cable_attachment_update_system.py",
    python_dir / "cable_attachment_cache_system.py",
    python_dir / "cable_slack_system.py",
    python_dir / "pbd_cable_constraint_solver.py",
    python_dir / "pbd_resolve_cable_over_corrections.py",
    python_dir / "cable_friction_system.py",
    python_dir / "cable_joints_components.py",
    python_dir / "geometry.py",
    examples_python_path / "flipper" / "flipper_common.py",
]

def _copy_usd_on_change(changed_file: Path, root_dir: Path):
    """Copy slideprinter.usda to the public dir for vite when it changes."""
    try:
        source_path = root_dir / "examples" / "usd_scenes" / "slideprinter.usda"
        if changed_file.resolve() != source_path.resolve():
            return

        dest_path = (
            root_dir
            / "public"
            / "examples"
            / "usd_scenes"
            / "slideprinter_copy_for_vite.usda.txt"
        )
        lock_path = dest_path.parent / (dest_path.name + ".lock")

        dest_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            lock_fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        except FileExistsError:
            return
        else:
            try:
                import shutil
                shutil.copy(source_path, dest_path)
                print(
                    f"Copied {source_path.relative_to(root_dir)} to {dest_path.relative_to(root_dir)}"
                )
            finally:
                os.close(lock_fd)
                os.remove(lock_path)
    except Exception as e:  # pragma: no cover - best effort only
        print(f"Could not copy {changed_file}: {e}", file=sys.stderr)


async def watch_and_restart(files, interval=1.0):
    """Monitor *files* and restart the process if any change."""
    mtimes = {}
    for f in files:
        try:
            mtimes[f] = Path(f).stat().st_mtime
        except FileNotFoundError:
            mtimes[f] = 0
    while True:
        await asyncio.sleep(interval)
        for f in files:
            try:
                mtime = Path(f).stat().st_mtime
            except FileNotFoundError:
                mtime = 0
            if mtime != mtimes.get(f):
                _copy_usd_on_change(Path(f), root_dir)
                print(f"{f} changed, restarting server...")
                os.execv(sys.executable, [sys.executable] + sys.argv)
        for f in files:
            try:
                mtimes[f] = Path(f).stat().st_mtime
            except FileNotFoundError:
                mtimes[f] = 0


# --- Helper entity creation ---

def create_distance_constraint(world, eA, eB, compliance=0.0):
    constraint = world.create_entity()
    pos_a = world.get_component(eA, PositionComponent).pos
    pos_b = world.get_component(eB, PositionComponent).pos
    rest = np.linalg.norm(pos_a - pos_b)
    world.add_component(constraint, DistanceConstraintComponent(eA, eB, rest, compliance))
    world.add_component(constraint, RenderableComponent('line', 'purple'))
    return constraint


# --- Scene setup ---

def load_slideprinter_stage():
    """Load the slideprinter demo USD stage."""
    scene_path = root_dir / "examples" / "usd_scenes" / "slideprinter.usda"
    return Usd.Stage.Open(str(scene_path))


def _material_properties(stage, prim):
    """Return (color_hex, friction, restitution) from a prim's bound material."""
    rel = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel()
    if not rel.GetTargets():
        return None, None, None

    mat_prim = stage.GetPrimAtPath(rel.GetTargets()[0])
    color = None
    friction = None
    restitution = None

    if not mat_prim:
        return color, friction, restitution

    shader_prim = mat_prim.GetChild("Shader")
    if shader_prim:
        color_attr = shader_prim.GetAttribute("inputs:diffuseColor")
        if color_attr and color_attr.Get() is not None:
            c = color_attr.Get()
            color = "#%02x%02x%02x" % (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))

    fric_attr = mat_prim.GetAttribute("physics:staticFriction")
    if fric_attr and fric_attr.Get() is not None:
        friction = fric_attr.Get()

    rest_attr = mat_prim.GetAttribute("physics:restitution")
    if rest_attr and rest_attr.Get() is not None:
        restitution = rest_attr.Get()

    return color, friction, restitution


def stage_to_world(world, stage):
    """Populate *world* using entities described in *stage*."""
    world.clear()

    dt = 1.0 / stage.GetTimeCodesPerSecond()
    world.set_resource("dt", dt)
    world.set_resource("pauseState", PauseStateComponent(True))
    world.set_resource("debugRenderPoints", {})
    world.set_resource("grabbedBall", None)

    physics_scene = stage.GetPrimAtPath("/World/PhysicsScene")
    if physics_scene:
        gdir_attr = physics_scene.GetAttribute("physics:gravityDirection")
        gmag_attr = physics_scene.GetAttribute("physics:gravityMagnitude")
        if gdir_attr.Get() is not None and gmag_attr.Get() is not None:
            gdir = np.array(gdir_attr.Get())
            gmag = gmag_attr.Get()
            world.set_resource("gravity", gdir * gmag)
        else:
            world.set_resource("gravity", np.array([0.0, 0.0, 0.0]))
    else:
        world.set_resource("gravity", np.array([0.0, 0.0, 0.0]))

    scene_root = stage.GetPrimAtPath("/World/SlideprinterScene")
    name_to_entity = {}

    for prim in scene_root.GetChildren():
        tags_attr = prim.GetAttribute("ecs:tags")
        tags = tags_attr.Get() if tags_attr else []
        if not tags:
            continue

        xform = UsdGeom.Xformable(prim)
        pos = np.zeros(3)
        for op in xform.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                pos = np.array(op.Get())

        color, fric, rest = _material_properties(stage, prim)

        if "Spool" in tags:
            ent = world.create_entity()
            radius = prim.GetAttribute("radius").Get()
            mass = prim.GetAttribute("physics:mass").Get()
            inertia_tensor = prim.GetAttribute("physics:inertiaTensor").Get()
            inertia = inertia_tensor[2][2]
            vel = np.array(prim.GetAttribute("physics:velocity").Get())
            ang_vel_attr = prim.GetAttribute("physics:angularVelocity")
            ang_vel = ang_vel_attr.Get()[2] if ang_vel_attr and ang_vel_attr.Get() is not None else 0.0

            world.add_component(ent, SpoolTagComponent())
            world.add_component(ent, BallTagComponent()) # For serialization
            world.add_component(ent, PositionComponent(pos.copy()))
            world.add_component(ent, VelocityComponent(vel.copy()))
            world.add_component(ent, RadiusComponent(radius))
            world.add_component(ent, MassComponent(mass))
            world.add_component(ent, RenderableComponent("circle", color or '#a0a0a0'))
            world.add_component(ent, OrientationComponent(0.0))
            world.add_component(ent, AngularVelocityComponent(ang_vel))
            world.add_component(ent, MomentOfInertiaComponent(inertia))
            world.add_component(ent, PrevFinalPosComponent(pos.copy()))
            world.add_component(ent, PrevFinalOrientationComponent(0.0))
            if prim.GetAttribute("cable:linkable").Get():
                world.add_component(ent, CableLinkComponent())
            if rest is not None:
                world.add_component(ent, RestitutionComponent(rest))
            if fric is not None:
                world.add_component(ent, CoefficientOfFrictionComponent(fric))
            name_to_entity[prim.GetName()] = ent

        elif "Anchor" in tags:
            ent = world.create_entity()
            world.add_component(ent, PositionComponent(pos.copy()))
            world.add_component(ent, RadiusComponent(0.01))
            world.add_component(ent, MassComponent(-1.0))
            world.add_component(ent, RenderableComponent("circle", color or '#aaaaaa'))
            if prim.GetAttribute("cable:linkable").Get():
                world.add_component(ent, CableLinkComponent())
            name_to_entity[prim.GetName()] = ent

    # Cable joints and path
    joint_prims = [p for p in scene_root.GetChildren() if p.GetTypeName() == 'CableJoint']
    joint_entities = {}
    for jp in joint_prims:
        body0 = jp.GetRelationship("physics:body0").GetTargets()[0].name
        body1 = jp.GetRelationship("physics:body1").GetTargets()[0].name
        attach_a = np.array(jp.GetAttribute("localPos0").Get())
        attach_b = np.array(jp.GetAttribute("localPos1").Get())
        rest_len = jp.GetAttribute("restLength").Get()
        ent = world.create_entity()
        world.add_component(ent, CableJointComponent(name_to_entity[body0], name_to_entity[body1], rest_len, attach_a, attach_b))
        joint_entities[jp.GetPath()] = ent

    path_prims = [p for p in scene_root.GetChildren() if p.GetAttribute("apiSchemas").Get() and "CablePathAPI" in p.GetAttribute("apiSchemas").Get()]
    for cable_path_prim in path_prims:
        joint_paths = cable_path_prim.GetRelationship("cablePath:joints").GetTargets()
        ordered = [joint_entities[p] for p in joint_paths if p in joint_entities]
        link_types = list(cable_path_prim.GetAttribute("cablePath:linkTypes").Get() or [])
        cw = list(cable_path_prim.GetAttribute("cablePath:clockwise").Get() or [])
        stored = list(cable_path_prim.GetAttribute("cablePath:stored").Get() or [])
        stiffness = cable_path_prim.GetAttribute("stiffness").Get()
        if ordered:
            cid = world.create_entity()
            path_comp = create_cable_path_component(world, ordered, link_types, cw, stiffness, stored=stored)
            world.add_component(cid, path_comp)

    return name_to_entity


def setup_scene(world: World):
    stage = load_slideprinter_stage()
    name_to_entity = stage_to_world(world, stage)

    spool_names = sorted([name for name, entity_id in name_to_entity.items() if world.has_component(entity_id, SpoolTagComponent)])

    if len(spool_names) == 3:
        spool_entities = [name_to_entity[name] for name in spool_names]
        create_distance_constraint(world, spool_entities[0], spool_entities[1])
        create_distance_constraint(world, spool_entities[1], spool_entities[2])
        create_distance_constraint(world, spool_entities[2], spool_entities[0])

    if not world.systems:
        # 1. Cache state from previous step
        world.register_system(PrevFinalPosSystem())
        world.register_system(PrevFinalOrientationSystem())

        # 2. Handle user input and non-physics state changes
        # This is where we could animate spool movements for example

        # 3. PREDICTION: Apply forces and integrate velocity to get predicted positions
        world.register_system(MovementSystem())
        world.register_system(AngularMovementSystem())

        # 4. Update derived geometry and cable state
        world.register_system(XPBDDistanceConstraintSystem())
        world.register_system(CableAttachmentUpdateSystem())
        world.register_system(CableAttachmentCacheSystem())
        world.register_system(CableSlackSystem());

        # 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
        world.register_system(PBDCableConstraintSolver())
        world.register_system(PBDResolveCableOverCorrections());

        # 6. POST-SOLVE CABLE DYNAMICS: Handle friction-based slip using accurate tension
        world.register_system(CableFrictionSystem());

        # 7. UPDATE VELOCITY: Derive final velocities from the position changes
        world.register_system(PBDVelocityUpdateSystem())
        world.register_system(PBDAngularVelocityUpdateSystem())

        # 8. VELOCITY SOLVERS: Apply restitution and dynamic friction
        # Velocity-level solvers (which might also do positional adjustments)

        # 9. Game Logic or similar. Counters and stuff


# --- Serialization ---

def world_to_json(world: World) -> str:
    state = {'balls': [], 'cables': [], 'isPaused': True}

    for ball_id in world.query([BallTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(ball_id, PositionComponent).pos
        radius = world.get_component(ball_id, RadiusComponent).radius
        mass = world.get_component(ball_id, MassComponent).mass
        renderable = world.get_component(ball_id, RenderableComponent)
        color = renderable.color if renderable else '#888888'
        ball_data = {'x': pos[0], 'y': pos[1], 'radius': radius, 'mass': mass, 'color': color}
        orientation_comp = world.get_component(ball_id, OrientationComponent)
        if orientation_comp:
            ball_data['angle'] = orientation_comp.angle
        state['balls'].append(ball_data)

    pause_comp = world.get_resource('pauseState')
    if pause_comp:
        state['isPaused'] = pause_comp.paused

    # Cables
    path_entities = world.query([CablePathComponent])
    for pid in path_entities:
        path = world.get_component(pid, CablePathComponent)
        if not path.joint_entities:
            continue
        cable_render = {'joints': []}
        for jid in path.joint_entities:
            joint = world.get_component(jid, CableJointComponent)
            cable_render['joints'].append({
                'pA': joint.attachment_point_a_world.tolist()[:2],
                'pB': joint.attachment_point_b_world.tolist()[:2],
                'restLength': joint.rest_length
            })
        state['cables'].append(cable_render)

    return json.dumps(state)


# --- WebSocket handler ---
async def handler(websocket):
    world = World()
    setup_scene(world)

    await websocket.send(world_to_json(world))

    async for message in websocket:
        data = json.loads(message)
        action = data.get('action')
        pause_state = world.get_resource('pauseState')

        if action == 'step':
            if not pause_state.paused:
                steps = data.get('steps', 1)
                dt = world.get_resource('dt')
                for _ in range(steps):
                    world.update(dt)
        elif action == 'reset':
            setup_scene(world)
        elif action == 'pause':
            pause_state.paused = data['paused']

        await websocket.send(world_to_json(world))


async def main():
    import websockets
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8766, help="WebSocket port")
    args = parser.parse_args()

    print(f"Starting Slideprinter server on ws://localhost:{args.port}")

    async with websockets.serve(handler, "localhost", args.port):
        asyncio.create_task(watch_and_restart(WATCHED_FILES))
        await asyncio.Future()  # run forever

if __name__ == '__main__':
    asyncio.run(main())
