import asyncio
import json
import functools
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
    PauseStateComponent
)
from slideprinter.slideprinter_common import (
    SpoolTagComponent,
    SpoolStateComponent
)

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
    examples_python_path / "slideprinter" / "slideprinter_common.py",
]

# --- Server-Side Systems ---
class RemoteSpoolSystem:
    def __init__(self):
        self.commands = []
        self.axis_to_entity = {}

    def add_command(self, command):
        self.commands.append(command)

    def update(self, world, dt):
        if not self.axis_to_entity:
            spool_entities = world.query([SpoolTagComponent, SpoolStateComponent])
            for e in spool_entities:
                state = world.get_component(e, SpoolStateComponent)
                if state.axis:
                    self.axis_to_entity[state.axis] = e

        if not self.commands:
            return

        command = self.commands.pop(0)
        if command['type'] == 'G1':
            for axis, entity_id in self.axis_to_entity.items():
                if axis in command:
                    target_angle = command[axis]
                    orientation = world.get_component(entity_id, OrientationComponent)
                    orientation.angle = target_angle

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


# --- Scene and World Setup ---

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
    world.set_resource("pauseState", PauseStateComponent(False))
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

            world.add_component(ent, SpoolTagComponent())
            axis_name = prim.GetName()[-1] # Assumes names like 'SpoolA'
            world.add_component(ent, SpoolStateComponent(axis=axis_name))
            world.add_component(ent, PositionComponent(pos.copy()))
            world.add_component(ent, VelocityComponent(np.zeros(3)))
            world.add_component(ent, RadiusComponent(radius))
            world.add_component(ent, MassComponent(mass))
            world.add_component(ent, RenderableComponent("circle", color or '#a0a0a0'))
            world.add_component(ent, OrientationComponent(0.0))
            world.add_component(ent, AngularVelocityComponent(0.0))
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

    # Process distance joints
    for prim in scene_root.GetChildren():
        if prim.GetTypeName() == 'DistancePhysicsJoint':
            body0_rel = prim.GetRelationship("physics:body0")
            body1_rel = prim.GetRelationship("physics:body1")

            if not (body0_rel.GetTargets() and body1_rel.GetTargets()):
                continue

            body0_path = body0_rel.GetTargets()[0]
            body1_path = body1_rel.GetTargets()[0]

            body0_name = body0_path.name
            body1_name = body1_path.name

            if body0_name not in name_to_entity or body1_name not in name_to_entity:
                continue

            entityA = name_to_entity[body0_name]
            entityB = name_to_entity[body1_name]

            min_dist_attr = prim.GetAttribute("physics:minDistance")
            max_dist_attr = prim.GetAttribute("physics:maxDistance")

            if min_dist_attr.Get() is not None and max_dist_attr.Get() is not None:
                min_dist = min_dist_attr.Get()
                max_dist = max_dist_attr.Get()

                if abs(min_dist - max_dist) < 1e-6:  # It's a fixed distance constraint
                    rest_length = (min_dist + max_dist) / 2.0

                    constraint_entity = world.create_entity()
                    world.add_component(constraint_entity, DistanceConstraintComponent(entityA, entityB, rest_length, 0.0))
                    world.add_component(constraint_entity, RenderableComponent('line', 'green'))

    for cable_path_prim in [
            p for p in scene_root.GetChildren()
            if p.GetRelationship("cablePath:joints")
    ]:
        joint_paths = cable_path_prim.GetRelationship("cablePath:joints").GetTargets()
        joint_entities = []

        for jpath in joint_paths:
            jp = stage.GetPrimAtPath(jpath)
            if not jp:
                continue

            body0_targets = jp.GetRelationship("physics:body0").GetTargets()
            body1_targets = jp.GetRelationship("physics:body1").GetTargets()
            if not (body0_targets and body1_targets):
                continue

            body0_name = body0_targets[0].name
            body1_name = body1_targets[0].name

            ent = world.create_entity()
            world.add_component(
                ent,
                CableJointComponent(
                    name_to_entity[body0_name],
                    name_to_entity[body1_name],
                    float(jp.GetAttribute("restLength").Get()),
                    np.array(jp.GetAttribute("localPos0").Get(), dtype=float),
                    np.array(jp.GetAttribute("localPos1").Get(), dtype=float),
                ),
            )
            joint_entities.append(ent)

        if joint_entities:
            world.add_component(
                world.create_entity(),
                create_cable_path_component(
                    world,
                    joint_entities,
                    list(cable_path_prim.GetAttribute("cablePath:linkTypes").Get() or []),
                    list(cable_path_prim.GetAttribute("cablePath:clockwise").Get() or []),
                    float(cable_path_prim.GetAttribute("stiffness").Get()),
                    stored=list(cable_path_prim.GetAttribute("cablePath:stored").Get() or []),
                ),
            )
    return name_to_entity

def setup_scene(world: World):
    stage = load_slideprinter_stage()
    stage_to_world(world, stage)

    if not world.systems:
        # 1. Cache state from previous step
        world.register_system(PrevFinalPosSystem())
        world.register_system(PrevFinalOrientationSystem())

        # 2. Handle user input and non-physics state changes

        # 3. PREDICTION: Apply forces and integrate velocity to get predicted positions
        world.register_system(MovementSystem())
        world.register_system(AngularMovementSystem())

        world.register_system(RemoteSpoolSystem())

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

    for ball_id in world.query([SpoolTagComponent, PositionComponent, RadiusComponent]):
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

    # Anchors
    for anchor_id in world.query([PositionComponent, RadiusComponent, MassComponent, RenderableComponent]):
        mass = world.get_component(anchor_id, MassComponent).mass
        if mass < 0:
            pos = world.get_component(anchor_id, PositionComponent).pos
            radius = world.get_component(anchor_id, RadiusComponent).radius
            render_comp = world.get_component(anchor_id, RenderableComponent)
            color = render_comp.color if render_comp else '#aaaaaa'
            state.setdefault('anchors', []).append({
                'x': pos[0], 'y': pos[1], 'radius': radius, 'color': color
            })

    pause_comp = world.get_resource('pauseState')
    if pause_comp:
        state['isPaused'] = pause_comp.paused

    # Distance Constraints
    for cid in world.query([DistanceConstraintComponent]):
        constraint = world.get_component(cid, DistanceConstraintComponent)
        posA = world.get_component(constraint.entityA, PositionComponent).pos
        posB = world.get_component(constraint.entityB, PositionComponent).pos
        renderComp = world.get_component(cid, RenderableComponent)
        color = renderComp.color if renderComp else '#ffffff'
        state.setdefault('distanceConstraints', []).append({
            'pA': [posA[0], posA[1]],
            'pB': [posB[0], posB[1]],
            'color': color
        })

    # Cables
    path_entities = world.query([CablePathComponent])
    for pid in path_entities:
        path = world.get_component(pid, CablePathComponent)
        if not path.joint_entities:
            continue
        cable_render = {'joints': [], 'arcs': [], 'links': []}
        for jid in path.joint_entities:
            joint = world.get_component(jid, CableJointComponent)
            cable_render['joints'].append({
                'pA': joint.attachment_point_a_world.tolist()[:2],
                'pB': joint.attachment_point_b_world.tolist()[:2],
                'restLength': joint.rest_length
            })
        # Compute arcs and links for rendering
        link_types = list(path.link_types)
        cw = list(path.cw)
        stored = list(path.stored or [])
        # Rolling arcs
        for i in range(1, len(link_types) - 1):
            if link_types[i] == 'rolling':
                j_prev = world.get_component(path.joint_entities[i - 1], CableJointComponent)
                j_next = world.get_component(path.joint_entities[i], CableJointComponent)
                center_id = j_prev.entity_b
                center_comp = world.get_component(center_id, PositionComponent)
                radius_comp = world.get_component(center_id, RadiusComponent)
                if center_comp and radius_comp:
                    p1 = j_prev.attachment_point_b_world.tolist()[:2]
                    p2 = j_next.attachment_point_a_world.tolist()[:2]
                    center = center_comp.pos.tolist()[:2]
                    radius = radius_comp.radius
                    anticlockwise = not cw[i]
                    dist_prev = np.linalg.norm(j_prev.attachment_point_a_world - j_prev.attachment_point_b_world)
                    dist_next = np.linalg.norm(j_next.attachment_point_a_world - j_next.attachment_point_b_world)
                    is_taut = dist_prev > (j_prev.rest_length + 1e-6) and dist_next > (j_next.rest_length + 1e-6)
                    cable_render['arcs'].append({
                        'center': center,
                        'radius': radius,
                        'p1': p1,
                        'p2': p2,
                        'anticlockwise': anticlockwise,
                        'is_taut': bool(is_taut)
                    })
        # Hybrid links and attachments
        for idx, lt in enumerate(link_types):
            link_data = {'type': lt}
            if lt == 'hybrid-attachment':
                ap = (world.get_component(path.joint_entities[0], CableJointComponent).attachment_point_a_world.tolist()[:2]
                      if idx == 0 else
                      world.get_component(path.joint_entities[-1], CableJointComponent).attachment_point_b_world.tolist()[:2])
                link_data['attachmentPoint'] = ap
            elif lt == 'hybrid':
                if idx == 0:
                    joint = world.get_component(path.joint_entities[0], CableJointComponent)
                    roller_id = joint.entity_a
                    attachment_point = joint.attachment_point_a_world
                else:
                    joint = world.get_component(path.joint_entities[-1], CableJointComponent)
                    roller_id = joint.entity_b
                    attachment_point = joint.attachment_point_b_world
                center_comp = world.get_component(roller_id, PositionComponent)
                radius_comp = world.get_component(roller_id, RadiusComponent)
                if center_comp and radius_comp:
                    center = center_comp.pos.tolist()[:2]
                    radius = radius_comp.radius
                    tangent = attachment_point.tolist()[:2]
                    stored_len = stored[idx]
                    cw_flag = cw[idx]
                    dist = np.linalg.norm(joint.attachment_point_a_world - joint.attachment_point_b_world)
                    is_taut = dist > (joint.rest_length + 1e-6)
                    link_data.update({
                        'center': center,
                        'radius': radius,
                        'tangentPoint': tangent,
                        'storedLength': stored_len,
                        'cw': cw_flag,
                        'is_taut': bool(is_taut)
                    })
            cable_render['links'].append(link_data)
        state['cables'].append(cable_render)

    return json.dumps(state)


# --- WebSocket handler ---
async def handler(websocket, world, remote_spool_system):
    await websocket.send(world_to_json(world))

    async for message in websocket:
        data = json.loads(message)
        action = data.get('action')
        pause_state = world.get_resource('pauseState')

        if action == 'step':
            steps = data.get('steps', 1)
            dt = world.get_resource('dt')
            old_paused = pause_state.paused
            pause_state.paused = False
            for _ in range(steps):
                world.update(dt)
            pause_state.paused = old_paused
        elif action == 'reset':
            setup_scene(world)
        elif action == 'pause':
            pause_state.paused = data['paused']
        elif action == 'gcode':
            remote_spool_system.add_command(data['command'])

        await websocket.send(world_to_json(world))


async def main():
    import websockets
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8766, help="WebSocket port")
    args = parser.parse_args()

    world = World()
    setup_scene(world)
    remote_spool_system = world.get_system(RemoteSpoolSystem)

    print(f"Starting Slideprinter server on ws://localhost:{args.port}")

    serve_handler = functools.partial(handler, world=world, remote_spool_system=remote_spool_system)
    async with websockets.serve(serve_handler, "localhost", args.port):
        asyncio.create_task(watch_and_restart(WATCHED_FILES))
        await asyncio.Future()  # run forever

if __name__ == '__main__':
    asyncio.run(main())
