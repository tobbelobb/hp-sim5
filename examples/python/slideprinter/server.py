import asyncio
import asyncio
import json
import functools
import os
import sys
from pathlib import Path
import dataclasses

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
    SpoolStateComponent,
    StepperMotorComponent,
    StepperMotorSystem
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
class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if dataclasses.is_dataclass(obj):
            return dataclasses.asdict(obj)
        if isinstance(obj, Path):
            return str(obj)
        return super(NpEncoder, self).default(obj)


class RemoteSpoolSystem:
    def __init__(self):
        self.commands = []
        self.axis_to_entity = {}

    def add_command(self, command):
        self.commands.append(command)

    def update(self, world, dt):
        # Cache axis-to-entity mapping if not already done
        if not self.axis_to_entity:
            spool_entities = world.query([SpoolTagComponent, SpoolStateComponent])
            for e in spool_entities:
                state = world.get_component(e, SpoolStateComponent)
                if state.axis:
                    self.axis_to_entity[state.axis] = e

        command = self.commands.pop(0) if self.commands else None

        for axis, entity_id in self.axis_to_entity.items():
            if command and command['type'] == 'Move' and axis in command:
                stepper_comp = world.get_component(entity_id, StepperMotorComponent)
                if stepper_comp:
                    stepper_comp.commanded_angle = command[axis]
            if command and command['type'] == 'Add to reference' and axis in command:
                stepper_comp = world.get_component(entity_id, StepperMotorComponent)
                if stepper_comp:
                    stepper_comp.delta_angle += command[axis]


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
            if "Stepper" in tags:
                world.add_component(ent, StepperMotorComponent())

        if "Anchor" in tags:
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
        world.register_system(RemoteSpoolSystem())
        world.register_system(StepperMotorSystem())

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

def world_to_full_state_json(world: World) -> str:
    state = {
        'entities': {},
        'components': {},
        'resources': {}
    }

    # Serialize entities and their component types
    for entity_id, component_classes in world.entities.items():
        state['entities'][str(entity_id)] = [c.__name__ for c in component_classes]

    # Serialize component data
    for component_class, component_map in world.components.items():
        class_name = component_class.__name__
        state['components'][class_name] = {}
        for entity_id, component_instance in component_map.items():
            comp_dict = dataclasses.asdict(component_instance)

            # Add world-space attachment points for JS client
            if class_name == 'CableJointComponent':
                def get_world_space_2d(body_pos, body_angle, local_pos):
                    c, s = np.cos(body_angle), np.sin(body_angle)
                    rotated_local = np.array([
                        local_pos[0] * c - local_pos[1] * s,
                        local_pos[0] * s + local_pos[1] * c
                    ])
                    return body_pos[:2] + rotated_local

                entity_a = component_instance.entity_a
                pos_a = world.get_component(entity_a, PositionComponent).pos
                rot_a_comp = world.get_component(entity_a, OrientationComponent)
                rot_a = rot_a_comp.angle if rot_a_comp else 0.0

                entity_b = component_instance.entity_b
                pos_b = world.get_component(entity_b, PositionComponent).pos
                rot_b_comp = world.get_component(entity_b, OrientationComponent)
                rot_b = rot_b_comp.angle if rot_b_comp else 0.0

                comp_dict['attachment_point_a_world'] = get_world_space_2d(pos_a, rot_a, component_instance.attachment_point_a_local)
                comp_dict['attachment_point_b_world'] = get_world_space_2d(pos_b, rot_b, component_instance.attachment_point_b_local)

            state['components'][class_name][str(entity_id)] = comp_dict

    # Serialize resources
    for name, resource in world.resources.items():
        if name in ['grabbedBall', 'debugRenderPoints']:
            continue
        if name in ['dt', 'gravity', 'pauseState']:
             state['resources'][name] = dataclasses.asdict(resource) if dataclasses.is_dataclass(resource) else resource

    return json.dumps(state, cls=NpEncoder)


# --- WebSocket handler ---
async def handler(websocket, world, remote_spool_system):
    await websocket.send(world_to_full_state_json(world))

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

        await websocket.send(world_to_full_state_json(world))


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
