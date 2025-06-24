import asyncio
import json
import os
import sys
from pathlib import Path

import numpy as np


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
    DistanceConstraintComponent
)
from cable_joints.cable_joints_components import (
    CableLinkComponent, CableJointComponent, CablePathComponent, create_cable_path_component
)
from cable_joints.geometry import tangent_from_point_to_circle
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
from examples.usd_scenes.slideprinter_usd_demo import parse_slideprinter
import re

from flipper.flipper_common import (
    PauseStateComponent, BallTagComponent,
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


def _read_timecodes_per_second(path: Path) -> float:
    """Parse timeCodesPerSecond from a USDA file."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                m = re.search(r"timeCodesPerSecond\s*=\s*(\d+(?:\.\d+)?)", line)
                if m:
                    return float(m.group(1))
    except Exception:
        pass
    return 200.0


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

def create_spool_entity(world, pos, vel, ang_vel, radius, mass, inertia, restitution, color="#a0a0a0"):
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
    return spool


def create_anchor_entity(world, pos, radius=0.01, color="#aaaaaa"):
    anchor = world.create_entity()
    world.add_component(anchor, BallTagComponent())
    world.add_component(anchor, PositionComponent(np.array([pos[0], pos[1], 0.0])))
    world.add_component(anchor, VelocityComponent(np.zeros(3)))
    world.add_component(anchor, RadiusComponent(radius))
    world.add_component(anchor, MassComponent(-1.0))
    world.add_component(anchor, RenderableComponent("circle", color))
    world.add_component(anchor, CableLinkComponent())
    return anchor


def create_cable_and_joint(world, anchor_e, spool_e, spool_radius, initial_stored, color="orange", stiffness=20000.0):
    anchor_pos = world.get_component(anchor_e, PositionComponent).pos
    spool_pos = world.get_component(spool_e, PositionComponent).pos

    joint = world.create_entity()
    tang = tangent_from_point_to_circle(anchor_pos, spool_pos, spool_radius, True)
    rest_len = np.linalg.norm(tang['a_attach'] - tang['a_circle'])
    world.add_component(joint, CableJointComponent(anchor_e, spool_e, rest_len, tang['a_attach'], tang['a_circle']))
    world.add_component(joint, RenderableComponent('line', color))

    path_e = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        [joint],
        ['attachment', 'hybrid'],
        [True, True],
        stiffness,
        [0.0, initial_stored]
    )
    world.add_component(path_e, path_comp)
    return joint, path_e


def create_distance_constraint(world, eA, eB, compliance=0.0):
    constraint = world.create_entity()
    pos_a = world.get_component(eA, PositionComponent).pos
    pos_b = world.get_component(eB, PositionComponent).pos
    rest = np.linalg.norm(pos_a - pos_b)
    world.add_component(constraint, DistanceConstraintComponent(eA, eB, rest, compliance))
    world.add_component(constraint, RenderableComponent('line', 'purple'))
    return constraint


# --- Scene setup ---

def setup_scene(world: World):
    world.clear()

    scene_path = root_dir / "examples" / "usd_scenes" / "slideprinter.usda"
    tcps = _read_timecodes_per_second(scene_path)
    _, entities, _j, _p, _ = parse_slideprinter(str(scene_path))

    sim_height = 1.7
    world.set_resource('gravity', np.array([0.0, 0.0, 0.0]))
    world.set_resource('dt', 1.0 / tcps)
    world.set_resource('simWidth', 1.0)
    world.set_resource('simHeight', sim_height)
    world.set_resource('pauseState', PauseStateComponent(True))
    world.set_resource('debugRenderPoints', {})
    world.set_resource('grabbedBall', None)

    turns = 5.0
    cable_stiffness = 20000.0

    spools = []
    spool_names = ['A', 'B', 'C']
    for n in spool_names:
        sp_info = entities.get(f'Spool{n}')
        an_info = entities.get(f'Anchor{n}')
        if not sp_info or not an_info:
            continue
        spool_radius = sp_info.get('radius', 0.03)
        spool_mass = sp_info.get('mass', 0.005)
        spool_inertia = 30 * 0.5 * spool_mass * spool_radius * spool_radius
        ball_restitution = 0.5
        anchor_radius = an_info.get('radius', 0.01)
        initial_stored = turns * spool_radius * np.pi * 2.0

        s = create_spool_entity(
            world,
            tuple(sp_info.get('pos', [0.0, 0.0])),
            (sp_info.get('velX', 0.0), sp_info.get('velY', 0.0)),
            sp_info.get('angVel', 0.0),
            spool_radius,
            spool_mass,
            spool_inertia,
            ball_restitution,
        )
        a = create_anchor_entity(world, tuple(an_info.get('pos', [0.0, 0.0])), anchor_radius)
        create_cable_and_joint(world, a, s, spool_radius, initial_stored, stiffness=cable_stiffness)
        spools.append(s)

    if len(spools) >= 3:
        create_distance_constraint(world, spools[0], spools[1])
        create_distance_constraint(world, spools[1], spools[2])
        create_distance_constraint(world, spools[2], spools[0])

    create_distance_constraint(world, spools[0], spools[1])
    create_distance_constraint(world, spools[1], spools[2])
    create_distance_constraint(world, spools[2], spools[0])

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
        state['balls'].append({'x': pos[0], 'y': pos[1], 'radius': radius, 'mass': mass, 'color': color})

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
                steps = data.get('steps', 0)
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
