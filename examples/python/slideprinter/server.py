import asyncio
import json
import numpy as np
import websockets

from cable_joints.ecs import (
    World, PauseStateComponent, PositionComponent, VelocityComponent,
    RadiusComponent, MassComponent, OrientationComponent, AngularVelocityComponent,
    MomentOfInertiaComponent, RenderableComponent, PrevFinalPosComponent,
    PrevFinalOrientationComponent, BallTagComponent, CableLinkComponent,
    DistanceConstraintComponent
)
from cable_joints.cable_joints_components import (
    CableJointComponent, CablePathComponent, create_cable_path_component
)
from cable_joints.geometry import tangent_from_point_to_circle
from cable_joints.common_systems import (
    PrevFinalPosSystem, PrevFinalOrientationSystem, MovementSystem,
    AngularMovementSystem, XPBDDistanceConstraintSystem,
    CableAttachmentUpdateSystem, PBDCableConstraintSolver,
    PBDVelocityUpdateSystem, PBDAngularVelocityUpdateSystem,
    PBDBallBallCollisions
)


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

    sim_height = 1.7
    world.set_resource('gravity', np.array([0.0, 0.0, 0.0]))
    world.set_resource('dt', 1.0 / 200.0)
    world.set_resource('simWidth', 1.0)
    world.set_resource('simHeight', sim_height)
    world.set_resource('pauseState', PauseStateComponent(True))
    world.set_resource('debugRenderPoints', {})
    world.set_resource('grabbedBall', None)

    spool_radius = 0.03
    spool_mass = 0.005
    spool_inertia = 30 * 0.5 * spool_mass * spool_radius * spool_radius
    ball_restitution = 0.5
    anchor_radius = 0.01
    turns = 5.0
    initial_stored = turns * spool_radius * np.pi * 2.0
    cable_stiffness = 20000.0
    dist = 0.1

    configs = [
        {
            'spoolPos': (0.0, -dist),
            'spoolVel': (1.0, 0.0),
            'spoolAng': 5.0,
            'anchorPos': (0.0, -dist - 2.0)
        },
        {
            'spoolPos': (dist*np.cos(np.pi/6), dist*np.sin(np.pi/6)),
            'spoolVel': (-1.0/np.sqrt(2), 1.0/np.sqrt(2)),
            'spoolAng': 5.0,
            'anchorPos': (2.05*np.cos(np.pi/6), 2.05*np.sin(np.pi/6))
        },
        {
            'spoolPos': (dist*np.cos(5*np.pi/6), dist*np.sin(5*np.pi/6)),
            'spoolVel': (0.0, 0.0),
            'spoolAng': 5.0,
            'anchorPos': (2.05*np.cos(5*np.pi/6), 2.05*np.sin(5*np.pi/6))
        }
    ]

    spools = []
    for cfg in configs:
        s = create_spool_entity(world, cfg['spoolPos'], cfg['spoolVel'], cfg['spoolAng'],
                                spool_radius, spool_mass, spool_inertia, ball_restitution)
        a = create_anchor_entity(world, cfg['anchorPos'], anchor_radius)
        create_cable_and_joint(world, a, s, spool_radius, initial_stored, stiffness=cable_stiffness)
        spools.append(s)

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
    print('Starting Slideprinter server on ws://localhost:8766')
    async with websockets.serve(handler, 'localhost', 8766):
        await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())
