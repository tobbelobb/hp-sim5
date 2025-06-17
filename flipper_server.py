import asyncio
import json
import functools
import os
import sys
import numpy as np
from pathlib import Path

# Assuming the python ECS code is in a 'python' directory
from python.ecs import (
    World, PauseStateComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent,
    RestitutionComponent, GravityAffectedComponent, OrientationComponent, AngularVelocityComponent,
    MomentOfInertiaComponent, ObstacleTagComponent, ScoredTagComponent, FlipperTagComponent,
    FlipperStateComponent, FlipperTipComponent, BorderComponent, RenderableComponent,
    PrevFinalPosComponent, PrevFinalOrientationComponent, BallTagComponent, ObstaclePushComponent,
    CableLinkComponent, CoefficientOfFrictionComponent
)
from python.usd_ecs import (
    ScoreComponent, ScoreSystem, FlipperMotionSystem, RemoteInputSystem,
    load_stage, stage_to_world, register_default_systems
)
from python.common_systems import (
    PrevFinalPosSystem, GravitySystem, MovementSystem, AngularMovementSystem,
    PBDBallObstacleCollisions, PBDBallBallCollisions, PBDVelocityUpdateSystem,
    PrevFinalOrientationSystem, PBDAngularVelocityUpdateSystem, FlipperTipLinkSystem
)
from python.cable_attachment_update_system import CableAttachmentUpdateSystem
from python.pbd_cable_constraint_solver import PBDCableConstraintSolver
from python.geometry import right_of_line
from python.cable_joints_components import CableJointComponent, CablePathComponent, create_cable_path_component
from python.ball_obstacle_bump_system import BallObstacleBumpSystem
from python.ball_border_or_flipper_velocity_contact_system import BallBorderOrFlipperVelocityContactSystem
from python.cable_attachment_cache_system import CableAttachmentCacheSystem
from python.cable_slack_system import CableSlackSystem
from python.cable_friction_system import CableFrictionSystem

# Files that trigger a server restart when modified
python_dir = Path(__file__).parent
WATCHED_FILES = [
    Path(__file__),
    Path(__file__).with_name("flipper_scene_typed.usda"),
    python_dir / "python" / "ball_obstacle_bump_system.py",
    python_dir / "python" / "ball_border_or_flipper_velocity_contact_system.py",
]

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
                print(f"{f} changed, restarting server...")
                os.execv(sys.executable, [sys.executable] + sys.argv)
        for f in files:
            try:
                mtimes[f] = Path(f).stat().st_mtime
            except FileNotFoundError:
                mtimes[f] = 0

# --- Server-Side Systems ---

class PBDBallFlipperCollisions:
    def _get_flipper_tip(self, flipper_pos, flipper_state):
        angle = flipper_state.rest_angle + flipper_state.sign * flipper_state.rotation
        direction = np.array([np.cos(angle), np.sin(angle), 0.0])
        return flipper_pos + direction * flipper_state.length

    def _closest_point_on_segment(self, p, a, b):
        ab = b - a
        ap = p - a
        if np.dot(ab, ab) == 0: return a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + ab * t

    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent])
        flipper_entities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent])

        contacts = world.get_resource('ball_flipper_contacts')
        if contacts is None:
            contacts = []
            world.set_resource('ball_flipper_contacts', contacts)
        contacts.clear()

        for ball_id in ball_entities:
            p1_comp = world.get_component(ball_id, PositionComponent)
            p1 = p1_comp.pos
            r1 = world.get_component(ball_id, RadiusComponent).radius
            mass_comp = world.get_component(ball_id, MassComponent)
            inv_mass = 1.0 / mass_comp.mass if mass_comp and mass_comp.mass > 0 else 0.0

            for flip_id in flipper_entities:
                fp = world.get_component(flip_id, PositionComponent).pos
                fr = world.get_component(flip_id, RadiusComponent).radius
                fs = world.get_component(flip_id, FlipperStateComponent)

                tip = self._get_flipper_tip(fp, fs)
                closest = self._closest_point_on_segment(p1, fp, tip)

                direction = p1 - closest
                d_sq = np.dot(direction, direction)
                r_sum = r1 + fr

                if d_sq == 0.0 or d_sq > r_sum * r_sum:
                    continue

                d = np.sqrt(d_sq)
                direction /= d

                # Resolve penetration
                corr = r_sum - d
                if inv_mass > 0:
                    p1 += direction * corr

                # Calculate positional impulse for the velocity solver
                delta_lambda = 0
                if inv_mass > 0:
                    # Since the flipper is treated as having infinite mass in this positional solve,
                    # the generalized inverse mass is just the ball's inverse mass.
                    w_inv = inv_mass
                    delta_lambda = corr / w_inv

                # Store contact info for velocity system
                contacts.append({
                    'ball_id': ball_id,
                    'flip_id': flip_id,
                    'normal': direction.copy(),
                    'contact_point_on_flipper': closest.copy(),
                    'delta_lambda': delta_lambda
                })

class PBDBallBorderCollisions:
    def _closest_point_on_segment(self, p, a, b):
        ab = b - a
        ap = p - a
        if np.dot(ab, ab) == 0: return a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + ab * t

    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent])
        border_entities = world.query([BorderComponent])
        if not border_entities: return

        border_id = border_entities[0]
        border_comp = world.get_component(border_id, BorderComponent)
        border_points = border_comp.points

        # Get physics properties from the border entity
        border_restitution_comp = world.get_component(border_id, RestitutionComponent)
        border_friction_comp = world.get_component(border_id, CoefficientOfFrictionComponent)
        restitution = border_restitution_comp.restitution if border_restitution_comp else None
        friction = border_friction_comp.mu if border_friction_comp else None

        contacts = world.get_resource('ball_border_contacts')
        if contacts is None:
            contacts = []
            world.set_resource('ball_border_contacts', contacts)
        contacts.clear()

        for ball_id in ball_entities:
            p1_comp = world.get_component(ball_id, PositionComponent)
            p1 = p1_comp.pos
            r1 = world.get_component(ball_id, RadiusComponent).radius
            mass_comp = world.get_component(ball_id, MassComponent)
            inv_mass = 1.0 / mass_comp.mass if mass_comp and mass_comp.mass > 0 else 0.0

            min_dist_sq = float('inf')
            closest_seg_point = np.zeros(3)
            edge_start, edge_end = None, None

            for i in range(len(border_points)):
                a = border_points[i]
                b = border_points[(i + 1) % len(border_points)]
                closest_pt = self._closest_point_on_segment(p1, a, b)
                dist_sq = np.sum((p1 - closest_pt)**2)
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    closest_seg_point = closest_pt
                    edge_start, edge_end = a, b

            if min_dist_sq > r1 * r1: continue

            ball_to_closest = p1 - closest_seg_point

            edge_vec = edge_end - edge_start
            normal = np.array([-edge_vec[1], edge_vec[0], 0.0])
            normal /= np.linalg.norm(normal)

            if np.sum(ball_to_closest**2) < 1e-9:
                collision_normal = normal
            else:
                collision_normal = ball_to_closest / np.linalg.norm(ball_to_closest)

            if np.dot(ball_to_closest, normal) < 0:
                collision_normal = normal

            dist = np.sqrt(min_dist_sq)
            penetration = r1 - dist
            delta_lambda = 0
            if penetration > 0:
                if inv_mass > 0:
                    p1 += collision_normal * penetration
                    # Since the border is static, the generalized inverse mass is just the ball's.
                    w_inv = inv_mass
                    delta_lambda = penetration / w_inv

            # Store contact info for the velocity-based system
            contacts.append({
                'ball_id': ball_id,
                'normal': collision_normal.copy(),
                'delta_lambda': delta_lambda,
                'restitution': restitution,
                'friction': friction
            })


# --- Scene and World Setup ---
def setup_scene(world, use_warp=False, device="cpu"):
    stage = load_stage(Path(__file__).with_name("flipper_scene_typed.usda"))
    stage_to_world(stage, world)
    register_default_systems(world, use_warp=use_warp, device=device)


def world_to_json(world):
    state = {
        'balls': [], 'obstacles': [], 'flippers': [], 'border': [], 'score': 0, 'cables': []
    }
    for ball_id in world.query([BallTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(ball_id, PositionComponent).pos
        radius = world.get_component(ball_id, RadiusComponent).radius
        ball_data = {'x': pos[0], 'y': pos[1], 'radius': radius}
        orientation_comp = world.get_component(ball_id, OrientationComponent)
        if orientation_comp:
            ball_data['angle'] = orientation_comp.angle
        state['balls'].append(ball_data)

    for obs_id in world.query([ObstacleTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(obs_id, PositionComponent).pos
        radius = world.get_component(obs_id, RadiusComponent).radius
        renderable = world.get_component(obs_id, RenderableComponent)
        color = renderable.color if renderable else '#ffffff'
        obs_data = {'x': pos[0], 'y': pos[1], 'radius': radius, 'color': color}
        orientation_comp = world.get_component(obs_id, OrientationComponent)
        if orientation_comp:
            obs_data['angle'] = orientation_comp.angle
        state['obstacles'].append(obs_data)

    for flip_id in world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]):
        pos = world.get_component(flip_id, PositionComponent).pos
        radius = world.get_component(flip_id, RadiusComponent).radius
        f_state = world.get_component(flip_id, FlipperStateComponent)
        state['flippers'].append({
            'x': pos[0], 'y': pos[1], 'radius': radius, 'length': f_state.length,
            'angle': f_state.rest_angle + f_state.sign * f_state.rotation
        })

    border_query = world.query([BorderComponent])
    if border_query:
        border_id = border_query[0]
        border_points = world.get_component(border_id, BorderComponent).points
        state['border'] = [p.tolist()[:2] for p in border_points]

    score_query = world.query([ScoreComponent])
    if score_query:
        score_id = score_query[0]
        state['score'] = world.get_component(score_id, ScoreComponent).value

    pause_comp = world.get_resource('pauseState')
    if pause_comp:
        state['isPaused'] = pause_comp.paused
    else:
        state['isPaused'] = True

    # Serialize Cables
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if not path.joint_entities:
            continue

        cable_render_data = {
            'joints': [],
            'arcs': [],
            'links': []
        }

        # 1. Joints
        for joint_id in path.joint_entities:
            joint = world.get_component(joint_id, CableJointComponent)
            pA = joint.attachment_point_a_world.tolist()[:2]
            pB = joint.attachment_point_b_world.tolist()[:2]
            rest_length = joint.rest_length
            cable_render_data['joints'].append({'pA': pA, 'pB': pB, 'restLength': rest_length})

        # 2. Rolling arcs
        for i in range(1, len(path.link_types) - 1):
            if path.link_types[i] == 'rolling':
                j_prev_id = path.joint_entities[i - 1]
                j_next_id = path.joint_entities[i]
                j_prev = world.get_component(j_prev_id, CableJointComponent)
                j_next = world.get_component(j_next_id, CableJointComponent)

                roller_id = j_prev.entity_b
                center_comp = world.get_component(roller_id, PositionComponent)
                radius_comp = world.get_component(roller_id, RadiusComponent)
                if not center_comp or not radius_comp:
                    continue

                center = center_comp.pos.tolist()[:2]
                radius = radius_comp.radius
                p1 = j_prev.attachment_point_b_world.tolist()[:2]
                p2 = j_next.attachment_point_a_world.tolist()[:2]
                anticlockwise = not path.cw[i]

                epsilon = 1e-6
                dist_prev = np.linalg.norm(j_prev.attachment_point_a_world - j_prev.attachment_point_b_world)
                tension_prev = dist_prev > (j_prev.rest_length + epsilon)
                dist_next = np.linalg.norm(j_next.attachment_point_a_world - j_next.attachment_point_b_world)
                tension_next = dist_next > (j_next.rest_length + epsilon)
                is_taut = tension_prev and tension_next

                cable_render_data['arcs'].append({
                    'center': center,
                    'radius': radius,
                    'p1': p1,
                    'p2': p2,
                    'anticlockwise': anticlockwise,
                    'is_taut': bool(is_taut)
                })

        # 3. Hybrid link markers and arcs
        for i, link_type in enumerate(path.link_types):
            link_data = {"type": link_type}
            if link_type == 'hybrid-attachment':
                attachment_point = None
                if i == 0:
                    joint = world.get_component(path.joint_entities[0], CableJointComponent)
                    attachment_point = joint.attachment_point_a_world
                else:  # Last link
                    joint = world.get_component(path.joint_entities[-1], CableJointComponent)
                    attachment_point = joint.attachment_point_b_world

                if attachment_point is not None:
                    link_data['attachmentPoint'] = attachment_point.tolist()[:2]

            elif link_type == 'hybrid':
                roller_id = None
                tangent_point = None
                is_taut = False
                epsilon = 1e-6

                if i == 0:
                    joint = world.get_component(path.joint_entities[0], CableJointComponent)
                    roller_id = joint.entity_a
                    tangent_point = joint.attachment_point_a_world
                    dist = np.linalg.norm(joint.attachment_point_a_world - joint.attachment_point_b_world)
                    is_taut = dist > (joint.rest_length + epsilon)
                else:  # Last link
                    joint = world.get_component(path.joint_entities[-1], CableJointComponent)
                    roller_id = joint.entity_b
                    tangent_point = joint.attachment_point_b_world
                    dist = np.linalg.norm(joint.attachment_point_a_world - joint.attachment_point_b_world)
                    is_taut = dist > (joint.rest_length + epsilon)

                center_comp = world.get_component(roller_id, PositionComponent)
                radius_comp = world.get_component(roller_id, RadiusComponent)

                if center_comp and radius_comp and tangent_point is not None:
                    link_data['center'] = center_comp.pos.tolist()[:2]
                    link_data['radius'] = radius_comp.radius
                    link_data['tangentPoint'] = tangent_point.tolist()[:2]
                    link_data['storedLength'] = path.stored[i]
                    link_data['cw'] = path.cw[i]
                    link_data['is_taut'] = bool(is_taut)

            cable_render_data['links'].append(link_data)

        state['cables'].append(cable_render_data)

    return json.dumps(state)

# --- WebSocket Handler ---

async def handler(websocket, path=None, use_warp=False, device='cpu'):
    world = World()
    setup_scene(world, use_warp=use_warp, device=device)
    input_system = world.get_system(RemoteInputSystem)

    # Send initial state
    await websocket.send(world_to_json(world))

    async for message in websocket:
        data = json.loads(message)
        action = data.get('action')

        pause_state = world.get_resource('pauseState')

        if action == 'step':
            if not pause_state.paused:
                steps = data.get('steps', 0)
                if steps > 0:
                    dt = world.get_resource('dt')
                    for _ in range(steps):
                        world.update(dt)
        elif action == 'single_step':
            pause_state.paused = False
            world.update(world.get_resource('dt'))
            pause_state.paused = True
        elif action == 'reset':
            setup_scene(world, use_warp=use_warp, device=device)
            input_system = world.get_system(RemoteInputSystem)
        elif action == 'pause':
            pause_state.paused = data['paused']
        elif action == 'input':
            if input_system:
                pos = np.array([data['x'], data['y'], 0.0])
                event_type = data['type']

                if event_type in ['pointerdown', 'pointermove', 'pointerup']:
                    input_system.add_event({'type': event_type, 'pos': pos})
                elif event_type == 'click':
                    input_system.clicks.append(pos)
                elif event_type == 'release':
                    input_system.releases.append(pos)

        await websocket.send(world_to_json(world))

async def main():
    import websockets
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--warp", action="store_true", help="Use Warp solver")
    parser.add_argument("--device", default="cpu", help="Warp device")
    parser.add_argument("--port", type=int, help="WebSocket port")
    args = parser.parse_args()

    port = args.port if args.port else (8767 if args.warp else 8765)
    print(f"Starting WebSocket server on ws://localhost:{port}")

    serve_handler = functools.partial(handler, use_warp=args.warp, device=args.device)

    async with websockets.serve(serve_handler, "localhost", port):
        asyncio.create_task(watch_and_restart(WATCHED_FILES))
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
