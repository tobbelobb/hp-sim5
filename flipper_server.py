import asyncio
import json
import numpy as np
import websockets

# Assuming the python ECS code is in a 'python' directory
from python.ecs import (
    World, PauseStateComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent,
    RestitutionComponent, GravityAffectedComponent, OrientationComponent, AngularVelocityComponent,
    MomentOfInertiaComponent, ObstacleTagComponent, ScoredTagComponent, FlipperTagComponent,
    FlipperStateComponent, BorderComponent, RenderableComponent, PrevFinalPosComponent,
    PrevFinalOrientationComponent, BallTagComponent, ObstaclePushComponent
)
from python.common_systems import (
    PrevFinalPosSystem, GravitySystem, MovementSystem, AngularMovementSystem,
    PBDBallObstacleCollisions, PBDBallBallCollisions, PBDVelocityUpdateSystem,
    PrevFinalOrientationSystem, PBDAngularVelocityUpdateSystem
)
from python.geometry import right_of_line

# --- Server-Side Systems ---

class ScoreComponent:
    def __init__(self, score=0):
        self.value = score

class FlipperMotionSystem:
    run_in_pause = True
    def update(self, world, dt):
        for entity_id in world.query([FlipperStateComponent]):
            state = world.get_component(entity_id, FlipperStateComponent)
            prev_rotation = state.rotation
            if state.pressed:
                state.rotation = min(state.rotation + dt * state.angular_velocity, state.max_rotation)
            else:
                state.rotation = max(state.rotation - dt * state.angular_velocity, 0.0)
            state.current_angular_velocity = state.sign * (state.rotation - prev_rotation) / dt if dt > 1e-6 else 0.0

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
        ball_entities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent])
        flipper_entities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent])

        for ball_id in ball_entities:
            p1 = world.get_component(ball_id, PositionComponent).pos
            v1 = world.get_component(ball_id, VelocityComponent).vel
            r1 = world.get_component(ball_id, RadiusComponent).radius

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
                p1 += direction * corr

                # Resolve velocity
                radius_vec = closest - fp
                contact_point_on_flipper = fp + radius_vec + direction * -fr
                radius_to_surface = contact_point_on_flipper - fp

                surface_vel = np.array([-fs.current_angular_velocity * radius_to_surface[1], fs.current_angular_velocity * radius_to_surface[0], 0.0])

                v_dot = np.dot(v1, direction)
                surf_vel_dot = np.dot(surface_vel, direction)

                v1 += direction * (surf_vel_dot - v_dot)

class PBDBallBorderCollisions:
    def _closest_point_on_segment(self, p, a, b):
        ab = b - a
        ap = p - a
        if np.dot(ab, ab) == 0: return a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + ab * t

    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, RestitutionComponent])
        border_entities = world.query([BorderComponent])
        if not border_entities: return

        border_comp = world.get_component(border_entities[0], BorderComponent)
        border_points = border_comp.points

        for ball_id in ball_entities:
            p1 = world.get_component(ball_id, PositionComponent).pos
            v1 = world.get_component(ball_id, VelocityComponent).vel
            r1 = world.get_component(ball_id, RadiusComponent).radius
            res1 = world.get_component(ball_id, RestitutionComponent).restitution

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
            if np.sum(ball_to_closest**2) < 1e-9:
                edge_vec = edge_end - edge_start
                collision_normal = np.array([-edge_vec[1], edge_vec[0], 0.0])
                collision_normal /= np.linalg.norm(collision_normal)
            else:
                collision_normal = ball_to_closest / np.linalg.norm(ball_to_closest)

            dist = np.sqrt(min_dist_sq)
            penetration = r1 - dist
            if penetration > 0:
                p1 += collision_normal * penetration

            v_dot = np.dot(v1, collision_normal)
            if v_dot < 0:
                v_new_dot = -v_dot * res1
                v1 += collision_normal * (v_new_dot - v_dot)

class ScoreSystem:
    run_in_pause = False
    def update(self, world, dt):
        score_entity_query = world.query([ScoreComponent])
        if not score_entity_query: return
        score_entity = score_entity_query[0]
        score_comp = world.get_component(score_entity, ScoreComponent)
        scored_entities = world.query([ScoredTagComponent])
        for scored_id in scored_entities:
            world.remove_component(scored_id, ScoredTagComponent)
            score_comp.value += 1

class RemoteInputSystem:
    run_in_pause = True
    def __init__(self):
        self.clicks = []
        self.releases = []

    def update(self, world, dt):
        if self.clicks:
            click_pos = self.clicks.pop(0)
            border_ents = world.query([BorderComponent])
            if not border_ents: return
            border_points = world.get_component(border_ents[0], BorderComponent).points

            right_click = right_of_line(click_pos, border_points[0], border_points[1]) and \
                          right_of_line(click_pos, border_points[1], border_points[2])
            left_click = right_of_line(click_pos, border_points[5], border_points[6]) and \
                         right_of_line(click_pos, border_points[6], border_points[7])

            flipper_entities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent])
            if not flipper_entities: return

            # Identify left and right flippers robustly
            flipper_data = []
            for fid in flipper_entities:
                pos = world.get_component(fid, PositionComponent).pos
                flipper_data.append({'id': fid, 'x': pos[0]})

            flipper_data.sort(key=lambda f: f['x'])
            left_flipper_id = flipper_data[0]['id']
            right_flipper_id = flipper_data[-1]['id']

            if right_click:
                world.get_component(right_flipper_id, FlipperStateComponent).pressed = True
            elif left_click:
                world.get_component(left_flipper_id, FlipperStateComponent).pressed = True
            else:
                # Direct click on a flipper - find the closest one
                closest_flipper_id = None
                min_dist_sq = float('inf')
                for fid in flipper_entities:
                    pos = world.get_component(fid, PositionComponent).pos
                    dist_sq = np.sum((click_pos - pos)**2)
                    if dist_sq < min_dist_sq:
                        min_dist_sq = dist_sq
                        closest_flipper_id = fid

                if closest_flipper_id is not None:
                    state = world.get_component(closest_flipper_id, FlipperStateComponent)
                    # Check if click is within flipper's radius of influence (e.g., its length)
                    if min_dist_sq < state.length**2:
                         world.get_component(closest_flipper_id, FlipperStateComponent).pressed = True

        if self.releases:
            release_pos = self.releases.pop(0)
            flipper_entities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent])
            closest_id = None
            min_dist_sq = float('inf')
            for id in flipper_entities:
                state = world.get_component(id, FlipperStateComponent)
                if not state.pressed: continue
                pos = world.get_component(id, PositionComponent).pos
                d2 = np.sum((release_pos - pos)**2)
                if d2 < min_dist_sq:
                    min_dist_sq = d2
                    closest_id = id
            if closest_id is not None:
                world.get_component(closest_id, FlipperStateComponent).pressed = False

# --- Scene and World Setup ---

def setup_scene(world):
    world.clear()

    sim_height = 1.7
    sim_width = 1.0 # Aspect ratio 1/1.7, will be updated by client

    world.set_resource('gravity', np.array([0.0, -2.0, 0.0]))
    world.set_resource('dt', 1.0 / 300.0)
    world.set_resource('simWidth', sim_width)
    world.set_resource('simHeight', sim_height)
    world.set_resource('pauseState', PauseStateComponent(True))

    offset = 0.02
    border_points = [
        np.array([0.74, 0.0, 0.0]), np.array([0.74, 0.25, 0.0]),
        np.array([1.0 - offset, 0.4, 0.0]), np.array([1.0 - offset, sim_height - offset, 0.0]),
        np.array([offset, sim_height - offset, 0.0]), np.array([offset, 0.4, 0.0]),
        np.array([0.26, 0.25, 0.0]), np.array([0.26, 0.0, 0.0])
    ]
    border_entity = world.create_entity()
    world.add_component(border_entity, BorderComponent(border_points))
    world.add_component(border_entity, RenderableComponent('border', '#000000'))

    ball_radius = 0.03
    ball_mass = np.pi * ball_radius**2
    ball_restitution = 0.4

    for pos in [(0.90, 0.95), (0.08, 0.5)]:
        ball = world.create_entity()
        world.add_component(ball, BallTagComponent())
        world.add_component(ball, PositionComponent(np.array([pos[0], pos[1], 0.0])))
        world.add_component(ball, VelocityComponent(np.zeros(3)))
        world.add_component(ball, RadiusComponent(ball_radius))
        world.add_component(ball, MassComponent(ball_mass))
        world.add_component(ball, RestitutionComponent(ball_restitution))
        world.add_component(ball, GravityAffectedComponent())
        world.add_component(ball, RenderableComponent('circle', '#a0a0a0'))
        world.add_component(ball, OrientationComponent(0.0))
        world.add_component(ball, AngularVelocityComponent(0.0))
        world.add_component(ball, MomentOfInertiaComponent(0.5 * ball_mass * ball_radius**2))
        world.add_component(ball, PrevFinalOrientationComponent(0.0))
        world.add_component(ball, PrevFinalPosComponent(np.array([pos[0], pos[1], 0.0])))

    obs_push = 2.7
    obstacles_data = [
        (0.25, 0.6, 0.1, "#0F7090", 0), (0.75, 0.5, 0.1, "#0F7090", 0),
        (0.7, 1.0, 0.12, "#FF8000", 200.0), (0.2, 1.2, 0.1, "#FF8000", -200.0)
    ]
    for x, y, r, color, ang_vel in obstacles_data:
        obs = world.create_entity()
        world.add_component(obs, ObstacleTagComponent())
        world.add_component(obs, PositionComponent(np.array([x, y, 0.0])))
        world.add_component(obs, MassComponent(-1.0))
        world.add_component(obs, RadiusComponent(r))
        world.add_component(obs, ObstaclePushComponent(obs_push))
        world.add_component(obs, RenderableComponent('circle', color))
        if ang_vel != 0:
            world.add_component(obs, OrientationComponent(0.0))
            world.add_component(obs, AngularVelocityComponent(ang_vel))
            world.add_component(obs, MomentOfInertiaComponent(0.020 * r**2))
            world.add_component(obs, PrevFinalOrientationComponent(0.0))

    flip_radius, flip_length, flip_max_rot, flip_ang_vel = 0.03, 0.2, 1.0, 20.0
    flipper1 = world.create_entity()
    world.add_component(flipper1, FlipperTagComponent())
    world.add_component(flipper1, PositionComponent(np.array([0.26, 0.22, 0.0])))
    world.add_component(flipper1, RadiusComponent(flip_radius))
    world.add_component(flipper1, FlipperStateComponent(flip_length, -0.5, flip_max_rot, flip_ang_vel))
    world.add_component(flipper1, RenderableComponent('flipper', '#FF0000'))

    flipper2 = world.create_entity()
    world.add_component(flipper2, FlipperTagComponent())
    world.add_component(flipper2, PositionComponent(np.array([0.74, 0.22, 0.0])))
    world.add_component(flipper2, RadiusComponent(flip_radius))
    world.add_component(flipper2, FlipperStateComponent(flip_length, np.pi + 0.5, -flip_max_rot, flip_ang_vel, sign: -1))
    world.add_component(flipper2, RenderableComponent('flipper', '#FF0000'))

    score_entity = world.create_entity()
    world.add_component(score_entity, ScoreComponent(0))

    if not world.systems:
        world.register_system(PrevFinalPosSystem())
        world.register_system(PrevFinalOrientationSystem())
        world.register_system(RemoteInputSystem())
        world.register_system(FlipperMotionSystem())
        world.register_system(GravitySystem())
        world.register_system(MovementSystem())
        world.register_system(AngularMovementSystem())
        world.register_system(PBDVelocityUpdateSystem())
        world.register_system(PBDAngularVelocityUpdateSystem())
        world.register_system(PBDBallBorderCollisions())
        world.register_system(PBDBallBallCollisions())
        world.register_system(PBDBallObstacleCollisions())
        world.register_system(PBDBallFlipperCollisions())
        world.register_system(ScoreSystem())

def world_to_json(world):
    state = {
        'balls': [], 'obstacles': [], 'flippers': [], 'border': [], 'score': 0
    }
    for ball_id in world.query([BallTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(ball_id, PositionComponent).pos
        radius = world.get_component(ball_id, RadiusComponent).radius
        state['balls'].append({'x': pos[0], 'y': pos[1], 'radius': radius})

    for obs_id in world.query([ObstacleTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(obs_id, PositionComponent).pos
        radius = world.get_component(obs_id, RadiusComponent).radius
        renderable = world.get_component(obs_id, RenderableComponent)
        color = renderable.color if renderable else '#ffffff'
        state['obstacles'].append({'x': pos[0], 'y': pos[1], 'radius': radius, 'color': color})

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

    return json.dumps(state)

# --- WebSocket Handler ---

async def handler(websocket):
    world = World()
    setup_scene(world)

    # Send initial state
    await websocket.send(world_to_json(world))

    async for message in websocket:
        data = json.loads(message)
        action = data.get('action')

        pause_state = world.get_resource('pauseState')

        if action == 'step':
            if not pause_state.paused:
                # The JS client sends dt in ms, the python sim expects seconds
                world.update(data['dt'] / 1000.0)
        elif action == 'single_step':
            pause_state.paused = False
            world.update(world.get_resource('dt'))
            pause_state.paused = True
        elif action == 'reset':
            setup_scene(world)
        elif action == 'pause':
            pause_state.paused = data['paused']
        elif action == 'input':
            input_system = world.get_system(RemoteInputSystem)
            if input_system:
                pos = np.array([data['x'], data['y'], 0.0])
                if data['type'] == 'click':
                    input_system.clicks.append(pos)
                elif data['type'] == 'release':
                    input_system.releases.append(pos)

        await websocket.send(world_to_json(world))

async def main():
    print("Starting WebSocket server on ws://localhost:8765")
    async with websockets.serve(handler, "localhost", 8765):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
