import math
from pathlib import Path
from typing import Tuple, Optional

import numpy as np
from pxr import Usd, UsdGeom, UsdShade

from dataclasses import dataclass
from .geometry import right_of_line

from .ecs import (
    World, PauseStateComponent, PositionComponent, VelocityComponent, RadiusComponent,
    MassComponent, RestitutionComponent, GravityAffectedComponent, OrientationComponent,
    AngularVelocityComponent, MomentOfInertiaComponent, ObstacleTagComponent,
    FlipperTagComponent, FlipperStateComponent, FlipperTipComponent, BorderComponent,
    RenderableComponent, PrevFinalPosComponent, PrevFinalOrientationComponent,
    BallTagComponent, ObstaclePushComponent, CableLinkComponent, CoefficientOfFrictionComponent,
    ScoredTagComponent
)
from .cable_joints_components import CableJointComponent, create_cable_path_component, CablePathComponent
from .common_systems import (
    PrevFinalPosSystem, GravitySystem, MovementSystem, AngularMovementSystem,
    PBDBallObstacleCollisions, PBDBallBallCollisions, PBDVelocityUpdateSystem,
    PrevFinalOrientationSystem, PBDAngularVelocityUpdateSystem, FlipperTipLinkSystem
)
from .cable_attachment_update_system import CableAttachmentUpdateSystem
from .pbd_cable_constraint_solver import PBDCableConstraintSolver
from .ball_obstacle_bump_system import BallObstacleBumpSystem
from .ball_border_or_flipper_velocity_contact_system import BallBorderOrFlipperVelocityContactSystem
from .cable_attachment_cache_system import CableAttachmentCacheSystem
from .cable_slack_system import CableSlackSystem
from .cable_friction_system import CableFrictionSystem


class FlipperMotionSystem:
    run_in_pause = True

    def update(self, world: World, dt: float) -> None:
        for eid in world.query([FlipperStateComponent]):
            state = world.get_component(eid, FlipperStateComponent)
            prev_rot = state.rotation
            if state.pressed:
                state.rotation = min(state.rotation + dt * state.angular_velocity, state.max_rotation)
            else:
                state.rotation = max(state.rotation - dt * state.angular_velocity, 0.0)
            if dt > 1e-6:
                state.current_angular_velocity = state.sign * (state.rotation - prev_rot) / dt
            else:
                state.current_angular_velocity = 0.0


class RemoteInputSystem:
    run_in_pause = True

    def __init__(self) -> None:
        self.clicks = []
        self.releases = []
        self.grab_spring = None
        self.events_queue = []

    def add_event(self, event) -> None:
        self.events_queue.append(event)

    def _handle_pointer_down(self, world: World, pos: np.ndarray) -> None:
        if self.grab_spring:
            return

        closest_ball = None
        closest_dist_sq = float('inf')
        for ball_id in world.query([BallTagComponent, PositionComponent, RadiusComponent]):
            ball_pos = world.get_component(ball_id, PositionComponent).pos
            radius = world.get_component(ball_id, RadiusComponent).radius
            dist_sq = np.sum((pos - ball_pos) ** 2)
            if dist_sq <= radius ** 2 and dist_sq < closest_dist_sq:
                closest_ball = ball_id
                closest_dist_sq = dist_sq

        if closest_ball is not None:
            ptr_e = world.create_entity()
            world.add_component(ptr_e, PositionComponent(pos.copy()))
            world.add_component(ptr_e, CableLinkComponent(prev_cable_attachment_time_pos=pos.copy()))

            ball_pos = world.get_component(closest_ball, PositionComponent).pos
            joint_e = world.create_entity()
            world.add_component(
                joint_e,
                CableJointComponent(
                    entity_a=closest_ball,
                    entity_b=ptr_e,
                    rest_length=0.1,
                    attachment_point_a_world=ball_pos.copy(),
                    attachment_point_b_world=pos.copy(),
                ),
            )

            path_e = world.create_entity()
            path_comp = create_cable_path_component(
                world,
                joint_entities=[joint_e],
                link_types=['attachment', 'attachment'],
                cw=[True, True],
                spring_constant=10.0,
            )
            world.add_component(path_e, path_comp)

            self.grab_spring = {'ptr_e': ptr_e, 'joint_e': joint_e, 'path_e': path_e, 'ball_e': closest_ball}
            world.set_resource('grabbedBall', closest_ball)

            pause_state = world.get_resource('pauseState')
            if pause_state:
                pause_state.paused = False
        else:
            self.clicks.append(pos)

    def _handle_pointer_move(self, world: World, pos: np.ndarray) -> None:
        if self.grab_spring:
            ptr_e = self.grab_spring['ptr_e']
            ptr_pos_comp = world.get_component(ptr_e, PositionComponent)
            if ptr_pos_comp:
                ptr_pos_comp.pos[:] = pos

    def _handle_pointer_up(self, world: World, pos: np.ndarray) -> None:
        if self.grab_spring:
            ball_e = self.grab_spring['ball_e']
            vel_comp = world.get_component(ball_e, VelocityComponent)
            pos_comp = world.get_component(ball_e, PositionComponent)
            prev_final_pos_comp = world.get_component(ball_e, PrevFinalPosComponent)
            dt = world.get_resource('dt')

            if vel_comp and pos_comp and prev_final_pos_comp and dt > 1e-9:
                np.subtract(pos_comp.pos, prev_final_pos_comp.pos, out=vel_comp.vel)
                np.divide(vel_comp.vel, dt, out=vel_comp.vel)

            world.destroy_entity(self.grab_spring['path_e'])
            world.destroy_entity(self.grab_spring['joint_e'])
            world.destroy_entity(self.grab_spring['ptr_e'])
            self.grab_spring = None
            world.set_resource('grabbedBall', None)
        else:
            self.releases.append(pos)

    def update(self, world: World, dt: float) -> None:
        for event in self.events_queue:
            event_type = event.get('type')
            pos = event.get('pos')
            if event_type == 'pointerdown':
                self._handle_pointer_down(world, pos)
            elif event_type == 'pointermove':
                self._handle_pointer_move(world, pos)
            elif event_type == 'pointerup':
                self._handle_pointer_up(world, pos)
        self.events_queue.clear()

        if self.clicks:
            click_pos = self.clicks.pop(0)
            border_ents = world.query([BorderComponent])
            if not border_ents:
                return
            border_points = world.get_component(border_ents[0], BorderComponent).points

            right_click = right_of_line(click_pos, border_points[0], border_points[1]) and \
                          right_of_line(click_pos, border_points[1], border_points[2])
            left_click = right_of_line(click_pos, border_points[5], border_points[6]) and \
                         right_of_line(click_pos, border_points[6], border_points[7])

            flipper_entities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent])
            if not flipper_entities:
                return

            flipper_data = [{'id': fid, 'x': world.get_component(fid, PositionComponent).pos[0]} for fid in flipper_entities]
            flipper_data.sort(key=lambda f: f['x'])
            left_flipper_id = flipper_data[0]['id']
            right_flipper_id = flipper_data[-1]['id']

            if right_click:
                world.get_component(right_flipper_id, FlipperStateComponent).pressed = True
            elif left_click:
                world.get_component(left_flipper_id, FlipperStateComponent).pressed = True
            else:
                closest_flipper_id, min_dist_sq = None, float('inf')
                for fid in flipper_entities:
                    fp = world.get_component(fid, PositionComponent).pos
                    dist_sq = np.sum((click_pos - fp) ** 2)
                    if dist_sq < min_dist_sq:
                        min_dist_sq, closest_flipper_id = dist_sq, fid
                if closest_flipper_id is not None:
                    state = world.get_component(closest_flipper_id, FlipperStateComponent)
                    if min_dist_sq < state.length ** 2:
                        state.pressed = True

        if self.releases:
            release_pos = self.releases.pop(0)
            flipper_entities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent])
            closest_id, min_dist_sq = None, float('inf')
            for fid in flipper_entities:
                state = world.get_component(fid, FlipperStateComponent)
                if not state.pressed:
                    continue
                fp = world.get_component(fid, PositionComponent).pos
                d2 = np.sum((release_pos - fp) ** 2)
                if d2 < min_dist_sq:
                    min_dist_sq, closest_id = d2, fid
            if closest_id is not None:
                world.get_component(closest_id, FlipperStateComponent).pressed = False


@dataclass
class ScoreComponent:
    value: int = 0


class ScoreSystem:
    run_in_pause = False

    def update(self, world: World, dt: float) -> None:
        score_entities = world.query([ScoreComponent])
        if not score_entities:
            return
        score_entity = score_entities[0]
        score_comp = world.get_component(score_entity, ScoreComponent)
        for eid in world.query([ScoredTagComponent]):
            world.remove_component(eid, ScoredTagComponent)
            score_comp.value += 1


def load_stage(path: Path) -> Usd.Stage:
    """Load the USD stage at *path*."""
    return Usd.Stage.Open(str(path))


def _quat_z_angle(quat) -> float:
    if quat is None:
        return 0.0
    return float(2.0 * math.atan2(quat[3], quat[0]))


def _material_props(stage: Usd.Stage, prim) -> Tuple[str, Optional[float], Optional[float]]:
    """Return (color_hex, friction, restitution) for *prim*."""
    api = UsdShade.MaterialBindingAPI(prim)
    rel = api.GetDirectBindingRel()
    color = "#ffffff"
    friction = None
    restitution = None
    if rel and rel.GetTargets():
        mat_prim = stage.GetPrimAtPath(rel.GetTargets()[0])
        if mat_prim:
            shader_prim = mat_prim.GetPrimAtPath("Shader")
            if shader_prim:
                c_attr = shader_prim.GetAttribute("inputs:diffuseColor")
                if c_attr and c_attr.Get() is not None:
                    c = c_attr.Get()
                    color = "#%02x%02x%02x" % (int(c[0]*255), int(c[1]*255), int(c[2]*255))
            f_attr = mat_prim.GetAttribute("physics:staticFriction")
            if f_attr and f_attr.Get() is not None:
                friction = f_attr.Get()
            r_attr = mat_prim.GetAttribute("physics:restitution")
            if r_attr and r_attr.Get() is not None:
                restitution = r_attr.Get()
    return color, friction, restitution


def stage_to_world(stage: Usd.Stage, world: World) -> None:
    """Populate *world* with entities and resources from *stage*."""
    world.set_resource('pauseState', PauseStateComponent(True))
    world.set_resource('debugRenderPoints', {})
    world.set_resource('grabbedBall', None)
    world.set_resource('ball_obstacle_contacts', [])
    world.set_resource('ball_border_contacts', [])
    world.set_resource('ball_flipper_contacts', [])

    # dt from stage metadata
    tps = stage.GetTimeCodesPerSecond()
    if tps:
        world.set_resource('dt', 1.0 / float(tps))
    # gravity
    phys = stage.GetPrimAtPath("/World/PhysicsScene")
    if phys:
        dir_attr = phys.GetAttribute("physics:gravityDirection")
        mag_attr = phys.GetAttribute("physics:gravityMagnitude")
        g_dir = np.array(dir_attr.Get()) if dir_attr and dir_attr.Get() is not None else np.array([0.0, -1.0, 0.0])
        g_mag = mag_attr.Get() if mag_attr and mag_attr.Get() is not None else 0.0
        world.set_resource('gravity', g_dir * g_mag)

    # Border
    border_prim = stage.GetPrimAtPath("/World/FlipperScene/Border")
    if border_prim:
        mesh = UsdGeom.Mesh(border_prim)
        pts = np.array(mesh.GetPointsAttr().Get())
        border_pts = pts[:8]
        border_e = world.create_entity()
        world.add_component(border_e, BorderComponent(border_pts))
        color, friction, restitution = _material_props(stage, border_prim)
        world.add_component(border_e, RenderableComponent('border', color))
        if restitution is not None:
            world.add_component(border_e, RestitutionComponent(restitution))
        if friction is not None:
            world.add_component(border_e, CoefficientOfFrictionComponent(friction))
        min_x, max_x = border_pts[:,0].min(), border_pts[:,0].max()
        min_y, max_y = border_pts[:,1].min(), border_pts[:,1].max()
        world.set_resource('simWidth', max_x - min_x)
        world.set_resource('simHeight', max_y - min_y)

    # Balls
    name_to_entity = {}
    for name in ["Ball1", "Ball2"]:
        prim = stage.GetPrimAtPath(f"/World/FlipperScene/{name}")
        if not prim:
            continue
        xf = UsdGeom.Xformable(prim)
        pos = np.array([0.0, 0.0, 0.0])
        for op in xf.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                pos = np.array(op.Get())
        radius = prim.GetAttribute("radius").Get()
        mass = prim.GetAttribute("physics:mass").Get()
        orient_attr = prim.GetAttribute("xformOp:orient")
        angle = _quat_z_angle(orient_attr.Get() if orient_attr else None)
        inertia = prim.GetAttribute("physics:inertiaTensor").Get()
        moi = inertia[2][2] if inertia is not None else 0.5 * mass * radius**2
        color, friction, restitution = _material_props(stage, prim)
        ang_vel_attr = prim.GetAttribute("physics:angularVelocity")
        ang_vel = np.deg2rad(ang_vel_attr.Get()[2]) if ang_vel_attr and ang_vel_attr.Get() is not None else 0.0

        ent = world.create_entity()
        world.add_component(ent, BallTagComponent())
        world.add_component(ent, PositionComponent(pos.copy()))
        world.add_component(ent, VelocityComponent(np.zeros(3)))
        world.add_component(ent, RadiusComponent(radius))
        world.add_component(ent, MassComponent(mass))
        world.add_component(ent, GravityAffectedComponent())
        world.add_component(ent, RenderableComponent('circle', color))
        world.add_component(ent, OrientationComponent(angle))
        world.add_component(ent, AngularVelocityComponent(ang_vel))
        world.add_component(ent, MomentOfInertiaComponent(moi))
        world.add_component(ent, PrevFinalOrientationComponent(angle))
        world.add_component(ent, PrevFinalPosComponent(pos.copy()))
        if restitution is not None:
            world.add_component(ent, RestitutionComponent(restitution))
        if friction is not None:
            world.add_component(ent, CoefficientOfFrictionComponent(friction))
        link_attr = prim.GetAttribute('cable:linkable')
        if link_attr and link_attr.Get():
            world.add_component(ent, CableLinkComponent(prev_cable_attachment_time_pos=pos.copy()))
        name_to_entity[name] = ent

    # Obstacles
    for i in range(1,5):
        name = f"Obs{i}"
        prim = stage.GetPrimAtPath(f"/World/FlipperScene/{name}")
        if not prim:
            continue
        xf = UsdGeom.Xformable(prim)
        pos = np.array([0.0,0.0,0.0])
        for op in xf.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                pos = np.array(op.Get())
        radius = prim.GetAttribute("radius").Get()
        color, friction, restitution = _material_props(stage, prim)
        ang_vel_attr = prim.GetAttribute("physics:angularVelocity")
        ang_vel = np.deg2rad(ang_vel_attr.Get()[2]) if ang_vel_attr and ang_vel_attr.Get() is not None else 0.0
        inertia_attr = prim.GetAttribute("physics:inertiaTensor")
        moi = inertia_attr.Get()[2][2] if inertia_attr and inertia_attr.Get() is not None else 0.020 * radius**2
        push_attr = prim.GetAttribute("obstacle:pushVel")
        push_vel = push_attr.Get() if push_attr and push_attr.Get() is not None else 0.0
        ent = world.create_entity()
        world.add_component(ent, ObstacleTagComponent())
        world.add_component(ent, PositionComponent(pos.copy()))
        world.add_component(ent, MassComponent(-1.0))
        world.add_component(ent, RadiusComponent(radius))
        world.add_component(ent, ObstaclePushComponent(push_vel))
        world.add_component(ent, RenderableComponent('circle', color))
        if friction is not None:
            world.add_component(ent, CoefficientOfFrictionComponent(friction))
        if ang_vel != 0.0:
            world.add_component(ent, OrientationComponent(0.0))
            world.add_component(ent, AngularVelocityComponent(ang_vel))
            world.add_component(ent, MomentOfInertiaComponent(moi))
            world.add_component(ent, PrevFinalOrientationComponent(0.0))
        name_to_entity[name] = ent

    # Flippers
    for name in ["Flipper1", "Flipper2"]:
        prim = stage.GetPrimAtPath(f"/World/FlipperScene/{name}")
        if not prim:
            continue
        xf = UsdGeom.Xformable(prim)
        pos = np.array([0.0,0.0,0.0])
        for op in xf.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                pos = np.array(op.Get())
        color, friction, restitution = _material_props(stage, prim)
        length = prim.GetAttribute("flipper:length").Get()
        rest_angle = prim.GetAttribute("flipper:restAngle").Get()
        max_rot = prim.GetAttribute("flipper:maxRotation").Get()
        ang_vel = prim.GetAttribute("flipper:angularVelocity").Get()
        geom = prim.GetChild("Geom")
        radius_attr = geom.GetAttribute("radius") if geom else None
        radius = radius_attr.Get() if radius_attr and radius_attr.Get() is not None else 0.03
        ent = world.create_entity()
        world.add_component(ent, FlipperTagComponent())
        world.add_component(ent, PositionComponent(pos.copy()))
        world.add_component(ent, RadiusComponent(radius))
        world.add_component(ent, FlipperStateComponent(length, rest_angle, max_rot, ang_vel))
        if restitution is not None:
            world.add_component(ent, RestitutionComponent(restitution))
        world.add_component(ent, RenderableComponent('flipper', color))
        link_attr = prim.GetAttribute('cable:linkable')
        if link_attr and link_attr.Get():
            world.add_component(ent, CableLinkComponent(prev_cable_attachment_time_pos=pos.copy()))
        tip = prim.GetChild("Tip")
        if tip:
            tip_pos_attr = tip.GetAttribute("xformOp:translate")
            tip_offset = np.array(tip_pos_attr.Get()) if tip_pos_attr and tip_pos_attr.Get() is not None else np.array([length,0,0])
            tip_e = world.create_entity()
            world.add_component(tip_e, PositionComponent())
            world.add_component(tip_e, RadiusComponent(radius))
            world.add_component(tip_e, FlipperTipComponent(ent))
            world.add_component(tip_e, CableLinkComponent())
            if friction is not None:
                world.add_component(tip_e, CoefficientOfFrictionComponent(friction))
        name_to_entity[name] = ent

    # Score entity
    score_ent = world.create_entity()
    world.add_component(score_ent, ScoreComponent())

    # Cable joints and path
    cable_path_prim = stage.GetPrimAtPath("/World/FlipperScene/CablePath")
    if cable_path_prim:
        joint_rels = cable_path_prim.GetRelationship("cablePath:joints").GetTargets()
        joint_entities = []
        for jpath in joint_rels:
            jprim = stage.GetPrimAtPath(jpath)
            if not jprim:
                continue
            b0 = jprim.GetRelationship("physics:body0").GetTargets()[0].name
            b1 = jprim.GetRelationship("physics:body1").GetTargets()[0].name
            ent_a = name_to_entity.get(b0)
            ent_b = name_to_entity.get(b1)
            if ent_a is None or ent_b is None:
                continue
            attach_a = np.array(jprim.GetAttribute("localPos0").Get())
            attach_b = np.array(jprim.GetAttribute("localPos1").Get())
            rest_len = jprim.GetAttribute("restLength").Get()
            je = world.create_entity()
            world.add_component(je, CableJointComponent(ent_a, ent_b, rest_len, attach_a, attach_b))
            joint_entities.append(je)
        link_types = list(cable_path_prim.GetAttribute("cablePath:linkTypes").Get() or [])
        cw = list(cable_path_prim.GetAttribute("cablePath:clockwise").Get() or [])
        stored = list(cable_path_prim.GetAttribute("cablePath:stored").Get() or [])
        stiffness_attr = cable_path_prim.GetAttribute("stiffness")
        stiffness = stiffness_attr.Get() if stiffness_attr and stiffness_attr.Get() is not None else 1e6
        if joint_entities:
            path_e = world.create_entity()
            comp = create_cable_path_component(world, joint_entities, link_types, cw, stiffness, stored=stored)
            world.add_component(path_e, comp)


def register_default_systems(world: World, use_warp: bool = False, device: str = 'cpu') -> None:
    """Register the default flipper systems on *world* if none are registered."""
    if world.systems:
        return

    world.register_system(PrevFinalPosSystem())
    world.register_system(PrevFinalOrientationSystem())
    world.register_system(RemoteInputSystem())
    world.register_system(FlipperMotionSystem())
    world.register_system(GravitySystem())
    world.register_system(MovementSystem())
    world.register_system(AngularMovementSystem())
    world.register_system(FlipperTipLinkSystem())
    world.register_system(CableAttachmentUpdateSystem())
    world.register_system(CableAttachmentCacheSystem())
    world.register_system(CableSlackSystem())
    solver = PBDCableConstraintSolver() if not use_warp else __import__('python_warp.cable_solver_warp', fromlist=['WarpCableConstraintSolver']).WarpCableConstraintSolver(device)
    world.register_system(solver)
    world.register_system(PBDBallBorderCollisions())
    world.register_system(PBDBallBallCollisions())
    world.register_system(PBDBallObstacleCollisions())
    world.register_system(PBDBallFlipperCollisions())
    world.register_system(CableFrictionSystem())
    world.register_system(PBDVelocityUpdateSystem())
    world.register_system(PBDAngularVelocityUpdateSystem())
    world.register_system(BallObstacleBumpSystem())
    world.register_system(BallBorderOrFlipperVelocityContactSystem())
    world.register_system(ScoreSystem())
