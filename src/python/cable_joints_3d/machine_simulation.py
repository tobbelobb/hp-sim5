"""Headless composition root for the specialized Hangprinter simulation."""
from usd.cable_scene_loader import open_cable_scene

from . import common_systems as common
from .cable_attachment_cache_system import CableAttachmentCacheSystem
from .cable_attachment_update_system import CableAttachmentUpdateSystem
from .cable_friction_system import CableFrictionSystem
from .ecs import RigidBodyComponent, World
from .extruder import ExtruderSystem
from .machine_scene import populate_machine_scene
from .motor_diagnostics import MissedStepTrackingSystem
from .pbd_cable_constraint_solver import PBDCableConstraintSolver
from .pbd_resolve_cable_over_corrections import PBDResolveCableOverCorrections
from .remote_spool_system import RemoteSpoolSystem
from .stepper_motor import StepperMotorSystem
from .torque_mode_system import TorqueModeSystem
from .cable_joints_components import CablePathComponent


def register_machine_systems(world, recording=None, *, cable_solver_device=None):
    if not world.systems:
        if world.get_resource('physicsSubsteps') is None:
            world.set_resource('physicsSubsteps', 1)
        world.register_system(RemoteSpoolSystem())
        for system in [
            common.PrevFinalPosSystem(), common.PrevFinalOrientationSystem(),
            StepperMotorSystem(), common.GravitySystem(),
            common.MovementSystem(), common.AngularMovementSystem(), common.RigidBodySyncSystem(),
            CableAttachmentUpdateSystem(False), CableAttachmentCacheSystem(), CableFrictionSystem(),
            PBDCableConstraintSolver(device=cable_solver_device), PBDResolveCableOverCorrections(),
            common.PBDVelocityUpdateSystem(), common.PBDAngularVelocityUpdateSystem(),
            TorqueModeSystem(), common.EncoderUpdateSystem(), MissedStepTrackingSystem(),
        ]:
            system.physics_substep = True
            world.register_system(system)
        world.register_system(ExtruderSystem())
        world.systems[1].physics_substeps = lambda state: (
            state.get_resource('physicsSubsteps')
            if state.get_resource('physicsSubsteps') > 1
            and state.query([RigidBodyComponent]) and state.query([CablePathComponent]) else 1)
    recorder = None
    if recording is not None:
        from .rerun_system import RerunSystem
        recorder = world.get_system(RerunSystem)
        if recorder is None:
            recorder = RerunSystem(recording)
            world.register_system(recorder)
        elif recorder.recording is not recording:
            raise ValueError('This world already has a different recording stream')
    extruder = world.get_system(ExtruderSystem)
    if extruder is not None:
        extruder.update(world, 0.)
    if recorder is not None:
        recorder.update(world, 0.)
        recorder.flush()


def load_machine_world(path, scene_prim_path=None, *, recording=None, cable_solver_device=None, **options):
    stage = open_cable_scene(path)
    if scene_prim_path is None:
        default = stage.GetMetadata('defaultPrim')
        scene_prim_path = '/World/' + default if default else '/World/SlideprinterScene'
    world = World()
    populate_machine_scene(world, stage, scene_prim_path, **options)
    register_machine_systems(world, recording, cable_solver_device=cable_solver_device)
    return world
