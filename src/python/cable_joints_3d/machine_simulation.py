"""Headless composition root for the specialized Hangprinter simulation."""
from usd.cable_scene_loader import open_cable_scene

from . import common_systems as common
from .cable_attachment_cache_system import CableAttachmentCacheSystem
from .cable_attachment_update_system import CableAttachmentUpdateSystem
from .cable_friction_system import CableFrictionSystem
from .ecs import World
from .extruder import ExtruderSystem
from .machine_scene import populate_machine_scene
from .motor_diagnostics import MissedStepTrackingSystem
from .pbd_cable_constraint_solver import PBDCableConstraintSolver
from .pbd_resolve_cable_over_corrections import PBDResolveCableOverCorrections
from .remote_spool_system import RemoteSpoolSystem
from .stepper_motor import StepperMotorSystem
from .torque_mode_system import TorqueModeSystem


def register_machine_systems(world, recording=None):
    if not world.systems:
        for system in [
            common.PrevFinalPosSystem(), common.PrevFinalOrientationSystem(),
            RemoteSpoolSystem(), StepperMotorSystem(), common.GravitySystem(),
            common.MovementSystem(), common.AngularMovementSystem(), common.RigidBodySyncSystem(),
            CableAttachmentUpdateSystem(False), CableAttachmentCacheSystem(), CableFrictionSystem(),
            PBDCableConstraintSolver(), PBDResolveCableOverCorrections(),
            common.PBDVelocityUpdateSystem(), common.PBDAngularVelocityUpdateSystem(),
            TorqueModeSystem(), ExtruderSystem(), common.EncoderUpdateSystem(), MissedStepTrackingSystem(),
        ]:
            world.register_system(system)
        if recording is not None:
            from .rerun_system import RerunSystem
            world.register_system(RerunSystem(recording))
    extruder = world.get_system(ExtruderSystem)
    if extruder is not None:
        extruder.update(world, 0.)


def load_machine_world(path, scene_prim_path=None, *, recording=None, **options):
    stage = open_cable_scene(path)
    if scene_prim_path is None:
        default = stage.GetMetadata('defaultPrim')
        scene_prim_path = '/World/' + default if default else '/World/SlideprinterScene'
    world = World()
    populate_machine_scene(world, stage, scene_prim_path, **options)
    register_machine_systems(world, recording)
    return world
