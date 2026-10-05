// The headless simulation pipeline shared by the app and differential oracle.
import { CableAttachmentUpdateSystem, PBDCableConstraintSolver } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { PBDResolveCableOverCorrections } from '../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';
import { CableAttachmentCacheSystem } from '../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableFrictionSystem } from '../../src/js/cable_joints_3d/cable_friction_system.js';
import { ExtruderSystem } from './hangprinter_extruder.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import { StepperMotorSystem } from './hangprinter_stepper_motor.js';
import { TorqueModeSystem } from './torqueModeSystem.js';
import { MissedStepTrackingSystem } from './motor-diagnostics.js';
import {
    PrevFinalPosSystem, PrevFinalOrientationSystem, EncoderUpdateSystem,
    GravitySystem, MovementSystem, AngularMovementSystem,
    PBDVelocityUpdateSystem, PBDAngularVelocityUpdateSystem, RigidBodySyncSystem,
} from '../../src/js/cable_joints_3d/commonSystems.js';

export function registerSimulationSystems(world) {
    world.registerSystem(new PrevFinalPosSystem());
    world.registerSystem(new PrevFinalOrientationSystem());
    world.registerSystem(new RemoteSpoolSystem());
    world.registerSystem(new StepperMotorSystem());
    world.registerSystem(new GravitySystem());
    world.registerSystem(new MovementSystem());
    world.registerSystem(new AngularMovementSystem());
    world.registerSystem(new RigidBodySyncSystem());
    world.registerSystem(new CableAttachmentUpdateSystem(false));
    world.registerSystem(new CableAttachmentCacheSystem());
    world.registerSystem(new CableFrictionSystem());
    world.registerSystem(new PBDCableConstraintSolver());
    world.registerSystem(new PBDResolveCableOverCorrections());
    world.registerSystem(new PBDVelocityUpdateSystem());
    world.registerSystem(new PBDAngularVelocityUpdateSystem());
    world.registerSystem(new TorqueModeSystem());
    world.registerSystem(new ExtruderSystem());
    world.registerSystem(new EncoderUpdateSystem());
    world.registerSystem(new MissedStepTrackingSystem());
    const flightRecorder = world.getResource('flightRecorder');
    if (flightRecorder) world.registerSystem(flightRecorder);
}
