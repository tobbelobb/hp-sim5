// The headless simulation pipeline shared by the app and differential oracle.
import { CableAttachmentUpdateSystem, CablePathComponent, PBDCableConstraintSolver } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { PBDResolveCableOverCorrections } from '../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';
import { CableAttachmentCacheSystem } from '../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableFrictionSystem } from '../../src/js/cable_joints_3d/cable_friction_system.js';
import { ExtruderSystem } from './hangprinter_extruder.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import { StepperMotorSystem } from './hangprinter_stepper_motor.js';
import { TorqueModeSystem } from './torqueModeSystem.js';
import { MissedStepTrackingSystem } from './motor-diagnostics.js';
import { createDefaultPhysicsBackend, RustPhysicsSystem } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import {
    PrevFinalPosSystem, PrevFinalOrientationSystem, EncoderUpdateSystem,
    GravitySystem, MovementSystem, AngularMovementSystem,
    PBDVelocityUpdateSystem, PBDAngularVelocityUpdateSystem, RigidBodySyncSystem,
} from '../../src/js/cable_joints_3d/commonSystems.js';

export function registerSimulationSystems(world, { cableSolverBackend = undefined } = {}) {
    // HP4's stiff cables and light effector rock at a single 2 ms solve.
    // Integrate at 0.5 ms, keeping firmware commands and recordings at 2 ms.
    if (world.getResource('physicsSubsteps') === undefined) world.setResource('physicsSubsteps', 4);
    const substeps = state => state.query([RigidBodyComponent]).length && state.query([CablePathComponent]).length
        ? state.getResource('physicsSubsteps') : 1;
    const firstSystem = world.systems.length;
    if (cableSolverBackend === undefined) cableSolverBackend = world.getResource('cableSolverBackend');
    if (cableSolverBackend === undefined) cableSolverBackend = createDefaultPhysicsBackend();
    world.setResource('cableSolverBackend', cableSolverBackend);
    world.setResource('physicsEngine', cableSolverBackend?.physicsEngine ?? (cableSolverBackend ? 'wasm-cable-kernel' : 'javascript'));
    world.setResource('physicsArtifact', cableSolverBackend?.artifactInfo ?? null);
    if (cableSolverBackend?.fullEngine) {
        world.registerSystem(new RemoteSpoolSystem());
        world.registerSystem(new RustPhysicsSystem(cableSolverBackend));
        world.systems.at(-1).physicsSubstep = true;
        world.systems.at(-1).physicsSubsteps = substeps;
        world.registerSystem(new ExtruderSystem());
        const recorder = world.getResource('flightRecorder');
        if (recorder) world.registerSystem(recorder);
        return;
    }
    world.registerSystem(new RemoteSpoolSystem());
    world.registerSystem(new PrevFinalPosSystem());
    world.registerSystem(new PrevFinalOrientationSystem());
    world.registerSystem(new StepperMotorSystem());
    world.registerSystem(new GravitySystem());
    world.registerSystem(new MovementSystem());
    world.registerSystem(new AngularMovementSystem());
    world.registerSystem(new RigidBodySyncSystem());
    world.registerSystem(new CableAttachmentUpdateSystem(false));
    world.registerSystem(new CableAttachmentCacheSystem());
    world.registerSystem(new CableFrictionSystem());
    world.registerSystem(new PBDCableConstraintSolver(cableSolverBackend));
    world.registerSystem(new PBDResolveCableOverCorrections());
    world.registerSystem(new PBDVelocityUpdateSystem());
    world.registerSystem(new PBDAngularVelocityUpdateSystem());
    world.registerSystem(new TorqueModeSystem());
    world.registerSystem(new EncoderUpdateSystem());
    world.registerSystem(new MissedStepTrackingSystem());
    for (const system of world.systems.slice(firstSystem + 1)) system.physicsSubstep = true;
    world.systems[firstSystem + 1].physicsSubsteps = substeps;
    world.registerSystem(new ExtruderSystem());
    const flightRecorder = world.getResource('flightRecorder');
    if (flightRecorder) world.registerSystem(flightRecorder);
}
