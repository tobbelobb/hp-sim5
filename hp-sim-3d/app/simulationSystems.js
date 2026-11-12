// The headless simulation pipeline shared by the app and differential oracle.
import { ExtruderSystem } from './hangprinter_extruder.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import { RustPhysicsSystem, createDefaultPhysicsBackend } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';

export function registerSimulationSystems(world, { cableSolverBackend = undefined } = {}) {
    // Scene loading sets machine defaults; firmware ticks remain 2 ms.
    if (world.getResource('physicsSubsteps') === undefined) world.setResource('physicsSubsteps', 1);
    if (cableSolverBackend === undefined) cableSolverBackend = world.getResource('cableSolverBackend');
    if (cableSolverBackend === undefined) cableSolverBackend = createDefaultPhysicsBackend();
    world.setResource('cableSolverBackend', cableSolverBackend);
    world.setResource('physicsEngine', cableSolverBackend.physicsEngine);
    world.setResource('physicsArtifact', cableSolverBackend.artifactInfo ?? null);
    world.registerSystem(new RemoteSpoolSystem());
    world.registerSystem(new RustPhysicsSystem(cableSolverBackend));
    world.systems.at(-1).physicsSubstep = true;
    world.systems.at(-1).physicsSubsteps = state => {
        const count = state.getResource('physicsSubsteps');
        return count > 1 && state.query([RigidBodyComponent]).length && state.query([CablePathComponent]).length
            ? count : 1;
    };
    world.registerSystem(new ExtruderSystem());
    const recorder = world.getResource('flightRecorder');
    if (recorder) world.registerSystem(recorder);
}
