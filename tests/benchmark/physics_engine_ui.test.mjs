import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { createFeatureFlagsController } from '../../hp-sim-3d/app/featureFlagsController.js';
import { NativePhysicsEngine } from '../../src/js/cable_joints_3d/native_physics_engine.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { CableJointComponent, CablePathComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { compareCheckpoints } from '../../scripts/profile_headless.mjs';

const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const attachments = world => world.query([CableJointComponent]).map(id => {
  const joint = world.getComponent(id, CableJointComponent);
  return { a: [joint.attachmentPointA_world.x, joint.attachmentPointA_world.y, joint.attachmentPointA_world.z],
    b: [joint.attachmentPointB_world.x, joint.attachmentPointB_world.y, joint.attachmentPointB_world.z],
    rest: joint.restLength };
});

for (const [name, createBackend] of [
  ['Wasm', () => new WasmPhysicsEngine(module)],
  ['native', () => new NativePhysicsEngine()],
]) test(`${name} supports layering, iteration edits and rebuilds between physics steps`, () => {
  const backend = createBackend();
  const reference = new NativePhysicsEngine();
  try {
    // Reuse the backend as the browser does when resetting or changing machines.
    for (const machine of ['hp4', 'hp3', 'hp4']) {
      const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${machine}.usda`, import.meta.url), 'utf8');
      const actual = createHeadlessWorld(scene, { cableSolverBackend: backend });
      const expected = createHeadlessWorld(scene, { cableSolverBackend: reference });
      const ui = createFeatureFlagsController({ world: actual.world, state: {} });
      const referenceUi = createFeatureFlagsController({ world: expected.world, state: {} });
      for (const [enabled, iterations] of [[true, null], [false, 8], [true, 16], [true, null]]) {
        ui.setLineLayeringEnabledState(enabled);
        referenceUi.setLineLayeringEnabledState(enabled);
        ui.setSolverIterationsState(iterations);
        referenceUi.setSolverIterationsState(iterations);
        assert(attachments(actual.world).length > 0);
        actual.world.update(actual.dt);
        expected.world.update(expected.dt);
        const count = iterations ?? (machine === 'hp4' ? 10 : 1);
        assert(actual.world.query([CablePathComponent]).every(id => actual.world.getComponent(id, CablePathComponent).solverIterations === count));
        compareCheckpoints(JSON.parse(JSON.stringify(attachments(actual.world))),
          JSON.parse(JSON.stringify(attachments(expected.world))), { atol: 1e-8, rtol: 1e-7 });
      }
    }
  } finally {
    backend.dispose();
    reference.dispose();
  }
});
