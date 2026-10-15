import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { createFeatureFlagsController } from '../../hp-sim-3d/app/featureFlagsController.js';
import { NativePhysicsEngine } from '../../src/js/cable_joints_3d/native_physics_engine.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { CableJointComponent, CablePathComponent, calculateAttachmentPoints } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { compareCheckpoints } from '../../scripts/profile_headless.mjs';

const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const attachments = world => world.query([CableJointComponent]).map(id => {
  const joint = world.getComponent(id, CableJointComponent);
  return { a: joint.attachmentPointA_world, b: joint.attachmentPointB_world, rest: joint.restLength };
});

for (const [name, createBackend] of [
  ['Wasm', () => new WasmPhysicsEngine(module)],
  ['native', () => new NativePhysicsEngine()],
]) test(`${name} supports scene initialization, layering edits and rebuilds between physics steps`, () => {
  const backend = createBackend();
  try {
    // Reuse the backend as the browser does when resetting or changing machines.
    for (const machine of ['hp4', 'hp3', 'hp4']) {
      const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${machine}_rigid_body.usda`, import.meta.url), 'utf8');
      const actual = createHeadlessWorld(scene, { cableSolverBackend: backend });
      const expected = createHeadlessWorld(scene, { cableSolverBackend: null });
      const ui = createFeatureFlagsController({ world: actual.world, state: {} });
      const referenceUi = createFeatureFlagsController({ world: expected.world, state: {} });
      for (const enabled of [true, false, true]) {
        ui.setLineLayeringEnabledState(enabled);
        referenceUi.setLineLayeringEnabledState(enabled);
        const snapshot = attachments(actual.world);
        assert(snapshot.length > 0);
        compareCheckpoints(JSON.parse(JSON.stringify(snapshot)), JSON.parse(JSON.stringify(attachments(expected.world))),
          { atol: 1e-8, rtol: 1e-7 });
        // The public helper also runs outside a full-engine physics step.
        const pathId = actual.world.query([CablePathComponent])[0];
        const path = actual.world.getComponent(pathId, CablePathComponent);
        const joint = actual.world.getComponent(path.jointEntities[0], CableJointComponent);
        const { attachmentA_current, attachmentB_current } = calculateAttachmentPoints(actual.world, joint, path, 0);
        assert([attachmentA_current, attachmentB_current].every(point =>
          point && [point.x, point.y, point.z].every(Number.isFinite)));
        actual.world.update(actual.dt);
        expected.world.update(expected.dt);
      }
    }
  } finally {
    backend.dispose();
  }
});
