import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { PositionComponent, RigidBodyComponent, SceneEntityInfoComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent, CableJointComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';

const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp4.usda', import.meta.url), 'utf8');
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

for (const [name, createBackend] of [
  ['native', () => undefined],
  ['browser Wasm', () => new WasmPhysicsEngine(module)],
]) test(`HP4 catalogue default resolves symmetric pretension without XY drift: ${name}`, () => {
  const { world, remote, dt } = createHeadlessWorld(scene, {
    cableSolverBackend: createBackend(), lineLayeringEnabled: false, closedLoopMotorsEnabled: true,
  });
  try {
    assert.equal(world.getResource('physicsSubsteps'), 4);
    const body = world.query([RigidBodyComponent])[0];
    const pos = () => world.getComponent(body, PositionComponent).pos.clone();
    for (let step = 0; step < 500; step++) world.update(dt);
    const start = pos();
    // MA²-corrected firmware home flex, in effector-distance mm. Convert to
    // spool angles using MA [2,2,2,4] and 30 mm radius; ramp 1 s, hold 4 s.
    // Research: the former 1-substep default creates 0.228 mm of XY bias and
    // realizes only ~3.3 N per ABC strand, versus ~5 N when converged.
    const target = [-.56587, -.56587, -.56587, -.08332].map((lp, i) => lp * [2, 2, 2, 4][i] / 30);
    remote.commands = Array.from({ length: 2500 }, (_, step) => {
      const ramp = Math.min(1, (step + 1) / 500);
      return { type: 'Move', ...Object.fromEntries(['A', 'B', 'C', 'D'].map((axis, i) => [axis, target[i] * ramp])) };
    });
    for (let step = 0; step < 2500; step++) world.update(dt);
    const displacement = pos().subtract(start);
    assert(Math.hypot(displacement.x, displacement.y) < .00001, `XY bias: ${displacement.length() * 1000} mm`);
    assert(Math.abs(displacement.z) < .00002, `Z bias: ${displacement.z * 1000} mm`);
    for (const id of world.query([CablePathComponent])) {
      const path = world.getComponent(id, CablePathComponent);
      const axis = world.getComponent(id, SceneEntityInfoComponent).name;
      const expected = axis.startsWith('CablePathD') ? .795 : 5;
      for (const joint of path.jointEntities) {
        const tension = world.getComponent(joint, CableJointComponent).constraintForceMagnitude;
        assert(Math.abs(tension / expected - 1) < .02, `${axis}: tension ${tension} N, expected ~${expected} N`);
      }
    }
    assert.equal(remote.getQueueLength(), 0);
    assert(!world.getResource('errorState')?.hasError);
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});
