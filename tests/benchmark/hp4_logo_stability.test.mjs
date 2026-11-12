import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { StepperMotorComponent } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { PositionComponent, OrientationComponent, VelocityComponent, AngularVelocityComponent,
  RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { commands } from './hp4_logo_commands.mjs';

const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp4.usda', import.meta.url), 'utf8');
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

// The normal Print Logo settings: HP4, layering on, open loop, catalogue substeps.
// Check every step, including motion: a finite final pose or a quiet startup
// alone missed the explosive regression introduced by removing wheel slip.
for (const [name, createBackend, steps] of [
  ['native', () => undefined, 10000],
  ['browser Wasm', () => new WasmPhysicsEngine(module), 10000],
]) for (const physicsSubsteps of [undefined, 1]) test(`HP4 Print Logo remains stable: ${name}, ${physicsSubsteps ?? 'catalogue'} substeps`, () => {
  const { world, remote, dt } = createHeadlessWorld(scene, { cableSolverBackend: createBackend(), physicsSubsteps });
  try {
    assert.equal(dt, .002);
    assert.equal(world.getResource('enableLayering'), true);
    assert.equal(world.getResource('physicsSubsteps'), physicsSubsteps ?? 4);
    assert(world.query([StepperMotorComponent]).every(id => !world.getComponent(id, StepperMotorComponent).closedLoop));
    remote.commands = commands.slice(0, steps);
    const body = world.query([RigidBodyComponent])[0];
    for (let step = 0; step < steps; step++) {
      world.update(dt);
      const p = world.getComponent(body, PositionComponent).pos;
      const q = world.getComponent(body, OrientationComponent).quaternion;
      const speed = world.getComponent(body, VelocityComponent).vel.length();
      const omega = world.getComponent(body, AngularVelocityComponent).omega.length();
      const where = `${name}, logo step ${step + 1}`;
      assert([p.x, p.y, p.z, q.x, q.y, q.z, q.w, speed, omega].every(Number.isFinite), `${where}: non-finite body state`);
      assert(Math.abs(p.x) < 1 && Math.abs(p.y) < 1, `${where}: left logo workspace`);
      assert(Math.abs(p.z - .0903) < .005, `${where}: left print plane: z=${p.z} m`);
      assert(speed < .5, `${where}: speed=${speed} m/s`);
      // Smaller steps remove numerical damping: the converged opening-move
      // transient reaches ~0.12 rad/s at four substeps (~0.20 at sixteen),
      // but stays below 0.002 rad of tilt and decays to rest. Keep the old
      // one-substep bound and check amplitude/settling as well as peak speed.
      assert(omega < (physicsSubsteps === 1 ? .1 : .15), `${where}: rocking=${omega} rad/s`);
      const tilt = 2 * Math.acos(Math.min(1, Math.abs(q.w)));
      assert(tilt < .002, `${where}: tilt=${tilt} rad`);
      assert(!world.getResource('pauseState')?.paused, `${where}: physics paused before completing`);
      assert(!world.getResource('errorState')?.hasError, `${where}: physics error`);
    }
    assert.equal(remote.getQueueLength(), 0, 'all scheduled logo commands must execute');
    assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps, 0);
    for (let step = 0; step < 10000; step++) {
      world.update(dt);
      if (step >= 7500) {
        assert(world.getComponent(body, AngularVelocityComponent).omega.length() < 1e-6, 'rocking must settle');
        assert(world.getComponent(body, VelocityComponent).vel.length() < 1e-8, 'motion must settle');
      }
    }
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});
