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

const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp4_rigid_body.usda', import.meta.url), 'utf8');
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

// The normal Print Logo settings: HP4, layering on, open loop, ONE substep.
// Check every step, including motion: a finite final pose or a quiet startup
// alone missed the explosive regression introduced by removing wheel slip.
for (const [name, createBackend, steps] of [
  ['native', () => undefined, 10000],
  ['browser Wasm', () => new WasmPhysicsEngine(module), 10000],
  ['JavaScript fallback', () => null, 1000],
]) test(`HP4 Print Logo remains stable at browser defaults: ${name}`, () => {
  const { world, remote, dt } = createHeadlessWorld(scene, { cableSolverBackend: createBackend() });
  try {
    assert.equal(dt, .002);
    assert.equal(world.getResource('enableLayering'), true);
    assert.equal(world.getResource('physicsSubsteps'), 1);
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
      assert(omega < .1, `${where}: rocking=${omega} rad/s`);
      assert(!world.getResource('pauseState')?.paused, `${where}: physics paused before completing`);
      assert(!world.getResource('errorState')?.hasError, `${where}: physics error`);
    }
    assert.equal(remote.getQueueLength(), 0, 'all scheduled logo commands must execute');
    assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps, 0);
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});
