import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { StepperMotorComponent } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';

import { commands } from './hp4_logo_commands.mjs';

const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp4_rigid_body.usda', import.meta.url), 'utf8');
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

for (const [name, createBackend, steps, substeps] of [
  ['JavaScript', () => null, 150, 4],
  ['Wasm', () => new WasmPhysicsEngine(module), 10000, 2],
  ['Wasm', () => new WasmPhysicsEngine(module), 10000, 4],
  ['Wasm', () => new WasmPhysicsEngine(module), 10000, 16],
]) test(`HP4 open-loop layered logo stays in step: ${name}, ${substeps} substeps`, () => {
  const backend = createBackend();
  try {
    const { world, remote, dt } = createHeadlessWorld(scene, {
      cableSolverBackend: backend, physicsSubsteps: substeps,
      lineLayeringEnabled: true, closedLoopMotorsEnabled: false,
    });
    remote.commands = commands.slice(0, steps);
    for (let step = 0; step < steps; step++) world.update(dt);
    assert.equal(remote.getQueueLength(), 0);
    assert(world.query([StepperMotorComponent]).every(id => !world.getComponent(id, StepperMotorComponent).closedLoop));
    assert.equal(world.getResource('dt'), .002);
    assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps, 0);
  } finally {
    backend?.dispose();
  }
});
