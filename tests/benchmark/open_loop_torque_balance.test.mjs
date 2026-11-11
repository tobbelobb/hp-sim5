import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { runFixture } from '../parity3d/oracle.mjs';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { NativePhysicsEngine } from '../../src/js/cable_joints_3d/native_physics_engine.js';

const fixture = JSON.parse(fs.readFileSync(new URL('../fixtures/engine_parity/open_loop_static_load.json', import.meta.url)));
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

// A 30 mm spool and EA = 10 kN cable over span length d must balance
// H*sin(p*theta) = r*(EA/d)*(extension-r*theta), as the firmware's
// springKPerUnitLength / distance. Use one projection per step to isolate
// motor/cable torque balance from nonlinear repeated-projection error.
// The physical law has no timestep factor.
const [, , rest, anchor, tangent] = fixture.entities.find(e => e.name === 'joint').components.CableJointComponent;
const span = Math.hypot(...anchor.map((x, i) => x - tangent[i]));
let low = 0, high = .02;
for (let i = 0; i < 60; i++) {
  const middle = (low + high) / 2;
  if (.5 * Math.sin(50 * middle) < .03 * 1e4 / span * (span - rest - .03 * middle)) low = middle;
  else high = middle;
}
const expectedAngle = (low + high) / 2;

for (const [engine, createBackend] of [
  ['Wasm physics', () => new WasmPhysicsEngine(module)],
  ['native physics', () => new NativePhysicsEngine()],
]) for (const substeps of [1, 2, 4, 8, 16]) {
  test(`${engine}: physical open-loop torque balance, ${substeps} substeps`, () => {
    const input = structuredClone(fixture);
    const dt = .002 / substeps;
    input.resources.dt = dt;
    input.steps = Array.from({ length: 500 * substeps }, () => ({ dt }));
    const backend = createBackend();
    try {
      const state = runFixture(input, { cableSolverBackend: backend }).snapshots.at(-1).entities;
      const angle = state.spool.EncoderComponent.angle;
      const motorTorque = .5 * Math.sin(50 * angle);
      const cableTorque = .03 * state.joint.CableJointComponent.constraintForceMagnitude;
      assert(Math.abs(angle - expectedAngle) < 1e-10, `${angle} != ${expectedAngle}`);
      assert(Math.abs(motorTorque - cableTorque) < 1e-9, `${motorTorque} != ${cableTorque} Nm`);
    } finally {
      backend?.dispose?.();
    }
  });
}
