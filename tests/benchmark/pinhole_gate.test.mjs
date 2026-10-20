import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {
  AngularVelocityComponent,
  PositionComponent,
  RenderableComponent,
  RigidBodyComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
  GateWheelComponent,
  PinholeGateComponent,
  advanceGateFlow,
  gateWheelVisualAngle,
} from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { commands } from './hp4_logo_commands.mjs';

const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const source = fs.readFileSync(new URL('../../public/usd_scenes/hp4_pinhole_gate.usda', import.meta.url), 'utf8');

test('stateful gate flow has static friction, inertia, drag and unilateral stopping', () => {
  const gate = {
    effectiveInertia: 2.0,
    damping: 3.0,
    coulombForce: 4.0,
    flow: 0.0,
    flowVelocity: 0.0,
  };
  advanceGateFlow(gate, 4.0, 0.1);
  assert.equal(gate.flow, 0.0);
  assert.equal(gate.flowVelocity, 0.0);

  advanceGateFlow(gate, 4.1, 0.1);
  assert(gate.flowVelocity > 0.0);
  assert(gate.flow > 0.0);
  const flowWhileStopping = gate.flow;
  advanceGateFlow(gate, -20.0, 0.1);
  assert.equal(gate.flowVelocity, 0.0);
  assert.equal(gate.flow, flowWhileStopping);

  const inertial = {
    effectiveInertia: 2.0,
    damping: 0.0,
    coulombForce: 0.0,
    flow: 0.0,
    flowVelocity: 0.0,
  };
  advanceGateFlow(inertial, 1.0, 0.1);
  assert(Math.abs(inertial.flowVelocity - 0.05) < 1e-12);
  assert(Math.abs(inertial.flow - 0.005) < 1e-12);
  advanceGateFlow(inertial, 1.0, 0.1);
  assert(Math.abs(inertial.flowVelocity - 0.1) < 1e-12);

  const dragged = {
    effectiveInertia: 1e-6,
    damping: 2.0,
    coulombForce: 0.5,
    flow: 0.0,
    flowVelocity: 0.0,
  };
  for (let i = 0; i < 2000; i++) advanceGateFlow(dragged, 1.5, 0.01);
  assert(Math.abs(dragged.flowVelocity - 0.5) < 1e-5);
});

test('HP4 gate scene has six condensed units and twelve visual cylinders', () => {
  const { world } = createHeadlessWorld(source, { cableSolverBackend: null });
  try {
    const gateIds = world.query([PinholeGateComponent]);
    const wheelIds = world.query([GateWheelComponent]);
    assert.equal(gateIds.length, 6);
    assert.equal(wheelIds.length, 12);
    assert.equal(world.query([CableJointComponent]).length, 33);
    assert.equal(world.query([CablePathComponent]).filter(id =>
      world.getComponent(id, CablePathComponent).linkTypes.includes('pinhole_gate')).length, 6);

    for (const gateId of gateIds) {
      const gate = world.getComponent(gateId, PinholeGateComponent);
      assert(Math.abs(gate.effectiveInertia - 0.023668639053254437) < 1e-15);
      assert(Math.abs(gate.damping - 0.9467455621301777) < 1e-15);
      assert(Math.abs(gate.coulombForce - 0.00004 / 0.0065) < 1e-15);
      assert.equal(gate.visualRoute.length, 8);
    }

    for (const wheelId of wheelIds) {
      const render = world.getComponent(wheelId, RenderableComponent);
      const wheel = world.getComponent(wheelId, GateWheelComponent);
      assert.equal(render.shape, 'cylinder');
      assert(Math.abs(render.height - 0.004) < 1e-15);
      assert(world.getComponent(wheel.pathEntity, CablePathComponent));
      assert(world.getComponent(wheel.gateEntity, PinholeGateComponent));
      assert(wheel.rotationScale === 1.0 || Math.abs(wheel.rotationScale + 0.338) < 1e-15);
    }
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});

const steps = 200;
const trajectories = [];
for (const [name, makeBackend] of [
  ['Native', () => undefined],
  ['Wasm', () => new WasmPhysicsEngine(module)],
  ['JS', () => null],
]) {
  test(`HP4 gate production prefix agrees across backends: ${name}`, () => {
    const backend = makeBackend();
    const { world, remote, dt } = createHeadlessWorld(source, { cableSolverBackend: backend });
    try {
      const body = world.query([RigidBodyComponent])[0];
      const pathIds = world.query([CablePathComponent]).filter(id =>
        world.getComponent(id, CablePathComponent).linkTypes.includes('pinhole_gate'));
      const row = [];
      let maxForceDifference = 0.0;
      remote.commands = commands.slice(0, steps);
      for (let i = 0; i < steps; i++) {
        world.update(dt);
        const p = world.getComponent(body, PositionComponent).pos;
        const omega = world.getComponent(body, AngularVelocityComponent).omega.length();
        assert([p.x, p.y, p.z, omega].every(Number.isFinite), `${name} non-finite state at ${i}`);
        assert(Math.abs(p.z - 0.09) < 0.01, `${name} left the print plane at ${i}`);
        for (const pathId of pathIds) {
          const path = world.getComponent(pathId, CablePathComponent);
          const gateIndex = path.linkTypes.indexOf('pinhole_gate');
          const entry = world.getComponent(path.jointEntities[0], CableJointComponent).constraintForceMagnitude;
          const exit = world.getComponent(path.jointEntities[gateIndex], CableJointComponent).constraintForceMagnitude;
          maxForceDifference = Math.max(maxForceDifference, Math.abs(entry - exit));
        }
        if (i % 20 === 0) row.push([p.x, p.y, p.z]);
      }
      assert.equal(remote.getQueueLength(), 0);
      assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps, 0);
      assert(maxForceDifference > 1e-6, `${name} did not expose the hidden bearing state`);
      assert(maxForceDifference < 0.5, `${name} produced an implausible gate force offset`);

      const wheelAngles = world.query([GateWheelComponent]).map(id => {
        const wheel = world.getComponent(id, GateWheelComponent);
        return {
          gate: wheel.gateEntity,
          scale: wheel.rotationScale,
          angle: gateWheelVisualAngle(wheel, world.getComponent(wheel.pathEntity, CablePathComponent)),
        };
      });
      assert(wheelAngles.every(({ angle }) => Math.abs(angle) > 0.1), `${name} wheel animation was not transport driven`);
      for (const gateId of world.query([PinholeGateComponent])) {
        const pair = wheelAngles.filter(({ gate }) => gate === gateId);
        assert.equal(pair.length, 2);
        assert(Math.sign(pair[0].angle) !== Math.sign(pair[1].angle), `${name} bearing pair rotated in the same direction`);
        const top = pair.find(({ scale }) => scale === 1.0);
        const bottom = pair.find(({ scale }) => scale !== 1.0);
        assert(Math.abs(bottom.angle + 0.338 * top.angle) < 1e-12, `${name} bearing speed ratio changed`);
      }

      trajectories.push({ name, row });
      const native = trajectories.find(entry => entry.name === 'Native').row;
      for (let i = 0; i < row.length; i++) {
        for (let axis = 0; axis < 3; axis++) {
          assert(Math.abs(row[i][axis] - native[i][axis]) < 1e-6, `${name} disagrees with native at sample ${i}`);
        }
      }
    } finally {
      world.getResource('cableSolverBackend')?.dispose();
    }
  });
}
