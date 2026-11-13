import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { BearingFrictionComponent, EncoderComponent, PositionComponent, RigidBodyComponent,
  VelocityComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { commands } from './hp4_logo_commands.mjs';

const steps = 4000;
function printLogo(scene) {
  const source = fs.readFileSync(new URL(`../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
  const { world, remote, dt } = createHeadlessWorld(source);
  try {
    const body = world.query([RigidBodyComponent])[0];
    const worn = world.query([BearingFrictionComponent]);
    const torques = worn.map(id => world.getComponent(id, BearingFrictionComponent).torque);
    remote.commands = commands.slice(0, steps);
    const path = [];
    for (let step = 0; step < steps; step++) {
      world.update(dt);
      const p = world.getComponent(body, PositionComponent).pos;
      assert([p.x, p.y, p.z].every(Number.isFinite), `${scene}, logo step ${step + 1}: non-finite position`);
      assert(world.getComponent(body, VelocityComponent).vel.length() < .5, `${scene}, logo step ${step + 1}: speed`);
      path.push(p.clone());
    }
    return { path, torques, missedSteps: getMachineMotorDiagnostics(world).totalMissedSteps,
      turned: worn.map(id => world.getComponent(id, EncoderComponent).angle) };
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
}

// bearing:frictionTorque on a Wheel prim makes its bearing resist rotation.
test('HP4 prints the logo on worn bearings', () => {
  const ideal = printLogo('hp4');
  const worn = printLogo('hp4_worn_bearings');
  assert.equal(ideal.torques.length, 0);
  assert.equal(worn.torques.length, 24);
  assert.equal(Math.min(...worn.torques), 1e-5);
  assert.equal(Math.max(...worn.torques), 3e-3);
  assert.equal(worn.missedSteps, 0);
  assert(worn.turned.every(angle => Math.abs(angle) > .1), 'every wheel must still turn with its line');
  const error = Math.max(...worn.path.map((p, step) => p.clone().subtract(ideal.path[step]).length()));
  assert(error > 1e-5, `bearing friction must load the lines: ${error} m`);
  assert(error < 2e-3, `worn bearings moved the effector ${error} m`);
});
