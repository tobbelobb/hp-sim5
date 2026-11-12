import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { PositionComponent, AngularVelocityComponent, RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { StepperMotorComponent } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';

// A loaded HP4 rocked indefinitely with fixed motor targets. The numeric state
// avoids replaying 584,000 steps; idle/initial-tilt tests do not expose this case.
// The damping displacement must be frozen once per step: re-deriving it on every
// Gauss-Seidel visit makes the damping term feed on its own corrections and the
// effector limit-cycles instead of settling (worst at the default 1 substep).
const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp4.usda', import.meta.url), 'utf8');
const saved = JSON.parse(fs.readFileSync(new URL('../fixtures/hp4_loaded_stability.json', import.meta.url)),
  (key, value) => ['NaN', 'Infinity', '-Infinity'].includes(value) ? Number(value) : value);

function settle(substeps, holdingTorque) {
  assert.equal(saved.formatVersion, 1);
  assert.equal(createHash('sha256').update(scene).digest('hex'), saved.sceneSha256,
    'Regenerate the loaded checkpoint when the HP4 scene changes');
  const { world, remote, dt } = createHeadlessWorld(scene, {
    lineLayeringEnabled: false, closedLoopMotorsEnabled: true, physicsSubsteps: substeps,
  });
  assert.equal(dt, .002);
  assert.equal(world.getResource('physicsSubsteps'), substeps);
  world.update(dt); // Prepare the native world's editable numeric regions.
  const backend = world.getResource('cableSolverBackend');
  assert.deepEqual(backend.regions.map(region => region.data.length), saved.regions.map(row => row.length));
  backend.regions.forEach((region, index) => region.data.set(saved.regions[index]));
  if (holdingTorque !== undefined) {
    for (const motor of world.query([StepperMotorComponent])) {
      world.getComponent(motor, StepperMotorComponent).holdingTorque = holdingTorque;
    }
  }
  const body = world.query([RigidBodyComponent])[0];
  assert(world.getComponent(body, AngularVelocityComponent).omega.length() > 1, 'start with real rocking');
  let minZ = Infinity, maxZ = -Infinity, maxOmega = 0;
  for (let step = 0; step < 10000; step++) {
    world.update(dt);
    if (step >= 7500) {
      const z = world.getComponent(body, PositionComponent).pos.z;
      minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
      maxOmega = Math.max(maxOmega, world.getComponent(body, AngularVelocityComponent).omega.length());
    }
  }
  assert(Number.isFinite(minZ) && Number.isFinite(maxZ));
  assert.equal(remote.getQueueLength(), 0);
  assert(maxZ - minZ < 1e-8, `vertical motion persisted at ${substeps} substeps: ${maxZ - minZ} m`);
  assert(maxOmega < 1e-6, `rocking persisted at ${substeps} substeps: ${maxOmega} rad/s`);
  assert.equal(world.getResource('dt'), .002);
}

for (const substeps of [1, 2, 4]) {
  test(`HP4 loaded recording state settles at production firmware cadence: ${substeps} substeps`,
    () => settle(substeps));
}
// The servo is tuned from the motor's holding torque. Its current limit must
// not turn a stronger or weaker motor into a limit cycle at the 2 ms step.
for (const holdingTorque of [0.5, 4]) {
  test(`HP4 loaded recording state settles with ${holdingTorque} Nm closed-loop motors at 1 substep`,
    () => settle(1, holdingTorque));
}
