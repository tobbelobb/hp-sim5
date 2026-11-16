import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { PositionComponent, RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent, CableJointComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';

for (const name of ['hp5', 'hp5_pulley_reduced']) test(`${name} holds near home with pretension`, () => {
  const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${name}.usda`, import.meta.url), 'utf8');
  const { world, remote, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, closedLoopMotorsEnabled: true });
  try {
    const body = world.query([RigidBodyComponent])[0];
    const start = world.getComponent(body, PositionComponent).pos.clone();
    for (let i = 0; i < 500; i++) world.update(dt);
    const angle = -.00025 * (name.includes('reduced') ? 2 : 1) / .03;
    remote.commands = Array.from({ length: 3000 }, (_, i) => {
      const value = angle * Math.min(1, (i + 1) / 500);
      return { type: 'Move', A: value, B: value, C: value, D: value, I: 0 };
    });
    for (let i = 0; i < 3000; i++) world.update(dt);
    const displacement = world.getComponent(body, PositionComponent).pos.clone().subtract(start);
    assert(displacement.length() < .0002, `displacement ${displacement.length()} m`);
    const paths = world.query([CablePathComponent]);
    assert.equal(paths.length, 12);
    for (const id of paths) {
      const path = world.getComponent(id, CablePathComponent);
      assert(world.getComponent(path.jointEntities[0], CableJointComponent).constraintForceMagnitude > .01);
    }
    assert(!world.getResource('errorState')?.hasError);
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});
