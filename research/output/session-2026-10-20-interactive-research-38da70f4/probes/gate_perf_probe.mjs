import fs from 'node:fs';
import { performance } from 'node:perf_hooks';
import { PositionComponent, RigidBodyComponent } from '../../../../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent } from '../../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { NativePhysicsEngine } from '../../../../../src/js/cable_joints_3d/native_physics_engine.js';
import { createHeadlessWorld } from '../../../../../hp-sim-3d/app/headlessWorld.js';
import { commands } from '../../../../../tests/benchmark/hp4_logo_commands.mjs';

const steps = 1000;
const scenes = {
  original: 'public/usd_scenes/hp4_rigid_body.usda',
  gate: 'public/usd_scenes/hp4_pinhole_gate.usda',
};
const runs = { original: 2, gate: 3 };

function measure(scenePath) {
  const source = fs.readFileSync(new URL(`../../../../../${scenePath}`, import.meta.url), 'utf8');
  const backend = new NativePhysicsEngine();
  const { world, remote, dt } = createHeadlessWorld(source, { cableSolverBackend: backend });
  try {
    const body = world.query([RigidBodyComponent])[0];
    remote.commands = commands.slice(0, steps);
    const start = performance.now();
    for (let i = 0; i < steps; i++) world.update(dt);
    const elapsedMs = performance.now() - start;
    return {
      elapsedMs,
      finalZ: world.getComponent(body, PositionComponent).pos.z,
      joints: world.query([CableJointComponent]).length,
    };
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
}

measure(scenes.gate);
const results = {};
for (const [name, scenePath] of Object.entries(scenes)) {
  results[name] = [];
  for (let i = 0; i < runs[name]; i++) results[name].push(measure(scenePath));
}
const median = values => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)];
const summary = Object.fromEntries(Object.entries(results).map(([name, values]) => [name, {
  samples: values,
  medianElapsedMs: median(values.map(value => value.elapsedMs)),
  medianFinalZ: median(values.map(value => value.finalZ)),
}]));
console.log(JSON.stringify({ steps, recording: 'none', backend: 'Native', runs: summary }, null, 2));
