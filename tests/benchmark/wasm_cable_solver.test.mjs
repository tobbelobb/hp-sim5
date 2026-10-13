import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { runFixture } from '../parity3d/oracle.mjs';
import { compileCableSolverWasm, WasmCableSolver } from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import { World, PositionComponent, MassComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent, PBDCableConstraintSolver } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { runTrajectory } from '../../scripts/profile_headless.mjs';

const module = await compileCableSolverWasm(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const directory = new URL('../fixtures/python_3d_parity/', import.meta.url);
test('backend survives empty updates, clear, reused IDs and absent pose/inertia stores', () => {
  const world = new World(), solver = new PBDCableConstraintSolver(new WasmCableSolver(module));
  solver.update(world);
  for (const end of [3, 5]) {
    world.clear(); world.setResource('dt', 1);
    const a = world.createEntity(), b = world.createEntity(), joint = world.createEntity(), path = world.createEntity();
    world.addComponent(a, new PositionComponent()); world.addComponent(b, new PositionComponent(end, 0, 0));
    for (const id of [a, b]) world.addComponent(id, new MassComponent(1));
    world.addComponent(joint, new CableJointComponent(a, b, 1, new Vector3(), new Vector3(end, 0, 0)));
    world.addComponent(path, new CablePathComponent(world, [joint], ['attachment', 'attachment'], [false, false], Infinity));
    solver.update(world);
    assert.equal(world.getComponent(a, PositionComponent).pos.x, (end - 1) / 2);
    assert.equal(world.getComponent(b, PositionComponent).pos.x, (end + 1) / 2);
  }
});
test('WASM vector length matches V8 for extreme magnitudes and deterministic samples', () => {
  const { hypot3 } = new WasmCableSolver(module).exports;
  for (const values of [[0, -0, 0], [Infinity, NaN, 0], [NaN, 0, 0],
    [1e308, -1e308, 1e308], [Number.MIN_VALUE, Number.MIN_VALUE, 0]]) {
    assert.equal(hypot3(...values), Math.hypot(...values));
  }
  let seed = 12345;
  const random = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 2 ** 32; };
  for (let i = 0; i < 10000; i++) {
    const values = Array.from({ length: 3 }, () => (random() - .5) * 10 ** (random() * 610 - 305));
    assert.equal(hypot3(...values), Math.hypot(...values));
  }
});
for (const file of fs.readdirSync(directory).filter(file => file.endsWith('.json')).sort()) {
  const fixture = JSON.parse(fs.readFileSync(new URL(file, directory)));
  if (!fixture.pipeline && !fixture.systems.some(system =>
    ['PBDCableConstraintSolver', 'CableAttachmentUpdateSystem', 'PBDResolveCableOverCorrections'].includes(system.name ?? system))) continue;
  test(`WASM matches JS every timestep: ${file}`, () => {
    const expected = runFixture(fixture);
    const actual = runFixture(fixture, { cableSolverBackend: new WasmCableSolver(module) });
    // Use the same JSON representation as the profiling checkpoints (-0 -> 0).
    assert.deepEqual(JSON.parse(JSON.stringify(actual)), JSON.parse(JSON.stringify(expected)));
  });
}

for (const name of ['hp3', 'hp4', 'four_high_anchors', 'cubecorners', 'slideprinter',
  'slideprinter_multi_unit', 'slideprinter_hexagon', 'slideprinter_single_pinholes']) {
  for (const lineLayeringEnabled of [true, false]) {
    test(`WASM matches 200 commanded steps: ${name}, layering ${lineLayeringEnabled}`, () => {
      const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${name}_rigid_body.usda`, import.meta.url), 'utf8');
      const options = { lineLayeringEnabled };
      const commands = Array.from({ length: 200 }, (_, n) => ({ type: 'Move',
        A: Math.sin(n / 20) * .001, B: -Math.sin(n / 20) * .001, C: n * .00001, D: -n * .00001 }));
      const workload = { boundaries: [{ step: 0, commands }] };
      const expected = runTrajectory(createHeadlessWorld(scene, options), workload, 200, 1);
      const actual = runTrajectory(createHeadlessWorld(scene, { ...options, cableSolverBackend: new WasmCableSolver(module) }), workload, 200, 1);
      assert.deepEqual(actual.checkpoints, expected.checkpoints);
    });
  }
}
