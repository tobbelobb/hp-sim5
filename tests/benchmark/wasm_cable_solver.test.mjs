import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { runFixture } from '../parity3d/oracle.mjs';
import { compileCableSolverWasm, WasmCableSolver } from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { World, PositionComponent, MassComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent, PBDCableConstraintSolver } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { registerSimulationSystems } from '../../hp-sim-3d/app/simulationSystems.js';
import { runTrajectory, compareCheckpoints } from '../../scripts/profile_headless.mjs';

const module = await compileCableSolverWasm(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const directory = new URL('../fixtures/python_3d_parity/', import.meta.url);
const referenceWorld = new World();
registerSimulationSystems(referenceWorld, { cableSolverBackend: null });
const productionSystems = new Set(referenceWorld.systems.map(system => system.constructor.name));
test('the full browser engine has no JavaScript imports', () => {
  assert.deepEqual(WebAssembly.Module.imports(module), []);
  const world = createHeadlessWorld(fs.readFileSync(new URL('../../public/usd_scenes/hp4_rigid_body.usda', import.meta.url), 'utf8'),
    { cableSolverBackend: new WasmPhysicsEngine(module) });
  assert(world.world.systems.some(s => s.constructor.name === 'RustPhysicsSystem'));
  assert(!world.world.systems.some(s => s.constructor.name === 'PBDCableConstraintSolver'));
  world.world.getResource('cableSolverBackend').update(world.world, world.dt);
  assert.equal(world.world.getResource('cableHybridTransitionStep'), 1);
});
test('persistent numeric views accept edits, refresh after memory growth, and detach on clear', () => {
  const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3_rigid_body.usda', import.meta.url), 'utf8');
  const instance = createHeadlessWorld(scene, { cableSolverBackend: new WasmPhysicsEngine(module) });
  const second = createHeadlessWorld(scene, { cableSolverBackend: new WasmPhysicsEngine(module) });
  instance.world.update(instance.dt);
  const backend = instance.world.getResource('cableSolverBackend');
  const entity = instance.world.query([PositionComponent])[0], position = instance.world.getComponent(entity, PositionComponent).pos;
  backend.exports.memory.grow(4);
  const old = position.x; position.x += .1;
  assert.equal(position.x, old + .1);
  assert.notEqual(second.world.getComponent(entity, PositionComponent).pos.x, position.x);
  instance.world.clear(); instance.world.update(instance.dt);
  assert.equal(position.x, old + .1);
  assert.equal(backend.ids.length, 0);
});
test('rigid cables in torque mode retain finite load coefficients', () => {
  const fixture = JSON.parse(fs.readFileSync(new URL('torque_motor_cables.json', directory)));
  fixture.pipeline = true; fixture.systems = [];
  for (const entity of fixture.entities) {
    const path = entity.components.CablePathComponent;
    if (path) path[3] = 'Infinity';
  }
  const expected = runFixture(fixture), actual = runFixture(fixture, { cableSolverBackend: new WasmPhysicsEngine(module) });
  for (const result of [actual, expected]) for (const snapshot of result.snapshots) delete snapshot.systemOrder;
  compareCheckpoints(JSON.parse(JSON.stringify(actual)), JSON.parse(JSON.stringify(expected)), { atol: 1e-8, rtol: 1e-7 });
});
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
  const systems = fixture.systems.map(system => system.name ?? system);
  const usesCableKernel = systems.some(name =>
    ['PBDCableConstraintSolver', 'CableAttachmentUpdateSystem', 'PBDResolveCableOverCorrections'].includes(name));
  // The old kernel filter omitted friction, integration and standalone motors.
  // Optional collisions, distance constraints and slack systems are not part of
  // hp-sim-3d's production pipeline or Rust core; keep their separate JS tests.
  const usesFullEngine = systems.length > 0 && systems.every(name => productionSystems.has(name));
  if (fixture.pipeline || usesCableKernel) test(`WASM matches JS every timestep: ${file}`, () => {
    const expected = runFixture(fixture);
    const actual = runFixture(fixture, { cableSolverBackend: fixture.pipeline ? new WasmPhysicsEngine(module) : new WasmCableSolver(module) });
    // Use the same JSON representation as the profiling checkpoints (-0 -> 0).
    if (fixture.pipeline) {
      assert(actual.snapshots.every(s => s.systemOrder.includes('RustPhysicsSystem')));
      for (const result of [actual, expected]) for (const snapshot of result.snapshots) delete snapshot.systemOrder;
    }
    compareCheckpoints(JSON.parse(JSON.stringify(actual)), JSON.parse(JSON.stringify(expected)), { atol: 1e-8, rtol: 1e-7 });
  });
  if (!fixture.pipeline && usesFullEngine) test(`full Rust pipeline covers ${file}`, () => {
    const full = { ...fixture, systems: [], pipeline: true };
    const expected = runFixture(full), actual = runFixture(full, { cableSolverBackend: new WasmPhysicsEngine(module) });
    assert(actual.snapshots.every(snapshot => snapshot.systemOrder.includes('RustPhysicsSystem')));
    for (const result of [actual, expected]) for (const snapshot of result.snapshots) delete snapshot.systemOrder;
    compareCheckpoints(JSON.parse(JSON.stringify(actual)), JSON.parse(JSON.stringify(expected)), { atol: 1e-8, rtol: 1e-7 });
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
      const expected = runTrajectory(createHeadlessWorld(scene, { ...options, cableSolverBackend: null }), workload, 200, 1);
      const actual = runTrajectory(createHeadlessWorld(scene, { ...options, cableSolverBackend: new WasmPhysicsEngine(module) }), workload, 200, 1);
      compareCheckpoints(actual.checkpoints, expected.checkpoints, { atol: 1e-8, rtol: 1e-7 });
    });
  }
}
