import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { setImmediate } from 'node:timers/promises';
import { runFixture } from '../parity3d/oracle.mjs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { NativePhysicsEngine, loadNativePhysicsAddon } from '../../src/js/cable_joints_3d/native_physics_engine.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { PositionComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { compareCheckpoints, runTrajectory } from '../../scripts/profile_headless.mjs';

const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const fixtures = new URL('../fixtures/python_3d_parity/', import.meta.url);
test('local defaults are native, isolated, editable and safe to clear', () => {
  const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3_rigid_body.usda', import.meta.url), 'utf8');
  const first = createHeadlessWorld(scene), second = createHeadlessWorld(scene);
  assert.equal(first.world.getResource('physicsEngine'), 'rust-native');
  assert(['release', 'profiling', 'dev'].includes(first.world.getResource('physicsArtifact').build.profile));
  first.world.update(first.dt);
  const backend = first.world.getResource('cableSolverBackend');
  const id = first.world.query([PositionComponent])[0];
  const pos = first.world.getComponent(id, PositionComponent).pos;
  const old = pos.x; pos.x += .1;
  assert.equal(backend.entityRegion.data[0], old + .1);
  assert.notEqual(pos.x, second.world.getComponent(id, PositionComponent).pos.x);
  const retained = backend.entityRegion.data;
  assert.throws(() => structuredClone(retained, { transfer: [retained.buffer] }), /transfer|clone/i);
  first.world.clear(); first.world.update(first.dt);
  assert.equal(pos.x, old + .1);
  assert.equal(retained.byteLength, 0);
  assert.equal(backend.ids.length, 0);
  backend.dispose(); second.world.getResource('cableSolverBackend').dispose();
});
test('native boundary rejects malformed records, indices and disposed handles', () => {
  const { addon } = loadNativePhysicsAddon();
  assert.throws(() => addon.create(new Float64Array(1), new Float64Array(), 0), /records/);
  assert.throws(() => addon.create(new Float64Array(), new Float64Array([1, 0]), 1), /path/);
  const cycle = new Float64Array(320); cycle[50] = 1; cycle[210] = 0;
  assert.throws(() => addon.create(cycle, new Float64Array(), 0), /parent/);
  const host = addon.create(new Float64Array(), new Float64Array(), 0);
  const options = host.options();
  assert.throws(() => host.path(-1), /index/);
  assert.throws(() => host.step(NaN), /timestep/);
  host.dispose(); assert.equal(options.byteLength, 0);
  assert.throws(() => host.step(.002), /disposed/);
});
test('native views retain their allocation after their host is collected', { skip: !global.gc }, async () => {
  let host = loadNativePhysicsAddon().addon.create(new Float64Array(), new Float64Array(), 0);
  const reference = new WeakRef(host), options = host.options();
  options[0] = 37; host = null;
  await setImmediate(); global.gc(); await setImmediate(); global.gc(); await setImmediate();
  assert.equal(reference.deref(), undefined);
  assert.equal(options[0], 37);
  options[0] = 38; assert.equal(options[0], 38);
});
for (const file of fs.readdirSync(fixtures).filter(file => file.endsWith('.json')).sort()) {
  const fixture = JSON.parse(fs.readFileSync(new URL(file, fixtures)));
  if (!fixture.pipeline && !fixture.systems.some(system => ['PBDCableConstraintSolver', 'CableAttachmentUpdateSystem',
    'PBDResolveCableOverCorrections'].includes(system.name ?? system))) continue;
  test(`native full pipeline matches Wasm: ${file}`, () => {
    const full = { ...fixture, systems: [], pipeline: true };
    const expected = runFixture(full, { cableSolverBackend: new WasmPhysicsEngine(module) });
    const backend = new NativePhysicsEngine();
    let detachments = 0;
    const advance = backend.stepPhysics.bind(backend);
    backend.stepPhysics = (world, dt) => {
      const old = backend.regions.slice(2), version = backend.topologyVersion;
      advance(world, dt);
      if (old.length && version !== backend.topologyVersion) {
        assert(old.every(region => region.data.byteLength === 0)); detachments++;
      }
    };
    const actual = runFixture(full, { cableSolverBackend: backend });
    compareCheckpoints(JSON.parse(JSON.stringify(actual)), JSON.parse(JSON.stringify(expected)), { atol: 1e-8, rtol: 1e-7 });
    backend.dispose();
    if (file === 'topology_feature_flags.json') assert(detachments > 0);
  });
}
for (const machine of ['hp3', 'hp4']) for (const lineLayeringEnabled of [true, false]) {
  test(`native commanded trajectory matches Wasm: ${machine}, layering ${lineLayeringEnabled}`, () => {
    const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${machine}_rigid_body.usda`, import.meta.url), 'utf8');
    const workload = { boundaries: [{ step: 0, commands: Array.from({ length: 200 }, (_, i) => ({
      type: 'Move', A: Math.sin(i / 20) * .001, B: -Math.sin(i / 20) * .001, C: i * .00001, D: -i * .00001,
    })) }] };
    const native = createHeadlessWorld(scene, { lineLayeringEnabled });
    const wasm = createHeadlessWorld(scene, { lineLayeringEnabled, cableSolverBackend: new WasmPhysicsEngine(module) });
    compareCheckpoints(runTrajectory(native, workload, 200, 1).checkpoints,
      runTrajectory(wasm, workload, 200, 1).checkpoints, { atol: 1e-8, rtol: 1e-7 });
    native.world.getResource('cableSolverBackend').dispose(); wasm.world.getResource('cableSolverBackend').dispose();
  });
}
