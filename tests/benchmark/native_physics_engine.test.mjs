import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { setImmediate } from 'node:timers/promises';
import { runFixture } from '../parity3d/oracle.mjs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { NativePhysicsEngine, loadNativePhysicsAddon } from '../../src/js/cable_joints_3d/native_physics_engine.js';
import { PositionComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { runTrajectory } from '../../scripts/profile_headless.mjs';

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
test('native and Wasm compile the same unconditional Rust physics sources', () => {
  const { info: { build } } = loadNativePhysicsAddon();
  assert(!build.target.startsWith('wasm'), 'The addon must be compiled for a native target');
  const crate = new URL('../../src/wasm/cable_solver/', import.meta.url);
  const profile = build.profile === 'dev' ? 'debug' : build.profile;
  // Cargo's dep-info lists the files actually compiled, rather than a parallel
  // hand-maintained source list. CI rebuilds both artifacts before this check.
  const dependencies = file => fs.readFileSync(new URL(file, crate), 'utf8').split('\n')[0].split(': ').slice(1).join(': ');
  const native = dependencies(`target/${build.target}/${profile}/libhp_sim5_cable_solver.d`);
  const wasm = dependencies('target/wasm32-unknown-unknown/release/hp_sim5_cable_solver.d');
  assert(native.length > 0);
  assert.equal(native, wasm);
  assert.match(native, /src\/engine\.rs/);
  assert.deepEqual(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)),
    fs.readFileSync(new URL('target/wasm32-unknown-unknown/release/hp_sim5_cable_solver.wasm', crate)),
    'The browser must use the Wasm artifact built from these sources');
  // Identical files must not hide alternate physics behind target/feature cfgs.
  for (const file of fs.readdirSync(new URL('src/', crate), { recursive: true }).filter(file => file.endsWith('.rs'))) {
    assert.doesNotMatch(fs.readFileSync(new URL(`src/${file}`, crate), 'utf8'),
      /#\s*!?\[\s*(?:cfg|cfg_attr)\b|\bcfg!\s*\(/, `Conditional physics in ${file}`);
  }
});

test('native topology changes detach obsolete path views', () => {
  const fixture = JSON.parse(fs.readFileSync(new URL('topology_feature_flags.json', fixtures)));
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
  try {
    runFixture({ ...fixture, systems: [], pipeline: true }, { cableSolverBackend: backend });
    assert(detachments > 0);
  } finally {
    backend.dispose();
  }
});

for (const machine of ['hp3', 'hp4']) {
  test(`native smoke: ${machine} consumes commands and advances encoders`, () => {
    const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${machine}_rigid_body.usda`, import.meta.url), 'utf8');
    const instance = createHeadlessWorld(scene);
    const commands = Array.from({ length: 20 }, (_, i) => ({ type: 'Move', A: (i + 1) * .0001 }));
    try {
      const { checkpoints } = runTrajectory(instance, { boundaries: [{ step: 0, commands }] }, 20, 1);
      const first = checkpoints[0].snapshot, last = checkpoints.at(-1).snapshot;
      assert.equal(last.queueLength, 0);
      assert.equal(instance.remote.history.length, commands.length);
      assert(last.motors.some((motor, i) => Math.abs(motor.encoder - first.motors[i].encoder) > 1e-6));
      assert(last.cables.length > 0);
      assert(last.cables.some(cable => cable.segments.some(segment => segment.force_n > 0)));
      assert(checkpoints.every(checkpoint => checkpoint.snapshot.motors.every(motor => Number.isFinite(motor.encoder))));
    } finally {
      instance.world.getResource('cableSolverBackend').dispose();
    }
  });
}
