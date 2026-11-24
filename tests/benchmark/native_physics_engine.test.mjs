import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { setImmediate } from 'node:timers/promises';
import { runFixture } from '../parity3d/oracle.mjs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { NativePhysicsEngine, loadNativePhysicsAddon } from '../../src/js/cable_joints_3d/native_physics_engine.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { CableJointComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { runTrajectory } from '../../scripts/profile_headless.mjs';

const fixtures = new URL('../fixtures/engine_parity/', import.meta.url);
test('local defaults are native, isolated, editable and safe to clear', () => {
  const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
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

for (const [name, createBackend] of [
  ['native', () => new NativePhysicsEngine()],
  ['Wasm', () => new WasmPhysicsEngine(new WebAssembly.Module(
    fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url))))],
]) {
  test(`${name} rejects non-finite positions without advancing physics`, () => {
    const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
    const backend = createBackend(), control = createBackend();
    const snapshot = engine => engine.regions.map(region => region.data.slice());
    try {
      const { world, dt } = createHeadlessWorld(scene, { cableSolverBackend: backend });
      const reference = createHeadlessWorld(scene, { cableSolverBackend: control }).world;
      const step = () => world.update(dt);
      for (let i = 0; i < 200; i++) {
        step();
        reference.update(dt);
      }
      const joints = world.query([CableJointComponent]).map(id => world.getComponent(id, CableJointComponent));
      const joint = joints.find(joint => joint.constraintForceMagnitude > 0);
      assert.ok(joint, 'the settled reference cable carries a load');
      const pos = world.getComponent(joint.entityA, PositionComponent).pos;
      const row = backend.index.get(joint.entityA);
      for (const [axis, invalid] of [['x', NaN], ['x', Infinity], ['y', -Infinity], ['z', NaN]]) {
        const original = pos[axis], topology = backend.topologyVersion;
        pos[axis] = invalid;
        const before = snapshot(backend);
        assert.throws(step, { name: 'RangeError', message: `Non-finite physics position at entity row ${row}` });
        assert.deepEqual(snapshot(backend), before);
        assert.equal(backend.topologyVersion, topology);
        pos[axis] = original;
        step();
        reference.update(dt);
        assert.deepEqual(snapshot(backend), snapshot(control));
      }
    } finally {
      backend.dispose();
      control.dispose();
    }
  });
}
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
  const browserWasm = fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url));
  const cargoWasm = fs.readFileSync(new URL('target/wasm32-unknown-unknown/release/hp_sim5_cable_solver.wasm', crate));
  // Compare bytes without generating a diff of the entire binaries.
  assert.ok(browserWasm.equals(cargoWasm), 'The browser must use the Wasm artifact built from these sources');
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
    const scene = fs.readFileSync(new URL(`../../public/usd_scenes/${machine}.usda`, import.meta.url), 'utf8');
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

test('USDA stepper stick-slip and cable elongation hysteresis attributes load into native physics', async () => {
  const { CablePathComponent } = await import('../../src/js/cable_joints_3d/cable_joints_core.js');
  const { StepperMotorComponent } = await import('../../hp-sim-3d/app/hangprinter_stepper_motor.js');
  let scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
  scene = scene.replace(/(^\s*custom double stepper:holdingTorque\s*=\s*[-+\d.eE]+)\s*$/gm,
    '$1\n            custom double stepper:coulombFriction = 0.008\n            custom double stepper:stictionTorque = 0.012\n            custom double stepper:stictionSpeed = 0.25');
  scene = scene.replace(/(^\s*custom double cablePath:stiffnessPerUnitLength\s*=\s*[-+\d.eE]+)\s*$/gm,
    '$1\n            custom double cablePath:hysteresisFraction = 0.35\n            custom double cablePath:hysteresisRecoveryTime = 90');
  const instance = createHeadlessWorld(scene);
  try {
    const { world } = instance;
    const motors = world.query([StepperMotorComponent]).map(id => world.getComponent(id, StepperMotorComponent));
    const paths = world.query([CablePathComponent]).map(id => world.getComponent(id, CablePathComponent));
    assert(motors.length === 4 && motors.every(m => m.coulombFriction === 0.008 && m.stictionTorque === 0.012 && m.stictionSpeed === 0.25));
    assert(paths.length > 0 && paths.every(p => p.hysteresisFraction === 0.35 && p.hysteresisRecoveryTime === 90));
    const L0 = paths.map(p => p.totalRestLength);
    for (const m of motors) { m.torqueMode = true; m.targetTorque = 0.4; }
    for (let i = 0; i < 500; i++) world.update(0.002);
    const L_load = paths.map(p => p.totalRestLength);
    assert(L_load.some((l, i) => (l - L0[i]) * 1000 > 0.1));
  } finally {
    instance.world.getResource('cableSolverBackend').dispose();
  }
});
