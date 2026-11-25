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

test('spool physics preserves history across ECS rebuilds, backend transfer, and matches Native vs Wasm', async () => {
  const { CablePathComponent } = await import('../../src/js/cable_joints_3d/cable_joints_core.js');
  const { StepperMotorComponent } = await import('../../hp-sim-3d/app/hangprinter_stepper_motor.js');
  const wasmModule = new WebAssembly.Module(
    fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url))
  );
  let scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
  scene = scene.replace(/(^\s*custom double cablePath:stiffnessPerUnitLength\s*=\s*[-+\d.eE]+)\s*$/gm,
    '$1\n            custom double cablePath:spoolRadialFlattening = 0.000000422\n            custom bool cablePath:strainedSpoolStorage = true\n            custom double cablePath:initialStorageTension = 25');

  const runCase = (mode) => {
    const backend = mode === 'wasm' ? new WasmPhysicsEngine(wasmModule) : new NativePhysicsEngine();
    const instance = createHeadlessWorld(scene, { cableSolverBackend: backend });
    const { world, dt } = instance;
    let activeBackend = backend;
    try {
      const motors = world.query([StepperMotorComponent]).map(id => world.getComponent(id, StepperMotorComponent));
      const paths = world.query([CablePathComponent]).map(id => world.getComponent(id, CablePathComponent));
      assert(paths.length > 0 && paths.every(p =>
        Math.abs(p.spoolRadialFlattening - 0.422e-6) < 1e-12
        && p.strainedSpoolStorage === true
        && p.initialStorageTension === 25));

      // Phase 1: Wind at elevated torque to build non-trivial LIFO coils.
      for (const m of motors) { m.torqueMode = true; m.targetTorque = 0.25; }
      for (let i = 0; i < 80; i++) world.update(dt);

      if (mode === 'rebuild') {
        // Force a full Rust PhysicsWorld recreation via ECS structureVersion bump.
        world.structureVersion += 1;
      } else if (mode === 'transfer') {
        // Dispose Native backend (which snapshots spoolHistory into ECS) and continue on Wasm.
        activeBackend.dispose();
        const wasmBackend = new WasmPhysicsEngine(wasmModule);
        const sys = world.systems.find(s => s.backend === activeBackend);
        sys.backend = wasmBackend;
        world.setResource('cableSolverBackend', wasmBackend);
        activeBackend = wasmBackend;
      }

      // Phase 2: Reverse torque to unwind the stored coils at a different tension.
      for (const m of motors) { m.targetTorque = -0.10; }
      for (let i = 0; i < 80; i++) world.update(dt);

      activeBackend.syncSpoolHistory(world);
      return {
        stored: paths.map(p => [...p.stored]),
        totalRest: paths.map(p => p.totalRestLength),
        histories: paths.map(p => Array.from(p.spoolHistory ?? [])),
      };
    } finally {
      activeBackend.dispose();
    }
  };

  const nativeCont = runCase('native');
  const wasmCont = runCase('wasm');
  const rebuilt = runCase('rebuild');
  const transferred = runCase('transfer');

  assert(nativeCont.histories.every(h => h.length >= 14), 'spoolHistory must be exported');
  for (const [label, other] of [['wasm', wasmCont], ['rebuild', rebuilt], ['transfer', transferred]]) {
    for (let p = 0; p < nativeCont.stored.length; p++) {
      for (let k = 0; k < nativeCont.stored[p].length; k++) {
        assert(Math.abs(nativeCont.stored[p][k] - other.stored[p][k]) < 1e-12,
          `${label}: stored[${p}][${k}] mismatch (${nativeCont.stored[p][k]} vs ${other.stored[p][k]})`);
      }
      assert(Math.abs(nativeCont.totalRest[p] - other.totalRest[p]) < 1e-12,
        `${label}: totalRest[${p}] mismatch`);
    }
  }
});

test('JS cableStoredLengthAfterRotation and cableTargetLengthDelta match layered strained Rust solver', async () => {
  const {
    CablePathComponent,
    CableJointComponent,
    cableGeometricStoredLength,
    cableStoredLengthAfterRotation,
  } = await import('../../src/js/cable_joints_3d/cable_joints_core.js');
  const { cableTargetLengthDelta } = await import('../../hp-sim-3d/app/flightRecorderSnapshot.js');
  const { StepperMotorComponent } = await import('../../hp-sim-3d/app/hangprinter_stepper_motor.js');
  const { EncoderComponent } = await import('../../src/js/cable_joints_3d/ecs.js');

  let scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
  // Use exaggerated flattening + strained storage on a layered HP3 spool so geom_stored clearly diverges from stored.
  scene = scene.replace(/(^\s*custom double cablePath:stiffnessPerUnitLength\s*=\s*[-+\d.eE]+)\s*$/gm,
    '$1\n            custom double cablePath:spoolRadialFlattening = 0.00001\n            custom bool cablePath:strainedSpoolStorage = true\n            custom double cablePath:initialStorageTension = 50');

  const instance = createHeadlessWorld(scene, { lineLayeringEnabled: true, closedLoopMotorsEnabled: true });
  const { world, dt } = instance;
  const backend = world.getResource('cableSolverBackend');
  try {
    const pathId = world.query([CablePathComponent])[0];
    const path = world.getComponent(pathId, CablePathComponent);
    const firstJoint = world.getComponent(path.jointEntities[0], CableJointComponent);
    const spoolId = firstJoint.entityA;
    const motor = world.getComponent(spoolId, StepperMotorComponent);
    const encoder = world.getComponent(spoolId, EncoderComponent);
    // Keep other motors gently slackened and give the tested motor enough torque to track
    // commanded angles cleanly while maintaining ~60-150 N tension on the layered spool.
    for (const mId of world.query([StepperMotorComponent])) {
      const m = world.getComponent(mId, StepperMotorComponent);
      if (mId === spoolId) {
        m.holdingTorque = 25.0;
      } else {
        m.torqueMode = true;
        m.targetTorque = -0.15;
      }
    }

    // Wind ~1.5 revolutions in the true winding direction (negative angle on cw=true spool)
    // against tension to build strained coils and clearly separate geom_stored from stored.
    for (let i = 0; i < 80; i++) {
      motor.commandedAngle -= 0.12;
      world.update(dt);
    }
    const geom0 = cableGeometricStoredLength(world, path, 0);
    const stored0 = path.stored[0];
    assert(geom0 > stored0 + 1e-4, `expected geom_stored (${geom0}) > stored (${stored0}) after strained winding`);

    // Now test both winding (-0.4 rad) and unwinding (+0.6 rad) predictions against actual solver execution.
    for (const deltaAngle of [-0.4, 0.6]) {
      for (let i = 0; i < 40; i++) world.update(dt);
      const startStored = path.stored[0];
      const startAngle = encoder.angle - (motor.missedStepEncoderOffset || 0);
      const predictedStored = cableStoredLengthAfterRotation(world, path, 0, spoolId, deltaAngle);
      const targetBefore = motor.commandedAngle;
      motor.commandedAngle = targetBefore + deltaAngle;
      const predictedTargetDelta = cableTargetLengthDelta(world, path);

      // Step until closed-loop motor settles at the new commanded angle.
      for (let i = 0; i < 200; i++) world.update(dt);
      const actualAngleDelta = (encoder.angle - (motor.missedStepEncoderOffset || 0)) - startAngle;
      const actualStoredDelta = path.stored[0] - startStored;
      const predictedStoredDelta = predictedStored - startStored;
      assert(Math.abs(actualAngleDelta - deltaAngle) < 0.02,
        `motor did not reach target angle delta ${deltaAngle} (got ${actualAngleDelta})`);
      // Predicted material transfer matches the solver's actual stored change tightly.
      assert(Math.abs(actualStoredDelta - predictedStoredDelta) < Math.max(2e-4, 0.02 * Math.abs(actualStoredDelta)),
        `deltaAngle=${deltaAngle}: predicted stored delta ${predictedStoredDelta} vs actual ${actualStoredDelta}`);
      // Flight recorder cableTargetLengthDelta sign & magnitude match -actualStoredDelta.
      assert(Math.abs(predictedTargetDelta + actualStoredDelta) < Math.max(3e-4, 0.03 * Math.abs(actualStoredDelta)),
        `deltaAngle=${deltaAngle}: cableTargetLengthDelta ${predictedTargetDelta} vs -actual ${-actualStoredDelta}`);
    }
  } finally {
    backend.dispose();
  }
});

test('exportCheckpoint/importCheckpoint round-trip across a fresh Node process matches uninterrupted run', async () => {
  const { execFileSync } = await import('node:child_process');
  const os = await import('node:os');
  const pathMod = await import('node:path');
  const { CablePathComponent, CableJointComponent } = await import('../../src/js/cable_joints_3d/cable_joints_core.js');
  const { StepperMotorComponent } = await import('../../hp-sim-3d/app/hangprinter_stepper_motor.js');
  const { EncoderComponent, RigidBodyComponent } = await import('../../src/js/cable_joints_3d/ecs.js');

  let scene = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
  scene = scene.replace(/(^\s*custom double cablePath:stiffnessPerUnitLength\s*=\s*[-+\d.eE]+)\s*$/gm,
    '$1\n            custom double cablePath:spoolRadialFlattening = 0.000002\n            custom bool cablePath:strainedSpoolStorage = true\n            custom double cablePath:initialStorageTension = 20');

  const tmpDir = fs.mkdtempSync(pathMod.join(os.tmpdir(), 'hp-sim5-ckpt-'));
  const ckptPath = pathMod.join(tmpDir, 'checkpoint.json');
  const scenePath = pathMod.join(tmpDir, 'scene.usda');
  fs.writeFileSync(scenePath, scene, 'utf8');

  const instance = createHeadlessWorld(scene);
  const { world, dt } = instance;
  const backend = world.getResource('cableSolverBackend');
  try {
    const motors = world.query([StepperMotorComponent]).map(id => world.getComponent(id, StepperMotorComponent));
    // Phase 1: Wind under 3 distinct tensions to build multi-entry LIFO stacks.
    for (const torque of [0.15, 0.35, 0.22]) {
      for (const m of motors) { m.torqueMode = true; m.targetTorque = torque; }
      for (let i = 0; i < 40; i++) world.update(dt);
    }

    // Export full resumable checkpoint while running (no disposal).
    const ckpt = backend.exportCheckpoint();
    fs.writeFileSync(ckptPath, JSON.stringify(ckpt), 'utf8');

    // Verify corrupt spool state fails loudly rather than silently resetting.
    assert.throws(() => backend.importCheckpoint({ ...ckpt, spoolStates: ckpt.spoolStates.map(() => [999, 0]) }),
      /Failed to import spool state/);

    // Phase 2 (parent uninterrupted): unwind at a different tension.
    for (const m of motors) { m.targetTorque = -0.12; }
    for (let i = 0; i < 90; i++) world.update(dt);

    const readSummary = (w, eng) => {
      const mIds = w.query([StepperMotorComponent]);
      const pIds = w.query([CablePathComponent]);
      const jIds = w.query([CableJointComponent]);
      const bodyId = w.query([RigidBodyComponent])[0];
      const pos = w.getComponent(bodyId, PositionComponent).pos;
      return {
        motorAngles: mIds.map(id => w.getComponent(id, EncoderComponent).angle),
        geomStored: pIds.map(id => eng.spoolGeomStored(w.getComponent(id, CablePathComponent), 0)),
        spoolStates: pIds.map((_, idx) => Array.from(eng.transport.exportSpoolState(idx))),
        restLengths: jIds.map(id => w.getComponent(id, CableJointComponent).restLength),
        forces: jIds.map(id => w.getComponent(id, CableJointComponent).constraintForceMagnitude),
        effectorPos: [pos.x, pos.y, pos.z],
      };
    };
    const uninterrupted = readSummary(world, backend);

    // Restore in a completely fresh Node process and run identical Phase 2 unwinding.
    const childScript = `
      import fs from 'node:fs';
      import { createHeadlessWorld } from './hp-sim-3d/app/headlessWorld.js';
      import { CablePathComponent, CableJointComponent } from './src/js/cable_joints_3d/cable_joints_core.js';
      import { StepperMotorComponent } from './hp-sim-3d/app/hangprinter_stepper_motor.js';
      import { EncoderComponent, PositionComponent, RigidBodyComponent } from './src/js/cable_joints_3d/ecs.js';
      const scene = fs.readFileSync(${JSON.stringify(scenePath)}, 'utf8');
      const ckpt = JSON.parse(fs.readFileSync(${JSON.stringify(ckptPath)}, 'utf8'));
      const { world, dt } = createHeadlessWorld(scene);
      const eng = world.getResource('cableSolverBackend');
      world.update(0);
      eng.importCheckpoint(ckpt);
      for (const id of world.query([StepperMotorComponent])) {
        const m = world.getComponent(id, StepperMotorComponent);
        m.torqueMode = true;
        m.targetTorque = -0.12;
      }
      for (let i = 0; i < 90; i++) world.update(dt);
      const mIds = world.query([StepperMotorComponent]);
      const pIds = world.query([CablePathComponent]);
      const jIds = world.query([CableJointComponent]);
      const bodyId = world.query([RigidBodyComponent])[0];
      const pos = world.getComponent(bodyId, PositionComponent).pos;
      process.stdout.write(JSON.stringify({
        motorAngles: mIds.map(id => world.getComponent(id, EncoderComponent).angle),
        geomStored: pIds.map(id => eng.spoolGeomStored(world.getComponent(id, CablePathComponent), 0)),
        spoolStates: pIds.map((_, idx) => Array.from(eng.transport.exportSpoolState(idx))),
        restLengths: jIds.map(id => world.getComponent(id, CableJointComponent).restLength),
        forces: jIds.map(id => world.getComponent(id, CableJointComponent).constraintForceMagnitude),
        effectorPos: [pos.x, pos.y, pos.z],
      }));
      eng.dispose();
    `;
    const childOut = execFileSync(process.execPath, ['--input-type=module', '-e', childScript], {
      cwd: new URL('../../', import.meta.url),
      encoding: 'utf8',
    });
    const restored = JSON.parse(childOut);

    const assertCloseArr = (label, a, b, tol = 1e-11) => {
      assert.equal(a.length, b.length, `${label} length mismatch`);
      for (let i = 0; i < a.length; i++) {
        assert(Math.abs(a[i] - b[i]) <= tol, `${label}[${i}] mismatch: ${a[i]} vs ${b[i]}`);
      }
    };
    assertCloseArr('motorAngles', uninterrupted.motorAngles, restored.motorAngles);
    assertCloseArr('geomStored', uninterrupted.geomStored, restored.geomStored);
    assertCloseArr('restLengths', uninterrupted.restLengths, restored.restLengths);
    assertCloseArr('forces', uninterrupted.forces, restored.forces, 1e-9);
    assertCloseArr('effectorPos', uninterrupted.effectorPos, restored.effectorPos);
    assert.equal(uninterrupted.spoolStates.length, restored.spoolStates.length);
    for (let i = 0; i < uninterrupted.spoolStates.length; i++) {
      assertCloseArr(`spoolStates[${i}]`, uninterrupted.spoolStates[i], restored.spoolStates[i]);
    }
  } finally {
    backend.dispose();
    fs.rmSync(tmpDir, { recursive: true, force: true });
  }
});

test('preloaded hybrid drums retain their cable and tension on the first stationary step', async () => {
  const { CablePathComponent } = await import('../../src/js/cable_joints_3d/cable_joints_core.js');
  const scene = fs.readFileSync(new URL('../../public/usd_scenes/copacabana.usda', import.meta.url), 'utf8');
  const { world, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, closedLoopMotorsEnabled: true });
  try {
    const paths = world.query([CablePathComponent]).map(id => world.getComponent(id, CablePathComponent));
    const stored = paths.map(path => path.stored[0]);
    assert.equal(paths.length, 8);
    world.update(dt);
    paths.forEach((path, i) => {
      assert.ok(Math.abs(path.stored[0] - stored[i]) < 1e-6);
      const span = world.getComponent(path.jointEntities.at(-1), CableJointComponent);
      assert.ok(Math.abs(span.constraintForceMagnitude - path.initialStorageTension) < 1);
    });
  } finally {
    world.getResource('cableSolverBackend').dispose();
  }
});
