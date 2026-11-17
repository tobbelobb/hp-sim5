// Shared parity measurement used by repro-parity.mjs.
import { state, numericDiff, sceneText, makeWorld, makeBackend, loadWasmModule } from './lib.mjs';
import { OrientationComponent, EncoderComponent, VelocityComponent, PositionComponent, RigidBodyComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

const module = loadWasmModule();

export function nextUp(x) {
  const view = new DataView(new ArrayBuffer(8));
  view.setFloat64(0, x || 0);
  let bits = view.getBigUint64(0);
  bits = (x >= 0) ? bits + 1n : bits - 1n;
  view.setBigUint64(0, bits);
  return view.getFloat64(0);
}

function maxAbs(a, b) {
  const n = Math.min(a.length, b.length);
  let max = 0, at = -1, nonFinite = null;
  for (let i = 0; i < n; i++) {
    const x = a[i], y = b[i];
    const xn = Number.isNaN(x), yn = Number.isNaN(y);
    if (xn && yn) continue;              // unused/undefined slots are NaN in both
    if (x === y) continue;               // identical, including +/-Infinity
    if (!Number.isFinite(x) || !Number.isFinite(y)) { nonFinite ??= { index: i, left: String(x), right: String(y) }; continue; }
    const d = Math.abs(x - y);
    if (d > max) { max = d; at = i; }
  }
  return { max, at, nonFinite, lengthMismatch: a.length !== b.length };
}

function jointCount(backend) {
  let joints = 0;
  for (let p = 0; p < (backend.pathIds?.length ?? 0); p++) {
    const data = backend.transport.path(p);
    joints += data[0];
  }
  return joints;
}

function maxQuatAngle(worldA, worldB) {
  let max = 0;
  for (const id of worldA.query([OrientationComponent])) {
    const qa = worldA.getComponent(id, OrientationComponent)?.quaternion;
    const qb = worldB.getComponent(id, OrientationComponent)?.quaternion;
    if (!qa || !qb) continue;
    const dot = Math.min(1, Math.abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w));
    max = Math.max(max, 2 * Math.acos(dot));
  }
  return max;
}

function quaternionSignFlips(worldA, worldB) {
  let count = 0;
  for (const id of worldA.query([OrientationComponent])) {
    const qa = worldA.getComponent(id, OrientationComponent)?.quaternion;
    const qb = worldB.getComponent(id, OrientationComponent)?.quaternion;
    if (!qa || !qb) continue;
    if (qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w < 0) count++;
  }
  return count;
}

function encoders(world) {
  return world.query([EncoderComponent]).map(id => world.getComponent(id, EncoderComponent).angle);
}

function missed(world) {
  return world.query([StepperMotorComponent]).map(id => world.getComponent(id, StepperMotorComponent).missedSteps);
}

// Runs native control, Wasm, and (optionally) a 1-ULP-perturbed native twin.
// The perturbed twin separates "Wasm math differs" from "this trajectory is
// chaotic": both should amplify an initially tiny difference the same way.
export function measure({ machine, layering = true, closedLoop = false, steps, commands = null, probe = false, probeSteps = 3000,
  probeField = 'velocity.x', probeScale = 1e-12, sampleEvery = 25 }) {
  const scene = sceneText(machine);
  const options = { lineLayeringEnabled: layering, closedLoopMotorsEnabled: closedLoop };
  const native = makeWorld(scene, { ...options, cableSolverBackend: makeBackend('native') });
  const wasm = makeWorld(scene, { ...options, cableSolverBackend: makeBackend('wasm', module) });
  const twin = makeWorld(scene, { ...options, cableSolverBackend: makeBackend('native') });
  const worlds = [native, wasm, twin];
  for (const instance of worlds) {
    if (commands) instance.remote.commands = commands.slice();
  }

  const nBackend = native.world.getResource('cableSolverBackend');
  const wBackend = wasm.world.getResource('cableSolverBackend');
  const tBackend = twin.world.getResource('cableSolverBackend');
  // Prepare both without stepping so the scene adapter itself is compared.
  nBackend.prepare(native.world);
  wBackend.prepare(wasm.world);
  const initial = maxAbs(nBackend.entityRegion.data, wBackend.entityRegion.data);
  const initialRecordLengths = [nBackend.entityRegion.data.length, wBackend.entityRegion.data.length];

  const rows = [];
  const events = { topologyFirstDifference: null, topologyChanges: { native: 0, wasm: 0 }, missedStepDifference: null,
    quaternionSignFlips: 0, lengthMismatch: null, nonFinite: null };
  let lastNativeTopology = nBackend.topologyVersion, lastWasmTopology = wBackend.topologyVersion;
  let perturbed = null;
  let initialMaxSpeedIndex = -1;

  const dt = native.dt;
  for (let stepn = 1; stepn <= steps; stepn++) {
    for (const instance of worlds) {
      if (instance === twin && stepn > probeSteps) continue;
      instance.world.update(dt);
    }
    if (stepn === 1 && probe) {
      // Perturb the fastest rigid body, the state that the cable solver integrates.
      const data = tBackend.entityRegion.data;
      let best = -1;
      for (let i = 0; i < tBackend.ids.length; i++) {
        if (!twin.world.hasComponent(tBackend.ids[i], RigidBodyComponent)) continue;
        const speed = Math.hypot(data[i * 160 + 69], data[i * 160 + 70], data[i * 160 + 71]);
        if (speed > best) { best = speed; initialMaxSpeedIndex = i; }
      }
      if (initialMaxSpeedIndex < 0) throw new Error('No rigid body to perturb');
      const offset = initialMaxSpeedIndex * 160 + (probeField === 'position.x' ? 0 : 69);
      const before = data[offset];
      const delta = probeField === 'ulp' ? nextUp(before) - before : probeScale * Math.max(1, Math.abs(before));
      data[offset] = before + delta;
      perturbed = { entityIndex: initialMaxSpeedIndex, entityId: tBackend.ids[initialMaxSpeedIndex],
        field: probeField, before, after: data[offset], delta, speedAtPerturbation: best };
    }
    const nd = nBackend.entityRegion.data, wd = wBackend.entityRegion.data;
    const diff = maxAbs(nd, wd);
    if (diff.lengthMismatch && !events.lengthMismatch) events.lengthMismatch = { step: stepn, native: nd.length, wasm: wd.length };
    if (diff.nonFinite && !events.nonFinite) events.nonFinite = { step: stepn, backend: 'native/wasm', ...diff.nonFinite };
    let twinDiff = null;
    if (probe && stepn <= probeSteps) {
      const td = tBackend.entityRegion.data;
      const d = maxAbs(nd, td);
      twinDiff = d.max;
      if (d.lengthMismatch && !events.lengthMismatch) events.lengthMismatch = { step: stepn, native: nd.length, twin: td.length };
      if (d.nonFinite && !events.nonFinite) events.nonFinite = { step: stepn, backend: 'native/twin', ...d.nonFinite };
    }
    if (nBackend.topologyVersion !== lastNativeTopology) { events.topologyChanges.native++; lastNativeTopology = nBackend.topologyVersion; }
    if (wBackend.topologyVersion !== lastWasmTopology) { events.topologyChanges.wasm++; lastWasmTopology = wBackend.topologyVersion; }
    if (nBackend.topologyVersion !== wBackend.topologyVersion && events.topologyFirstDifference === null) {
      events.topologyFirstDifference = { step: stepn, native: nBackend.topologyVersion, wasm: wBackend.topologyVersion };
    }
    if (stepn % 10 === 0 || stepn === 1) events.quaternionSignFlips += quaternionSignFlips(native.world, wasm.world);
    const mNative = missed(native.world), mWasm = missed(wasm.world);
    if (events.missedStepDifference === null && mNative.some((value, i) => value !== mWasm[i])) {
      events.missedStepDifference = { step: stepn, native: mNative, wasm: mWasm };
    }
    if (stepn % sampleEvery === 0 || stepn === 1) {
      const encN = encoders(native.world), encW = encoders(wasm.world);
      const dEnc = encN.reduce((m, value, i) => Math.max(m, Math.abs(value - encW[i])), 0);
      const pN = native.world.query([VelocityComponent]);
      let dVel = 0;
      for (const id of pN) {
        const a = native.world.getComponent(id, VelocityComponent)?.vel;
        const b = wasm.world.getComponent(id, VelocityComponent)?.vel;
        if (a && b) dVel = Math.max(dVel, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
      }
      const qAngle = maxQuatAngle(native.world, wasm.world);
      let twinEncoder = null, twinPosition = null;
      if (twinDiff !== null) {
        const encT = encoders(twin.world);
        twinEncoder = encN.reduce((m, value, i) => Math.max(m, Math.abs(value - encT[i])), 0);
        twinPosition = 0;
        for (const id of native.world.query([PositionComponent])) {
          const a = native.world.getComponent(id, PositionComponent)?.pos;
          const b = twin.world.getComponent(id, PositionComponent)?.pos;
          if (a && b) twinPosition = Math.max(twinPosition, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
        }
      }
      let dPosition = 0;
      for (const id of native.world.query([PositionComponent])) {
        const a = native.world.getComponent(id, PositionComponent)?.pos;
        const b = wasm.world.getComponent(id, PositionComponent)?.pos;
        if (a && b) dPosition = Math.max(dPosition, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
      }
      rows.push({ step: stepn, dMax: diff.max, at: diff.at, dEncoder: dEnc, dVelocity: dVel, dPosition,
        quatAngle: qAngle, twin: twinDiff, twinEncoder, twinPosition, topology: [nBackend.topologyVersion, wBackend.topologyVersion],
        joints: [jointCount(nBackend), jointCount(wBackend)] });
    }
  }

  const checkpoint = { native: state(native.world), wasm: state(wasm.world) };
  const finalDiff = numericDiff(checkpoint.native, checkpoint.wasm);
  const finalEncoders = encoders(native.world).map((value, i) => ({ native: value, wasm: encoders(wasm.world)[i] }));

  for (const instance of worlds) instance.world.getResource('cableSolverBackend')?.dispose?.();
  return { case: { machine, layering, closedLoop, steps, commands: commands?.length ?? 0, probe },
    initialRecordMaxDifference: initial.max, initialRecordLengths,
    perturbed, rows, events, finalMaxAbs: finalDiff.maxAbs, finalMaxKey: finalDiff.maxKey,
    finalMismatchedKeys: finalDiff.mismatchedKeys.slice(0, 10),
    finalEncoders, lastRow: rows.at(-1) };
}

// Re-synchronize the Wasm world from native before every step. The remaining
// difference is the one-step operator difference, with feedback amplification
// removed.
export function oneStepFork({ machine, layering = true, steps = 300, commands = null }) {
  const native = makeWorld(sceneText(machine), { lineLayeringEnabled: layering, cableSolverBackend: makeBackend('native') });
  const wasm = makeWorld(sceneText(machine), { lineLayeringEnabled: layering, cableSolverBackend: makeBackend('wasm', module) });
  for (const instance of [native, wasm]) if (commands) instance.remote.commands = commands.slice();
  const nBackend = native.world.getResource('cableSolverBackend');
  const wBackend = wasm.world.getResource('cableSolverBackend');
  nBackend.prepare(native.world); wBackend.prepare(wasm.world);
  const dt = native.dt;
  let maxEntity = 0, maxPath = 0, maxEncoder = 0, pathLengthMismatch = null;
  const rows = [];
  for (let stepn = 1; stepn <= steps; stepn++) {
    // Copy native entity and path state into Wasm so both start this step equal.
    const nRegions = nBackend.regions, wRegions = wBackend.regions;
    for (let i = 0; i < Math.min(nRegions.length, wRegions.length); i++) {
      if (i === 1) continue; // options are rewritten from resources each step
      const target = wRegions[i].data, source = nRegions[i].data;
      if (target.length !== source.length) { pathLengthMismatch ??= { step: stepn, index: i, native: source.length, wasm: target.length }; continue; }
      target.set(source);
    }
    native.world.update(dt);
    wasm.world.update(dt);
    const d = maxAbs(nBackend.entityRegion.data, wBackend.entityRegion.data);
    maxEntity = Math.max(maxEntity, d.max);
    if (d.nonFinite && !pathLengthMismatch) pathLengthMismatch = { step: stepn, nonFinite: d.nonFinite };
    let dp = 0;
    for (let p = 0; p < Math.min(nRegions.length, wRegions.length); p++) {
      if (p < 2) continue;
      const a = nRegions[p].data, b = wRegions[p].data;
      if (a.length !== b.length) continue;
      for (let i = 0; i < a.length; i++) {
        const x = a[i], y = b[i];
        if (Number.isNaN(x) && Number.isNaN(y)) continue;
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        dp = Math.max(dp, Math.abs(x - y));
      }
    }
    maxPath = Math.max(maxPath, dp);
    const encN = encoders(native.world), encW = encoders(wasm.world);
    maxEncoder = Math.max(maxEncoder, encN.reduce((m, value, i) => Math.max(m, Math.abs(value - encW[i])), 0));
    if (stepn % 25 === 0) rows.push({ step: stepn, entity: d.max, path: dp,
      encoder: encN.reduce((m, value, i) => Math.max(m, Math.abs(value - encW[i])), 0) });
  }
  native.world.getResource('cableSolverBackend').dispose();
  wasm.world.getResource('cableSolverBackend').dispose();
  return { machine, layering, steps, maxEntity, maxPath, maxEncoder, pathLengthMismatch, rows };
}

// Sensitivity scaling: perturb a native world by different small amounts and
// measure how the deviation evolves. Linear scaling with the perturbation means
// exponential amplification of infinitesimal noise; saturation at one size for
// every perturbation means a discrete branch decision.
export function perturbationScaling({ machine, layering = true, steps = 1500, commands = null,
  deltas = [1e-15, 1e-12, 1e-9], sampleEvery = 100 }) {
  const options = { lineLayeringEnabled: layering };
  const control = makeWorld(sceneText(machine), { ...options, cableSolverBackend: makeBackend('native') });
  const twins = deltas.map(() => makeWorld(sceneText(machine), { ...options, cableSolverBackend: makeBackend('native') }));
  const all = [control, ...twins];
  for (const instance of all) if (commands) instance.remote.commands = commands.slice();
  const backends = all.map(instance => instance.world.getResource('cableSolverBackend'));
  backends.forEach((backend, i) => backend.prepare(all[i].world));
  const dt = control.dt;
  const rows = [];
  const perturbed = [];
  for (let stepn = 1; stepn <= steps; stepn++) {
    if (stepn === 1) {
      const data = backends[0].entityRegion.data;
      let index = -1, best = -1;
      for (let i = 0; i < backends[0].ids.length; i++) {
        if (!control.world.hasComponent(backends[0].ids[i], RigidBodyComponent)) continue;
        const speed = Math.hypot(data[i * 160 + 69], data[i * 160 + 70], data[i * 160 + 71]);
        if (speed > best) { best = speed; index = i; }
      }
      deltas.forEach((delta, k) => {
        const target = backends[k + 1].entityRegion.data;
        const before = target[index * 160 + 69];
        target[index * 160 + 69] = before + delta;
        perturbed.push({ delta, before, after: target[index * 160 + 69], entityId: backends[k + 1].ids[index] });
      });
    }
    for (const instance of all) instance.world.update(dt);
    if (stepn % sampleEvery === 0 || stepn === 1) {
      const encC = encoders(control.world);
      const row = { step: stepn, deltas: [] };
      twins.forEach((twin, k) => {
        const d = maxAbs(backends[0].entityRegion.data, backends[k + 1].entityRegion.data);
        const encT = encoders(twin.world);
        let dpos = 0;
        for (const id of control.world.query([PositionComponent])) {
          const a = control.world.getComponent(id, PositionComponent)?.pos;
          const b = twin.world.getComponent(id, PositionComponent)?.pos;
          if (a && b) dpos = Math.max(dpos, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
        }
        row.deltas.push({ delta: deltas[k], entity: d.max,
          encoder: encC.reduce((m, value, i) => Math.max(m, Math.abs(value - encT[i])), 0), position: dpos });
      });
      rows.push(row);
    }
  }
  for (const instance of all) instance.world.getResource('cableSolverBackend').dispose();
  return { machine, layering, steps, perturbed, rows };
}
