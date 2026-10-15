// Explicit native/Wasm/JS profiling. See tests/benchmark/HEADLESS_JS_PROFILING.md.
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { PerformanceObserver, constants } from 'node:perf_hooks';
import { Session } from 'node:inspector/promises';
import { setImmediate } from 'node:timers/promises';
import { createHeadlessWorld } from '../hp-sim-3d/app/headlessWorld.js';
import { captureFlightRecorderSnapshot } from '../hp-sim-3d/app/flightRecorderSnapshot.js';
import { setClosedLoopMotorFeatureFlags } from '../hp-sim-3d/app/closed-loop-flags.js';
import { EncoderComponent, VelocityComponent, AngularVelocityComponent } from '../src/js/cable_joints_3d/ecs.js';
import { StepperMotorComponent } from '../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { compileCableSolverWasm } from '../src/js/cable_joints_3d/wasm_cable_solver.js';
import { WasmPhysicsEngine } from '../src/js/cable_joints_3d/wasm_physics_engine.js';
import { NativePhysicsEngine, loadNativePhysicsAddon } from '../src/js/cable_joints_3d/native_physics_engine.js';

const root = fileURLToPath(new URL('../', import.meta.url));
const sha = value => createHash('sha256').update(value).digest('hex');
const finiteJson = value => JSON.stringify(value, (key, item) => {
  if (typeof item === 'number' && !Number.isFinite(item)) throw new Error(`Non-finite observation: ${key}`);
  return item;
});

// Rust uses its own transcendental functions. Compare physical observations
// numerically, while requiring identical metadata, topology and sample clocks.
export function compareCheckpoints(actual, expected, { atol = 1e-6, rtol = 1e-6, allowNumericalDrift = false } = {}) {
  const result = { atol, rtol, maxAbsoluteError: 0, maxRelativeError: 0, worstField: null, exact: true,
    matchesTolerance: true, outsideTolerance: 0, firstDifference: null, physicalErrors: {} };
  function compare(a, b, path = '', key = '') {
    if (typeof a === 'number' && typeof b === 'number') {
      if (!Number.isFinite(a) || !Number.isFinite(b)) throw new Error(`Non-finite checkpoint: ${path}`);
      const error = Math.abs(a - b), scale = Math.max(Math.abs(a), Math.abs(b));
      const metadata = /^(step|id|queueLength)$/.test(key);
      const discrete = metadata || /^(missedSteps|currentMissedSteps)$/.test(key);
      if (error > (discrete ? 0 : atol + rtol * scale)) {
        result.matchesTolerance = false; result.outsideTolerance++;
        result.firstDifference ??= { path, actual: a, expected: b };
        if (metadata || !allowNumericalDrift) throw new Error(`Numerical trajectory changed at ${path}: ${a} vs ${b}`);
      }
      const category = path.includes('.position.') || path.includes('.points.') || /rest_length|stored/.test(path) ? 'length_m'
        : key === 'encoder' ? 'encoder_rad' : path.includes('.angles.') ? 'encoder_deg'
        : key === 'force_n' ? 'force_n' : path.includes('.omega.') ? 'angular_velocity_rad_s'
        : path.includes('.vel.') ? 'velocity_m_s' : path.includes('.quaternion.') ? 'quaternion_component' : null;
      if (category) result.physicalErrors[category] = Math.max(result.physicalErrors[category] ?? 0, error);
      if (error) result.exact = false;
      if (error > result.maxAbsoluteError) { result.maxAbsoluteError = error; result.worstField = path; }
      if (scale > atol) result.maxRelativeError = Math.max(result.maxRelativeError, error / scale);
      return;
    }
    if (a && b && typeof a === 'object' && typeof b === 'object') {
      const ka = Object.keys(a), kb = Object.keys(b);
      if (Array.isArray(a) !== Array.isArray(b) || ka.length !== kb.length || ka.some(k => !Object.hasOwn(b, k))) throw new Error(`Checkpoint shape changed: ${path}`);
      for (const k of ka) compare(a[k], b[k], `${path}.${k}`, k);
    } else if (a !== b) throw new Error(`Checkpoint value changed: ${path}`);
  }
  compare(actual, expected);
  return result;
}

// Preserve arrival steps AND FIFO consumption: several commands can arrive at
// one step; RemoteSpoolSystem consumes one on each subsequent physics update.
export function loadWorkload(source, events = false) {
  const boundaries = [];
  if (!events) {
    const commands = JSON.parse(source);
    if (!Array.isArray(commands) || !commands.length) throw new Error('Expected a nonempty command array');
    return { steps: commands.length, boundaries: [{ step: 0, commands }], kind: 'scheduled_commands' };
  }
  let previous = 0, epoch, steps = 0;
  for (const line of source.trim().split('\n')) {
    const event = JSON.parse(line);
    if (!Number.isInteger(event.step) || event.step < previous) throw new Error('Events must have monotonic integer steps');
    if (epoch !== undefined && event.epoch !== epoch) throw new Error('Replay requires one epoch; split resets into separate workloads');
    epoch = event.epoch;
    previous = steps = event.step;
    const payload = event.type === 'bridge_payload' ? event.payload : null;
    if (payload?.type === 'reset') throw new Error('Replay requires one world without resets');
    if (payload?.type === 'simulation_settings' && typeof payload.lineLayeringEnabled === 'boolean') {
      throw new Error('Select layering via --layering and split traces containing layering changes');
    }
    if (payload?.type === 'command' || Array.isArray(payload?.commands)) {
      const commands = payload.type === 'command' ? [payload.command] : payload.commands;
      boundaries.push({ step: event.step, commands });
    } else if (payload?.type === 'simulation_settings' && typeof payload.closedLoopMotorsEnabled === 'boolean') {
      boundaries.push({ step: event.step, closedLoop: payload.closedLoopMotorsEnabled });
    } else if (event.type === 'encoder_response' && event.axes?.length) {
      boundaries.push({ step: event.step, encoderRead: { axes: event.axes, expected: event.angles_deg } });
    }
  }
  if (!steps || !boundaries.some(row => row.commands?.length)) throw new Error('Trace must contain motion and elapsed physics steps');
  return { steps, boundaries, kind: 'frozen_autocal_replay' };
}

function observe(world, remote) {
  const snapshot = captureFlightRecorderSnapshot(world, { geometryDetail: 'compact' });
  snapshot.motors = world.query([StepperMotorComponent]).map(id => {
    const motor = world.getComponent(id, StepperMotorComponent);
    return { id, encoder: world.getComponent(id, EncoderComponent)?.angle,
      target: motor.commandedAngle, delta: motor.deltaAngle, torqueMode: motor.torqueMode,
      torque: motor.targetTorque, missedSteps: motor.missedSteps, currentMissedSteps: motor.currentMissedSteps };
  });
  snapshot.velocities = [VelocityComponent, AngularVelocityComponent].map(Type =>
    world.query([Type]).map(id => [id, structuredClone(world.getComponent(id, Type))]));
  snapshot.queueLength = remote.getQueueLength();
  return snapshot;
}

export function runTrajectory(instance, workload, steps, checkpointEvery = 1000, observations = true) {
  const { world, remote, dt } = instance;
  const checkpoints = [];
  let step = 0, index = 0, physicsMs = 0, observationMs = 0, encoderReads = 0, maxEncoderErrorDeg = 0;
  const checkpoint = () => {
    const started = performance.now();
    checkpoints.push({ step, snapshot: observe(world, remote) });
    // Validate before JSON can silently convert Infinity/NaN to null.
    finiteJson(checkpoints.at(-1));
    observationMs += performance.now() - started;
  };
  if (observations) checkpoint();
  while (step <= steps) {
    while (index < workload.boundaries.length && workload.boundaries[index].step === step) {
      const boundary = workload.boundaries[index++];
      for (const command of boundary.commands ?? []) remote.addCommand(command);
      if (boundary.closedLoop !== undefined) setClosedLoopMotorFeatureFlags(world, boundary.closedLoop);
      if (observations && boundary.encoderRead) {
        const started = performance.now();
        const { axes, expected } = boundary.encoderRead;
        const angles = axes.map(axis => world.getComponent(remote.axisToEntity[axis]?.[0], EncoderComponent)?.angle * 180 / Math.PI);
        finiteJson(angles);
        if (expected.length !== angles.length) throw new Error('Encoder read shape changed');
        maxEncoderErrorDeg = Math.max(maxEncoderErrorDeg, ...angles.map((angle, i) => Math.abs(angle - expected[i])));
        checkpoints.push({ step, axes, angles });
        encoderReads++;
        observationMs += performance.now() - started;
      }
    }
    if (step === steps) break;
    const stop = Math.min(steps, (Math.floor(step / checkpointEvery) + 1) * checkpointEvery,
      workload.boundaries[index]?.step ?? steps);
    const started = performance.now();
    while (step < stop) { world.update(dt); step++; }
    physicsMs += performance.now() - started;
    if (world.getResource('errorState')?.hasError || world.getResource('pauseState')?.paused) {
      throw new Error(`Physics stopped at step ${step}`);
    }
    if (observations && (step % checkpointEvery === 0 || step === steps)) checkpoint();
  }
  return { physicsMs, observationMs, encoderReads, maxEncoderErrorDeg, checkpoints };
}

async function main() {
  const { values: args } = parseArgs({ options: {
    solver: { type: 'string', default: 'native' }, 'native-addon': { type: 'string' },
    machine: { type: 'string', default: 'hp4' }, scene: { type: 'string' },
    commands: { type: 'string' }, events: { type: 'string' }, output: { type: 'string' },
    layering: { type: 'string', default: 'on' }, 'closed-loop': { type: 'boolean', default: false },
    steps: { type: 'string' }, repeats: { type: 'string', default: '3' },
    warmup: { type: 'string', default: '5000' }, 'checkpoint-every': { type: 'string', default: '1000' },
    systems: { type: 'boolean', default: false }, 'heap-sampling': { type: 'boolean', default: false },
    compare: { type: 'string' }, 'absolute-tolerance': { type: 'string', default: '0.000001' },
    'relative-tolerance': { type: 'string', default: '0.000001' },
    'allow-numerical-drift': { type: 'boolean', default: false },
  } });
  if (!args.output || Boolean(args.commands) === Boolean(args.events) || !['hp3', 'hp4'].includes(args.machine)
    || !['on', 'off'].includes(args.layering)) {
    throw new Error('Usage: node [--cpu-prof] scripts/profile_headless.mjs --solver native|wasm|js --machine hp3|hp4 --commands ARRAY.json|--events TRACE.jsonl --layering on|off --output NEW.json [--native-addon PATH] [--systems] [--heap-sampling] [--compare BASELINE.json]');
  }
  const numeric = (key, fallback, minimum = 1) => {
    const value = Number(args[key] ?? fallback);
    if (!Number.isSafeInteger(value) || value < minimum) throw new Error(`${key} must be an integer >= ${minimum}`);
    return value;
  };
  const source = fs.readFileSync(args.commands ?? args.events, 'utf8');
  const workload = loadWorkload(source, Boolean(args.events));
  const steps = numeric('steps', workload.steps);
  if (steps > workload.steps) throw new Error('Requested steps exceed workload; do not pad a trajectory silently');
  const repeats = numeric('repeats', 3), warmup = numeric('warmup', 5000, 0);
  const checkpointEvery = numeric('checkpoint-every', 1000);
  const sceneFile = args.scene ?? path.join(root, `public/usd_scenes/${args.machine}_rigid_body.usda`);
  const scene = fs.readFileSync(sceneFile, 'utf8');
  if (!['js', 'wasm', 'native'].includes(args.solver)) throw new Error('Expected --solver js, wasm or native');
  if (args['native-addon'] && args.solver !== 'native') throw new Error('--native-addon requires --solver native');
  const nativeArtifact = args.solver === 'native' ? loadNativePhysicsAddon(args['native-addon']).artifact : null;
  const wasmBytes = args.solver === 'wasm' ? fs.readFileSync(path.join(root, 'public/wasm/cable_solver.wasm')) : null;
  const wasmModule = wasmBytes ? await compileCableSolverWasm(wasmBytes) : null;
  const options = { lineLayeringEnabled: args.layering === 'on', closedLoopMotorsEnabled: args['closed-loop'],
    profiling: { enabled: args.systems, logEverySteps: 0, slowStepMs: Infinity } };
  const createInstance = (profiling = options.profiling) => createHeadlessWorld(scene, {
    ...options, profiling, cableSolverBackend: nativeArtifact ? new NativePhysicsEngine({ addonPath: nativeArtifact })
      : wasmModule ? new WasmPhysicsEngine(wasmModule) : null,
  });
  if (fs.existsSync(args.output)) throw new Error('Output already exists; preserve earlier measurements');
  fs.mkdirSync(path.dirname(args.output), { recursive: true });
  const baseline = args.compare ? JSON.parse(fs.readFileSync(args.compare)) : null;
  const tolerance = { atol: Number(args['absolute-tolerance']), rtol: Number(args['relative-tolerance']) };
  if (Object.values(tolerance).some(value => !Number.isFinite(value) || value < 0)) throw new Error('Numerical tolerances must be finite and nonnegative');
  const baselineCheckpoints = baseline ? JSON.parse(fs.readFileSync(baseline.runs[0].checkpointFile)) : null;
  const contract = { kind: workload.kind, machine: args.machine, sceneSha256: sha(scene), inputSha256: sha(source),
    steps, layering: args.layering, closedLoop: args['closed-loop'], checkpointEvery, warmup };
  if (baseline && finiteJson(baseline.contract) !== finiteJson(contract)) throw new Error('Baseline inputs/settings differ');
  if (warmup) {
    const instance = createInstance({ enabled: false });
    try { runTrajectory(instance, workload, Math.min(warmup, steps), checkpointEvery, false); }
    finally { instance.world.getResource('cableSolverBackend')?.dispose?.(); }
  }
  const gcEntries = [];
  const observer = new PerformanceObserver(list => gcEntries.push(...list.getEntries()));
  observer.observe({ entryTypes: ['gc'] });
  const sourceFiles = new Set(execFileSync('git', ['ls-files', '-z', 'src/js', 'hp-sim-3d'], { cwd: root, encoding: 'utf8' }).split('\0').filter(Boolean));
  sourceFiles.add('scripts/profile_headless.mjs');
  for (const file of ['src/js/cable_joints_3d/wasm_cable_solver.js', 'src/js/cable_joints_3d/wasm_physics_engine.js',
    'src/js/cable_joints_3d/rust_physics_engine.js', 'src/js/cable_joints_3d/native_physics_engine.js',
    'src/wasm/cable_solver/src/lib.rs', 'src/wasm/cable_solver/src/engine.rs', 'src/wasm/cable_solver/src/abi.rs',
    'src/wasm/cable_solver/Cargo.toml', 'scripts/build_cable_wasm.mjs']) sourceFiles.add(file);
  for (const file of ['src/wasm/cable_solver/native/node_bridge.c', 'scripts/build_native_physics.mjs', 'scripts/train_native_physics.mjs']) sourceFiles.add(file);
  const sourceHash = createHash('sha256');
  for (const file of [...sourceFiles].sort()) sourceHash.update(file).update('\0').update(fs.readFileSync(path.join(root, file)));
  const result = { contract, node: process.version, v8: process.versions.v8, platform: `${os.platform()} ${os.release()} ${os.arch()}`,
    cpu: os.cpus()[0]?.model, execArgv: process.execArgv, revision: execFileSync('git', ['rev-parse', 'HEAD'], { cwd: root, encoding: 'utf8' }).trim(),
    sourceSha256: sourceHash.digest('hex'), solver: args.solver, wasmSha256: wasmBytes ? sha(wasmBytes) : null,
    physicsEngine: args.solver === 'native' ? 'rust-native' : args.solver === 'wasm' ? 'rust-wasm' : 'javascript',
    nativeArtifact, nativeSha256: nativeArtifact ? sha(fs.readFileSync(nativeArtifact)) : null,
    nativeBuild: nativeArtifact && fs.existsSync(`${nativeArtifact}.json`) ? JSON.parse(fs.readFileSync(`${nativeArtifact}.json`)) : null,
    runs: [] };
  for (let repeat = 0; repeat < repeats; repeat++) {
    const loadStart = performance.now();
    const instance = createInstance();
    const loadMs = performance.now() - loadStart;
    let inspector;
    if (args['heap-sampling']) {
      inspector = new Session(); inspector.connect();
      await inspector.post('HeapProfiler.startSampling', { samplingInterval: 32768,
        includeObjectsCollectedByMajorGC: true, includeObjectsCollectedByMinorGC: true });
    }
    const heapBefore = process.memoryUsage();
    const cpuBefore = process.cpuUsage();
    const startUs = Number(process.hrtime.bigint() / 1000n);
    const startMs = performance.now();
    const trajectory = runTrajectory(instance, workload, steps, checkpointEvery);
    const endMs = performance.now();
    const endUs = Number(process.hrtime.bigint() / 1000n);
    const cpuUsageUs = process.cpuUsage(cpuBefore), heapAfter = process.memoryUsage();
    let heapProfile;
    if (inspector) {
      heapProfile = `${args.output}.${repeat}.heapprofile`;
      const { profile } = await inspector.post('HeapProfiler.stopSampling');
      inspector.disconnect(); fs.writeFileSync(heapProfile, JSON.stringify(profile));
    }
    // GC observers deliver asynchronously even when the measured loop is sync.
    await setImmediate(); await setImmediate();
    const gc = gcEntries.filter(entry => entry.startTime >= startMs && entry.startTime < endMs);
    const kinds = { minor: constants.NODE_PERFORMANCE_GC_MINOR, major: constants.NODE_PERFORMANCE_GC_MAJOR,
      incremental: constants.NODE_PERFORMANCE_GC_INCREMENTAL, weakCallbacks: constants.NODE_PERFORMANCE_GC_WEAKCB };
    const { checkpoints, ...metrics } = trajectory;
    const checkpointSha256 = sha(finiteJson(checkpoints));
    const checkpointFile = `${args.output}.${repeat}.checkpoints.json`;
    fs.writeFileSync(checkpointFile, finiteJson(checkpoints) + '\n');
    for (const expected of [result.runs[0]]) {
      if (expected && expected.checkpointSha256 !== checkpointSha256) {
        throw new Error(`Numerical trajectory changed; inspect ${checkpointFile} before accepting speed results`);
      }
    }
    const numericalComparison = baselineCheckpoints ? compareCheckpoints(checkpoints, baselineCheckpoints,
      { ...tolerance, allowNumericalDrift: args['allow-numerical-drift'] }) : null;
    const run = { ...metrics, loadMs, wallMs: endMs - startMs, startMs, endMs, startUs, endUs, cpuUsageUs,
      heapBefore, heapAfter, heapProfile, checkpointFile, checkpointSha256, numericalComparison,
      gc: { count: gc.length, durationMs: gc.reduce((sum, entry) => sum + entry.duration, 0),
        kinds: Object.fromEntries(Object.entries(kinds).map(([name, kind]) => [name, gc.filter(entry => entry.detail.kind === kind).length])) },
      systems: instance.world.getResource('performanceMonitor').snapshot(), dt: instance.dt,
      bakedSceneSha256: sha(instance.bakedScene) };
    result.runs.push(run);
    instance.world.getResource('cableSolverBackend')?.dispose?.();
    fs.writeFileSync(args.output, finiteJson(result) + '\n');
    console.log(`run ${repeat + 1}: ${(run.physicsMs / 1000).toFixed(3)} s physics, ${(steps / run.physicsMs * 1000).toFixed(0)} steps/s, GC ${run.gc.count}/${run.gc.durationMs.toFixed(1)} ms`);
  }
  observer.disconnect();
  const sorted = result.runs.map(run => run.physicsMs).sort((a, b) => a - b);
  result.medianPhysicsMs = (sorted[Math.floor((sorted.length - 1) / 2)] + sorted[Math.floor(sorted.length / 2)]) / 2;
  result.realtimeFactor = steps * result.runs[0].dt * 1000 / result.medianPhysicsMs;
  result.numericalMatch = result.runs.every(run => run.numericalComparison?.matchesTolerance !== false);
  fs.writeFileSync(args.output, finiteJson(result) + '\n');
}

if (process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])) await main();
