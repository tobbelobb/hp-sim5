import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import { execFileSync } from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { loadWorkload, runTrajectory, compareCheckpoints } from '../../scripts/profile_headless.mjs';
import { summarizeCpu, summarizeHeap } from '../../scripts/summarize_v8_profile.mjs';

const events = rows => rows.map(row => JSON.stringify({ epoch: 0, ...row })).join('\n');

test('profiling selects the requested engine and records the loaded native artifact', () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'hp-sim5-profile-'));
  const input = path.join(directory, 'commands.json');
  fs.writeFileSync(input, JSON.stringify(Array.from({ length: 10 }, (_, i) => ({ type: 'Move', A: i * .0001 }))));
  const script = fileURLToPath(new URL('../../scripts/profile_headless.mjs', import.meta.url));
  for (const solver of ['js', 'wasm', 'native']) {
    const output = path.join(directory, `${solver}.json`);
    execFileSync(process.execPath, [script, '--solver', solver, '--machine', 'hp3', '--commands', input,
      '--warmup', '0', '--repeats', '1', '--checkpoint-every', '2', '--output', output], {
      // Explicit Wasm/JS profiling must work even if native cannot be loaded.
      env: { ...process.env, ...(solver === 'native' ? {} : { HP_SIM5_NATIVE_ENGINE: path.join(directory, 'missing.node') }) },
    });
    const result = JSON.parse(fs.readFileSync(output));
    assert.equal(result.solver, solver);
    assert.equal(result.physicsEngine, { js: 'javascript', wasm: 'rust-wasm', native: 'rust-native' }[solver]);
    assert.equal(Boolean(result.nativeSha256), solver === 'native');
    assert.equal(Boolean(result.wasmSha256), solver === 'wasm');
    if (solver === 'native') assert.equal(result.nativeBuild.artifactSha256, result.nativeSha256);
  }
  const selected = path.join(directory, 'selected.node');
  const native = JSON.parse(fs.readFileSync(path.join(directory, 'native.json')));
  fs.copyFileSync(native.nativeArtifact, selected);
  fs.copyFileSync(`${native.nativeArtifact}.json`, `${selected}.json`);
  const output = path.join(directory, 'selected.json');
  execFileSync(process.execPath, [script, '--solver', 'native', '--native-addon', selected,
    '--machine', 'hp3', '--commands', input, '--warmup', '0', '--repeats', '1', '--output', output], {
    env: { ...process.env, HP_SIM5_NATIVE_ENGINE: path.join(directory, 'missing.node') },
  });
  const result = JSON.parse(fs.readFileSync(output));
  assert.equal(result.nativeArtifact, selected);
  assert.equal(result.nativeSha256, native.nativeSha256);
});

test('replay preserves same-step FIFO, idle settling, settings and encoder reads', () => {
  const source = fs.readFileSync(fileURLToPath(new URL('../../public/usd_scenes/hp3_rigid_body.usda', import.meta.url)), 'utf8');
  const workload = loadWorkload(events([
    { step: 0, type: 'bridge_payload', payload: { type: 'simulation_settings', closedLoopMotorsEnabled: true } },
    ...[.01, .02].map(A => ({ step: 1, type: 'bridge_payload', payload: { type: 'command', command: { type: 'Move', A } } })),
    { step: 5, type: 'encoder_response', axes: ['A'], angles_deg: [0] },
    { step: 10, type: 'service_stopped' },
  ]), true);
  const instance = createHeadlessWorld(source);
  const replay = runTrajectory(instance, workload, 10, 2);
  const direct = createHeadlessWorld(source, { closedLoopMotorsEnabled: true });
  direct.world.update(direct.dt);
  direct.remote.addCommand({ type: 'Move', A: .01 }); direct.remote.addCommand({ type: 'Move', A: .02 });
  const explicit = runTrajectory(direct, { boundaries: [] }, 9, 9);
  assert.deepEqual(replay.checkpoints.at(-1).snapshot, explicit.checkpoints.at(-1).snapshot);
  assert.equal(replay.encoderReads, 1);
  assert.deepEqual(instance.remote.history.map(command => command.A), [.01, .02]);
  assert.notDeepEqual(replay.checkpoints[0].snapshot.velocities, replay.checkpoints.at(-1).snapshot.velocities);
});

test('reviewed drift records a failed tolerance check and still rejects clock changes', () => {
  const expected = [{ step: 1, encoder: 0.1 }], actual = [{ step: 1, encoder: 0.102 }];
  assert.throws(() => compareCheckpoints(actual, expected), /Numerical trajectory/);
  const result = compareCheckpoints(actual, expected, { allowNumericalDrift: true });
  assert.equal(result.matchesTolerance, false);
  assert.equal(result.outsideTolerance, 1);
  assert(result.physicalErrors.encoder_rad > .001);
  assert.throws(() => compareCheckpoints([{ step: 2, encoder: .1 }], expected, { allowNumericalDrift: true }), /trajectory/);
});

test('replay rejects reset epochs and backwards clocks', () => {
  assert.throws(() => loadWorkload(events([{ step: 2 }, { step: 1 }]), true), /monotonic/);
  assert.throws(() => loadWorkload(events([{ step: 0 }, { step: 1, epoch: 1 }]), true), /epoch/);
});

test('CPU summary excludes startup and attributes leaf and ancestor time', () => {
  const frame = functionName => ({ functionName, url: 'physics.js', lineNumber: 0 });
  const summary = summarizeCpu({ startTime: 0, nodes: [
    { id: 1, callFrame: frame('system'), children: [2] }, { id: 2, callFrame: frame('math') },
  ], samples: [1, 2, 2], timeDeltas: [10, 10, 10] }, [{ startUs: 10, endUs: 25 }]);
  assert.equal(summary.sampledMs, .015);
  assert.equal(summary.bySelf[0].selfUs, 15); assert.equal(summary.bySelf[0].selfPercent, 100);
  assert.equal(summary.byInclusive.find(row => row.function.startsWith('system')).inclusiveUs, 15);
});

test('heap summary aggregates allocation sites across call stacks', () => {
  const summary = summarizeHeap({ head: { callFrame: { functionName: 'root', lineNumber: -1 }, selfSize: 20,
    children: [1, 2].map(() => ({ callFrame: { functionName: 'vector', url: 'math.js', lineNumber: 0 }, selfSize: 40 })) } });
  assert.equal(summary.estimatedAllocatedBytes, 100);
  assert.equal(summary.bySelf[0].selfBytes, 80);
});
