// Sequential replay matrix: never run tests/profiles concurrently with timings.
import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
const { values: args } = parseArgs({ options: {
  inputs: { type: 'string' }, output: { type: 'string' },
  repeats: { type: 'string', default: '3' },
  'allow-numerical-drift': { type: 'boolean', default: false },
} });
if (!args.inputs || !args.output) throw new Error('Usage: node scripts/benchmark_cable_wasm.mjs --inputs DIR --output NEW_DIR');
const output = path.resolve(args.output);
fs.mkdirSync(output, { recursive: true });
const matrix = [];
for (const machine of ['hp3', 'hp4']) for (const workload of ['logo', 'autocal']) for (const layering of ['on', 'off']) {
  const name = `${machine}-${workload}-${layering}`;
  const input = path.resolve(args.inputs, `${machine}-${workload}.${workload === 'logo' ? 'json' : 'jsonl'}`);
  const baseline = path.join(output, `${name}-js.json`);
  for (const solver of ['js', 'wasm']) {
    console.log(`START ${name} ${solver}`);
    execFileSync(process.execPath, ['scripts/profile_headless.mjs', '--machine', machine,
      workload === 'logo' ? '--commands' : '--events', input, '--layering', layering,
      '--solver', solver, '--repeats', args.repeats, '--output', path.join(output, `${name}-${solver}.json`),
      ...(solver === 'wasm' ? ['--compare', baseline] : []),
      ...(args['allow-numerical-drift'] ? ['--allow-numerical-drift'] : [])], { cwd: root, stdio: 'inherit' });
  }
  const js = JSON.parse(fs.readFileSync(baseline));
  const wasm = JSON.parse(fs.readFileSync(path.join(output, `${name}-wasm.json`)));
  matrix.push({ name, steps: js.contract.steps, jsMs: js.runs.map(run => run.physicsMs),
    wasmMs: wasm.runs.map(run => run.physicsMs), jsMedianMs: js.medianPhysicsMs,
    wasmMedianMs: wasm.medianPhysicsMs, speedup: js.medianPhysicsMs / wasm.medianPhysicsMs,
    reductionPercent: 100 * (1 - wasm.medianPhysicsMs / js.medianPhysicsMs),
    jsCheckpointSha256: js.runs[0].checkpointSha256, wasmCheckpointSha256: wasm.runs[0].checkpointSha256,
    numericalComparison: wasm.runs.map(run => run.numericalComparison), numericalMatch: wasm.numericalMatch });
  fs.writeFileSync(path.join(output, 'matrix.json'), JSON.stringify(matrix, null, 2) + '\n');
}
console.table(matrix.map(({ name, jsMedianMs, wasmMedianMs, speedup, reductionPercent }) => ({
  name, jsSeconds: (jsMedianMs / 1000).toFixed(3), wasmSeconds: (wasmMedianMs / 1000).toFixed(3),
  speedup: speedup.toFixed(2), reductionPercent: reductionPercent.toFixed(1),
})));
