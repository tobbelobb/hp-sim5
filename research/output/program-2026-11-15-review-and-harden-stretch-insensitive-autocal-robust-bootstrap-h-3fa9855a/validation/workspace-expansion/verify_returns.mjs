// Parent verification of the independent review's return counterexamples.
// Uses real current collector/return code, with only sweep acquisition stubbed.
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, mkdtempSync, existsSync, rmSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { pathToFileURL, fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';

const root = process.cwd();
const sourcePath = 'autocal/control/behaviors/sweep_data_collection.mjs';
const stub = `data:text/javascript;base64,${Buffer.from('export async function collectWithSlipRecovery(...args) { return globalThis.__returnCheckSlip(...args); }').toString('base64')}`;
const source = readFileSync(sourcePath, 'utf8').replace(/from (['"])(\.[^'"]+)\1/g, (_, quote, specifier) => {
  const absolute = new URL(specifier, pathToFileURL(`${root}/${sourcePath}`)).href;
  return `from ${quote}${absolute.endsWith('/behaviors/slip_recovery.mjs') ? stub : absolute}${quote}`;
});
const { collectSweepData, MACHINE_CONFIGS } = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const { UnmeasurableSweepError } = await import(pathToFileURL(`${root}/autocal/control/behaviors/spool_reserve.mjs`));
const { returnMotorsToOriginOneAtATime } = await import(pathToFileURL(`${root}/autocal/control/primitives/uncalibrated_actions.mjs`));
const directory = mkdtempSync(path.join(os.tmpdir(), 'verify-autocal-returns-'));
const logs = [];
const log = console.log;
console.log = (...args) => logs.push(args.join(' '));
function stationary(failMovement) {
  let now = 0;
  const commands = [];
  const send = async line => {
    commands.push(line);
    if (line === 'M666') return { reply: `M666 R${Array(4).fill(180 / Math.PI).join(':')} U1:1:1:1 O1:1:1:1` };
    if (line.startsWith('M569.3')) return { reply: '100 200 300 400' };
    if (line.startsWith('G1') && failMovement) throw new Error('Injected return movement failure');
    return { reply: '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  return { send, commands };
}
async function trial(name, { rejection, failMovement, maxSweeps = 1 }) {
  const { send, commands } = stationary(failMovement);
  const outputFile = path.join(directory, `${name}.json`);
  globalThis.__returnCheckSlip = async () => {
    if (rejection) throw new UnmeasurableSweepError('Unresolved local coupling');
    return { forces: { forceLow: .01, forceMid: .2, forceMax: 2 },
      recoveries: [], dataPoints: [{ l_drive: 1, l_sensor: 2 }] };
  };
  let error = null;
  try {
    await collectSweepData(send, { args: { outputFile, maxSweeps, fixedTargets: '0,0',
      forceLow: .01, forceMid: .2, forceMax: 2, noAutoTuneForce: true, completedSweeps: 3, returnToOrigin: true },
      machineType: 'hangprinter_3', machineConfig: MACHINE_CONFIGS.hangprinter_3,
      motorIds: ['40.0', '41.0', '42.0', '43.0'], speedup: 1 });
  } catch (caught) { error = caught.message; }
  assert.ok(error, `${name} must fail`);
  const published = existsSync(outputFile);
  if (rejection || maxSweeps > 1) assert.equal(published, false);
  else assert.equal(JSON.parse(readFileSync(outputFile)).sweeps.length, 1);
  return { error, partial_measurement_file: published,
    movements: commands.filter(line => line.startsWith('G1')) };
}
try {
  const { send, commands } = stationary(false);
  await assert.rejects(returnMotorsToOriginOneAtATime(send, {
    motorIds: ['40.0', '41.0', '42.0', '43.0'], axes: ['X', 'Y', 'Z', 'U'],
    mmPerDeg: [1,1,1,1], feed: 1400,
  }), /did not reach its encoder origin/);
  assert.equal(commands.filter(line => line.startsWith('G1')).length, 1);
  const results = {};
  results.boundary_unmoved = await trial('boundary-unmoved', { rejection: true, failMovement: false });
  results.boundary_exception = await trial('boundary-exception', { rejection: true, failMovement: true });
  results.final_unmoved = await trial('final-unmoved', { rejection: false, failMovement: false });
  results.final_exception = await trial('final-exception', { rejection: false, failMovement: true });
  results.between_exception = await trial('between-exception', { rejection: false, failMovement: true, maxSweeps: 2 });
  const files = [sourcePath, 'autocal/control/primitives/uncalibrated_actions.mjs'];
  const output = { source_revision: execFileSync('git', ['rev-parse', 'HEAD'], { encoding: 'utf8' }).trim(),
    scope: 'Offline failure-injection verification derived from the independent review. No physics or hardware.',
    sources: Object.fromEntries(files.map(file => [file, createHash('sha256').update(readFileSync(file)).digest('hex')])),
    primitive_failed_after_first_move: true, results };
  writeFileSync(path.join(path.dirname(fileURLToPath(import.meta.url)), 'return-verification.json'), `${JSON.stringify(output, null, 2)}\n`);
  log(JSON.stringify(output, null, 2));
} finally { console.log = log; rmSync(directory, { recursive: true, force: true }); }
