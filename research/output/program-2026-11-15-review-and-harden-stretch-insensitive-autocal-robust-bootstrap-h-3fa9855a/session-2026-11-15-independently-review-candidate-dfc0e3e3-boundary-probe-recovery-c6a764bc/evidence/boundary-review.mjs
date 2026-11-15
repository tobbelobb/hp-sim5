// Deterministic command/encoder mocks; no services, physics, or hardware.
// Execute from repository root: node <this file>
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';

const directory = path.dirname(fileURLToPath(import.meta.url));
const revision = 'dfc0e3e3f47be4e7ae915211a42645d40e5e8746';
const manifest = JSON.parse(readFileSync(path.join(directory, 'source-manifest.json')));
for (const [file, digest] of Object.entries(manifest.files)) {
  assert.equal(createHash('sha256').update(readFileSync(file)).digest('hex'), digest, file);
}
execFileSync('git', ['diff', '--quiet', revision, '--', 'autocal/control/primitives',
  'autocal/control/behaviors/force_tuning.mjs', 'autocal/control/behaviors/slip_recovery.mjs', 'integrations']);

const urls = new Map();
const dataModule = source => `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
const slipPath = pathToFileURL(`${process.cwd()}/autocal/control/behaviors/slip_recovery.mjs`).href;
urls.set(slipPath, dataModule('export async function collectWithSlipRecovery(...args) { return globalThis.__boundaryReviewSlip(...args); }'));
const frozen = async file => {
  const source = execFileSync('git', ['show', `${revision}:${file}`], { encoding: 'utf8' })
    .replace(/from (['"])(\.[^'"]+)\1/g, (_, quote, specifier) => {
      const absolute = new URL(specifier, pathToFileURL(`${process.cwd()}/${file}`)).href;
      return `from ${quote}${urls.get(absolute) ?? absolute}${quote}`;
    });
  const url = dataModule(source);
  urls.set(pathToFileURL(`${process.cwd()}/${file}`).href, url);
  return import(url);
};
const { probeSpoolInterval, UnmeasurableSweepError } = await frozen('autocal/control/behaviors/spool_reserve.mjs');
const { collectSweepData, performForceSweep, MACHINE_CONFIGS, sensorForceAt } =
  await frozen('autocal/control/behaviors/sweep_data_collection.mjs');
const { returnMotorsToOriginOneAtATime } = await import(pathToFileURL(`${process.cwd()}/autocal/control/primitives/uncalibrated_actions.mjs`).href);

const logs = [];
const originalLog = console.log.bind(console);
const originalWarn = console.warn.bind(console);
console.log = (...values) => logs.push(values.join(' '));
console.warn = (...values) => logs.push(values.join(' '));

function staticEncoders({ failMovement = false, angles = [100, 200, 300, 400] } = {}) {
  let now = 0;
  const commands = [];
  const send = async line => {
    commands.push(line);
    if (line === 'M666') return { reply: `M666 R${Array(4).fill(180 / Math.PI).join(':')} U1:1:1:1 O1:1:1:1` };
    if (line.startsWith('M569.3')) return { reply: angles.join(' ') };
    if (line.startsWith('G1') && failMovement) throw new Error('Review injected movement failure');
    // Deliberately acknowledged, stationary encoders: commanded moves do not happen.
    return { reply: '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  return { send, commands, angles };
}

const returnTrial = staticEncoders();
const returned = await returnMotorsToOriginOneAtATime(returnTrial.send, {
  motorIds: ['40.0', '41.0', '42.0', '43.0'], axes: ['X', 'Y', 'Z', 'U'],
  mmPerDeg: [1, 1, 1, 1], feed: 1400, speedup: 1,
});
assert.deepEqual(returned, [100, 200, 300, 400]);
assert.equal(returnTrial.commands.filter(command => command.startsWith('G1')).length, 4);

async function collectTrial(name, { rejection = false, failMovement = false, returnToOrigin = false,
  completedSweeps = 3, failGeneric = false } = {}) {
  const sendTrial = staticEncoders({ failMovement });
  const outputFile = path.join(directory, 'tmp', `${name}.json`);
  globalThis.__boundaryReviewSlip = async () => {
    if (failGeneric) throw new Error('Review injected transport failure');
    if (rejection) throw new UnmeasurableSweepError('Review locally unresolved coupling', { retained_points: 0 });
    return { forces: { forceLow: .01, forceMid: .2, forceMax: 2, sensorCollectionForce: 1 },
      recoveries: [], dataPoints: [{ l_drive: 1, l_sensor: 2 }] };
  };
  let error = null;
  try {
    await collectSweepData(sendTrial.send, {
      args: { outputFile, maxSweeps: 1, sensorCollectionForce: 1, fixedTargets: '0,0',
        forceLow: .01, forceMid: .2, forceMax: 2, noAutoTuneForce: true,
        completedSweeps, returnToOrigin },
      machineType: 'hangprinter_3', machineConfig: MACHINE_CONFIGS.hangprinter_3,
      motorIds: ['40.0', '41.0', '42.0', '43.0'], speedup: 1,
    });
  } catch (caught) { error = caught.message; }
  let dataset = null;
  try { dataset = JSON.parse(readFileSync(outputFile)); } catch {}
  return { error, dataset, unchanged_encoder_lengths_mm: sendTrial.angles,
    movements: sendTrial.commands.filter(command => command.startsWith('G1')) };
}

const stationaryRecovery = await collectTrial('stationary-recovery', { rejection: true });
assert.equal(stationaryRecovery.error, null);
assert.equal(stationaryRecovery.dataset.sweeps.length, 0);
assert.equal(stationaryRecovery.dataset.acquisition_rejections.length, 1);
assert.equal(stationaryRecovery.movements.length, 4);
const explicitRecoveryFailure = await collectTrial('explicit-recovery-failure', { rejection: true, failMovement: true });
assert.match(explicitRecoveryFailure.error, /movement failure/);
assert.equal(explicitRecoveryFailure.dataset, null);
const earlyRejection = await collectTrial('initial-unmeasurable', { rejection: true, completedSweeps: 2 });
assert.match(earlyRejection.error, /unresolved coupling/);
assert.equal(earlyRejection.dataset, null);
const genericFailure = await collectTrial('generic-failure', { failGeneric: true });
assert.match(genericFailure.error, /transport failure/);
assert.equal(genericFailure.dataset, null);
const finalReturnFailure = await collectTrial('final-return-failure', { failMovement: true, returnToOrigin: true });
assert.equal(finalReturnFailure.error, null);
assert.equal(finalReturnFailure.dataset.sweeps.length, 1);
assert.ok(logs.some(line => /Warning: failed to return to encoder origin: Review injected movement failure/.test(line)));

async function localProbe(mode) {
  const visited = [];
  let result, error = null;
  try {
    result = await probeSpoolInterval(async target => {
      visited.push(target);
      if (mode === 'transport' && visited.length === 2) throw new Error('Review transport');
      let drive = target, sensor = -3 * target;
      if (mode === 'drive-return' && visited.length === 3) drive += 1;
      if (mode === 'sensor-return' && visited.length === 3) sensor += 1;
      if (mode === 'flat') sensor = 10;
      return { muByMotorDeg: [drive, sensor], sigmaByMotorDeg: [.01, .01] };
    }, { start: 100, away: 120, toward: 80, driveAnchor: 0, sensorAnchor: 1,
      mmPerDeg: [1, 1], sigmaFloorDeg: .01 });
  } catch (caught) { error = caught.message; }
  return { mode, visited, result, error };
}
const localProbes = await Promise.all(['coupled', 'flat', 'sensor-return', 'drive-return', 'transport'].map(localProbe));
assert.equal(localProbes[0].result.accepted, true);
assert.equal(localProbes[1].result.accepted, false);
assert.equal(localProbes[2].result.accepted, false);
assert.match(localProbes[3].error, /held drive/);
assert.match(localProbes[4].error, /transport/);

async function forceSweep({ profile = { drive_mm: [-200, -100], force_n: [1, 3] },
  later = true, gain = 4, meanOffset = 0, rawOffset = 0, targetShift = 0 } = {}) {
  let now = 0, modes = [0, 0, 0, 0], angles = [0, 0, 0, 0], lastSleep = null;
  const commands = [], published = [];
  const send = async line => {
    commands.push({ line, now, angles: [...angles] });
    if (line.startsWith('M569.4')) modes = line.split(' T')[1].split(':').map(Number);
    if (line.startsWith('M569.3')) return { reply: angles.join(' ') };
    const move = line.match(/^G1 H2 Y([-\d.]+)/);
    if (move) {
      const delta = Number(move[1]);
      angles[1] += delta + targetShift;
      angles[2] -= gain * delta;
    }
    return { reply: '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => {
    now += ms;
    if (modes[1] > 0) {
      const next = Math.max(-600, angles[1] - ms / 1000);
      angles[2] -= next - angles[1]; angles[1] = next;
    }
    if (ms === 40 && lastSleep !== 40) angles[1] += meanOffset;
    lastSleep = ms;
  } };
  let result, error = null;
  try {
    result = await performForceSweep(send, { driveAnchor: 1, sensorAnchor: 2 }, {
      axes: ['X', 'Y', 'Z', 'U'], motorIds: ['40.0', '41.0', '42.0', '43.0'],
      sweepPoints: 5, feed: 1400, speedup: 1, mmPerDeg: [1, 1, 1, 1], datasetStartMs: 0,
      forceLow: .01, forceMid: .23, forceMax: 4, fixedAnchors: [0, 3], fixedTargets: [0, 0],
      forbiddenForceAnchors: [3], baseRadiusMm: 30, noiseSampleCount: 16,
      noiseSampleIntervalMs: 40, noiseMinSamples: 4, noiseSigmaFloorDeg: .01,
      sensorForceProfile: profile, probeSpoolReserve: later,
      onPoint: point => published.push(point),
    });
  } catch (caught) { error = { type: caught.constructor.name, message: caught.message }; }
  return { error, result, published, command_count: commands.length };
}

const supported = await forceSweep();
assert.equal(supported.error, null);
assert.equal(supported.result.dataPoints.length, 5);
for (const point of supported.result.dataPoints) {
  assert.ok(point.l_drive >= -200 && point.l_drive <= -100);
  assert.ok(point.l_drive_mu >= -200 && point.l_drive_mu <= -100);
  assert.equal(point.sensor_force_n, sensorForceAt({ drive_mm: [-200, -100], force_n: [1, 3] }, point.drive_setpoint_mm));
}
const unsupported = await forceSweep({ profile: { drive_mm: [100, 200], force_n: [1, 3] } });
assert.equal(unsupported.error.type, 'UnmeasurableSweepError');
assert.equal(unsupported.published.length, 0);
const unresponsive = await forceSweep({ profile: { drive_mm: [-600, 0], force_n: [1, 3] }, gain: 0 });
assert.equal(unresponsive.error.type, 'UnmeasurableSweepError');
assert.equal(unresponsive.published.length, 0);
const lostDrive = await forceSweep({ targetShift: 2 });
assert.notEqual(lostDrive.error?.type, 'UnmeasurableSweepError');
assert.match(lostDrive.error.message, /Drive did not track/);

const output = { revision, evidence_kind: 'deterministic offline command/encoder mocks',
  limits: 'No static/mechanical simulation or hardware validation. The slip wrapper alone is stubbed for collector-boundary tests; recovery primitives are real.',
  returnPrimitive: { returned_lengths_mm: returned, movements: returnTrial.commands.filter(command => command.startsWith('G1')) },
  stationaryRecovery, explicitRecoveryFailure, earlyRejection, genericFailure, finalReturnFailure,
  localProbes, supported, unsupported, unresponsive, lostDrive,
  logs };
mkdirSync(path.join(directory, 'tmp'), { recursive: true });
writeFileSync(path.join(directory, 'boundary-review.json'), JSON.stringify(output, null, 2));
console.log = originalLog; console.warn = originalWarn;
originalLog(JSON.stringify({ assertion_status: 'passed', stationary_recovery_publishes_rejection: true,
  final_return_error_is_swallowed: true, supported_points: supported.result.dataPoints.length,
  output: path.join(directory, 'boundary-review.json') }));
