// Pure command/encoder mocks. No simulator, service, hardware, or physics.
// Run from the repository root at candidate 6ab979c3:
// node <investigation>/evidence/reserve-review.mjs
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { pathToFileURL } from 'node:url';

const candidateRevision = '6ab979c367c286451d05a06b839ca770c51eeab3';
const modulePath = `${process.cwd()}/autocal/control/behaviors/sweep_data_collection.mjs`;
// Freeze the reviewed entrypoint despite concurrent fixes in the shared checkout.
// All other imported implementations must still match the candidate revision.
execFileSync('git', ['diff', '--quiet', candidateRevision, '--', 'autocal/control/primitives',
  'autocal/control/behaviors/spool_reserve.mjs', 'autocal/control/behaviors/force_tuning.mjs',
  'autocal/control/behaviors/slip_recovery.mjs', 'integrations']);
const candidateSource = execFileSync('git', ['show',
  `${candidateRevision}:autocal/control/behaviors/sweep_data_collection.mjs`], { encoding: 'utf8' });
const sourceForImport = candidateSource.replace(/from (['"])(\.[^'"]+)\1/g,
  (_, quote, specifier) => `from ${quote}${new URL(specifier, pathToFileURL(modulePath))}${quote}`);
const { performForceSweep } = await import(`data:text/javascript;base64,${Buffer.from(sourceForImport).toString('base64')}`);
const collectorLog = [];
const writeResult = console.log.bind(console);
console.log = (...args) => collectorLog.push(args.join(' '));

async function trial({ pulloutLimit = 600, sensorGain = 4, meanCrossing = false,
  probeSpoolReserve = false, sensorForceProfile = null, forceLosesCoupling = false,
  sweepPoints = 3 } = {}) {
  let now = 0;
  let modes = [0, 0, 0, 0];
  let angles = [0, 0, 0, 0];
  let lastSleepMs = null;
  let noiseBatch = 0;
  const commands = [];
  const points = [];
  const sensorAt = drive => pulloutLimit - sensorGain * (drive + pulloutLimit);
  const send = async line => {
    commands.push({ line, now, modes: [...modes], angles: [...angles] });
    if (line.startsWith('M569.4')) {
      modes = line.split(' T')[1].split(':').map(Number);
      // Emulate a repeatable force-dependent loss of sensor coupling only at
      // the larger force later used for measurement. The local probe is linear.
      if (forceLosesCoupling && modes[1] === 0 && modes[2] > 2.5) angles[2] = pulloutLimit;
    }
    if (line.startsWith('M569.3')) return { reply: angles.join(' ') };
    const move = line.match(/^G1 H2 Y([-\d.]+)/);
    if (move) {
      angles[1] += Number(move[1]);
      angles[2] = forceLosesCoupling && modes[2] > 2.5 ? pulloutLimit : sensorAt(angles[1]);
    }
    return { reply: '' };
  };
  send.simulationClock = {
    now: () => now,
    sleep: async ms => {
      now += ms;
      if (modes[1] > 0) {
        const next = Math.max(-pulloutLimit, angles[1] - ms / 1000);
        angles[2] -= next - angles[1];
        angles[1] = next;
      }
      if (ms === 40 && lastSleepMs !== 40) {
        noiseBatch += 1;
        // The point's settled raw value passes by .01 deg. A .1 deg jump
        // immediately afterward puts the mean used by fitting outside reserve.
        if (meanCrossing && noiseBatch === 2) angles[2] += .1;
      }
      lastSleepMs = ms;
    },
  };
  let result;
  let error = null;
  try {
    result = await performForceSweep(send, { driveAnchor: 1, sensorAnchor: 2 }, {
      axes: ['X', 'Y', 'Z', 'U'], motorIds: ['40.0', '41.0', '42.0', '43.0'],
      sweepPoints, feed: 1400, speedup: 1, mmPerDeg: [1, 1, 1, 1], datasetStartMs: 0,
      forceLow: .01, forceMid: .23, forceMax: 4, fixedAnchors: [0, 3], fixedTargets: [0, 0],
      forbiddenForceAnchors: [3], baseRadiusMm: 30,
      noiseSampleCount: 16, noiseSampleIntervalMs: 40, noiseMinSamples: 4,
      noiseSigmaFloorDeg: .01, probeSpoolReserve, sensorForceProfile,
      onPoint: point => points.push(point),
    });
  } catch (caught) { error = caught.message; }
  return {
    error,
    driveRange: result?.driveRange,
    spoolReserve: result?.spoolReserve,
    points: points.map(point => ({ drive_setpoint_mm: point.drive_setpoint_mm,
      l_drive: point.l_drive, l_sensor: point.l_sensor,
      l_sensor_mu: point.l_sensor_mu, sensor_force_n: point.sensor_force_n,
      raw_rewind_deg: pulloutLimit - point.raw_angles_deg[2],
      mean_rewind_deg: pulloutLimit - point.mu[2], sample_count: point.sample_count })),
    relative_moves_mm: commands.map(({line}) => line.match(/^G1 H2 Y([-\d.]+)/)?.[1])
      .filter(value => value !== undefined).map(Number),
    // Keep the decisive measurement-force transitions without large command logs.
    measurement_mode_changes: commands.filter(({line}) => line.startsWith('M569.4'))
      .map(({line, now, angles}) => ({ line, now, drive_mm: angles[1] })),
  };
}

const meanCrossing = await trial({ meanCrossing: true, sensorGain: 180.01 / 47.124 });
assert.equal(meanCrossing.error, null);
assert.ok(meanCrossing.points[0].raw_rewind_deg >= 180);
assert.ok(meanCrossing.points[0].mean_rewind_deg < 180);
const meanBaseline = await trial({ sensorGain: 180.01 / 47.124 });
assert.equal(meanBaseline.error, null);
assert.ok(meanBaseline.points.every(point => point.mean_rewind_deg >= 180));

const consumedInterval = await trial({ pulloutLimit: 20, sensorGain: 10 });
assert.equal(consumedInterval.error, null);
assert.equal(new Set(consumedInterval.points.map(point => point.drive_setpoint_mm)).size, 1);
assert.equal(consumedInterval.points[0].drive_setpoint_mm, 0);
const shortBaseline = await trial({ pulloutLimit: 20, sensorGain: 4 });
assert.match(shortBaseline.error, /too short/);
assert.equal(shortBaseline.points.length, 0);

const profile = { drive_anchor: 1, sensor_anchor: 2, drive_mm: [-600, 0], force_n: [1, 3] };
const changedPreload = await trial({ probeSpoolReserve: true, sensorForceProfile: profile,
  forceLosesCoupling: true, sweepPoints: 4 });
assert.equal(changedPreload.error, null);
assert.equal(changedPreload.spoolReserve.probe.accepted, true);
assert.ok(changedPreload.points.some(point => point.sensor_force_n > 2.5
  && point.mean_rewind_deg < 180));
const changedPreloadWithoutProbe = await trial({ sensorForceProfile: profile,
  forceLosesCoupling: true, sweepPoints: 4 });
assert.match(changedPreloadWithoutProbe.error, /left the reserved half-turn interval/);

const rejectedWithoutCoupling = await trial({ probeSpoolReserve: true, sensorGain: 0 });
assert.match(rejectedWithoutCoupling.error, /too short/);
assert.equal(rejectedWithoutCoupling.points.length, 0);

writeResult(JSON.stringify({
  candidate_revision: candidateRevision,
  source_sha256: createHash('sha256').update(candidateSource).digest('hex'),
  evidence_type: 'synthetic command/encoder mocks; no physics or hardware',
  collector_log: collectorLog,
  meanCrossing, meanBaseline, consumedInterval, shortBaseline, changedPreload,
  changedPreloadWithoutProbe, rejectedWithoutCoupling,
}, null, 2));
