// Frozen-candidate encoder/command mocks. No physics, service, or hardware.
// Run from repository root: node <this file>
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { pathToFileURL } from 'node:url';

const revision = '8338be8f';
const sources = {};
const read = file => {
  const source = execFileSync('git', ['show', `${revision}:${file}`], { encoding: 'utf8' });
  sources[file] = createHash('sha256').update(source).digest('hex');
  return source;
};
// Guard live imports so concurrent changes cannot silently affect the review.
execFileSync('git', ['diff', '--quiet', revision, '--', 'autocal/control/primitives',
  'autocal/control/behaviors/force_tuning.mjs', 'autocal/control/behaviors/slip_recovery.mjs',
  'integrations']);
const moduleURLs = new Map();
const importFrozen = async file => {
  const source = read(file).replace(/from (['"])(\.[^'"]+)\1/g,
    (_, quote, specifier) => {
      const url = new URL(specifier, pathToFileURL(`${process.cwd()}/${file}`)).href;
      return `from ${quote}${moduleURLs.get(url) ?? url}${quote}`;
    });
  const url = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  moduleURLs.set(pathToFileURL(`${process.cwd()}/${file}`).href, url);
  return import(url);
};
const { probeSpoolInterval } = await importFrozen('autocal/control/behaviors/spool_reserve.mjs');
const { performForceSweep } = await importFrozen('autocal/control/behaviors/sweep_data_collection.mjs');
const logs = [];
const writeResult = console.log.bind(console);
console.log = (...args) => logs.push(args.join(' '));

async function trial({ pulloutLimit = 600, sensorGain = 4, meanCrossing = false,
  probeSpoolReserve = false, sensorForceProfile = null, forceLosesCoupling = false,
  sweepPoints = 3, forceDependentOffsets = false, sensorSpoolGearRatio = 1 } = {}) {
  let now = 0, modes = [0, 0, 0, 0], angles = [0, 0, 0, 0];
  let lastSleepMs = null, noiseBatch = 0;
  const commands = [], points = [];
  const sensorAt = drive => pulloutLimit - sensorGain * (drive + pulloutLimit)
    + (forceDependentOffsets ? modes[2] * 3 : 0);
  const send = async line => {
    commands.push({ line, now, modes: [...modes], angles: [...angles] });
    if (line.startsWith('M569.4')) {
      modes = line.split(' T')[1].split(':').map(Number);
      if (forceLosesCoupling && modes[1] === 0 && modes[2] > 2.5) angles[2] = pulloutLimit;
      else if (forceDependentOffsets && modes[1] === 0) angles[2] = sensorAt(angles[1]);
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
        if (meanCrossing && noiseBatch === 2) angles[2] += .1;
      }
      lastSleepMs = ms;
    },
  };
  let result, error = null;
  try {
    result = await performForceSweep(send, { driveAnchor: 1, sensorAnchor: 2 }, {
      axes: ['X', 'Y', 'Z', 'U'], motorIds: ['40.0', '41.0', '42.0', '43.0'],
      sweepPoints, feed: 1400, speedup: 1, mmPerDeg: [1, 1, 1, 1], datasetStartMs: 0,
      forceLow: .01, forceMid: .23, forceMax: 4, fixedAnchors: [0, 3], fixedTargets: [0, 0],
      forbiddenForceAnchors: [3], baseRadiusMm: 30,
      noiseSampleCount: 16, noiseSampleIntervalMs: 40, noiseMinSamples: 4,
      noiseSigmaFloorDeg: .01, probeSpoolReserve, sensorForceProfile,
      sensorSpoolGearRatio,
      onPoint: point => points.push(point),
    });
  } catch (caught) { error = caught.message; }
  return {
    error, driveRange: result?.driveRange, spoolReserve: result?.spoolReserve,
    points: points.map(point => ({ drive_setpoint_mm: point.drive_setpoint_mm,
      l_drive: point.l_drive, l_drive_mu: point.l_drive_mu, l_sensor: point.l_sensor,
      l_sensor_mu: point.l_sensor_mu, sensor_force_n: point.sensor_force_n,
      spool_reserve_method: point.spool_reserve_method,
      raw_rewind_deg: pulloutLimit - point.raw_angles_deg[2],
      mean_rewind_deg: pulloutLimit - point.mu[2], sample_count: point.sample_count,
      clock_time_ms: point.clock_time_ms, timestamp_ms: point.timestamp_ms,
      noise_clock_start_ms: point.noise_clock_start_ms, noise_clock_end_ms: point.noise_clock_end_ms,
      quiet_clock_matches_encoder_read: commands.some(c => c.line.startsWith('M569.3')
        && c.now === point.clock_time_ms && c.angles.every((v, i) => v === point.raw_angles_deg[i])),
      mode_changes_after_raw: commands.filter(c => c.line.startsWith('M569.4')
        && c.now > point.clock_time_ms && c.now < point.noise_clock_end_ms).length,
    })),
    relative_moves_mm: commands.map(({line}) => line.match(/^G1 H2 Y([-\d.]+)/)?.[1])
      .filter(value => value !== undefined).map(Number),
    measurement_mode_changes: commands.filter(({line}) => line.startsWith('M569.4'))
      .map(({line, now, angles}) => ({ line, now, drive_mm: angles[1] })),
  };
}

const meanCrossing = await trial({ meanCrossing: true, sensorGain: 180.01 / 47.124 });
assert.match(meanCrossing.error, /half-turn interval/);
assert.equal(meanCrossing.points.length, 0);
const meanBaseline = await trial({ sensorGain: 180.01 / 47.124 });
assert.equal(meanBaseline.error, null);
assert.ok(meanBaseline.points.every(p => p.mean_rewind_deg >= 180));
const gearedInitial = await trial({ sensorSpoolGearRatio: 2 });
assert.equal(gearedInitial.error, null);
assert.equal(gearedInitial.spoolReserve.required_sensor_rewind_mm, 360);
assert.ok(gearedInitial.points.every(p => p.raw_rewind_deg >= 360 && p.mean_rewind_deg >= 360));
const consumedInterval = await trial({ pulloutLimit: 20, sensorGain: 10 });
assert.match(consumedInterval.error, /No measurement interval/);
assert.equal(consumedInterval.points.length, 0);
const shortBaseline = await trial({ pulloutLimit: 20, sensorGain: 4 });
assert.match(shortBaseline.error, /too short/);
const profile = { drive_anchor: 1, sensor_anchor: 2, drive_mm: [-600, 0], force_n: [1, 3] };
const changedPreload = await trial({ probeSpoolReserve: true, sensorForceProfile: profile,
  forceLosesCoupling: true, sweepPoints: 4 });
assert.match(changedPreload.error, /Insufficient coupled measurement points/);
assert.ok(changedPreload.points.every(p => p.sensor_force_n <= 2.5));
const changedPreloadWithoutProbe = await trial({ sensorForceProfile: profile,
  forceLosesCoupling: true, sweepPoints: 4 });
assert.match(changedPreloadWithoutProbe.error, /half-turn interval/);
const laterShortArc = await trial({ probeSpoolReserve: true, pulloutLimit: 20,
  sensorGain: 10, sweepPoints: 5 });
assert.equal(laterShortArc.error, null);
assert.equal(laterShortArc.points.length, 5);
assert.equal(new Set(laterShortArc.points.map(p => p.l_drive)).size, 5);
assert.ok(laterShortArc.points.every(p => p.spool_reserve_method === 'local_encoder_coupling'));
const unresponsiveFrontier = await trial({ probeSpoolReserve: true, sensorGain: 0, sweepPoints: 5 });
assert.match(unresponsiveFrontier.error, /Insufficient coupled measurement points/);
assert.equal(unresponsiveFrontier.points.length, 0);
const tableSupport = [-200, -100];
const outsideProfileSupport = await trial({ probeSpoolReserve: true, sensorGain: 1, sweepPoints: 5,
  sensorForceProfile: { drive_anchor: 1, sensor_anchor: 2, drive_mm: tableSupport, force_n: [1, 3] } });
assert.equal(outsideProfileSupport.error, null);
assert.ok(outsideProfileSupport.points.some(p => p.drive_setpoint_mm < tableSupport[0]));
const preloadAndReuse = await trial({ probeSpoolReserve: true, sensorGain: 1,
  sensorForceProfile: profile, sweepPoints: 10, forceDependentOffsets: true });
assert.equal(preloadAndReuse.error, null);
assert.ok(preloadAndReuse.points.every(p => p.quiet_clock_matches_encoder_read
  && p.mode_changes_after_raw === 0 && p.noise_clock_start_ms === p.clock_time_ms
  && p.noise_clock_end_ms - p.noise_clock_start_ms === 640));
for (const p of preloadAndReuse.points.filter(p => p.spool_reserve_method === 'local_encoder_coupling')) {
  const proof = preloadAndReuse.spoolReserve.coupling_probes.find(probe => probe.accepted
    && probe.sensor_force_n === p.sensor_force_n
    && Math.abs(probe.tested_drive_mm[0] - p.drive_setpoint_mm) < .001);
  assert.ok(proof);
}

async function probeCase(mode) {
  const visited = [];
  const move = async target => {
    visited.push(target);
    if (mode === 'transport' && visited.length === 2) throw new Error('Disconnected');
    let sensor = -4 * target;
    if (mode === 'flat') sensor = 0;
    if (mode === 'unresolved') sensor = -.0001 * target;
    if (mode === 'drift' && visited.length > 1 && target === 100) sensor += 1;
    return { muByMotorDeg: [target, sensor], sigmaByMotorDeg: [.01, .01] };
  };
  let result, error = null;
  try { result = await probeSpoolInterval(move, { start: 100, away: 120, toward: 80,
    driveAnchor: 0, sensorAnchor: 1, mmPerDeg: [1, 1], sigmaFloorDeg: .01 }); }
  catch (caught) { error = caught.message; }
  return { result, error, visited };
}
const probeCases = Object.fromEntries(await Promise.all(['coupled', 'flat', 'unresolved', 'drift', 'transport']
  .map(async mode => [mode, await probeCase(mode)])));
assert.equal(probeCases.coupled.result.accepted, true);
assert.ok(['flat', 'unresolved'].every(mode => probeCases[mode].result.accepted === false
  && probeCases[mode].visited.at(-1) === 100));
assert.match(probeCases.drift.error, /did not return/);
assert.match(probeCases.transport.error, /Disconnected/);
assert.deepEqual(probeCases.transport.visited, [100, 120]);

writeResult(JSON.stringify({ candidate_revision: revision,
  evidence_type: 'synthetic command/encoder mocks; no physics, service, or hardware',
  source_sha256: sources, logs, meanCrossing, meanBaseline, consumedInterval, shortBaseline,
  gearedInitial,
  changedPreload, changedPreloadWithoutProbe, laterShortArc, unresponsiveFrontier,
  tableSupport, outsideProfileSupport,
  preloadAndReuse, probeCases }, null, 2));
