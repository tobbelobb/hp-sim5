import { performForceSweep, sensorForceAt, sensorForceProfileFor } from '../../behaviors/sweep_data_collection.mjs';
import { EncoderStabilityTimeoutError, FixedAnchorDriftError } from '../../primitives/uncalibrated_actions.mjs';

// Endpoint travel may continue indefinitely under force. Measurements must still
// settle after the drive is held, and every fixed-anchor reading stays guarded.
function movingPullout({ fixedDrift = false, measurementDrift = false, transportError = null, stalled = false, preparationDrift = false, sensorForceProfile = null, sensorSpoolGearRatio = 1, probeSpoolReserve = false, pulloutLimit = Infinity, sensorGain = 4, meanCrossing = false, forceLosesCoupling = false, sweepPoints = 2 } = {}) {
  let now = 0;
  let modes = preparationDrift ? [0, 2.4, 2.4, 0] : [0, 0, 0, 0];
  let preparing = preparationDrift;
  let angles = [0, 0, 0, 0];
  const commands = [];
  const points = [];
  let lastSleepMs = null, noiseBatch = 0;
  const send = async line => {
    commands.push({ line, now });
    if (line.startsWith('M569.4')) {
      modes = line.split(' T')[1].split(':').map(Number);
      if (forceLosesCoupling && modes[2] > 2.5) angles[2] = 600.5;
      preparing = false;
    }
    if (line.startsWith('M569.3')) {
      if (modes[1] > 0 && transportError) throw transportError;
      return { reply: angles.join(' ') };
    }
    const move = line.match(/^G1 H2 Y([-\d.]+)/);
    if (move) {
      angles[1] += Number(move[1]);
      if (!(forceLosesCoupling && modes[2] > 2.5)) angles[2] -= sensorGain * Number(move[1]);
    }
    return { reply: '' };
  };
  send.simulationClock = {
    now: () => now,
    sleep: async ms => {
      if (stalled && modes[1] > 0) throw new Error('Simulation advance stalled');
      now += ms;
      if (preparing && now >= 60000) return;
      if (modes[1] > 0) {
        const next = Math.max(-pulloutLimit, angles[1] - ms / 1000);
        angles[2] -= next - angles[1];
        angles[1] = next;
        if (fixedDrift) angles[0] = 2;
      } else if (measurementDrift && modes[2] > 0) angles[2] += ms / 1000;
      if (ms === 40 && lastSleepMs !== 40) {
        noiseBatch += 1;
        if (meanCrossing && noiseBatch === 2) angles[2] += .1;
      }
      lastSleepMs = ms;
    },
  };
  const run = () => performForceSweep(send, { driveAnchor: 1, sensorAnchor: 2 }, {
    axes: ['X', 'Y', 'Z', 'U'], motorIds: ['40.0', '41.0', '42.0', '43.0'],
    sweepPoints, feed: 1400, speedup: 1, mmPerDeg: [1, 1, 1, 1], datasetStartMs: 0,
    forceLow: .01, forceMid: .23, forceMax: 2.4, fixedAnchors: [0, 3], fixedTargets: [0, 0],
    forbiddenForceAnchors: [3], baseRadiusMm: 30, projectZeroTension: false,
    noiseSampleCount: 16, noiseSampleIntervalMs: 40, noiseMinSamples: 4, noiseSigmaFloorDeg: .01,
    onPoint: point => points.push(point), sensorForceProfile, sensorSpoolGearRatio, probeSpoolReserve,
  });
  return { run, commands, points };
}

test('bounds moving endpoint search, then holds and settles before recording all samples', async () => {
  const trial = movingPullout();
  const result = await trial.run();
  const hold = trial.commands.find(command => command.now > 600000 && command.line === 'M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.46:0.0');
  expect(hold).toBeDefined();
  expect(result.dataPoints).toHaveLength(2);
  expect(result.driveRange.start).toBeCloseTo(-600.5);
  expect(result.dataPoints[0].raw_angles_deg[2]).toBeLessThanOrEqual(600.5 - 180);
  expect(trial.points[0].timestamp_ms).toBeGreaterThanOrEqual(hold.now + 1500);
  expect(result.dataPoints.every(point => point.sample_count === 16)).toBe(true);
});

test('continues to reject measurement drift after a bounded pullout', async () => {
  const trial = movingPullout({ measurementDrift: true });
  await expect(trial.run()).rejects.toBeInstanceOf(EncoderStabilityTimeoutError);
  expect(trial.points).toHaveLength(0);
});

test('half-turn sensor reserve follows spindle gearing, not a drive radius guess', async () => {
  const result = await movingPullout({ sensorSpoolGearRatio: 2 }).run();
  expect(result.dataPoints[0].raw_angles_deg[2]).toBeLessThanOrEqual(600.5 - 360);
  expect(result.spoolReserve.guarded_sensor_rewind_mm).toBeGreaterThanOrEqual(360);
  expect(result.spoolReserve.coupling_probes).toEqual([]);
});

test('later collection can reclaim only a reversibly tested part of the reserved interval', async () => {
  const early = await movingPullout({ sensorGain: 1, sweepPoints: 4 }).run();
  const later = await movingPullout({ probeSpoolReserve: true, sensorGain: 1, sweepPoints: 4 }).run();
  expect(early.spoolReserve.initial_half_turn_required).toBe(true);
  expect(early.spoolReserve.coupling_probes).toEqual([]);
  expect(later.spoolReserve.coupling_probes.some(probe => probe.accepted)).toBe(true);
  expect(later.spoolReserve.initial_half_turn_required).toBe(false);
  expect(later.dataPoints[0].l_drive).toBeLessThan(early.dataPoints[0].l_drive);
});

test.each([0, 20])('does not invent or overshoot a %s mm measured interval to satisfy reserve', async limit => {
  const trial = movingPullout({ pulloutLimit: limit });
  await expect(trial.run()).rejects.toThrow(limit === 0 ? 'No measured drive interval' : 'too short');
  expect(trial.points).toHaveLength(0);
  const total = trial.commands.reduce((sum, {line}) => sum + Number(line.match(/^G1 H2 Y([-\d.]+)/)?.[1] ?? 0), 0);
  expect(total).toBeLessThanOrEqual(limit);
});

test('drive movement without a sensor response never satisfies the initial reserve', async () => {
  const trial = movingPullout({ sensorGain: 0 });
  await expect(trial.run()).rejects.toThrow('too short');
  expect(trial.points).toHaveLength(0);
});

test('does not publish an encoder mean that crosses the initial half-turn boundary', async () => {
  const trial = movingPullout({ meanCrossing: true, sensorGain: 180.01 / 47.124 });
  await expect(trial.run()).rejects.toThrow('half-turn interval');
  expect(trial.points).toHaveLength(0);
});

test('rejects a guard that consumes all available travel instead of repeating one point', async () => {
  const trial = movingPullout({ pulloutLimit: 20, sensorGain: 10 });
  await expect(trial.run()).rejects.toThrow('No measurement interval');
  expect(trial.points).toHaveLength(0);
});

test('probe approval cannot authorize later samples outside its preload and interval', async () => {
  const trial = movingPullout({ probeSpoolReserve: true, forceLosesCoupling: true, sweepPoints: 4,
    sensorForceProfile: { drive_mm: [-600, 0], force_n: [1, 3] } });
  await expect(trial.run()).rejects.toThrow('Insufficient coupled measurement points');
  expect(trial.points.every(point => point.sensor_force_n < 2.5)).toBe(true);
});

test('later short arcs need successful local evidence at each unreserved point', async () => {
  const result = await movingPullout({ probeSpoolReserve: true, pulloutLimit: 20,
    sensorGain: 10, sweepPoints: 5 }).run();
  expect(result.dataPoints).toHaveLength(5);
  expect(new Set(result.dataPoints.map(point => point.drive_setpoint_mm)).size).toBe(5);
  expect(result.dataPoints.every(point => point.spool_reserve_method === 'local_encoder_coupling')).toBe(true);
});

test('unreserved samples with different profile forces receive fresh local probes', async () => {
  const trial = movingPullout({ probeSpoolReserve: true, sensorGain: 1, sweepPoints: 10,
    sensorForceProfile: { drive_mm: [-600, 0], force_n: [1, 3] } });
  const result = await trial.run();
  const accepted = result.spoolReserve.coupling_probes.filter(probe => probe.accepted);
  expect(accepted.length).toBeGreaterThan(1);
  expect(new Set(accepted.map(probe => probe.sensor_force_n)).size).toBe(accepted.length);
  const force = result.dataPoints[0].sensor_force_n;
  const changes = trial.commands.filter(command => command.line.startsWith('M569.4'));
  const first = changes.findIndex(command => Number(command.line.split(' T')[1].split(':')[2]) === force);
  const duringProbe = changes.slice(first).filter(command => command.now <= result.dataPoints[0].clock_time_ms);
  expect(duringProbe.every(command => Number(command.line.split(' T')[1].split(':')[2]) === force)).toBe(true);
});

test('allows slow fixed-target preparation before starting the bounded pullout', async () => {
  const trial = movingPullout({ preparationDrift: true });
  const result = await trial.run();
  expect(trial.commands.find(command => command.line.startsWith('M569.4')).now).toBeGreaterThan(60000);
  expect(result.dataPoints).toHaveLength(2);
});

test('does not recover fixed-anchor slip during endpoint search', async () => {
  const trial = movingPullout({ fixedDrift: true });
  await expect(trial.run()).rejects.toBeInstanceOf(FixedAnchorDriftError);
  expect(trial.points).toHaveLength(0);
});

test.each(['Transport disconnected', 'Timed out waiting for encoder stability (wall-clock deadline)'])(
  'does not recover an unrelated failure: %s', async message => {
    const error = new Error(message);
    const trial = movingPullout({ transportError: error });
    await expect(trial.run()).rejects.toBe(error);
    expect(trial.points).toHaveLength(0);
  },
);

test('does not recover stalled simulation advances', async () => {
  const trial = movingPullout({ stalled: true });
  await expect(trial.run()).rejects.toThrow('Simulation advance stalled');
  expect(trial.points).toHaveLength(0);
});

test('sensor force follows the profile at every point of the sweep', async () => {
  const profile = { drive_anchor: 1, sensor_anchor: 2, drive_mm: [-600, 0], force_n: [1, 3] };
  const trial = movingPullout({ sensorForceProfile: profile });
  const result = await trial.run();
  const forces = result.dataPoints.map(point => point.sensor_force_n);
  expect(forces).toEqual(result.dataPoints.map(point => sensorForceAt(profile, point.drive_setpoint_mm)));
  expect(new Set(forces).size).toBe(2);
  for (const point of result.dataPoints) {
    const applied = trial.commands.filter(command => command.line.startsWith('M569.4')
      && Math.abs(Number(command.line.split(' T')[1].split(':')[2]) - point.sensor_force_n) < 1e-9);
    expect(applied.length).toBeGreaterThan(0);
    expect(applied[0].now).toBeLessThan(point.timestamp_ms);
  }
});

test('sensor force profile interpolates, holds its end values and must match the sweep direction', () => {
  const profile = { drive_anchor: 1, sensor_anchor: 2, fixed_anchors: [0, 3], fixed_lengths: [390, -100],
    drive_mm: [-100, 0, 200], force_n: [1, 2, 4] };
  expect(sensorForceAt(profile, -500)).toBe(1);
  expect(sensorForceAt(profile, -50)).toBeCloseTo(1.5);
  expect(sensorForceAt(profile, 100)).toBeCloseTo(3);
  expect(sensorForceAt(profile, 900)).toBe(4);
  const otherHeight = { ...profile, fixed_lengths: [390, -200], force_n: [2, 3, 5] };
  expect(sensorForceProfileFor([profile, otherHeight], 1, 2, [0, 3], [390, -200])).toBe(otherHeight);
  expect(() => sensorForceProfileFor([profile], 2, 1, [0, 3], [390, -100])).toThrow('found 0');
  expect(() => sensorForceProfileFor([profile, profile], 1, 2, [0, 3], [390, -100])).toThrow('found 2');
});
