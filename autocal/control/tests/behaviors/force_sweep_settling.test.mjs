import { performForceSweep, sensorForceAt, sensorForceProfileFor } from '../../behaviors/sweep_data_collection.mjs';
import { EncoderStabilityTimeoutError, FixedAnchorDriftError } from '../../primitives/uncalibrated_actions.mjs';

// Endpoint travel may continue indefinitely under force. Measurements must still
// settle after the drive is held, and every fixed-anchor reading stays guarded.
function movingPullout({ fixedDrift = false, measurementDrift = false, transportError = null, stalled = false, preparationDrift = false, sensorForceProfile = null } = {}) {
  let now = 0;
  let modes = preparationDrift ? [0, 2.4, 2.4, 0] : [0, 0, 0, 0];
  let preparing = preparationDrift;
  let angles = [0, 0, 0, 0];
  const commands = [];
  const points = [];
  const send = async line => {
    commands.push({ line, now });
    if (line.startsWith('M569.4')) {
      modes = line.split(' T')[1].split(':').map(Number);
      preparing = false;
    }
    if (line.startsWith('M569.3')) {
      if (modes[1] > 0 && transportError) throw transportError;
      return { reply: angles.join(' ') };
    }
    const move = line.match(/^G1 H2 Y([-\d.]+)/);
    if (move) angles[1] += Number(move[1]);
    return { reply: '' };
  };
  send.simulationClock = {
    now: () => now,
    sleep: async ms => {
      if (stalled && modes[1] > 0) throw new Error('Simulation advance stalled');
      now += ms;
      if (preparing && now >= 60000) return;
      if (modes[1] > 0) {
        angles[1] += ms / 1000;
        angles[2] -= ms / 1000;
        if (fixedDrift) angles[0] = 2;
      } else if (measurementDrift && modes[2] > 0) angles[2] += ms / 1000;
    },
  };
  const run = () => performForceSweep(send, { driveAnchor: 1, sensorAnchor: 2 }, {
    axes: ['X', 'Y', 'Z', 'U'], motorIds: ['40.0', '41.0', '42.0', '43.0'],
    sweepPoints: 2, feed: 1400, speedup: 1, mmPerDeg: [1, 1, 1, 1], datasetStartMs: 0,
    forceLow: .01, forceMid: .23, forceMax: 2.4, fixedAnchors: [0, 3], fixedTargets: [0, 0],
    forbiddenForceAnchors: [3], baseRadiusMm: 30, projectZeroTension: false,
    noiseSampleCount: 16, noiseSampleIntervalMs: 40, noiseMinSamples: 4, noiseSigmaFloorDeg: .01,
    onPoint: point => points.push(point), sensorForceProfile,
  });
  return { run, commands, points };
}

test('bounds moving endpoint search, then holds and settles before recording all samples', async () => {
  const trial = movingPullout();
  const result = await trial.run();
  const hold = trial.commands.find(command => command.now > 600000 && command.line === 'M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.46:0.0');
  expect(hold).toBeDefined();
  expect(result.dataPoints).toHaveLength(2);
  expect(result.driveRange.start).toBeCloseTo(600.5);
  expect(trial.points[0].timestamp_ms).toBeGreaterThanOrEqual(hold.now + 1500);
  expect(result.dataPoints.every(point => point.sample_count === 16)).toBe(true);
});

test('continues to reject measurement drift after a bounded pullout', async () => {
  const trial = movingPullout({ measurementDrift: true });
  await expect(trial.run()).rejects.toBeInstanceOf(EncoderStabilityTimeoutError);
  expect(trial.points).toHaveLength(0);
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
  const profile = { drive_anchor: 1, sensor_anchor: 2, drive_mm: [0, 600], force_n: [1, 3] };
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
  const profile = { drive_anchor: 1, sensor_anchor: 2, drive_mm: [-100, 0, 200], force_n: [1, 2, 4] };
  expect(sensorForceAt(profile, -500)).toBe(1);
  expect(sensorForceAt(profile, -50)).toBeCloseTo(1.5);
  expect(sensorForceAt(profile, 100)).toBeCloseTo(3);
  expect(sensorForceAt(profile, 900)).toBe(4);
  expect(sensorForceProfileFor([profile], 1, 2)).toBe(profile);
  expect(() => sensorForceProfileFor([profile], 2, 1)).toThrow('no entry for drive 2, sensor 1');
});
