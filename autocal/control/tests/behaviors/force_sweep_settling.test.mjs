import { performForceSweep } from '../../behaviors/sweep_data_collection.mjs';
import { EncoderStabilityTimeoutError, FixedAnchorDriftError } from '../../primitives/uncalibrated_actions.mjs';

// Endpoint travel may continue indefinitely under force. Measurements must still
// settle after the drive is held, and every fixed-anchor reading stays guarded.
function movingPullout({ fixedDrift = false, measurementDrift = false, transportError = null, stalled = false } = {}) {
  let now = 0;
  let modes = [0, 0, 0, 0];
  let angles = [0, 0, 0, 0];
  const commands = [];
  const points = [];
  const send = async line => {
    commands.push({ line, now });
    if (line.startsWith('M569.4')) modes = line.split(' T')[1].split(':').map(Number);
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
    onPoint: point => points.push(point),
  });
  return { run, commands, points };
}

test('bounds moving endpoint search, then holds and settles before recording all samples', async () => {
  const trial = movingPullout();
  const result = await trial.run();
  const hold = trial.commands.find(command => command.now > 120000 && command.line === 'M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.46:0.0');
  expect(hold).toBeDefined();
  expect(result.dataPoints).toHaveLength(2);
  expect(result.driveRange.start).toBeCloseTo(120.5);
  expect(trial.points[0].timestamp_ms).toBeGreaterThanOrEqual(hold.now + 1500);
  expect(result.dataPoints.every(point => point.sample_count === 16)).toBe(true);
});

test('continues to reject measurement drift after a bounded pullout', async () => {
  const trial = movingPullout({ measurementDrift: true });
  await expect(trial.run()).rejects.toBeInstanceOf(EncoderStabilityTimeoutError);
  expect(trial.points).toHaveLength(0);
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
