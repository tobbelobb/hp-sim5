import {
  calculateReturnOrder,
  collectDataPoint,
  waitForStableEncoders,
} from '../../primitives/uncalibrated_actions.mjs';

function createCollectDataSend(reply = '1 2 3') {
  const commands = [];
  const send = jest.fn(async (command) => {
    commands.push(command);
    if (command.startsWith('M569.3 ')) {
      return { reply };
    }
    return { reply: '' };
  });
  return { commands, send };
}

function createSettleOptions() {
  let nowMs = 0;
  return {
    pollIntervalMs: 1,
    stableWindowMs: 2,
    vibrationWindowMs: 2,
    toleranceDeg: 0.01,
    sleepFn: async ms => { nowMs += ms; },
    nowFn: () => nowMs,
  };
}

describe('uncalibrated_actions', () => {
  test('a settled measurement retains noise sampling and the fixed-angle guard', async () => {
    const { commands, send } = createCollectDataSend('1 2 3');
    const validateAngles = jest.fn();
    const recordPoint = jest.fn();
    const settledMeasurement = { anglesDeg: [1, 2, 3], elapsedMs: 1500 };
    const clock = createSettleOptions();
    const result = await collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      fixedAnchors: [2],
      forceMid: 0.2,
      forceMax: 10,
      settledMeasurement,
      settleOptions: { validateAngles },
      recordPoint,
      encoderNoiseOptions: { sampleCount: 16, sleepFn: clock.sleepFn, nowFn: clock.nowFn },
    });
    expect(result.stableData).toBe(settledMeasurement);
    expect(result.lengths).toEqual([1, 2, 3]);
    expect(result.noiseStats.sampleCount).toBe(16);
    expect(commands).toHaveLength(16);
    expect(commands.every(command => command.startsWith('M569.3 '))).toBe(true);
    expect(validateAngles).toHaveBeenCalledWith([1, 2, 3]);
    expect(recordPoint).toHaveBeenCalledTimes(1);
  });

  test('a settled reading cannot bypass a fixed-angle rejection', async () => {
    const { send } = createCollectDataSend();
    const recordPoint = jest.fn();
    await expect(collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      settledMeasurement: { anglesDeg: [1, 2, 3] },
      settleOptions: { validateAngles: () => { throw new Error('fixed drift'); } },
      recordPoint,
    })).rejects.toThrow('fixed drift');
    expect(recordPoint).not.toHaveBeenCalled();
  });

  test('zero-tension projection performs its force trials even with a settled reading', async () => {
    const { commands, send } = createCollectDataSend('1 2 3');
    const result = await collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      fixedAnchors: [2],
      forceMid: 2,
      forceMin: 1,
      forceMax: 10,
      projectZeroTension: true,
      settledMeasurement: { anglesDeg: [9, 9, 9] },
      settleOptions: createSettleOptions(),
    });
    expect(result.anglesDeg).toEqual([1, 2, 3]);
    expect(commands.filter(command => command.startsWith('M569.4 '))).toHaveLength(11);
  });

  test('calculateReturnOrder', () => {
    const order = calculateReturnOrder({
      fixedAnchors: [2],
      currentLengths: [5, 12, 1],
    });
    expect(order).toEqual([1, 0, 2]);
  });

  test('waitForStableEncoders', async () => {
    const send = async () => ({ reply: '1 1 1' });
    const result = await waitForStableEncoders(send, ['40.0', '41.0', '42.0'], 1, {
      pollIntervalMs: 1,
      stableWindowMs: 2,
      toleranceDeg: 0.01,
    });
    expect(result.anglesDeg).toEqual([1, 1, 1]);
    expect(result.samples).toBeGreaterThanOrEqual(2);
  });

  test('collectDataPoint tensions the sensor before reading and stays in measurement preload', async () => {
    const { commands, send } = createCollectDataSend();
    const recordPoint = jest.fn();

    const result = await collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      forceMid: 2,
      forceMax: 10,
      speedup: 1,
      settleOptions: createSettleOptions(),
      recordPoint,
      driveSetpointMm: 12,
      stepIndex: 0,
      stepCount: 4,
    });

    expect(result.anglesDeg).toEqual([1, 2, 3]);
    expect(recordPoint).toHaveBeenCalledWith([1, 2, 3], 12, 0, 4, null);
    expect(commands.filter((command) => command.startsWith('M569.4 '))).toEqual([
      'M569.4 P40.0:41.0:42.0 T0.0:4:2',
      'M569.4 P40.0:41.0:42.0 T0.0:10:2',
    ]);
  });

  test('collectDataPoint skips return prep but still boosts only the sensor collection force', async () => {
    const { commands, send } = createCollectDataSend('5 6 7');

    await collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      fixedAnchors: [2],
      forceMid: 2,
      forceMax: 10,
      speedup: 1,
      settleOptions: createSettleOptions(),
      skipReturnModePrep: true,
    });

    expect(commands.filter((command) => command.startsWith('M569.4 '))).toEqual([
      'M569.4 P40.0:41.0:42.0 T0.0:10:0.0',
    ]);
  });

  test('collectDataPoint bounds sensor force by the start-force preload', async () => {
    const { commands, send } = createCollectDataSend('5 6 7');

    await collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      forceMid: 0.2,
      forceMax: 10,
      speedup: 1,
      settleOptions: createSettleOptions(),
      skipReturnModePrep: true,
    });

    expect(commands.filter((command) => command.startsWith('M569.4 '))).toEqual([
      'M569.4 P40.0:41.0:42.0 T0.0:1:0.2',
    ]);
  });

  test('collectDataPoint accepts explicit sensor collection force', async () => {
    const { commands, send } = createCollectDataSend('5 6 7');

    await collectDataPoint(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      driveAnchor: 0,
      sensorAnchors: [1],
      forceMid: 0.2,
      forceMax: 10,
      sensorCollectionForce: 7,
      speedup: 1,
      settleOptions: createSettleOptions(),
      skipReturnModePrep: true,
    });

    expect(commands.filter((command) => command.startsWith('M569.4 '))).toEqual([
      'M569.4 P40.0:41.0:42.0 T0.0:7:0.2',
    ]);
  });
});
