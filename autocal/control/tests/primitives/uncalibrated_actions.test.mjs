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
  return {
    pollIntervalMs: 0,
    stableWindowMs: 0,
    vibrationWindowMs: 0,
    toleranceDeg: 0.01,
    sleepFn: async () => {},
    nowFn: () => 0,
  };
}

describe('uncalibrated_actions', () => {
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

  test('collectDataPoint defaults sensor collection force from force max', async () => {
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
      'M569.4 P40.0:41.0:42.0 T0.0:5:0.2',
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
