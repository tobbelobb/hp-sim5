import { collectWithSlipRecovery } from '../../behaviors/slip_recovery.mjs';
import { FixedAnchorDriftError } from '../../primitives/uncalibrated_actions.mjs';

const options = { motorIds: ['A', 'B'], forceLow: .01, forceMid: .2, forceMax: 10, sensorCollectionForce: 6 };

test('holds every motor, lowers force, discards failed result and recollects direction', async () => {
  const sequence = [];
  const send = jest.fn(async command => { sequence.push(command); });
  const collect = jest.fn(async (forces, attempt) => {
    sequence.push(`attempt ${attempt}`);
    if (!attempt) throw new FixedAnchorDriftError(1, 7.2, 1.5);
    expect(forces).toEqual({ forceLow: .01, forceMid: .1, forceMax: 5, sensorCollectionForce: 3 });
    return { dataPoints: ['new valid point'] };
  });
  const onRecovery = jest.fn();
  const result = await collectWithSlipRecovery(send, collect, { ...options, onRecovery });
  expect(sequence).toEqual(['attempt 0', 'M569.4 PA:B T0.0:0.0', 'attempt 1']);
  expect(result.dataPoints).toEqual(['new valid point']);
  expect(result.recoveries).toHaveLength(1);
  expect(onRecovery).toHaveBeenCalledWith(expect.objectContaining({ anchor: 1, discarded_attempt: 0 }));
});

test('repeated slips stop after a bounded number of retries with motors held', async () => {
  const send = jest.fn(async () => {});
  const collect = jest.fn(async () => { throw new FixedAnchorDriftError(0, 2, 1.5); });
  await expect(collectWithSlipRecovery(send, collect, options)).rejects.toThrow('exhausted after 3 retries');
  expect(collect).toHaveBeenCalledTimes(4);
  expect(send.mock.calls.at(-1)[0]).toContain('T0.0:0.0');
});

test('transport and invalid measurement failures are not treated as slips', async () => {
  const send = jest.fn();
  const collect = jest.fn(async () => { throw new Error('encoder disconnected'); });
  await expect(collectWithSlipRecovery(send, collect, options)).rejects.toThrow('encoder disconnected');
  expect(collect).toHaveBeenCalledTimes(1);
  expect(send).not.toHaveBeenCalled();
});

test('recovery never raises an explicit force that is already below idle preload', async () => {
  const collect = jest.fn(async (forces, attempt) => {
    if (!attempt) throw new FixedAnchorDriftError(1, 2, 1.5);
    expect(forces.sensorCollectionForce).toBe(.005);
    return { dataPoints: [] };
  });
  await collectWithSlipRecovery(async () => {}, collect, { ...options, sensorCollectionForce: .005 });
});
