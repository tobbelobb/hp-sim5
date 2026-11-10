import {
  assertFixedAnchorAngles,
  resolveCollectionForce,
  waitForStableEncoders,
} from '../../primitives/uncalibrated_actions.mjs';

const targets = { fixedTargetByAnchor: [null, 0, null, -489.075], mmPerDeg: [0.5, 0.5, 0.5, 0.5] };

describe('fixed-anchor protection', () => {
  test('caps collection preload without importing a geometry-specific edge force', () => {
    expect(resolveCollectionForce({ forceMid: 0.227271514624, forceMax: 12.88491133549 })).toBeCloseTo(1.13635757312);
    expect(resolveCollectionForce({ forceMid: 2, forceMax: 3 })).toBe(3);
    expect(resolveCollectionForce({ forceMid: .2, forceMax: 3, preloadMultiplier: 20 })).toBe(3);
    expect(resolveCollectionForce({ forceMax: 3 })).toBe(3);
    expect(resolveCollectionForce({ forceMid: .2, forceMax: 10, sensorCollectionForce: 7 })).toBe(7);
  });

  test('allows drive/sensor movement and small held-motor load deflections', () => {
    expect(() => assertFixedAnchorAngles([500, .16, -600, -978.14], targets)).not.toThrow();
  });

  test('rejects an encoder that is quiet after slipping to another motor detent', async () => {
    let time = 0;
    const send = jest.fn(async () => ({ reply: '0 7.2 0 -978.15' }));
    await expect(waitForStableEncoders(send, ['A', 'B', 'C', 'D'], {
      nowFn: () => time, sleepFn: async ms => { time += ms; },
      validateAngles: angles => assertFixedAnchorAngles(angles, targets),
    })).rejects.toThrow('Fixed anchor 1 drifted 7.200deg');
    // Reject before accepting a quiet window or issuing any corrective move.
    expect(send).toHaveBeenCalledTimes(1);
  });

  test('rejects trial-004 held B drift and invalid fixed-angle conversion', () => {
    expect(() => assertFixedAnchorAngles([0, 193.04, 0, -978.15], targets)).toThrow('Fixed anchor 1');
    expect(() => assertFixedAnchorAngles([0, 0, 0, -978.15], { ...targets, mmPerDeg: [] })).toThrow('Fixed anchor 1');
  });
});
