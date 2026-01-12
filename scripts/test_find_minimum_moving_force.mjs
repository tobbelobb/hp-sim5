#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { findMinimumMovingForce, FORCE_TUNING_CONSTANTS } from './force_tuning.mjs';

async function testFindMinimumMovingForce() {
  const threshold = 0.8;
  const calls = [];
  const trialFn = async (force) => {
    calls.push(force);
    return { moved: force >= threshold };
  };
  let returnCalls = 0;
  const returnToOriginFn = async () => {
    returnCalls += 1;
  };

  const result = await findMinimumMovingForce(() => {}, {
    motorIds: ['40.0', '41.0'],
    axes: ['A', 'B'],
    mmPerDeg: [1, 1],
    baseLow: 0.2,
    capForceLimit: 2.0,
    trialFn,
    returnToOriginFn,
  });

  assert.ok(result.forceStart >= threshold);
  const allowed = threshold * (1 + FORCE_TUNING_CONSTANTS.AUTO_TUNE_RELATIVE_TOLERANCE)
    + FORCE_TUNING_CONSTANTS.AUTO_TUNE_ABSOLUTE_TOLERANCE;
  assert.ok(result.forceStart <= allowed);
  assert.ok(calls.length > 0);
  assert.equal(returnCalls, 1);
}

await testFindMinimumMovingForce();
console.log('find_minimum_moving_force tests: PASSED');
