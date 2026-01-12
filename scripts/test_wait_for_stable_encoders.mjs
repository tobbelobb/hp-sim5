#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { waitForStableEncoders } from './uncalibrated_actions.mjs';

async function testWaitForStableEncoders() {
  const send = async () => ({ reply: '0 0' });
  const delayFn = (ms) => new Promise((resolve) => setTimeout(resolve, Math.max(1, ms)));

  const result = await waitForStableEncoders(send, ['40.0', '41.0'], {
    pollIntervalMs: 1,
    stableWindowMs: 2,
    delayFn,
  });

  assert.deepEqual(result.anglesDeg, [0, 0]);
  assert.ok(result.samples >= 2);
}

await testWaitForStableEncoders();
console.log('wait_for_stable_encoders tests: PASSED');
