#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';

async function testWaitForStableEncoders() {
  const send = async () => ({ reply: '0 0' });

  const result = await waitForStableEncoders(send, ['40.0', '41.0'], 1, {
    pollIntervalMs: 1,
    stableWindowMs: 2,
  });

  assert.deepEqual(result.anglesDeg, [0, 0]);
  assert.ok(result.samples >= 2);
}

await testWaitForStableEncoders();
console.log('wait_for_stable_encoders tests: PASSED');
