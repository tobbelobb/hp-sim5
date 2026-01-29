#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { calculateReturnOrder, waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';

function testCalculateReturnOrder() {
  const order = calculateReturnOrder({
    fixedAnchors: [2],
    currentLengths: [5, 12, 1],
  });
  assert.deepEqual(order, [1, 0, 2]);
}

async function testWaitForStableEncoders() {
  const send = async () => ({ reply: '1 1 1' });
  const result = await waitForStableEncoders(send, ['40.0', '41.0', '42.0'], 1, {
    pollIntervalMs: 1,
    stableWindowMs: 2,
    toleranceDeg: 0.01,
  });
  assert.deepEqual(result.anglesDeg, [1, 1, 1]);
  assert.ok(result.samples >= 2);
}

testCalculateReturnOrder();
await testWaitForStableEncoders();
console.log('uncalibrated_actions unit tests: PASSED');
