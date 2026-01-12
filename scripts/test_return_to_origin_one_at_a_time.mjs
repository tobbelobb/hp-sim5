#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { returnMotorsToOriginOneAtATime } from './uncalibrated_actions.mjs';

async function testReturnToOriginOneAtATime() {
  const sent = [];
  const encoderReply = '10 20 5';
  const send = async (line) => {
    sent.push(line);
    if (line.startsWith('M569.3')) {
      return { reply: encoderReply };
    }
    return { reply: '' };
  };
  const delayFn = () => Promise.resolve();
  const settleDelayFn = (ms) => new Promise((resolve) => setTimeout(resolve, Math.max(1, ms)));

  await returnMotorsToOriginOneAtATime(send, {
    motorIds: ['40.0', '41.0', '42.0'],
    axes: ['A', 'B', 'C'],
    mmPerDeg: [1, 1, 1],
    feed: 100,
    speedup: 1,
    lowForceNm: 0.01,
    fixedAnchors: [1],
    delayFn,
    settleOptions: { pollIntervalMs: 1, stableWindowMs: 2, delayFn: settleDelayFn },
  });

  const moveLines = sent.filter((line) => line.startsWith('G1 H2'));
  const axesOrder = moveLines.map((line) => {
    const match = line.match(/H2\s+([A-Z])/);
    return match ? match[1] : null;
  });

  assert.deepEqual(axesOrder, ['A', 'C', 'B']);
}

await testReturnToOriginOneAtATime();
console.log('return_to_origin_one_at_a_time tests: PASSED');
