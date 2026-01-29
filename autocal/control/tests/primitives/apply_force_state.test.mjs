#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { applyForceModeState } from '../../primitives/uncalibrated_actions.mjs';

async function testApplyForceModeState() {
  const sent = [];
  const send = async (line) => {
    sent.push(line);
    return { reply: '' };
  };

  await applyForceModeState(send, {
    motorIds: ['40.0', '41.0', '42.0'],
    modes: ['position', 0.05, 0]
  });

  assert.deepEqual(sent, [
    'M569.4 P40.0:41.0:42.0 T0.0:0.05:0.0'
  ]);
}

await testApplyForceModeState();
console.log('apply_force_state tests: PASSED');
