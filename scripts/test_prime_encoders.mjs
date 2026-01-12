#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import { primeEncoders } from './uncalibrated_actions.mjs';

async function testPrimeEncoders() {
  const sent = [];
  const send = async (line) => {
    sent.push(line);
    return { reply: '' };
  };

  await primeEncoders(send, {
    motorIds: ['40.0', '41.0'],
    axes: ['A', 'B'],
  });

  assert.deepEqual(sent, [
    'G92 A0 B0',
    'G91',
    'M569.3 P40.0:41.0 S',
  ]);
}

await testPrimeEncoders();
console.log('prime_encoders tests: PASSED');
