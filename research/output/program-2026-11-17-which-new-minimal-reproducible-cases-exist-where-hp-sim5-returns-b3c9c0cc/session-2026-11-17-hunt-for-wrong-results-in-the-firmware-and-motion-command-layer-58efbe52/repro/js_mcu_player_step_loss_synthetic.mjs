#!/usr/bin/env node
// Minimal repro: KlipperMcuCommandPlayer (the default hp-sim5 upload player for
// Klipper .txt/.serial MCU streams) silently drops the steps that a *later*
// stepper's first queue_step puts into buckets that were already emitted for an
// earlier stepper.
//
// Each axis's first queue_step in a batch dump spans the idle time from MCU
// clock 0 to the first step, so it is smeared over buckets 0..N.  The player
// flushes ready buckets after every parsed line, so when the second stepper's
// line is parsed the cursor `nextBucketToEmit` is already past 0..N-1 and those
// writes are dropped (`_emitBucket` already deleted the map entries and never
// revisits the bucket).
//
// Synthetic stream (2 ms buckets = 100000 ticks at 50 MHz):
//   stepper 0 (axis A): one step at tick 1000000   -> 1.000 steps
//   stepper 1 (axis B): one step at tick 1000000   -> 1.000 steps
// Expected: A = 1, B = 1.  Actual: A = 1, B = 1/10 (only the last bucket).

import path from 'node:path';
import { fileURLToPath } from 'node:url';

globalThis.postMessage = () => {};
globalThis.self = {
  addEventListener() {}, onmessage: null,
  postMessage: globalThis.postMessage, console,
};

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');
const { KlipperMcuCommandPlayer } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperMcuCommandPlayer.js'));
const { FileFormat } = await import(
  path.join(ROOT, 'integrations', 'shared', 'fileFormatUtils.js'));
const { STEP_ANGLE_RAD } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperFirmwareModel.js'));

const stream = [
  // step pins are the ones the shipped slideprinter config uses for a/b
  'config_stepper oid=0 step_pin=gpiochip2/gpio0 dir_pin=gpiochip2/gpio1 invert_step=0 step_pulse_ticks=100',
  'config_stepper oid=1 step_pin=gpiochip2/gpio3 dir_pin=gpiochip2/gpio4 invert_step=0 step_pulse_ticks=100',
  'set_next_step_dir oid=0 dir=1',
  'queue_step oid=0 interval=1000000 count=1 add=0',
  'set_next_step_dir oid=1 dir=1',
  'queue_step oid=1 interval=1000000 count=1 add=0',
].join('\n') + '\n';

async function run(text) {
  const player = new KlipperMcuCommandPlayer();
  player.setAsapMode(true);
  const commands = [];
  player.sendCommand = async (c) => { commands.push(c); };
  await player.run(new ReadableStream({
    start(c) { c.enqueue(new TextEncoder().encode(text)); c.close(); },
  }), FileFormat.MCU_TEXT);
  const moves = commands.filter((c) => c.type === 'Move');
  const last = moves.at(-1) || {};
  return {
    A: +((last.A || 0) / STEP_ANGLE_RAD).toFixed(4),
    B: +((last.B || 0) / STEP_ANGLE_RAD).toFixed(4),
    moveCommands: moves.length,
  };
}

const incremental = await run(stream);

// Control: hold back flushing until the whole stream is parsed.
const player2 = new KlipperMcuCommandPlayer();
player2.setAsapMode(true);
const realFlush = player2._flushReadyBuckets.bind(player2);
player2._flushReadyBuckets = async (force) => { if (!force) return; await realFlush(force); };
const cmds2 = [];
player2.sendCommand = async (c) => { cmds2.push(c); };
await player2.run(new ReadableStream({
  start(c) { c.enqueue(new TextEncoder().encode(stream)); c.close(); },
}), FileFormat.MCU_TEXT);
const last2 = cmds2.filter((c) => c.type === 'Move').at(-1) || {};
const deferred = {
  A: +((last2.A || 0) / STEP_ANGLE_RAD).toFixed(4),
  B: +((last2.B || 0) / STEP_ANGLE_RAD).toFixed(4),
};

console.log(JSON.stringify({ incrementalFlush: incremental, deferredFlush: deferred }, null, 2));

if (Math.abs(incremental.A - 1) > 1e-6 || Math.abs(incremental.B - 1) > 1e-6) {
  console.error('\nBUG REPRODUCED: KlipperMcuCommandPlayer lost a whole microstep from axis B');
  console.error(`  - A ends at ${incremental.A} steps (expected 1), B ends at ${incremental.B} steps (expected 1)`);
  console.error(`  - with a single deferred flush both axes end at ${deferred.A}/${deferred.B} steps`);
  process.exit(1);
}
console.log('OK: both axes replay their step');
