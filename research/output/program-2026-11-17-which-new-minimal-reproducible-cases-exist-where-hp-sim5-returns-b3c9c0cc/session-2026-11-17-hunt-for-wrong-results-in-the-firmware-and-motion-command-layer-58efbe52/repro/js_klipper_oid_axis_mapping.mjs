#!/usr/bin/env node
// Repro: two shipped parsers for the *same* Klipper MCU command stream
// disagree, and one of them moves the wrong axis.
//
//   integrations/klipper/klipperMcuCommandPlayer.js   maps config_stepper oid
//       -> axis through STEP_PIN_AXIS_MAP (gpio0->A ... gpio24->E).  Correct.
//   integrations/klipper/klipperMotionCore.js#applyBucketedParsedLine (used by
//       integrations/klipper/klipperPacerWorker.js for raw serial/text input)
//       maps queue_step oid -> DEFAULT_AXIS_ORDER[oid].  Wrong: the extruder is
//       the fourth stepper (oid 3) in every shipped Hangprinter config, so its
//       steps become spool axis D, and E never moves.
//
// Fixture: fixtures/klipper_slideprinter_x10_e5.txt = parsedump.py output of
//   klippy.py public/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg \
//       -i x10e5.gcode -o x10e5.serial -d public/klipper/linux_mcu/klipper.dict
// with x10e5.gcode = "G90 / G1 X10 E5 F600".
//
// Expected (for both parsers): E = 5.0041 mm (478 steps x 33.5/3200 mm),
// spool axes only from the anchor motors (b/c = -147/+147 steps).

globalThis.postMessage = () => {};
globalThis.self = {
  addEventListener() {},
  onmessage: null,
  postMessage: globalThis.postMessage,
  console,
};

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');
const { FileFormat } = await import(
  path.join(ROOT, 'integrations', 'shared', 'fileFormatUtils.js'));

const { KlipperBucketedMotionCore, applyBucketedParsedLine } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperMotionCore.js'));
const { KlipperMcuCommandPlayer } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperMcuCommandPlayer.js'));
const { STEP_ANGLE_RAD, EXTRUDER_MM_PER_STEP_KLIPPER } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperFirmwareModel.js'));

const text = fs.readFileSync(
  path.join(__dirname, 'fixtures', 'klipper_slideprinter_x10_e5.txt'), 'utf8');

// --- path 1: klipperMotionCore.applyBucketedParsedLine (pacer worker) ---
const core = new KlipperBucketedMotionCore({ bucketIntervalMs: 2 });
for (const line of text.split('\n')) applyBucketedParsedLine(core, line);
const coreMoves = core.flushCommands({ force: true });
const coreLast = coreMoves.filter((m) => m.type === 'Move').at(-1) || {};

// --- path 2: KlipperMcuCommandPlayer (default player path) ---
const player = new KlipperMcuCommandPlayer();
player.setAsapMode(true);
const playerCommands = [];
player.sendCommand = async (command) => { playerCommands.push(command); };
await player.run(new ReadableStream({
  start(controller) {
    controller.enqueue(new TextEncoder().encode(text));
    controller.close();
  },
}), FileFormat.MCU_TEXT);
const playerMoves = playerCommands.filter((c) => c.type === 'Move');
const playerLast = playerMoves.at(-1) || {};
// E is emitted as a per-bucket delta, so accumulate it.
const playerExtrusionMm = playerMoves.reduce((sum, m) => sum + (m.E || 0), 0);

const steps = (obj, axis) => +(((obj[axis] || 0) / STEP_ANGLE_RAD).toFixed(2));
const table = (obj) => ({
  A: steps(obj, 'A'), B: steps(obj, 'B'), C: steps(obj, 'C'),
  D: steps(obj, 'D'), I: steps(obj, 'I'), E_mm: +(obj.E || 0).toFixed(4),
});

const viaPacer = table(coreLast);
const viaPlayer = { ...table(playerLast), E_mm: +playerExtrusionMm.toFixed(4) };
const expectedExtrusion = 478 * EXTRUDER_MM_PER_STEP_KLIPPER;

console.log(JSON.stringify({
  firmware_stream: 'slideprinter: G1 X10 E5 F600 -> oid3 extruder 478 steps',
  via_pacer_worker_applyBucketedParsedLine: viaPacer,
  via_mcu_command_player: viaPlayer,
  expected: { B: -147, C: 147, D: 0, E_mm: +expectedExtrusion.toFixed(4) },
}, null, 2));

const problems = [];
if (Math.abs(viaPacer.E_mm - expectedExtrusion) > 0.02) {
  problems.push(`pacer-worker path extruded ${viaPacer.E_mm} mm, expected ${expectedExtrusion.toFixed(4)} mm`);
}
if (Math.abs(viaPacer.D) > 1e-9) {
  problems.push(`pacer-worker path turned spool axis D by ${viaPacer.D} steps (${(viaPacer.D * 188.496 / 3200).toFixed(2)} mm of line) instead of extruding`);
}
if (Math.abs(viaPlayer.E_mm - expectedExtrusion) > 0.05) {
  problems.push(`mcu-command-player path extruded ${viaPlayer.E_mm} mm, expected ${expectedExtrusion.toFixed(4)} mm (a 1-step shortfall here is the separate step-loss bug, not the oid mapping)`);
}
if (Math.abs(viaPlayer.D) > 1e-9) {
  problems.push(`mcu-command-player path turned spool axis D by ${viaPlayer.D} steps`);
}

if (problems.length) {
  console.error('\nBUG REPRODUCED: Klipper oid->axis mapping disagrees between the two shipped parsers:');
  for (const p of problems) console.error('  - ' + p);
  process.exit(1);
}
console.log('OK: both Klipper parsers agree and extrude on the extruder axis');
