#!/usr/bin/env node
// Repro: hp-sim5's RRF CAN-CSV path silently turns a multi-driver CAN board
// into a single (wrong) axis, dropping the other drives' steps entirely.
//
// Fixture `fixtures/rrf_two_drivers_one_board.csv` was produced by the real
// firmware (RRF/build/rrf_simulator) from
//   fixtures/rrf_two_drivers_one_board_config.g   (M584 X40.0 Y40.1 Z40.2 E40.3)
//   fixtures/rrf_two_drivers_one_board.gcode      (G92 X0 Y0 Z0 / G1 X10 Y5 F600)
// with:
//   ./RRF/build/rrf_simulator --vsd <vsd> \
//       --gcode <vsd>/gcodes/probe.gcode --can-log capture.csv \
//       -c sys/config_probe.g
// The firmware capture row is
//   0,40,0,2500,836026,2500,800,400,4.77e-10,4.77e-10
//   |  |  |    |     |    |   |   |    |
//   |  |  |    |     |    |   |   |    +-- deceleration
//   |  |  |    |     |    |   |   +------- acceleration
//   |  |  |    |     |    |   +----------- Y steps (400 = 5 mm @ 80 steps/mm)
//   |  |  |    |     |    +--------------- X steps (800 = 10 mm @ 80 steps/mm)
//   ... (RRF/host/can/CanCapture.cpp writes one steps column per drive)
//
// Two independent defects in integrations/rrf:
//  1. rrfFirmwareModel.parseRrfMotorAxisMapFromConfigText keys the map by CAN
//     *address* only and ignores the `.driver` suffix, so address 40 ends up
//     mapped to the last axis mentioned (E).  All 800 steps then become
//     extrusion instead of X rotation.
//  2. rrfCanPlayer._parseCsvRow assumes a fixed 9-column row, so a second
//     drive in the same CAN message is parsed as the acceleration field
//     (400 instead of 4.77e-10) and its 400 steps are never applied.

globalThis.postMessage = () => {};
globalThis.self = {
  addEventListener() {},
  postMessage: globalThis.postMessage,
};

import path from 'node:path';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');

const { RrfCanPlayer } = await import(
  path.join(ROOT, 'integrations', 'rrf', 'rrfCanPlayer.js'));
const { parseRrfMotorAxisMapFromConfigText, STEP_ANGLE_RAD,
        EXTRUDER_MM_PER_STEP_RRF } = await import(
  path.join(ROOT, 'integrations', 'rrf', 'rrfFirmwareModel.js'));
const { FileFormat } = await import(
  path.join(ROOT, 'integrations', 'shared', 'fileFormatUtils.js'));

const csv = fs.readFileSync(
  path.join(__dirname, 'fixtures', 'rrf_two_drivers_one_board.csv'), 'utf8');
const cfg = fs.readFileSync(
  path.join(__dirname, 'fixtures', 'rrf_two_drivers_one_board_config.g'), 'utf8');

const driverToAxis = parseRrfMotorAxisMapFromConfigText(cfg);
const player = new RrfCanPlayer();
player.setDriverToAxis(driverToAxis);
player.setAsapMode(true);

const commands = [];
player.sendCommand = async (command) => { commands.push(command); };
await player.run(new ReadableStream({
  start(controller) {
    controller.enqueue(new TextEncoder().encode(csv));
    controller.close();
  },
}), FileFormat.RRF_CAN);

const moves = commands.filter((c) => c.type === 'Move');
const final = moves.at(-1) || {};
const travelMm = {};
for (const axis of ['A', 'B', 'C', 'D', 'I', 'J', 'L', 'O']) {
  travelMm[axis] = +(((final[axis] || 0) / STEP_ANGLE_RAD) * 0).toFixed(6);
}
const stepsPerAxis = {};
for (const axis of ['A', 'B', 'C', 'D', 'I', 'J', 'L', 'O']) {
  stepsPerAxis[axis] = +((final[axis] || 0) / STEP_ANGLE_RAD).toFixed(4);
}
// STEP_ANGLE_RAD is the same for every spool axis here (200*16 microsteps).
const extrusionMm = (final.E || 0);

const row = csv.trim().split('\n')[1].split(',');
const parsedAcceleration = Number(row[7]);
const trueAcceleration = Number(row[8]);

console.log(JSON.stringify({
  config_m584: 'X40.0 Y40.1 Z40.2 E40.3',
  driverToAxisAfterParse: [...driverToAxis.entries()],
  firmwareRowColumns: row.length,
  parsedStepsPerAxis: stepsPerAxis,
  parsedExtrusionMm: +extrusionMm.toFixed(4),
  accelerationReadAs: parsedAcceleration,
  accelerationInFirmware: trueAcceleration,
  moveCommands: moves.length,
}, null, 2));

const problems = [];
if (driverToAxis.get(40) !== 'A') {
  problems.push(`CAN address 40 maps to '${driverToAxis.get(40)}' instead of 'A' (X); the .driver suffix is ignored, so the last axis on the board wins`);
}
if (Math.abs(extrusionMm) > 1e-9) {
  problems.push(`${extrusionMm.toFixed(3)} mm of extrusion was synthesised from axis steps (should be 0 mm)`);
}
if (Math.abs(stepsPerAxis.A - 800) > 1e-6) {
  problems.push(`X travel is ${stepsPerAxis.A} steps; expected 800 steps = 10.000 mm at 80 steps/mm`);
}
if (Math.abs(stepsPerAxis.B - 400) > 1e-6) {
  problems.push(`Y travel is ${stepsPerAxis.B} steps; expected 400 steps = 5.000 mm at 80 steps/mm (the second drive in the CAN message is dropped)`);
}
if (Math.abs(parsedAcceleration - trueAcceleration) > 1e-12) {
  problems.push(`acceleration ${parsedAcceleration} instead of ${trueAcceleration} (${(parsedAcceleration / trueAcceleration).toExponential(2)}x too large)`);
}

if (problems.length) {
  console.error('\nBUG REPRODUCED: multi-driver RRF CAN capture is misparsed:');
  for (const p of problems) console.error('  - ' + p);
  process.exit(1);
}
console.log('OK: multi-driver RRF capture parsed correctly');
