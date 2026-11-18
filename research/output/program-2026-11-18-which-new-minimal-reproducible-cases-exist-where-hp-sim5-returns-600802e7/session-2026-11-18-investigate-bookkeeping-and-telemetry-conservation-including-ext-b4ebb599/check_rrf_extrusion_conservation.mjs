#!/usr/bin/env node
// Negative control: the preset / RRF-CAN path conserves extrusion exactly.
//
// The shipped presets do not use MoveCommander: they play the machine's
// compiled RRF CAN capture. Summing the extruder driver's positive step counts
// from that capture and converting with the config's M92 E95.922 must reproduce
// the reference G-code's positive extrusion total.
//
// Run from anywhere:
//   node check_rrf_extrusion_conservation.mjs
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = process.env.HP_SIM5_ROOT || path.resolve(here, '../../../..');

const { parseRrfMotorAxisMapFromConfigText } = await import(
  path.join(root, 'integrations/rrf/rrfFirmwareModel.js'));
const { parseGcodeText } = await import(
  path.join(root, 'hp-sim-3d/app/inspectionToolsController.js'));

const gcodeFile = path.join(root, 'public/gcode/Hangprinter_logo6.gcode');
const canFile = path.join(root, 'public/RRF_CAN_commands/Hangprinter_logo6_hp4_no_buildup.can');
const configFile = path.join(root, 'RRF/run/vsd/sys/config_hp4.g');

const reference = parseGcodeText(fs.readFileSync(gcodeFile, 'utf8'))
  .segments.reduce((sum, segment) => sum + segment.extrusion, 0);

const driverToAxis = parseRrfMotorAxisMapFromConfigText(fs.readFileSync(configFile, 'utf8'));
const extruderDriver = [...driverToAxis.entries()].find(([, axis]) => axis === 'E')?.[0];
if (extruderDriver === undefined) throw new Error('config has no E driver');

const buffer = fs.readFileSync(canFile);
const view = new DataView(buffer.buffer, buffer.byteOffset, buffer.byteLength);
let offset = 0;
let contextTime = 0;
let positiveSteps = 0;
let netSteps = 0;
let extruderMoves = 0;
while (offset < buffer.byteLength) {
  const mask = view.getUint8(offset++);
  const count = view.getUint8(offset++);
  if (mask & 1) {
    const lower = view.getUint32(offset, true);
    const upper = view.getUint32(offset + 4, true);
    contextTime = upper * 2 ** 32 + lower;
    offset += 8;
  }
  if (mask & 2) offset += 4;
  if (mask & 4) offset += 4;
  if (mask & 8) offset += 4;
  if (mask & 16) offset += 4;
  if (mask & 32) offset += 4;
  for (let i = 0; i < count; i += 1) {
    const header = view.getUint8(offset++);
    const type = (header >> 6) & 3;
    const canId = header & 0x3f;
    let value;
    if (type === 0) { value = view.getInt8(offset); offset += 1; }
    else if (type === 1) { value = view.getInt16(offset, true); offset += 2; }
    else if (type === 2) { value = view.getInt32(offset, true); offset += 4; }
    else { value = view.getFloat32(offset, true); offset += 4; }
    if (canId === extruderDriver) {
      positiveSteps += Math.max(0, value);
      netSteps += value;
      extruderMoves += 1;
    }
  }
  if (!Number.isFinite(contextTime)) throw new Error('bad tick');
}

const mmPerStep = 1 / 95.922; // M92 E95.922 in every shipped RRF config
const compiledPositive = positiveSteps * mmPerStep;
const compiledNet = netSteps * mmPerStep;
const relative = (compiledPositive - reference) / reference;

console.log(`E driver: ${extruderDriver}, moves: ${extruderMoves}`);
console.log(`reference G-code positive extrusion: ${reference.toFixed(6)} mm`);
console.log(`RRF CAN positive extrusion:         ${compiledPositive.toFixed(6)} mm`);
console.log(`difference: ${(compiledPositive - reference).toFixed(6)} mm ` +
            `(${(relative * 100).toExponential(3)} %)`);
console.log(`RRF CAN net extrusion (after retracts): ${compiledNet.toFixed(6)} mm`);

const toleranceMm = 0.01;
if (Math.abs(compiledPositive - reference) > toleranceMm) {
  console.error(`\nFAIL: RRF compilation does not conserve extrusion ` +
                `(> ${toleranceMm} mm difference).`);
  process.exit(1);
}
console.log('\nOK: the RRF/CAN preset path conserves extrusion.');
