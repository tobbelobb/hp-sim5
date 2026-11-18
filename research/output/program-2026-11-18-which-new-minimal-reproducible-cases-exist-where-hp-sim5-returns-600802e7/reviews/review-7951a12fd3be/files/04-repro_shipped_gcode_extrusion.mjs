#!/usr/bin/env node
// Reproducer: a shipped PrusaSlicer G-code file loses 0.46 % of its extrusion
// when uploaded through the simulation path, because MoveCommander ignores the
// file's 121 `G92 E0` resets.
//
// Compares, on the same file:
//   * hp-sim-3d/app/inspectionToolsController.js parseGcodeText (the app's own
//     reference path / quality-monitor segmentation), and
//   * hp-sim-3d/app/workers/moveCommander.js (the .gcode upload simulation path).
//
// Run from anywhere:
//   node repro_shipped_gcode_extrusion.mjs
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = process.env.HP_SIM5_ROOT || path.resolve(here, '../../../..');

globalThis.self = { addEventListener() {} };
const sent = [];
globalThis.postMessage = (msg) => {
  if (msg?.action === 'gcode') sent.push(msg.command);
};

const { MoveCommander } = await import(path.join(root, 'hp-sim-3d/app/workers/moveCommander.js'));
const { parseGcodeText } = await import(path.join(root, 'hp-sim-3d/app/inspectionToolsController.js'));

const file = path.join(root, 'public/gcode/Hangprinter_logo6.gcode');
const text = fs.readFileSync(file, 'utf8');
const reference = parseGcodeText(text).segments.reduce((sum, segment) => sum + segment.extrusion, 0);

const commander = new MoveCommander();
commander.configure('M669 A0:0:0 B1:0:0 C0:1:0 D0:0:1\nM666 R39:39:39:39\n');
commander.dt = 10;
commander.fastMode = true;
commander.setSpeedScale(1e9);
await commander.run(new ReadableStream({
  start(controller) {
    controller.enqueue(new TextEncoder().encode(text));
    controller.close();
  },
}));
const simulation = sent.reduce((sum, cmd) => sum + (Number.isFinite(cmd.E) && cmd.E > 0 ? cmd.E : 0), 0);
const g92Count = (text.match(/^G92[^\n]*\bE/gm) ?? []).length;
const missing = reference - simulation;
const relative = missing / reference;

console.log(`file: ${path.relative(root, file)}`);
console.log(`G92 E resets: ${g92Count}`);
console.log(`reference (parseGcodeText) positive extrusion: ${reference.toFixed(6)} mm`);
console.log(`MoveCommander positive extrusion:              ${simulation.toFixed(6)} mm`);
console.log(`difference: ${missing.toFixed(6)} mm (${(relative * 100).toFixed(6)} %)`);

// The app's reference path and the simulation path describe the same file; they
// must agree to far better than the 0.1 mm scale of a deposited bead.
const toleranceMm = 0.01;
if (Math.abs(missing) > toleranceMm) {
  console.error(`\nBUG REPRODUCED: simulation loses ${missing.toFixed(6)} mm of extrusion ` +
                `(${(relative * 100).toFixed(3)} %), above the ${toleranceMm} mm tolerance.`);
  process.exit(1);
}
console.log('\nOK: extrusion agrees within tolerance.');
