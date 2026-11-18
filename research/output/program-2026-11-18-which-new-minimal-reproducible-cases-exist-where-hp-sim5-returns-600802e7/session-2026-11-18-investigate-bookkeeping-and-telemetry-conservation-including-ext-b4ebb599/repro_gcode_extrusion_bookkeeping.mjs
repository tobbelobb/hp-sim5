#!/usr/bin/env node
// Reproducer: the G-code upload path (hp-sim-3d/app/workers/moveCommander.js)
// disagrees with the app's own G-code reference parser
// (hp-sim-3d/app/inspectionToolsController.js, parseGcodeText) about the same
// extruder commands.
//
// MoveCommander starts in relative-E mode (the firmwares default to absolute),
// ignores M83, ignores `G92 E`, and keeps `last_e` across run()/configure()
// calls, so the extrusion it sends does not sum to the reference path's
// extrusion. This script asserts the two totals agree for representative files.
//
// Run from anywhere:
//   node repro_gcode_extrusion_bookkeeping.mjs
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

const CONFIG = 'M669 A0:0:0 B1:0:0 C0:1:0 D0:0:1\nM666 R39:39:39:39\n';
const streamOf = text => new ReadableStream({
  start(controller) {
    controller.enqueue(new TextEncoder().encode(text));
    controller.close();
  },
});

function newCommander() {
  const commander = new MoveCommander();
  commander.configure(CONFIG);
  // Large dt and no pacing: only the extrusion bookkeeping is under test.
  commander.dt = 10;
  commander.fastMode = true;
  commander.setSpeedScale(1e9);
  return commander;
}

async function simulationExtrusionMm(commander, text) {
  sent.length = 0;
  await commander.run(streamOf(text));
  return sent.reduce((sum, cmd) => sum + (Number.isFinite(cmd.E) && cmd.E > 0 ? cmd.E : 0), 0);
}

function referenceExtrusionMm(text) {
  return parseGcodeText(text).segments.reduce((sum, segment) => sum + segment.extrusion, 0);
}

const cases = [
  {
    name: 'control: absolute E with M82 (should agree)',
    text: 'M82\nG1 X10 E5\nG1 X20 E10\n',
  },
  {
    name: 'absolute E, firmware default, no M82 (RRF and Klipper default to absolute)',
    text: 'G92 E0\nG1 X10 E5\nG1 X20 E10\n',
  },
  {
    name: 'absolute E with a mid-file G92 E0 reset',
    text: 'M82\nG92 E0\nG1 X10 E5\nG1 X20 E10\nG92 E0\nG1 X30 E5\n',
  },
  {
    name: 'M82 then M83 (switch to relative E)',
    text: 'M82\nM83\nG1 X10 E5\nG1 X20 E5\n',
  },
];

const failures = [];
for (const testCase of cases) {
  const expected = referenceExtrusionMm(testCase.text);
  const actual = await simulationExtrusionMm(newCommander(), testCase.text);
  const ok = Math.abs(actual - expected) <= 1e-9;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${testCase.name}: reference ${expected} mm, ` +
              `MoveCommander ${actual} mm`);
  if (!ok) {
    failures.push(`${testCase.name}: reference ${expected} mm, simulation ${actual} mm ` +
                  `(${((actual / expected) * 100).toFixed(3)} %)`);
  }
}

// `last_e` survives run()/configure(); a second G-code job starts from the
// previous job's extruder odometer.
const commander = newCommander();
const first = await simulationExtrusionMm(commander, 'M82\nG92 E0\nG1 X10 E100\n');
const second = await simulationExtrusionMm(commander, 'M82\nG92 E0\nG1 X10 E5\n');
const secondExpected = referenceExtrusionMm('M82\nG92 E0\nG1 X10 E5\n');
console.log(`first job: ${first} mm (expect 100)`);
console.log(`second job on the same worker: ${second} mm (expect ${secondExpected})`);
if (Math.abs(second - secondExpected) > 1e-9) {
  failures.push(`second job on the same worker: reference ${secondExpected} mm, ` +
                `simulation ${second} mm; configure() does not reset last_e`);
}

if (failures.length > 0) {
  console.error('\nBUG REPRODUCED: G-code upload extrusion accounting disagrees with the reference path.');
  for (const failure of failures) console.error('  - ' + failure);
  process.exit(1);
}
console.log('\nOK: MoveCommander extrusion agrees with the reference parser.');
