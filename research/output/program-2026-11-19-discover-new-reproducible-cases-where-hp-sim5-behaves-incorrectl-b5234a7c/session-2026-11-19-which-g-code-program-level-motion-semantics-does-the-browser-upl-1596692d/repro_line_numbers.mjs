#!/usr/bin/env node
// FINDING 4 - lines carrying an N<line number> (and *<checksum>) are dropped
// wholesale, by both the simulator and the app's own reference parser.
//
// moveCommander takes the first whitespace-separated token as the command and
// compares it literally (hp-sim-3d/app/workers/moveCommander.js:209-220), so
// "N10" is the "command" and the whole line is skipped (line 222-224).
// inspectionToolsController.js:84-124 does the same with the first token, so
// the reference overlay is empty too.
//
// Firmware ground truth: this is valid input.
//   RRF    GCodeBuffer/StringParser.cpp:123-141 accepts "N<digits>" at the start
//          of a line and skips it; StringParser.cpp:186-192 / 272-285 accept and
//          strip a "*<checksum>" suffix.
//   Klipper klippy/gcode.py:199-205 skips the line number
//          (`cmd = ''.join(parts[3:5]).strip()`).
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, finish } from './harness.mjs';

// RS-274 XOR checksum (the classic RepRap/Marlin checksum).
const checksum = (line) => [...line].reduce((c, ch) => c ^ ch.charCodeAt(0), 0);

const l1 = 'N10 G1 X10 Y0 E1 F600';
const l2 = 'N20 G1 X20 Y0 E2 F600';
const GCODE = `M82
${l1}*${checksum(l1)}
${l2}*${checksum(l2)}
`;

console.log('Input lines carry valid N-line-numbers and XOR checksums:');
console.log(`  ${l1}*${checksum(l1)}`);
console.log(`  ${l2}*${checksum(l2)}\n`);

const m = await measure(GCODE);
printMeasurement('N line numbers + checksums', m);

console.log('\nContract (RRF/Klipper): 2 x 10 mm moves = 20 mm travel, 2 mm extrusion, 2.0 s.');
const failures = [];
expectClose(failures, 'travel (mm)', 20.0, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'final X (mm)', 20.0, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
expectClose(failures, 'extruded filament (mm)', 2.0, m.sim.extrusionMm, { rel: 0.01 });

// Isolation: the line number alone already kills the line (checksum is not the
// only trigger), and the reference overlay is empty for the same reason.
const m2 = await measure(`M82
N10 G1 X10 Y0 E1 F600
N20 G1 X20 Y0 E2 F600
`);
printMeasurement('N line numbers only', m2);
expectClose(failures, 'N-only travel (mm)', 20.0, m2.sim.pathLengthMm, { rel: 0.01 });
console.log('\n  note: the app reference overlay for the same file is %d segments, %s mm travel',
  m2.overlay.segments, m2.overlay.travelMm.toFixed(3));

finish('N line numbers / checksums dropped', failures);
