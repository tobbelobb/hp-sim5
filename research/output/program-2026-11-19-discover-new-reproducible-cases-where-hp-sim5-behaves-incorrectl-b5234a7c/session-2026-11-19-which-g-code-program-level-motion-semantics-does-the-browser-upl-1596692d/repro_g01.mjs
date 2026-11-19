#!/usr/bin/env node
// FINDING 3 - numeric command spellings G00/G01 (and G02/G03) are dropped.
//
// moveCommander compares the raw uppercased first token with the strings
// 'G1' / 'G6' / 'G92' (hp-sim-3d/app/workers/moveCommander.js:209-220), so the
// equally valid RS-274 spellings "G01", "G00", "G02", "G03" never match.
//
// The app's own reference parser parses the command number with
// Number.parseInt (hp-sim-3d/app/inspectionToolsController.js:84-86), so it
// accepts G01 and draws the path: the reference overlay shows 20 mm while the
// simulated machine does not move at all.
//
// Firmware ground truth:
//   RRF    GCodeBuffer/StringParser.cpp:985-1032 parses the command number
//          numerically, so G01 == G1 (GCodes2.cpp:230-233 dispatches on 0/1).
//   Klipper answers "Unknown command:\"G01\"" and does not move (gcode.py:272-306);
//          noted as a limitation.
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, expectBelow, finish } from './harness.mjs';

const GCODE = `M82
G01 X10 Y0 E1 F600
G01 X20 Y0 E2 F600
`;

const m = await measure(GCODE);
printMeasurement('G01 numeric spelling', m);

console.log('\nContract (RRF): G01 == G1, so 2 x 10 mm moves = 20 mm, 2 mm extrusion, 2.0 s.');
const failures = [];
expectClose(failures, 'travel (mm)', 20.0, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'final X (mm)', 20.0, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
expectClose(failures, 'extruded filament (mm)', 2.0, m.sim.extrusionMm, { rel: 0.01 });
expectClose(failures, 'reference overlay travel (mm)', 20.0, m.overlay.travelMm, { rel: 0.01 });

// G00 rapid, same bug, and here it is visible as wrong motion: the rapid is
// dropped, so the following G1 cuts the diagonal instead of the L.
const m2 = await measure(`M82
G00 X10 Y0 F9000
G1 X10 Y10 E1 F600
`);
printMeasurement('G00 numeric spelling', m2);
console.log('\nContract (RRF): G00 to (10, 0) then G1 to (10, 10) = 10 + 10 = 20 mm of travel,');
console.log('  with the filament laid along the vertical leg.');
expectClose(failures, 'G00 travel (mm)', 20.0, m2.sim.pathLengthMm, { rel: 0.01 });
expectBelow(failures, 'G00 max deviation from commanded path (mm)', 0.5, m2.sim.maxPathDeviationMm);

finish('G00/G01 numeric spellings dropped', failures);
