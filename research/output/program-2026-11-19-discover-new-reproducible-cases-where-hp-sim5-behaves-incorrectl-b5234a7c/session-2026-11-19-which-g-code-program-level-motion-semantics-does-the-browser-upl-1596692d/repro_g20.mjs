#!/usr/bin/env node
// FINDING 5 - G20 (inches) is silently interpreted as millimetres.
//
// Neither parser looks at G20/G21: moveCommander skips the line
// (hp-sim-3d/app/workers/moveCommander.js:222-224), and coordinates are taken
// as mm directly (line 229-234).  inspectionToolsController.js has no G20 case
// either (lines 88-126), so overlay and simulation agree with each other but
// both under-travel by 25.4x.
//
// Firmware ground truth:
//   RRF    GCodes2.cpp:376-380 "case 20: // Inches" -> gb.UseInches(true);
//          GCodeBuffer.cpp:1013-1016 ConvertDistance multiplies by InchToMm=25.4
//          (RepRapFirmware.h:632). ConvertSpeed does the same for F, and
//          GCodes.cpp:1985 converts the E word with ConvertDistance as well.
//   Klipper rejects G20 with an explicit error (gcode_move.py:159-161), i.e.
//          the user is told, instead of silently getting a 25.4x smaller print.
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, finish } from './harness.mjs';

// 1.0 in -> 25.4 mm, 1.5 in -> 38.1 mm, and E is a distance too:
// E1 -> 25.4 mm, E2 -> 50.8 mm (absolute E, M82).
const GCODE = `M82
G20
G1 X1 Y0 E1 F600
G1 X1.5 Y0 E2 F600
`;

const m = await measure(GCODE);
printMeasurement('G20 inches', m);

console.log('\nContract (RRF): X1 = 25.4 mm and X1.5 = 38.1 mm, so 38.1 mm of travel;');
console.log('  E is scaled the same way, so extrusion = 2 in = 50.8 mm.');
console.log('  Extrusion *density* is 50.8/38.1 = 2.0/1.5 = 1.333 mm/mm in both, so the');
console.log('  error is a uniform 25.4x shrink of the whole program, not a density error.');
const failures = [];
expectClose(failures, 'travel (mm)', 38.1, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'final X (mm)', 38.1, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
expectClose(failures, 'extruded filament (mm)', 50.8, m.sim.extrusionMm, { rel: 0.01 });

finish('G20 inches treated as mm', failures);
