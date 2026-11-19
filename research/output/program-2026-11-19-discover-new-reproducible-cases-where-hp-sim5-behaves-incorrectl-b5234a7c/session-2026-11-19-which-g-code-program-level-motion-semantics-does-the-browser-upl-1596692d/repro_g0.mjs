#!/usr/bin/env node
// FINDING 2 - G0 (rapid) lines are dropped; only literal "G1" moves.
//
// hp-sim-3d/app/workers/moveCommander.js:214  `else if (code === 'G1')`
// - "G0" never matches, falls through, and the position bookkeeping is never
//   updated (line 330).  The next G1 therefore starts from the stale point and
//   interpolates a straight line across both legs of the path.
// - An F word on a G0 line is dropped with the line, so the modal feed rate
//   stays at the previous value (line 124-128).
//
// The reference parser treats G0 and G1 identically
// (hp-sim-3d/app/inspectionToolsController.js:124).
//
// Firmware ground truth: RRF GCodes2.cpp:230-233 ("case 0: // Rapid move"),
// Klipper klipper/gcode.py:46 (`gcode.register_command('G0', self.cmd_G1)`).
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, expectBelow, finish } from './harness.mjs';

// A two-leg print: rapid out, print a leg, rapid over, print a leg.
const GCODE = `M82
G0 X10 Y0 F9000
G1 X10 Y10 E1 F600
G0 X20 Y10 F9000
G1 X20 Y20 E2 F600
`;

const m = await measure(GCODE);
printMeasurement('G0 rapids dropped', m);

console.log('\nContract (RRF/Klipper): 4 moves of 10 mm = 40 mm travel;');
console.log('  time = 2x(10 mm / 150 mm/s) + 2x(10 mm / 10 mm/s) = 0.1333 + 2.0 = 2.1333 s;');
console.log('  every simulated point lies on the commanded path.');
const failures = [];
expectClose(failures, 'travel (mm)', 40.0, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'duration (s)', 2.1333, m.sim.timeSec, { rel: 0.02, abs: 0.005 });
expectClose(failures, 'extruded filament (mm)', 2.0, m.sim.extrusionMm, { rel: 0.01 });
expectBelow(failures, 'max deviation from commanded path (mm)', 0.5, m.sim.maxPathDeviationMm);
// The reference overlay draws the two print legs as separate 10 mm segments;
// the simulation replaces them with two 14.14 mm diagonals.
expectClose(failures, 'reference overlay travel (mm)', 20.0, m.overlay.travelMm, { rel: 0.01 });

// Sub-case: the F word of a G0 line is modal feed rate for the following moves.
const m2 = await measure(`M82
G0 F9000
G1 X10 Y0 E1
`);
printMeasurement('G0 carries the modal feed rate', m2);
console.log('\nContract (RRF/Klipper): F9000 on the G0 sets the modal feed rate,');
console.log('  so the 10 mm G1 takes 10/150 = 0.0667 s.');
expectClose(failures, 'duration after G0 F9000 (s)', 0.0667, m2.sim.timeSec, { rel: 0.02, abs: 0.005 });

finish('G0 rapids dropped', failures);
