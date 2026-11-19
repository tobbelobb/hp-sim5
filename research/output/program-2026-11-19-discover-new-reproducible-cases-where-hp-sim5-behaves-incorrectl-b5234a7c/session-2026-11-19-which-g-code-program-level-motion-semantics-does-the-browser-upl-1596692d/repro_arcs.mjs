#!/usr/bin/env node
// FINDING 6 - G2/G3 arc moves are dropped (motion and extrusion lost).
//
// moveCommander only treats "G1" as a move (moveCommander.js:214); "G2"/"G3"
// fall through to `if (!command) continue;` (lines 222-224).  Consequences:
//   - the arc's travel is never executed, and the following move starts from
//     the stale pre-arc position, so it interpolates across the chord;
//   - the arc's E word never updates last_e (line 236-240), so a trailing arc
//     loses its extrusion entirely and a mid-file arc dumps the whole
//     accumulated E onto the next straight move.
// The app reference parser skips G2/G3 as well
// (inspectionToolsController.js:124), so overlay and simulation agree - both
// disagree with the machine.
//
// Firmware ground truth:
//   RRF    GCodes2.cpp:251-277 "case 2/3" -> DoArcMove (I/J or R form)
//   Klipper klippy/extras/gcode_arcs.py registers G2 and G3.
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, expectBelow, finish } from './harness.mjs';

// Quarter arc, r = 10: length = 10 * pi/2 = 15.708 mm, total 10 + 15.708.
const ARC_END = `M82
G1 X10 Y0 E1 F600
G2 X20 Y10 I10 J0 E2 F600
`;

const m = await measure(ARC_END);
printMeasurement('trailing G2 arc', m);

console.log('\nContract (RRF/Klipper): 10 mm line + quarter arc of r=10 (15.708 mm)');
console.log('  = 25.708 mm travel, 2 mm extrusion, ending at (20, 10).');
const failures = [];
expectClose(failures, 'travel (mm)', 25.708, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'final X (mm)', 20.0, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
expectClose(failures, 'final Y (mm)', 10.0, m.sim.endMm[1], { rel: 0.001, abs: 0.05 });
expectClose(failures, 'extruded filament (mm)', 2.0, m.sim.extrusionMm, { rel: 0.01 });

// Mid-file arc: half circle r=10 (31.416 mm) followed by a 20 mm straight line.
const ARC_MID = `M82
G3 X20 Y0 I10 J0 E2 F600
G1 X20 Y20 E3 F600
`;
const m2 = await measure(ARC_MID);
printMeasurement('mid-file G3 arc', m2);
console.log('\nContract (RRF/Klipper): half circle r=10 (31.416 mm) + 20 mm = 51.416 mm travel.');
expectClose(failures, 'mid-file travel (mm)', 51.416, m2.sim.pathLengthMm, { rel: 0.01 });
expectBelow(failures, 'mid-file max deviation from commanded path (mm)', 0.5, m2.sim.maxPathDeviationMm);

finish('G2/G3 arcs dropped', failures);
