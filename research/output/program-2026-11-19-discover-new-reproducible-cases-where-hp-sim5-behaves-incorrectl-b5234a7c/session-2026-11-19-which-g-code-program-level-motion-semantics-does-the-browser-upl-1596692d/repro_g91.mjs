#!/usr/bin/env node
// FINDING 1 - G91 (relative positioning) is silently ignored.
//
// hp-sim-3d/app/workers/moveCommander.js:209-220 recognises only the literal
// tokens "G1", "G6", "G92", "M82", "M83".  "G91" falls through to
// `if (!command) continue;` (line 222-224) and the relative mode is never
// applied: every X/Y/Z word is treated as absolute (line 229-234).
//
// The app's own reference parser *does* implement G91
// (hp-sim-3d/app/inspectionToolsController.js:96-103), so the uploaded file
// draws a 20 mm path in the reference overlay while the simulated machine only
// travels 10 mm.
//
// Firmware ground truth:
//   RRF    RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:573-576
//   Klipper klipper/klippy/extras/gcode_move.py:174-176
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, finish } from './harness.mjs';

// Smallest reproducer: 2 commanded 10 mm moves in relative mode.
// Firmware: 0 -> X=10 -> X=20, 20 mm of travel, 2 mm of filament.
const GCODE = `M82
G91
G1 X10 Y0 E1 F600
G1 X10 Y0 E2 F600
`;

const m = await measure(GCODE);
printMeasurement('G91 relative positioning', m);

console.log('\nContract (RRF/Klipper): 2 x 10 mm relative moves at F600 = 20 mm travel in 2.0 s');
const failures = [];
expectClose(failures, 'travel (mm)', 20.0, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'final X (mm)', 20.0, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
expectClose(failures, 'extruded filament (mm)', 2.0, m.sim.extrusionMm, { rel: 0.01 });
expectClose(failures, 'duration (s)', 2.0, m.sim.timeSec, { rel: 0.02 });
expectClose(failures, 'reference overlay travel (mm)', 20.0, m.overlay.travelMm, { rel: 0.01 });

// Detail: the second commanded 10 mm move must be 500 steps at dt=0.002 s
// (10 mm / (600/60 mm/s) = 1.0 s = 500 steps).  The simulation emits 501
// commands in total: 500 for move 1 plus a single stationary extrusion step.
const perMove = Math.round((10.0 / (600 / 60)) / m.dt);
expectClose(failures, 'steps for move 2', perMove, m.sim.steps - perMove, { rel: 0.01, abs: 0.5 });

finish('G91 relative positioning ignored', failures);
