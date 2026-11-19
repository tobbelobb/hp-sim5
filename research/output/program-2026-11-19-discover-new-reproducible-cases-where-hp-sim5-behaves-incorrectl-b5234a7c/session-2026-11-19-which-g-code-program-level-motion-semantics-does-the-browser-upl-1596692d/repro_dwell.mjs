#!/usr/bin/env node
// FINDING 7 - G4 (dwell) contributes no simulated time.
//
// moveCommander recognises only G1/G6/G92 (moveCommander.js:209-220); "G4"
// falls through and produces no command at all, so the physics simply runs on.
// Time in this app is the number of consumed commands times the timestep
// (hp-sim-3d/app/remoteSpoolSystem.js:439-449 consumes exactly one command per
// physics step), so a dwell of 1 s is 1 s of missing print time.
//
// Firmware ground truth:
//   RRF    GCodes.cpp:3742-3790 DoDwell (P in ms, S in seconds; simulationTime
//          advances by the dwell)
//   Klipper klippy/toolhead.py:554-557 cmd_G4 -> toolhead.dwell()
//
// Classification: timing-only (no wrong motion), i.e. a silent unsupported
// command gap that changes the simulated print duration.
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, expectClose, finish } from './harness.mjs';

const GCODE = `M82
G1 X10 Y0 E1 F600
G4 P1000
G1 X20 Y0 E2 F600
`;

const m = await measure(GCODE);
printMeasurement('G4 P1000 dwell', m);

console.log('\nContract (RRF/Klipper): 1.0 s move + 1.0 s dwell + 1.0 s move = 3.0 s total.');
const failures = [];
expectClose(failures, 'duration (s)', 3.0, m.sim.timeSec, { rel: 0.01 });
expectClose(failures, 'travel (mm)', 20.0, m.sim.pathLengthMm, { rel: 0.01 });
expectClose(failures, 'extruded filament (mm)', 2.0, m.sim.extrusionMm, { rel: 0.01 });

finish('G4 dwell not simulated', failures);
