#!/usr/bin/env node
// FINDING 8 - the initial modal feed rate is 1000 mm/min, 3x slower than RRF
// and 1.5x slower than Klipper.
//
// hp-sim-3d/app/workers/moveCommander.js:32
//   this.last_speed_mm_per_min = 1000.0;
// A G1 with no F word therefore runs at 1000 mm/min.  Nothing in the repository
// documents 1000 mm/min as an intentional default (hp-sim-3d/README.md does not
// document a G-code subset or a default feed rate).
//
// Firmware ground truth:
//   RRF    Config/Configuration.h:220  `DefaultFeedRate = 3000.0` mm/min
//           (GCodeMachineState.cpp:16 initialises feedRate to it)
//   Klipper gcode_move.py:55 `self.speed = 25.` mm/s = 1500 mm/min
//
// Classification: timing-only (no wrong motion).
//
// Exit code: 1 when the bug is present.

import { measure, printMeasurement, finish } from './harness.mjs';

// No F word anywhere: 20 mm of travel.
const GCODE = `M82
G1 X10 Y0 E1
G1 X20 Y0 E2
`;

const m = await measure(GCODE);
printMeasurement('default feed rate (no F word)', m);

const rrf = 20.0 / (3000 / 60);
const klipper = 20.0 / (1500 / 60);
console.log('\nContract: no F word -> firmware default feed rate.');
console.log('  RRF 3000 mm/min -> %s s; Klipper 1500 mm/min -> %s s; simulated -> %s s',
  rrf.toFixed(4), klipper.toFixed(4), m.sim.timeSec.toFixed(4));
console.log('  simulated/commanded = %sx (RRF) and %sx (Klipper)',
  (m.sim.timeSec / rrf).toFixed(2), (m.sim.timeSec / klipper).toFixed(2));

const failures = [];
// The simulation must be no slower than the slowest of the two documented
// firmware defaults (Klipper's 1500 mm/min).
const limit = klipper * 1.01;
if (m.sim.timeSec > limit) {
  const line = `FAIL  duration: expected <= ${limit.toFixed(4)} s (Klipper default), got ${m.sim.timeSec.toFixed(4)} s`;
  console.log(`  ${line}`);
  failures.push(line);
} else {
  console.log(`  pass  duration: ${m.sim.timeSec.toFixed(4)} s <= ${limit.toFixed(4)} s`);
}

finish('default feed rate 1000 mm/min', failures);
