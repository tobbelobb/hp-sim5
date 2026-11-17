#!/usr/bin/env node
// Control experiment for the negative results: every non-layered catalog
// machine except CubeCorners (which F2 covers) must move the tool tip by the
// commanded magnitude on all three axes, with no cross-axis motion, and every
// motor must reach its commanded spool angle.
//
// Reads results.json produced by kinematics_e2e.mjs.  Exits 0 when correct.

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const data = JSON.parse(fs.readFileSync(path.join(__dirname, 'results.json'), 'utf8'));
const STEP_ANGLE_RAD = (2 * Math.PI) / 3200;
const TOL_REL = 0.015;      // 1.5 % of the commanded move
const TOL_CROSS = 0.10;     // mm
const TOL_STEP = 2;         // microsteps in the CAN step audit
const TOL_SPOOL = 2 * STEP_ANGLE_RAD;

const skipped = [];
const checked = [];
const failures = [];

for (const machine of data.machines) {
  if (machine.lineLayering) { skipped.push(`${machine.name}: layered (F1)`); continue; }
  if (machine.nicknames.includes('cubecorners')) { skipped.push(`${machine.name}: CubeCorners (F2)`); continue; }
  for (const row of machine.axes) {
    if (row.noStepMotion) {
      skipped.push(`${machine.name} ${row.axis}: firmware emits no motion (planar machine)`);
      continue;
    }
    checked.push({ machine: machine.name, row });
    const relErr = Math.abs(row.measuredMajorMm - row.commandedMm) / row.commandedMm;
    if (relErr > TOL_REL) {
      failures.push(`${machine.name} ${row.axis}: tip ${row.measuredMajorMm.toFixed(4)} mm for `
        + `${row.commandedMm} mm (${(100 * relErr).toFixed(2)} %, tol ${100 * TOL_REL} %)`);
    }
    if (row.measuredCrossMm > TOL_CROSS) {
      failures.push(`${machine.name} ${row.axis}: cross-axis ${row.measuredCrossMm.toFixed(4)} mm `
        + `(tol ${TOL_CROSS} mm)`);
    }
    for (const [axis, err] of Object.entries(row.spoolAngleErrorRad)) {
      if (Math.abs(err) > TOL_SPOOL) {
        failures.push(`${machine.name} ${row.axis}: spool ${axis} angle error ${err} rad`);
      }
    }
  }
}

for (const [key, rows] of Object.entries(data.stepAudit)) {
  for (const r of rows) {
    if (r.errorSteps !== undefined && Math.abs(r.errorSteps) > TOL_STEP) {
      failures.push(`${key} ${r.letter}: CAN steps differ from the RRF closed form by ${r.errorSteps}`);
    }
  }
}

console.log(`checked ${checked.length} machine/axis pairs`);
for (const s of skipped) console.log(`  skipped: ${s}`);
for (const c of checked) {
  const r = c.row;
  console.log(`  ${c.machine.padEnd(56)} ${r.axis}  `
    + `tip ${r.measuredMajorMm.toFixed(4)} mm  cross ${r.measuredCrossMm.toFixed(4)} mm  `
    + `osc ${Math.max(...r.measuredOscillationMm).toFixed(3)} mm`);
}

if (failures.length > 0) {
  console.error('\nFAIL: an axis outside the two findings is wrong:');
  for (const f of failures) console.error('  ' + f);
  process.exit(1);
}
console.log('\nPASS: every other catalogue machine/axis follows the commanded magnitude and direction');
