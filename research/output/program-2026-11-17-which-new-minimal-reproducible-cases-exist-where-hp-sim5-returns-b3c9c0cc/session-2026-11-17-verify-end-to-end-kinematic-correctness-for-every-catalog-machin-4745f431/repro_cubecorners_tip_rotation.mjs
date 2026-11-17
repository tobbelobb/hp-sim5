#!/usr/bin/env node
// F2: CubeCorners does not translate the tool tip by the commanded amount.
//
// A +5 mm X command through RRF -> RrfCanPlayer -> production headless world
// moves the body centre by about +5.05 mm (non-layered) / +5.14 mm (layered),
// but the tool tip by only +4.90 / +4.73 mm.  The effector rotates by
// 0.19 deg / 0.078 deg because the eight-cable layout is nearly singular in
// rotation and the quantised step stream is not exactly the continuous
// line-length change.
//
// The script also solves the exact 6-DOF rigid-body geometry for the actual
// step stream (`geom_cube_6dof.py`).  That geometric prediction reproduces the
// measured tip to < 0.01 mm, while the ideal (unquantised) geometry is exactly
// 5.000 mm.  So the simulator is geometrically faithful; the end-to-end
// command is not Cartesian-accurate.
//
// Exits nonzero while the tip magnitude error is present.

import fs from 'node:fs';
import path from 'node:path';
import { spawnSync, execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..', '..', '..', '..');
const HARNESS = path.join(__dirname, 'kinematics_e2e.mjs');
const PY = path.join(ROOT, '.venv', 'bin', 'python');
const TMP = 'repro-cubecorners.json';
const MOVE = 5;
const TOL_MM = 0.05; // 1 % of the 5 mm command

// The harness exits 1 because these presets fail its own magnitude check;
// that is expected.  We only need its JSON.
spawnSync(process.execPath, [
  HARNESS, `--machines=cubecorners,cubecorners_w_line_layers`, `--move=${MOVE}`, '--axes=X',
  '--settle=8000', '--avg=3000', `--out=${TMP}`,
], { cwd: __dirname, stdio: 'inherit' });

const data = JSON.parse(fs.readFileSync(path.join(__dirname, TMP), 'utf8'));
const rows = [];
for (const machine of data.machines) {
  const row = machine.axes[0];
  const slope = Math.abs(machine.winding[machine.config.anchorOrder[0]].paths[0].slope);
  const steps = {};
  machine.config.anchorOrder.forEach((letter, i) => {
    steps[letter] = row.stepTotals[40 + i] ?? 0;
  });
  const geom = JSON.parse(execFileSync(PY, [
    path.join(__dirname, 'geom_cube_6dof.py'),
    JSON.stringify({ R_eff: slope, steps, move_mm: MOVE, axis: 'X' }),
  ], { encoding: 'utf8' }));
  rows.push({
    machine: machine.name,
    tipMm: row.measuredMajorMm,
    bodyMm: row.bodyDeltaMm[0],
    rotationDeg: geom.actual.rotation_angle_deg,
    geometricTipMm: geom.actual.tip_mm[0],
    idealTipMm: geom.ideal.tip_mm[0],
    stepTotals: row.stepTotals,
    simRadiusMm: slope * 1000,
  });
}

console.log('\nCubeCorners +5 mm X: production path vs pure 6-DOF geometry');
for (const r of rows) {
  console.log(`${r.machine}`);
  console.log(`  step totals ${JSON.stringify(r.stepTotals)}  (sim radius ${r.simRadiusMm.toFixed(3)} mm)`);
  console.log(`  measured tip   ${r.tipMm.toFixed(4)} mm  (${(100 * (r.tipMm / MOVE - 1)).toFixed(2)} %)`);
  console.log(`  measured body  ${r.bodyMm.toFixed(4)} mm  (${(100 * (r.bodyMm / MOVE - 1)).toFixed(2)} %)`);
  console.log(`  body rotation  ${r.rotationDeg.toFixed(5)} deg`);
  console.log(`  6-DOF predicted tip (quantised steps) ${r.geometricTipMm.toFixed(4)} mm`);
  console.log(`  6-DOF predicted tip (ideal line lengths) ${r.idealTipMm.toFixed(4)} mm`);
}

const failures = [];
for (const r of rows) {
  if (Math.abs(r.tipMm - MOVE) > TOL_MM) {
    failures.push(`${r.machine}: tool tip moved ${r.tipMm.toFixed(4)} mm for a ${MOVE} mm +X command `
      + `(${(100 * (r.tipMm / MOVE - 1)).toFixed(2)} %, tolerance +/-${TOL_MM} mm)`);
  }
  if (Math.abs(r.tipMm - r.geometricTipMm) > 0.02) {
    failures.push(`${r.machine}: measured tip ${r.tipMm.toFixed(4)} mm disagrees with the exact `
      + `6-DOF geometric prediction ${r.geometricTipMm.toFixed(4)} mm`);
  }
  if (Math.abs(r.idealTipMm - MOVE) > 0.01) {
    failures.push(`${r.machine}: ideal continuous line lengths do not predict ${MOVE} mm `
      + `(${r.idealTipMm.toFixed(4)} mm)`);
  }
}

if (failures.length > 0) {
  console.error('\nFAIL: CubeCorners command is not Cartesian-accurate at the tool tip:');
  for (const f of failures) console.error('  ' + f);
  process.exit(1);
}
console.log('\nPASS: CubeCorners tip follows the commanded move');
