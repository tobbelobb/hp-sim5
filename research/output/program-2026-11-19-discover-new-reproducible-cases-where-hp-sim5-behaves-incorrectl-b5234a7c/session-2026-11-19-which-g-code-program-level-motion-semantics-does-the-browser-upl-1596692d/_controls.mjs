// Assertion liveness check: for each finding, run a *firmware-equivalent file
// that the shipped parser does handle* through the same expected numbers.  If
// the control passes, the repro's failures are caused by the G-code feature
// under test, not by an unattainable assertion.
//
// Not a deliverable; development evidence for report.md.
import { measure, expectClose, expectBelow, pct } from './harness.mjs';

const results = [];
function run(title, failures) {
  results.push({ title, failures });
  console.log(`${failures.length ? 'FAIL' : 'PASS'}  ${title}${failures.length ? ` (${failures.length} violations)` : ''}`);
}

// 1. G91 -> equivalent absolute file.
{
  const m = await measure('M82\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n');
  const f = [];
  expectClose(f, 'travel', 20.0, m.sim.pathLengthMm);
  expectClose(f, 'endX', 20.0, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
  expectClose(f, 'ext', 2.0, m.sim.extrusionMm);
  expectClose(f, 'time', 2.0, m.sim.timeSec);
  run('control G91 (absolute equivalent file)', f);
}

// 2. G0 -> same coordinates written as G1.
{
  const m = await measure('M82\nG1 X10 Y0 F9000\nG1 X10 Y10 E1 F600\nG1 X20 Y10 F9000\nG1 X20 Y20 E2 F600\n');
  const f = [];
  expectClose(f, 'travel', 40.0, m.sim.pathLengthMm);
  expectClose(f, 'time', 2.1333, m.sim.timeSec, { rel: 0.02, abs: 0.005 });
  expectBelow(f, 'maxDev', 0.5, m.sim.maxPathDeviationMm);
  run('control G0 (same path as G1)', f);
}

// 3. G01 -> G1.
{
  const m = await measure('M82\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n');
  const f = [];
  expectClose(f, 'travel', 20.0, m.sim.pathLengthMm);
  expectClose(f, 'ext', 2.0, m.sim.extrusionMm);
  run('control G01 (G1 spelling)', f);
}

// 4. N/checksum -> stripped lines.
{
  const m = await measure('M82\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n');
  const f = [];
  expectClose(f, 'travel', 20.0, m.sim.pathLengthMm);
  expectClose(f, 'ext', 2.0, m.sim.extrusionMm);
  run('control N/checksum (stripped lines)', f);
}

// 5. G20 -> values already in mm.
{
  const m = await measure('M82\nG1 X25.4 Y0 E1 F600\nG1 X38.1 Y0 E2 F600\n');
  const f = [];
  expectClose(f, 'travel', 38.1, m.sim.pathLengthMm);
  expectClose(f, 'endX', 38.1, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
  run('control G20 (mm equivalent)', f);
}

// 6. G2 quarter arc -> 16-chord polyline with interpolated E (M82 absolute E).
{
  const lines = ['M82', 'G1 X10 Y0 E1 F600'];
  const cx = 20, cy = 0, r = 10; // center = start (10,0) + I10 J0
  const n = 16;
  const a0 = Math.PI, a1 = Math.PI / 2; // quarter arc, clockwise (G2)
  for (let i = 1; i <= n; i++) {
    const t = i / n;
    const ang = a0 + (a1 - a0) * t;
    const x = cx + r * Math.cos(ang), y = cy + r * Math.sin(ang);
    const e = 1 + t;
    lines.push(`G1 X${x.toFixed(6)} Y${y.toFixed(6)} E${e.toFixed(6)} F600`);
  }
  const m = await measure(lines.join('\n') + '\n');
  const f = [];
  expectClose(f, 'travel', 25.708, m.sim.pathLengthMm, { rel: 0.01 });
  expectClose(f, 'endX', 20.0, m.sim.endMm[0], { rel: 0.001, abs: 0.05 });
  expectClose(f, 'endY', 10.0, m.sim.endMm[1], { rel: 0.001, abs: 0.05 });
  expectClose(f, 'ext', 2.0, m.sim.extrusionMm);
  expectBelow(f, 'maxDev', 0.5, m.sim.maxPathDeviationMm);
  console.log('  (16-chord control travel vs analytic arc: %s%%)', pct(25.708, m.sim.pathLengthMm).toFixed(2));
  run('control G2 (16-chord polyline)', f);
}

// 7. G4 dwell -> replaced by an equivalent 1 s of commanded time via F-scaled
//    motion is not equivalent, so the control only shows the dwell-free file
//    matches the oracle exactly (failure isolated to the dwell itself).
{
  const m = await measure('M82\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n');
  const f = [];
  expectClose(f, 'time without dwell', 2.0, m.sim.timeSec, { rel: 0.02, abs: 0.005 });
  run('control G4 (dwell-free file matches oracle)', f);
}

// 8. default feed rate -> add F3000.
{
  const m = await measure('M82\nG1 X10 Y0 E1 F3000\nG1 X20 Y0 E2\n');
  const f = [];
  expectClose(f, 'time at F3000', 0.4, m.sim.timeSec, { rel: 0.02, abs: 0.005 });
  run('control feedrate (F3000 present)', f);
}

const bad = results.filter((r) => r.failures.length);
console.log(`\n${results.length - bad.length}/${results.length} controls pass`);
process.exit(bad.length ? 1 : 0);
