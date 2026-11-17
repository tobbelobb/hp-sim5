#!/usr/bin/env node
// Feed a real Klipper MCU command stream (batch mode via klippy.py) through the
// JS bucketed motion core that the hp-sim5 Klipper player uses, and compare the
// commanded position-vs-time with the exact trapezoid.
//
// Fixture: fixtures/klipper_cartesian_x80_f600.txt is `parsedump.py` output for
//   SET_KINEMATIC_POSITION X=0 Y=0 Z=0 / G90 / G1 X80 F600
// on repro/klipper_cartesian.cfg (80 steps/mm, max_velocity 300, max_accel 3000).
//
// Analytic: 80 mm at F600 = 10 mm/s, accel 3000 mm/s^2, so 1.333 mm is spent
// ramping (t = 3.333 ms) and the cruise runs at exactly 10 mm/s.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');

const { KlipperBucketedMotionCore, applyBucketedParsedLine, wrap32 } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperMotionCore.js'));
const { STEP_ANGLE_RAD } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperFirmwareModel.js'));

const lines = fs.readFileSync(
  path.join(__dirname, 'fixtures', 'klipper_cartesian_x80_f600.txt'), 'utf8')
  .split('\n');

const core = new KlipperBucketedMotionCore({ bucketIntervalMs: 2 });
for (const line of lines) applyBucketedParsedLine(core, line);
const moves = core.flushCommands({
  force: true,
  buildTiming: (bucketIdx) => ({
    at: (core.getBucketEndTick(bucketIdx) / 50_000_000) * 1000,
    span: (core.ticksPerBucket / 50_000_000) * 1000,
  }),
});

// applyBucketedParsedLine maps queue_step oid -> DEFAULT_AXIS_ORDER[oid], so
// pick whichever axis received the motion instead of assuming 'A'.
let movingAxis = null;
let best = 0;
for (const m of moves) {
  if (m.type !== 'Move') continue;
  for (const [axis, value] of Object.entries(m)) {
    if (axis === 'type' || axis === 'at' || axis === 'span') continue;
    if (Math.abs(value) > best) { best = Math.abs(value); movingAxis = axis; }
  }
}

const points = [];
let cumulative = 0;
for (const m of moves) {
  if (m.type !== 'Move') continue;
  const steps = (m[movingAxis] || 0) / STEP_ANGLE_RAD;
  const delta = steps - cumulative;
  cumulative = steps;
  points.push({ t: (m.at || 0) / 1000, mm: steps / 80, delta });
}
const totalSteps = cumulative;

// Exact closed form (mm): v = 10 mm/s, a = 3000 mm/s^2, distance 80 mm.
const v = 10, a = 3000;
const tAcc = v / a, dAcc = 0.5 * a * tAcc * tAcc;
const cruise = 80 - 2 * dAcc;
const timeAtDistance = (d) => (d <= dAcc ? Math.sqrt(2 * d / a)
  : d <= dAcc + cruise ? tAcc + (d - dAcc) / v
  : tAcc + cruise / v + (v - Math.sqrt(Math.max(0, v * v - 2 * a * (d - dAcc - cruise)))) / a);
const distanceAtTime = (t) => (t <= tAcc ? 0.5 * a * t * t
  : t <= tAcc + cruise / v ? dAcc + v * (t - tAcc)
  : 80 - 0.5 * a * (tAcc + cruise / v + tAcc - t) ** 2);
const expectedTime = timeAtDistance(80);

// The stream starts at MCU clock 0 and the emulator smears the first (idle)
// interval over the preceding buckets, so align the analytic clock on a
// mid-cruise crossing instead of on the first bucket.
const anchorPoint = points.find((p) => p.mm >= 10) || points[0];
const tStart = anchorPoint.t - timeAtDistance(anchorPoint.mm);
let worst = { err: 0 };
for (const p of points) {
  if (p.t < tStart + 0.05) continue;   // skip the idle-smear / ramp start
  const expected = distanceAtTime(Math.min(80, Math.max(0, p.t - tStart)));
  const err = Math.abs(p.mm - expected);
  if (err > worst.err) worst = { err, t: p.t, actual: p.mm, expected };
}

console.log(JSON.stringify({
  movingAxis,
  moveCommands: points.length,
  totalSteps,
  expectedSteps: 6400,
  stepError: totalSteps - 6400,
  emulatedMoveStartSec: +tStart.toFixed(4),
  emulatedMoveEndSec: +(points.at(-1).t).toFixed(4),
  expectedMoveEndSec: +(tStart + expectedTime).toFixed(4),
  worstPositionErrorMmAfterRamp: +worst.err.toFixed(4),
  worstAt: { t: +(worst.t || 0).toFixed(4), actualMm: +(worst.actual || 0).toFixed(4),
             expectedMm: +(worst.expected || 0).toFixed(4) },
  maxStepsPerBucket: Math.max(...points.map((p) => Math.abs(p.delta))),
  analyticMaxStepsPerBucket: 10 * 0.002 * 80,
}, null, 2));

const problems = [];
if (Math.abs(totalSteps - 6400) > 1e-6) problems.push(`final step count ${totalSteps}, expected 6400`);
if (worst.err > 0.005) problems.push(`commanded position is off the analytic trapezoid by ${worst.err.toFixed(4)} mm after the ramp`);
const peak = Math.max(...points.map((p) => Math.abs(p.delta)));
if (Math.abs(peak - 1.6) > 0.05) problems.push(`peak commanded rate ${peak.toFixed(3)} steps/bucket, analytic 1.600`);
if (problems.length) {
  console.error('\nFAIL: ' + problems.join('; '));
  process.exit(1);
}
console.log('OK: bucketed emulation matches the analytic trapezoid within one bucket of travel');
