// repro_corner_overshoot_reports_offset.mjs
//
// The quality report claims corners are judged "after removing the mean offset"
// and that "Corners are judged on shape too: rounding and overshoot, not the
// offset" (hp-sim-3d/app/quality-monitor.js:128,681), and that "a rigidly
// shifted but otherwise perfect print still scores high" (:132-135).
//
// On a perfectly sharp, perfectly printed corner whose first move is parallel to
// the print's rigid offset, the report shows
//     Corner overshoot max = the full offset          (expected 0)
//     Corner settling max  = one sample step          (expected 0)
// and the score depends on the offset *direction*.
//
// Trigger (documented in report.md): the offset estimate is taken before the
// current sample joins the normal equations (:646 vs :659-667), so the first
// sample of a new direction is compared with an offset that does not yet
// constrain that direction. Whether that sample is scored as overshoot depends
// on its intended arc length being strictly greater than the corner's
// (:772). Here the 50 mm leg is divided into 500 steps of 0.1 mm; the E
// accumulator lands a float epsilon past the vertex, so the sample is scored.
// With 1024 steps (exact binary E steps) the same print reports 0.001 mm
// instead -- the displayed number flips between 0 and the full offset.
//
// Only the public API is used: setReferenceSegments + recordExtrusion, the same
// calls inspectionToolsController.setSegments / remoteSpoolSystem make, and the
// HUD is rendered into a stub element exactly like
// tests/js/hp-sim-3d/qualityMonitor.test.js does.
//
// Exit code 1 = the report is wrong; 0 = correct.
// Run: timeout 30 node repro_corner_overshoot_reports_offset.mjs
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';

const MM = 0.001;
const D = 1.0 * MM;        // 1 mm rigid offset
const LEN = 50 * MM;       // 50 mm legs
const STEP = 0.1 * MM;     // 0.1 mm of travel per extrusion sample; the app emits the
                           // tip at the start of each step's E chunk (RemoteSpoolSystem
                           // processes the command before the move it extrudes for).

function segment(a, b) {
  return { start: a, end: b, extrusion: Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) };
}

function stubHud() {
  const classes = new Set(['sim-hidden']);
  return {
    innerHTML: '', textContent: '',
    style: { setProperty() {} },
    classList: {
      add: (...c) => c.forEach((x) => classes.add(x)),
      remove: (...c) => c.forEach((x) => classes.delete(x)),
      contains: (c) => classes.has(c),
      toggle: (c, on) => (on ? classes.add(c) : classes.delete(c)),
    },
    addEventListener() {}, removeEventListener() {},
  };
}

// 50 mm +X leg then 50 mm +Y leg: one perfectly sharp 90 degree corner.
const reference = [segment([0, 0, 0], [LEN, 0, 0]), segment([LEN, 0, 0], [LEN, LEN, 0])];

function hudField(html, label) {
  const m = html.match(new RegExp(`${label.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}</span><span>([^<]*)</span>`));
  return m ? m[1] : '(not rendered)';
}

function run(offset) {
  const hud = stubHud();
  const m = new QualityMonitor({ hudElement: hud });
  m.setReferenceSegments(reference);
  for (const seg of reference) {
    const d = seg.end.map((v, i) => v - seg.start[i]);
    const count = Math.round(seg.extrusion / STEP);
    for (let i = 0; i < count; i += 1) {
      const p = seg.start.map((v, k) => v + d[k] * (i / count));
      m.recordExtrusion({ pos: p.map((v, k) => v + offset[k]), length: seg.extrusion / count });
    }
  }
  m.runFinalCheck();
  m.refreshHud(true);
  return {
    metrics: m.getMetrics(),
    hudOvershoot: hudField(hud.innerHTML, 'Corner overshoot avg / max'),
    hudSettling: hudField(hud.innerHTML, 'Corner settling avg / max'),
    hudOffset: hudField(hud.innerHTML, 'Mean offset X / Y / Z'),
    hudScore: hudField(hud.innerHTML, 'Quality').replace(/<[^>]*>/g, ''),
    hudQuality: (hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, '--'])[1],
  };
}

const along = run([D, 0, 0]);    // offset parallel to the first move
const across = run([0, D, 0]);   // identical shape error, offset perpendicular

const mm = (v) => v * 1000;
const failures = [];
function check(name, actualMm, expectedMm, tolMm = 1e-9) {
  const ok = Math.abs(actualMm - expectedMm) <= tolMm;
  console.log(`${ok ? 'ok  ' : 'FAIL'}  ${name}: expected ${expectedMm.toFixed(6)} mm, reported ${actualMm.toFixed(6)} mm`);
  if (!ok) failures.push(name);
}

console.log('reference: perfect 90 deg corner at (50,0) mm, 50 mm legs, 0.1 mm samples');
console.log('print: exactly on the reference path, rigidly shifted by 1.000 mm\n');

console.log('-- offset ALONG the first move (+X) --');
console.log(`   HUD "Corner overshoot avg / max": ${along.hudOvershoot}`);
console.log(`   HUD "Corner settling avg / max" : ${along.hudSettling}`);
console.log(`   HUD "Mean offset X / Y / Z"     : ${along.hudOffset}`);
console.log(`   HUD "Quality"                   : ${along.hudQuality}`);
check('cornerOvershootMax (offset along first move)', mm(along.metrics.cornerOvershootMax), 0);
check('cornerOvershootAvg (offset along first move)', mm(along.metrics.cornerOvershootAvg), 0);
check('settleDistanceMax  (offset along first move)', mm(along.metrics.settleDistanceMax), 0);
check('cornerDeviationMax', mm(along.metrics.cornerDeviationMax), 0);
check('shapeP95', mm(along.metrics.shapeP95), 0, 1e-3);

console.log('\n-- control: same 1.000 mm offset, perpendicular to the first move (+Y) --');
console.log(`   HUD "Corner overshoot avg / max": ${across.hudOvershoot}`);
console.log(`   HUD "Corner settling avg / max" : ${across.hudSettling}`);
console.log(`   HUD "Quality"                   : ${across.hudQuality}`);
check('cornerOvershootMax (offset perpendicular)', mm(across.metrics.cornerOvershootMax), 0);
check('settleDistanceMax  (offset perpendicular)', mm(across.metrics.settleDistanceMax), 0);

console.log(`\nscore for the same rigid 1.000 mm shift: along=${along.metrics.score.toFixed(3)} ` +
  `perpendicular=${across.metrics.score.toFixed(3)}  difference=${(across.metrics.score - along.metrics.score).toFixed(3)} points`);
console.log('mean offset magnitude reported for both prints: ' +
  `${mm(Math.hypot(...along.metrics.meanOffset)).toFixed(4)} mm / ${mm(Math.hypot(...across.metrics.meanOffset)).toFixed(4)} mm`);

if (failures.length) {
  console.log(`\nRESULT: WRONG. ${failures.length} reported value(s) differ from the closed-form expectation.`);
  process.exit(1);
}
console.log('\nRESULT: correct');
