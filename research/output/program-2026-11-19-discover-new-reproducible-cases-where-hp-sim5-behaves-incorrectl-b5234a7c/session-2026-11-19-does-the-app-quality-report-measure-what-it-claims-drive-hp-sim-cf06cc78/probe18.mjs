// Probe 18: closed form for the artefact: corner overshoot == |offset . firstMoveDir|,
// checked at 0/45/90 degrees, plus the exact scenario of the shipped test
// tests/js/hp-sim-3d/qualityMonitor.test.js "a rigidly shifted but otherwise
// perfect print still scores high".
import { QualityMonitor } from './harness.mjs';

function print(segments, offset = () => [0, 0, 0], step = 0.0001) {
  const m = new QualityMonitor();
  m.setReferenceSegments(segments);
  for (const seg of segments) {
    const d = seg.end.map((v, i) => v - seg.start[i]);
    const length = Math.hypot(...d);
    if (length === 0) { m.recordExtrusion({ pos: seg.start, length: seg.extrusion }); continue; }
    const count = Math.max(1, Math.round(length / step));
    for (let i = 0; i < count; i += 1) {
      const p = seg.start.map((v, k) => v + d[k] * (i / count));
      const o = offset(p);
      m.recordExtrusion({ pos: p.map((v, k) => v + o[k]), length: seg.extrusion / count });
    }
  }
  m.runFinalCheck();
  return m.getMetrics();
}

function seg(a, b) { return { start: a, end: b, extrusion: Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) }; }
// L with first leg along +x, second along +y (same as the shipped test).
const L = [seg([0, 0, 0], [0.05, 0, 0]), seg([0.05, 0, 0], [0.05, 0.05, 0])];

for (const deg of [0, 45, 90, 180]) {
  const th = deg * Math.PI / 180;
  const d = 0.001; // 1 mm
  const off = [d * Math.cos(th), d * Math.sin(th), 0];
  const m = print(L, () => off, 0.0001);
  const projected = Math.abs(off[0]); // first move is +x
  console.log(`offset ${deg} deg, |d|=${d * 1000}mm -> |offset.firstMoveDir| = ${(projected * 1000).toFixed(6)}mm`,
    ' cornerOvershootMax =', (m.cornerOvershootMax * 1000).toFixed(6), 'mm',
    ' cornerSettleMax =', (m.settleDistanceMax * 1000).toFixed(6), 'mm',
    ' cornerDeviationMax =', (m.cornerDeviationMax * 1000).toFixed(6), 'mm',
    ' score =', m.score.toFixed(3));
}

// The shipped test scenario, verbatim numbers.
const shipped = print(L, () => [0.001, -0.0005, 0.0003], 0.0001);
console.log('\nshipped test scenario [0.001,-0.0005,0.0003] m on a 50mm L:');
console.log({
  shapeP95_mm: shipped.shapeP95 * 1000,
  shapeRms_mm: shipped.shapeRms * 1000,
  cornerDeviationMax_mm: shipped.cornerDeviationMax * 1000,
  cornerOvershootMax_mm: shipped.cornerOvershootMax * 1000,
  settleMax_mm: shipped.settleDistanceMax * 1000,
  score: shipped.score,
});
console.log('shipped assertions: shapeP95<1e-4 m, cornerDeviationMax<1e-4 m, score>90 ->',
  shipped.shapeP95 < 1e-4, shipped.cornerDeviationMax < 1e-4, shipped.score > 90);
