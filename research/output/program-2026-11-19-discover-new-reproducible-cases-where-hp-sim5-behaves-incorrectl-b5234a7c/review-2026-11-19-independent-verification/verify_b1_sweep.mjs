// Reviewer scratch: independent re-run of the B1 trigger sweep and of the
// perpendicular control.  Same public API (setReferenceSegments/recordExtrusion),
// independent code.
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';

const MM = 0.001, LEN = 50 * MM, D = 1 * MM;
const seg = (a, b) => ({ start: a, end: b, extrusion: Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) });
const ref = [seg([0, 0, 0], [LEN, 0, 0]), seg([LEN, 0, 0], [LEN, LEN, 0])];

function run(stepsPerLeg, offset) {
  const m = new QualityMonitor({ hudElement: null });
  m.setReferenceSegments(ref);
  for (const s of ref) {
    const d = s.end.map((v, i) => v - s.start[i]);
    for (let i = 0; i < stepsPerLeg; i++) {
      const p = s.start.map((v, k) => v + d[k] * (i / stepsPerLeg) + offset[k]);
      m.recordExtrusion({ pos: p, length: s.extrusion / stepsPerLeg });
    }
  }
  m.runFinalCheck();
  const x = m.getMetrics();
  return { ov: x.cornerOvershootMax * 1000, settle: x.settleDistanceMax * 1000, score: x.score };
}

console.log('offset +X 1 mm, L with 50 mm legs; independent sweep');
for (const n of [2000, 2400, 2450, 2499, 2500, 2501, 2550, 2600, 3000, 4000]) {
  const r = run(n, [D, 0, 0]);
  console.log(`  steps/leg ${String(n).padStart(4)}  overshootMax ${r.ov.toFixed(6)} mm  settleMax ${r.settle.toFixed(6)} mm  score ${r.score.toFixed(3)}`);
}
const perp = run(500, [0, D, 0]);
console.log(`perpendicular control (500 steps): overshootMax ${perp.ov.toFixed(6)} mm  score ${perp.score.toFixed(3)}`);
