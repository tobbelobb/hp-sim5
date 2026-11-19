// Probe 10: differential test of rms / p95 / max / horizontal+vertical rms and
// the score against brute-force values on a path with well-separated lines.
import {
  QualityMonitor, polylineSegments, feed, metricsOf, pointToPolylineDistance,
  nearestRank, rms, samplesFromPoints, MM,
} from './harness.mjs';

const mm = (v) => v * MM;
let seed = 987654321;
const rand = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };

// square 40 mm, 0.25 mm sampling, random 3-D deviation of +-0.05 mm
const S = mm(40);
const sq = polylineSegments([[0, 0, 0], [S, 0, 0], [S, S, 0], [0, S, 0], [0, 0, 0]]);
const pts = [];
for (const seg of sq) {
  const n = Math.round(seg.length / 0.00025);
  for (let i = 0; i < n; i += 1) {
    const f = i / n;
    pts.push([
      seg.start[0] + f * (seg.end[0] - seg.start[0]) + (rand() - 0.5) * 0.0001,
      seg.start[1] + f * (seg.end[1] - seg.start[1]) + (rand() - 0.5) * 0.0001,
      (rand() - 0.5) * 0.0001,
    ]);
  }
}
// exact distances to the full polyline
const exact = pts.map((p) => pointToPolylineDistance(p, sq));
// Exact error vectors: recompute nearest point (dx,dy,dz) by brute force.
function nearest(p) {
  let best = Infinity; let d = null;
  for (const s of sq) {
    const [ax, ay, az] = s.start;
    const bx = s.end[0] - ax, by = s.end[1] - ay, bz = s.end[2] - az;
    const ll = bx * bx + by * by + bz * bz;
    let t = 0;
    if (ll > 0) { t = ((p[0] - ax) * bx + (p[1] - ay) * by + (p[2] - az) * bz) / ll; t = Math.min(Math.max(t, 0), 1); }
    const ex = p[0] - (ax + t * bx), ey = p[1] - (ay + t * by), ez = p[2] - (az + t * bz);
    const dd = ex * ex + ey * ey + ez * ez;
    if (dd < best) { best = dd; d = [ex, ey, ez]; }
  }
  return d;
}
const vecs = pts.map(nearest);
const exactH = Math.sqrt(vecs.reduce((a, v) => a + v[0] * v[0] + v[1] * v[1], 0) / vecs.length);
const exactV = Math.sqrt(vecs.reduce((a, v) => a + v[2] * v[2], 0) / vecs.length);

const m = new QualityMonitor();
m.setReferenceSegments(sq);
feed(m, samplesFromPoints(pts, sq));
const x = metricsOf(m);

console.log('samples', x.samples);
console.log('contourRms: exact', rms(exact), 'reported', x.contourRms, 'rel', (x.contourRms - rms(exact)) / rms(exact));
console.log('contourMax: exact', Math.max(...exact), 'reported', x.contourMax);
console.log('contourP95: exact', nearestRank(exact, 0.95), 'reported', x.contourP95,
  'rel%', (x.contourP95 - nearestRank(exact, 0.95)) / nearestRank(exact, 0.95) * 100);
console.log('hRms: exact', exactH, 'reported', x.contourHorizontalRms, 'rel', (x.contourHorizontalRms - exactH) / exactH);
console.log('vRms: exact', exactV, 'reported', x.contourVerticalRms, 'rel', (x.contourVerticalRms - exactV) / exactV);
console.log('cornerCount', x.cornerCount, '(square closed: 4 geometric corners, 3 interior)');
console.log('score', x.score);

// Recompute the score from the reported metric values with the documented
// formula and compare.
const terms = [
  ['Shape 95%', 45, 0.001, x.shapeP95],
  ['Shape RMS', 20, 0.0006, x.shapeRms],
  ['Corners', 10, 0.001, x.cornerCount > 0 ? Math.max(x.cornerDeviationAvg, x.cornerOvershootAvg) : 0],
  ['Ringing', 5, 0.005, x.cornerCount > 0 ? x.settleDistanceAvg : 0],
  ['Offset', 10, 0.005, Math.hypot(...x.meanOffset)],
  ['Missed steps', 10, 20, 0],
];
const recomputed = 100 - terms.reduce((a, [l, w, s, v]) => a + w * (1 - Math.exp(-v / s)), 0);
console.log('score recomputed from reported values', recomputed, 'reported score', x.score,
  'diff', x.score - recomputed);
console.log('reported deductions', x.scoreDeductions.map((d) => `${d.label}=${d.points.toFixed(3)}`).join(' '));
