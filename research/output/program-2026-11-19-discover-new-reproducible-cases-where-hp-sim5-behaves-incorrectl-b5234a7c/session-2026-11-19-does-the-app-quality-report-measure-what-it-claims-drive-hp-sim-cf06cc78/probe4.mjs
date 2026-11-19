import { QualityMonitor, polylineSegments, feed, metricsOf } from './harness.mjs';
const MM = 0.001; const mm = (v) => v * MM;

const segsL = polylineSegments([[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);
const pts = [];
for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
pts.push([mm(49.8), 0, 0]);
pts.push([mm(50), mm(0.2), 0]);
for (let y = 0.7; y <= 49.5; y += 0.5) pts.push([mm(50), mm(y), 0]);
pts.push([mm(50), mm(50), 0]);

// exact min distance from the corner vertex to the attained polyline
function vertexMin(points, v) {
  let best = Infinity, bestPair = null;
  for (let i = 0; i + 1 < points.length; i++) {
    const a = points[i], b = points[i + 1];
    const bx = b[0] - a[0], by = b[1] - a[1], bz = b[2] - a[2];
    const ll = bx * bx + by * by + bz * bz;
    let t = ll > 0 ? ((v[0] - a[0]) * bx + (v[1] - a[1]) * by + (v[2] - a[2]) * bz) / ll : 0;
    t = Math.max(0, Math.min(1, t));
    const d = Math.hypot(v[0] - (a[0] + t * bx), v[1] - (a[1] + t * by), v[2] - (a[2] + t * bz));
    if (d < best) { best = d; bestPair = [i, d]; }
  }
  return { best, bestPair };
}
console.log('exact min distance corner->attained polyline (mm):', vertexMin(pts, [mm(50), 0, 0]).best * 1000, vertexMin(pts, [mm(50), 0, 0]).bestPair);

const totalE = segsL.reduce((a, s) => a + s.extrusion, 0);
const eStep = totalE / pts.length;
const m = new QualityMonitor();
m.setReferenceSegments(segsL);
feed(m, pts.map((p) => ({ pos: p, E: eStep })));
const x = metricsOf(m);
console.log('reported cornerDeviationMax (mm):', x.cornerDeviationMax * 1000,
  'avg', x.cornerDeviationAvg * 1000);
console.log('meanOffset (mm):', x.meanOffset.map((v) => v * 1000));
console.log('cornerCount', x.cornerCount, 'overshoot', x.cornerOvershootMax * 1000, 'settle', x.settleDistanceMax * 1000);
console.log('shapeP95', x.shapeP95 * 1000, 'shapeRms', x.shapeRms * 1000, 'contourMax', x.contourMax * 1000);
