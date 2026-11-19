import { QualityMonitor, polylineSegments, feed, metricsOf } from './harness.mjs';
const MM = 0.001; const mm = (v) => v * MM;

const segsL = polylineSegments([[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);
const pts = [];
for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
pts.push([mm(49.8), 0, 0]);
pts.push([mm(50), mm(0.2), 0]);
for (let y = 0.7; y <= 49.5; y += 0.5) pts.push([mm(50), mm(y), 0]);
pts.push([mm(50), mm(50), 0]);

const totalE = segsL.reduce((a, s) => a + s.extrusion, 0);
const eStep = totalE / pts.length;

const m = new QualityMonitor();
m.setReferenceSegments(segsL);
const seen = [];
const orig = m._accumulateCorners.bind(m);
m._accumulateCorners = (x, y, z, arc) => { seen.push([x, y, z, arc]); return orig(x, y, z, arc); };
feed(m, pts.map((p) => ({ pos: p, E: eStep })));
const x = metricsOf(m);
console.log('cornerDeviationMax mm', x.cornerDeviationMax * 1000);
console.log('corner', m.corners.map((c) => ({ arc: c.arcLength, vertex: c.vertex.map((v) => v * 1000), win: [c.windowStart, c.windowEnd].map((v) => v * 1000) })));

// find pairs near the corner
const near = seen.filter((s) => Math.abs(s[3] - m.corners[0].arcLength) < 0.006);
console.log('samples near corner:', near.length);
let best = Infinity, bestPair = null;
for (let i = 0; i + 1 < near.length; i += 1) {
  const a = near[i], b = near[i + 1];
  if (b[3] - a[3] > 1e-9 /* any consecutive pair */) {
    const v = m.corners[0].vertex;
    const bx = b[0] - a[0], by = b[1] - a[1], bz = b[2] - a[2];
    const ll = bx * bx + by * by + bz * bz;
    let t = ll > 0 ? ((v[0] - a[0]) * bx + (v[1] - a[1]) * by + (v[2] - a[2]) * bz) / ll : 0;
    t = Math.max(0, Math.min(1, t));
    const d = Math.hypot(v[0] - (a[0] + t * bx), v[1] - (a[1] + t * by), v[2] - (a[2] + t * bz));
    if (d < best) { best = d; bestPair = [a.map((q) => q * 1000), b.map((q) => q * 1000)]; }
  }
}
console.log('min over pairs (mm)', best * 1000, 'pair', JSON.stringify(bestPair));
