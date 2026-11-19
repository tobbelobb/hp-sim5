// Probe 3: corners. Exact attained geometry, independent computation.
import { QualityMonitor, polylineSegments, samplePolyline, feed, metricsOf } from './harness.mjs';

const MM = 0.001;
function mm(v) { return v * MM; }

function run(name, segs, samples) {
  const m = new QualityMonitor();
  m.setReferenceSegments(segs);
  feed(m, samples);
  const x = metricsOf(m);
  console.log(`\n== ${name}`);
  console.log({
    samples: x.samples,
    cornerCount: x.cornerCount,
    devAvg_mm: x.cornerDeviationAvg * 1000,
    devMax_mm: x.cornerDeviationMax * 1000,
    overAvg_mm: x.cornerOvershootAvg * 1000,
    overMax_mm: x.cornerOvershootMax * 1000,
    settleAvg_mm: x.settleDistanceAvg * 1000,
    settleMax_mm: x.settleDistanceMax * 1000,
    shapeRms_mm: x.shapeRms * 1000,
    shapeP95_mm: x.shapeP95 * 1000,
    contourMax_mm: x.contourMax * 1000,
    score: x.score,
  });
  return x;
}

// L-shaped path: (0,0)->(50,0)->(50,50) mm, 1 sample per 0.5 mm.
const segsL = polylineSegments([[[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]][0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);
// perfect
run('L, perfect, 0.5mm sampling', segsL, samplePolyline(segsL, 100, { place: 0 }));

// Bevel: replace the corner by two points. Attained path: line 1 up to 49.8mm,
// then a straight chord to (50, 0.2), then line 2. Distance from the vertex
// (50,0) to that chord is 0.2/sqrt(2) = 0.1414 mm.
{
  const samples = [];
  // line 1 sampled at 0.5mm, but near the end use 49.8
  const pts = [];
  for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
  pts.push([mm(49.8), 0, 0]);
  pts.push([mm(50), mm(0.2), 0]);
  for (let y = 0.7; y <= 49.5; y += 0.5) pts.push([mm(50), mm(y), 0]);
  pts.push([mm(50), mm(50), 0]);
  const totalE = segsL.reduce((a, s) => a + s.extrusion, 0);
  const eStep = totalE / pts.length;
  for (const p of pts) samples.push({ pos: p, E: eStep });
  const x = run('L, 0.2mm bevel at corner, 0.5mm sampling', segsL, samples);
  console.log('expected cornerDeviationMax = 0.1414 mm; got', x.cornerDeviationMax * 1000);
  console.log('expected cornerOvershootMax = 0.1414 mm (dist from bevel to path); got', x.cornerOvershootMax * 1000);
}

// Overshoot: attained path goes past the corner: line1 to 50.5 then back? Classic
// overshoot: after the corner the tip keeps moving in +x while y increases, then
// comes back to x=50. At the corner local reference, max distance from path.
{
  const segs = polylineSegments([[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);
  const pts = [];
  for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
  // corner overshoot: bulge out to x=50.5 at y=0.5, back to x=50 at y=1
  pts.push([mm(50), 0, 0]);
  pts.push([mm(50.25), mm(0.25), 0]);
  pts.push([mm(50.5), mm(0.5), 0]);
  pts.push([mm(50.25), mm(0.75), 0]);
  pts.push([mm(50), mm(1), 0]);
  for (let y = 1.5; y <= 50; y += 0.5) pts.push([mm(50), mm(y), 0]);
  const eStep = polylineSegments([[0,0,0],[mm(50),0,0],[mm(50),mm(50),0]]).reduce((a, s) => a + s.extrusion, 0) / pts.length;
  const x = run('L, 0.5mm corner overshoot bulge', segs, pts.map((p) => ({ pos: p, E: eStep })));
  // exact max distance of the bulge polyline to the reference path = 0.5mm (apex x=50.5)
  console.log('expected overshoot/deviation max ~0.5 mm; got dev', x.cornerDeviationMax * 1000, 'over', x.cornerOvershootMax * 1000);
}

// Staircase: 90-degree corners every 5 mm. Windows (+-6mm) overlap.
{
  const pts = [[0, 0, 0]];
  let x = 0, y = 0;
  for (let k = 0; k < 10; k += 1) {
    if (k % 2 === 0) x += 5; else y += 5;
    pts.push([mm(x), mm(y), 0]);
  }
  const stairSegs = polylineSegments(pts);
  const sy = run('staircase 5mm, perfect, 0.1mm sampling', stairSegs, samplePolyline(stairSegs, 50, { place: 0 }));
  console.log('staircase has 9 interior corners; reported', sy.cornerCount,
    'devMax_mm', sy.cornerDeviationMax * 1000, 'settleMax_mm', sy.settleDistanceMax * 1000);
}
