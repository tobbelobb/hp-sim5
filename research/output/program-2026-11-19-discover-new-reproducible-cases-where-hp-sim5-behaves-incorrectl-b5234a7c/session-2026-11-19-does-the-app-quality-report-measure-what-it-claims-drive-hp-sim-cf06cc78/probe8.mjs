// Probe 8: corner metrics with E aligned to the attained path (simulator
// convention: sample position = start of its E chunk).
import {
  QualityMonitor, polylineSegments, feed, metricsOf, samplesFromPoints, vertexMinDistance, MM,
} from './harness.mjs';

const mm = (v) => v * MM;

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
    meanOffset_mm: x.meanOffset.map((v) => v * 1000),
    score: x.score,
  });
  return x;
}

const segsL = polylineSegments([[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);

// bevel 0.2 mm
{
  const pts = [];
  for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
  pts.push([mm(49.8), 0, 0]);
  pts.push([mm(50), mm(0.2), 0]);
  for (let y = 0.7; y <= 49.5; y += 0.5) pts.push([mm(50), mm(y), 0]);
  pts.push([mm(50), mm(50), 0]);
  const x = run('L, 0.2mm bevel, 0.5mm sampling', segsL, samplesFromPoints(pts, segsL));
  console.log('exact corner->attained polyline min (mm):', vertexMinDistance(pts, [mm(50), 0, 0]) * 1000);
}

// overshoot bulge 0.5 mm
{
  const pts = [];
  for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
  pts.push([mm(50), 0, 0]);
  pts.push([mm(50.25), mm(0.25), 0]);
  pts.push([mm(50.5), mm(0.5), 0]);
  pts.push([mm(50.25), mm(0.75), 0]);
  pts.push([mm(50), mm(1), 0]);
  for (let y = 1.5; y <= 50; y += 0.5) pts.push([mm(50), mm(y), 0]);
  const x = run('L, 0.5mm overshoot bulge', segsL, samplesFromPoints(pts, segsL));
  console.log('exact corner->attained min (mm):', vertexMinDistance(pts, [mm(50), 0, 0]) * 1000);
}

// perfect square, closed, 4 sharp corners
{
  const S = 0.05;
  const sq = polylineSegments([[0, 0, 0], [S, 0, 0], [S, S, 0], [0, S, 0], [0, 0, 0]]);
  run('perfect closed square 50mm, 0.5mm sampling', sq, samplesFromPoints(
    (() => { const p = []; for (const seg of sq) { const n = Math.round(seg.length / 0.0005); for (let i = 0; i < n; i++) p.push([seg.start[0] + i / n * (seg.end[0] - seg.start[0]), seg.start[1] + i / n * (seg.end[1] - seg.start[1]), 0]); } p.push([0, 0, 0]); return p; })(), sq));
}

// circle of radius 20mm, 720 samples: no corners; contour deviation 0
{
  const R = 0.02; const N = 720; const p = [];
  for (let i = 0; i < N; i += 1) { const a = 2 * Math.PI * i / N; p.push([R * Math.cos(a), R * Math.sin(a), 0]); }
  p.push(p[0]);
  const segs = polylineSegments(p);
  const x = run('circle R20mm, 720 samples, perfect', segs, samplesFromPoints(p, segs));
  console.log('circle: exact 0 everywhere; cornerCount', x.cornerCount);
}
