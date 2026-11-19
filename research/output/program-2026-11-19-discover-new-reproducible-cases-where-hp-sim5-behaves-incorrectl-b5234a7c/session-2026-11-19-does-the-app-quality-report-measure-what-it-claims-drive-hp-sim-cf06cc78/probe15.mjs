// Probe 15: rigid offset vs corner metrics. Contract: "Corners are judged on
// shape too: rounding and overshoot, not the offset." (quality-monitor.js:681)
// and "per sharp corner (after removing the mean offset)" (:128-131).
import {
  QualityMonitor, polylineSegments, feed, metricsOf, samplesFromPoints, MM,
} from './harness.mjs';

const mm = (v) => v * MM;

function run(name, segs, pts) {
  const m = new QualityMonitor();
  m.setReferenceSegments(segs);
  feed(m, samplesFromPoints(pts, segs));
  const x = metricsOf(m);
  console.log(`\n== ${name}`);
  console.log({
    samples: x.samples,
    cornerCount: x.cornerCount,
    cornerDeviation_max_mm: x.cornerDeviationMax * 1000,
    cornerOvershoot_max_mm: x.cornerOvershootMax * 1000,
    settle_max_mm: x.settleDistanceMax * 1000,
    shapeRms_mm: x.shapeRms * 1000,
    shapeP95_mm: x.shapeP95 * 1000,
    meanOffset_mm: x.meanOffset.map((v) => v * 1000),
    contourMax_mm: x.contourMax * 1000,
    score: x.score,
  });
  return x;
}

function LPoints(offset, { swap = false } = {}) {
  // first leg along +x, second along +y; 0.5 mm sampling
  const pts = [];
  const n = 100;
  for (let i = 0; i < n; i += 1) pts.push([mm(50) * i / n, 0, 0]);
  for (let i = 0; i < n; i += 1) pts.push([mm(50), mm(50) * i / n, 0]);
  pts.push([mm(50), mm(50), 0]);
  return pts.map((p) => [p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]]);
}

const L = polylineSegments([[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);

run('L perfect', L, LPoints([0, 0, 0]));
run('L rigid +x offset 0.3 mm (along first leg)', L, LPoints([mm(0.3), 0, 0]));
run('L rigid +y offset 0.3 mm (perpendicular to first leg)', L, LPoints([0, mm(0.3), 0]));
run('L rigid +x offset 1.0 mm', L, LPoints([mm(1), 0, 0]));

// Reversed L: first leg along +y, second along +x, shift along +x.
const L2 = polylineSegments([[0, 0, 0], [0, mm(50), 0], [mm(50), mm(50), 0]]);
{
  const pts = [];
  for (let i = 0; i < 100; i += 1) pts.push([0, mm(50) * i / 100, 0]);
  for (let i = 0; i < 100; i += 1) pts.push([mm(50) * i / 100, mm(50), 0]);
  pts.push([mm(50), mm(50), 0]);
  run('reversed L rigid +x offset 0.3 mm (perpendicular to first leg)',
    L2, pts.map((p) => [p[0] + mm(0.3), p[1], p[2]]));
  // same geometry but shift along the first leg (+y)
  run('reversed L rigid +y offset 0.3 mm (along first leg)',
    L2, pts.map((p) => [p[0], p[1] + mm(0.3), p[2]]));
}
