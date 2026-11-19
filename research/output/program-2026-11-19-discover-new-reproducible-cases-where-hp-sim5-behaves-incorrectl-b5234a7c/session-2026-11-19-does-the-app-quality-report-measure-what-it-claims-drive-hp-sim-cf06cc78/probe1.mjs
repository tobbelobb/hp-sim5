// Probe 1: does the report reproduce trivially-known closed forms?
import {
  QualityMonitor, polylineSegments, samplePolyline, feed, metricsOf, MM,
} from './harness.mjs';

function run(name, segments, samples) {
  const m = new QualityMonitor();
  m.setReferenceSegments(segments);
  feed(m, samples);
  const x = metricsOf(m);
  console.log(`\n== ${name}`);
  console.log(JSON.stringify({
    samples: x.samples,
    contourRms: x.contourRms, contourP95: x.contourP95, contourMax: x.contourMax,
    shapeRms: x.shapeRms, shapeP95: x.shapeP95,
    hRms: x.contourHorizontalRms, vRms: x.contourVerticalRms,
    trackingRms: x.trackingRms, trackingMax: x.trackingMax, meanLag: x.meanLag,
    meanOffset: x.meanOffset,
    withinTol: x.withinTolerance, progress: x.progress,
    cornerCount: x.cornerCount, cDevMax: x.cornerDeviationMax,
    score: x.score,
  }, null, 1));
  return x;
}

// --- 1. straight 0.2 m line along +X, constant lateral offset d
const L = 0.2;
const lineSegs = polylineSegments([[0, 0, 0], [L, 0, 0]]);
const d = 300 * MM; // 0.3 mm
run('straight line, 0.3 mm +Y offset, 200 samples, place=1',
  lineSegs,
  samplePolyline(lineSegs, 200, { place: 1, offset: () => [0, d, 0] }));

run('straight line, 0 offset (perfect), 200 samples, place=1',
  lineSegs,
  samplePolyline(lineSegs, 200, { place: 1 }));

run('straight line, 0 offset (perfect), 200 samples, place=0',
  lineSegs,
  samplePolyline(lineSegs, 200, { place: 0 }));

// --- 2. perfect square 4 x 50 mm, 100 samples/side
const S = 0.05;
const sq = polylineSegments([[0, 0, 0], [S, 0, 0], [S, S, 0], [0, S, 0], [0, 0, 0]]);
run('perfect square 50mm, 100 samples/side, place=0', sq, samplePolyline(sq, 100, { place: 0 }));
run('perfect square 50mm, 100 samples/side, place=1', sq, samplePolyline(sq, 100, { place: 1 }));

// --- 3. one outlier at D = 2 mm, 199 on-path samples
const D = 2 * MM;
{
  const samples = samplePolyline(lineSegs, 200, { place: 0 });
  samples[123].pos[1] = D;
  const x = run('single 2 mm outlier / 199 perfect', lineSegs, samples);
  console.log('expected contourMax exactly', D, 'got', x.contourMax);
}

// --- 4. all samples have zero distance (already covered), plus exact-zero check
{
  const x = run('all zero distance, 50 samples', lineSegs, samplePolyline(lineSegs, 50, { place: 0 }));
  console.log('zero-distance: rms', x.contourRms, 'max', x.contourMax, 'p95', x.contourP95,
    'expect 0,0,0; score expect 100 got', x.score);
}
