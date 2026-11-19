import { QualityMonitor, polylineSegments, samplePolyline, feed, metricsOf, MM } from './harness.mjs';

const L = 0.2;
const segs = polylineSegments([[0, 0, 0], [L, 0, 0]]);
for (const n of [2, 5, 10, 20, 50, 200, 2000]) {
  for (const dm of [0, 0.05, 0.1, 0.3, 1.0]) {
    const m = new QualityMonitor();
    m.setReferenceSegments(segs);
    feed(m, samplePolyline(segs, n, { place: 0, offset: () => [0, dm * MM, 0] }));
    const x = metricsOf(m);
    console.log(`n=${String(n).padStart(4)} d=${dm}mm  shapeRms=${(x.shapeRms * 1000).toFixed(4)}mm shapeP95=${(x.shapeP95 * 1000).toFixed(4)}mm off=${(Math.hypot(...x.meanOffset) * 1000).toFixed(4)}mm score=${x.score.toFixed(2)}  ded=[${x.scoreDeductions.map((d) => d.label + ':' + d.points.toFixed(2)).join(', ')}]`);
  }
}
