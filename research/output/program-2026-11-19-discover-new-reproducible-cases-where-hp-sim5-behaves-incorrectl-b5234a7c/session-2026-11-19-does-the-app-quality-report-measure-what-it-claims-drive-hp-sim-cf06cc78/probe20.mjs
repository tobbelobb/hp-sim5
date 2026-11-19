// Probe 20: corner-detection threshold, matched-segment window (lag), and the
// sample-vs-length meaning of "Path within X mm".
import {
  QualityMonitor, polylineSegments, feed, metricsOf, samplesFromPoints, MM,
} from './harness.mjs';
const mm = (v) => v * MM;

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
const seg = (a, b) => ({ start: a, end: b, extrusion: Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) });

// --- (1) corner threshold: two 40 mm legs with a turn of `deg`
for (const deg of [30, 44, 44.9, 45, 45.1, 46, 60, 90]) {
  const th = deg * Math.PI / 180;
  const a = [0, 0, 0];
  const b = [mm(40), 0, 0];
  const c = [b[0] + mm(40) * Math.cos(th), b[1] + mm(40) * Math.sin(th), 0];
  const x = print([seg(a, b), seg(b, c)]);
  console.log(`turn ${deg} deg -> cornerCount ${x.cornerCount}`);
}

// --- (2) lag beyond the +-5 mm search window on a perfect straight line.
{
  const ref = [seg([0, 0, 0], [mm(200), 0, 0])];
  for (const lagmm of [0, 2, 4.9, 5.5, 10, 20]) {
    const x = print(ref, () => [-lagmm * MM, 0, 0]);
    console.log(`pure lag ${lagmm} mm: contourMax=${(x.contourMax * 1000).toFixed(4)}mm ` +
      `shapeP95=${(x.shapeP95 * 1000).toFixed(4)}mm trackingMax=${(x.trackingMax * 1000).toFixed(4)}mm ` +
      `meanLag=${(x.meanLag * 1000).toFixed(4)}mm within0.5=${(x.withinTolerance * 100).toFixed(1)}% score=${x.score.toFixed(2)}`);
  }
}

// --- (3) "Path within 0.5 mm" is a share of samples, not of path length:
// first 10 mm printed slowly (dense samples), last 90 mm fast (10x fewer samples)
// and 1 mm out of tolerance.
{
  const ref = [seg([0, 0, 0], [mm(100), 0, 0])];
  const m = new QualityMonitor();
  m.setReferenceSegments(ref);
  const samples = [];
  // slow part: 0.01 mm steps over the first 10 mm, on path
  for (let i = 0; i < 1000; i += 1) samples.push({ pos: [i * 0.01 * MM, 0, 0], E: 0.01 });
  // fast part: 0.1 mm steps over the last 90 mm, 1 mm off the path
  for (let i = 0; i < 900; i += 1) samples.push({ pos: [(10 + i * 0.1) * MM, 1 * MM, 0], E: 0.1 });
  for (const s of samples) m.recordExtrusion({ pos: s.pos, length: s.E });
  m.runFinalCheck();
  const x = m.getMetrics();
  console.log('sample-vs-length: withinTolerance =', (x.withinTolerance * 100).toFixed(1), '% (samples);',
    'length share within 0.5 mm = 10.0 %; path deviation max', (x.contourMax * 1000).toFixed(3), 'mm');
  console.log('samples', x.samples, 'contourRms_mm', (x.contourRms * 1000).toFixed(4), 'score', x.score.toFixed(2));
}
