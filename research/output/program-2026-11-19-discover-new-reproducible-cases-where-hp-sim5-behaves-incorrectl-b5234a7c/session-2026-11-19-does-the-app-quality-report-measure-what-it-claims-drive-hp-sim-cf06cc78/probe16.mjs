// Probe 16: how the offset-along-first-move artefact shows up on a square
// (max vs average), and the score for the perpendicular control.
import {
  QualityMonitor, polylineSegments, feed, metricsOf, MM,
} from './harness.mjs';

const mm = (v) => v * MM;

// Same sampling convention as tests/js/hp-sim-3d/qualityMonitor.test.js:
// tip at the start of each 0.1 mm step together with that step's extrusion.
function print(segments, offset = () => [0, 0, 0], step = 0.0001) {
  const m = new QualityMonitor();
  m.setReferenceSegments(segments);
  for (const seg of segments) {
    const d = seg.end.map((v, i) => v - seg.start[i]);
    const length = Math.hypot(...d);
    const count = Math.max(1, Math.round(length / step));
    for (let i = 0; i < count; i += 1) {
      const p = seg.start.map((v, k) => v + d[k] * (i / count));
      const o = offset(p, d);
      m.recordExtrusion({ pos: p.map((v, k) => v + o[k]), length: seg.extrusion / count });
    }
  }
  m.runFinalCheck();
  return m.getMetrics();
}

const S = 0.05;
const square = [
  { start: [0, 0, 0], end: [S, 0, 0], extrusion: S },
  { start: [S, 0, 0], end: [S, S, 0], extrusion: S },
  { start: [S, S, 0], end: [0, S, 0], extrusion: S },
];
function report(name, x) {
  console.log(`\n== ${name}`);
  console.log({
    cornerCount: x.cornerCount,
    devAvg: (x.cornerDeviationAvg * 1000).toFixed(6),
    devMax: (x.cornerDeviationMax * 1000).toFixed(6),
    overAvg: (x.cornerOvershootAvg * 1000).toFixed(6),
    overMax: (x.cornerOvershootMax * 1000).toFixed(6),
    settleAvg: (x.settleDistanceAvg * 1000).toFixed(6),
    settleMax: (x.settleDistanceMax * 1000).toFixed(6),
    shapeRms: (x.shapeRms * 1000).toFixed(6),
    shapeP95: (x.shapeP95 * 1000).toFixed(6),
    meanOffset: x.meanOffset.map((v) => (v * 1000).toFixed(6)),
    score: x.score.toFixed(3),
  });
}
for (const dmm of [0.3, 1.0]) {
  report(`square perfect, shift +x ${dmm}mm (along first move)`,
    print(square, () => [dmm * MM, 0, 0]));
  report(`square perfect, shift +y ${dmm}mm (perpendicular to first move)`,
    print(square, () => [0, dmm * MM, 0]));
  report(`square perfect, shift +z ${dmm}mm`,
    print(square, () => [0, 0, dmm * MM]));
}
report('square perfect, no shift', print(square));
