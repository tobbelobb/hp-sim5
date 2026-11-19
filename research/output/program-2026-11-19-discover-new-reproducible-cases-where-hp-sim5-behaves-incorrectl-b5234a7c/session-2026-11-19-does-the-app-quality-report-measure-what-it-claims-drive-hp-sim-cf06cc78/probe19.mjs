import { QualityMonitor } from './harness.mjs';
function segment(start, end) { return { start, end, extrusion: Math.hypot(end[0]-start[0], end[1]-start[1], end[2]-start[2]) }; }
function print(monitor, segments, offset = () => [0,0,0], step = 0.0001) {
  for (const seg of segments) {
    const d = seg.end.map((value, i) => value - seg.start[i]);
    const length = Math.hypot(...d);
    if (length === 0) { const o = offset(seg.start, [0,0,0]); monitor.recordExtrusion({ pos: seg.start.map((v,i)=>v+o[i]), length: seg.extrusion }); continue; }
    const tangent = d.map((v) => v / length);
    const count = Math.round(length / step);
    for (let i = 0; i < count; i += 1) {
      const point = seg.start.map((value, k) => value + d[k] * (i / count));
      const o = offset(point, tangent);
      monitor.recordExtrusion({ pos: point.map((value, k) => value + o[k]), length: seg.extrusion / count });
    }
  }
  monitor.runFinalCheck();
  return monitor.metrics;
}
const reference = [segment([0,0,0],[0.05,0,0]), segment([0.05,0,0],[0.05,0.05,0]), segment([0.05,0.05,0],[0,0.05,0])];
const m = new QualityMonitor(); m.setReferenceSegments(reference);
const x = print(m, reference, () => [0.001, -0.0005, 0.0003]);
console.log({shapeP95:x.shapeP95, shapeRms:x.shapeRms, cornerCount:x.cornerCount,
  cornerDevMax:x.cornerDeviationMax, cornerOverAvg:x.cornerOvershootAvg, cornerOverMax:x.cornerOvershootMax,
  settleAvg:x.settleDistanceAvg, settleMax:x.settleDistanceMax, meanOffset:x.meanOffset, score:x.score});
console.log('assertions: shapeP95<1e-4', x.shapeP95<1e-4, '| cornerDevMax<1e-4', x.cornerDeviationMax<1e-4, '| score>90', x.score>90);
