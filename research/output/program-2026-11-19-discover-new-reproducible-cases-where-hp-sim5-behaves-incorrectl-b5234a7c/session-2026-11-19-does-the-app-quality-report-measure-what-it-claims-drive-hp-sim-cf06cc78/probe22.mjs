// Probe 22: the same artefact on the shipped "Print Squares" reference
// (public/gcode/draw_squares_bigger.gcode), parsed and converted exactly as
// inspectionToolsController.loadForPreset does.
import { readFileSync } from 'node:fs';
import { parseGcodeText } from '../../../../hp-sim-3d/app/inspectionToolsController.js';
import { QualityMonitor, samplePolyline } from './harness.mjs';

const text = readFileSync(new URL('../../../../public/gcode/draw_squares_bigger.gcode', import.meta.url), 'utf8');
const parsed = parseGcodeText(text);
const sim = parsed.segments.map((s) => ({
  start: s.start.map((v) => v * 0.001), end: s.end.map((v) => v * 0.001), extrusion: s.extrusion, length: s.length * 0.001,
}));
const first = sim[0];
const dir = first.end.map((v, i) => (v - first.start[i]) / first.length);
console.log('segments', sim.length, 'total length mm', (sim.reduce((a, s) => a + s.length, 0) * 1000).toFixed(1));
console.log('first extruding move direction', dir.map((v) => +v.toFixed(4)));

function run(offset) {
  const m = new QualityMonitor();
  m.setReferenceSegments(sim);
  const samplesPerSegment = (s) => Math.max(1, Math.round(s.length / 0.00025));
  const samples = samplePolyline(sim, samplesPerSegment, { place: 0, offset: () => offset });
  for (const s of samples) m.recordExtrusion({ pos: s.pos, length: s.E });
  m.runFinalCheck();
  const x = m.getMetrics();
  return {
    cornerCount: x.cornerCount,
    devMaxMm: x.cornerDeviationMax * 1000,
    overMaxMm: x.cornerOvershootMax * 1000,
    overAvgMm: x.cornerOvershootAvg * 1000,
    settleMaxMm: x.settleDistanceMax * 1000,
    meanOffsetMm: x.meanOffset.map((v) => v * 1000),
    score: x.score,
  };
}
const D = 0.001;
console.log('\nperfect print, 1.000 mm offset along the first move:',
  JSON.stringify(run([D * dir[0], D * dir[1], D * dir[2]]), null, 1));
console.log('\nperfect print, 1.000 mm offset perpendicular to the first move (in-plane):',
  JSON.stringify(run([-D * dir[1], D * dir[0], 0]), null, 1));
console.log('\nperfect print, no offset:', JSON.stringify(run([0, 0, 0]), null, 1));
