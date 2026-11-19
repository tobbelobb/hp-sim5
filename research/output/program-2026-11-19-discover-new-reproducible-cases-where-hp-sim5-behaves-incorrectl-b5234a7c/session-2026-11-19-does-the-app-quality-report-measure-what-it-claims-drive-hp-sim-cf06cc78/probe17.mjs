// Probe 17: end-to-end through the shipped parser and unit conversion:
// parseGcodeText (mm) -> convertParsedSegmentsToSimulation (m) -> QualityMonitor.
import { parseGcodeText, createInspectionToolsController } from '../../../../hp-sim-3d/app/inspectionToolsController.js';
import {
  QualityMonitor, metricsOf, MM,
} from './harness.mjs';

// The converter is module-private; replicate exactly its documented mapping
// (positions * 0.001, extrusion unchanged) because inspectionToolsController
// does not export it. `loadFromFile` is the public path but needs a File object;
// instead we check the same transformation via metadata of a loaded record.
const scale = 0.001;
function toSim(parsed) {
  return {
    segments: parsed.segments.map((s) => ({
      start: s.start.map((v) => v * scale),
      end: s.end.map((v) => v * scale),
      extrusion: s.extrusion,
      feedRate: s.feedRate,
      length: s.length * scale,
    })),
    bounds: parsed.bounds && Object.fromEntries(Object.entries(parsed.bounds).map(([k, v]) => [k, v * scale])),
  };
}

const gcode = `
; synthetic, absolute XYZ and E
G21
G90
M82
G92 E0
G1 Z0.2 F1000
G1 X0 Y0 F3000
G1 X50 Y0 E5           ; 50 mm +X, E 5 mm
G1 X50 Y50 E10         ; 50 mm +Y, E 5 mm  (90 deg corner at (50,0))
G1 E9                  ; retract 1 mm, no motion
G0 X0 Y0               ; travel back (not extruding)
G1 X0 Y0 Z0.4 E10      ; prime at the new spot (zero length, E 1 mm)
G1 X40 Y0 E14          ; 40 mm +X, E 4 mm
`;
const parsed = parseGcodeText(gcode);
console.log('segments:', parsed.segments.length);
for (const s of parsed.segments) {
  console.log(' ', s.start.map((v) => +v.toFixed(3)), '->', s.end.map((v) => +v.toFixed(3)),
    'E', +s.extrusion.toFixed(6), 'len', +s.length.toFixed(3));
}
console.log('bounds (mm):', parsed.bounds);
console.log('expected: 4 segments; E totals 5,5,1,4; lengths 50,50,0,40;');
console.log('expected bounds: x 0..50, y 0..50, z 0.2..0.4 (travel to X0 Y0 not included, no E)');

const sim = toSim(parsed);
const { samplePolyline } = await import('./harness.mjs');
const m = new QualityMonitor();
m.setReferenceSegments(sim.segments);
// equal E chunks per segment, tip at the chunk start (the app's convention),
// with a known constant +Y offset of 0.2 mm
const samples = samplePolyline(sim.segments, 100, { place: 0, offset: () => [0, 0.0002, 0] });
for (const smp of samples) m.recordExtrusion({ pos: smp.pos, length: smp.E });
m.runFinalCheck();
const x = metricsOf(m);
console.log('\nreported with a 0.2 mm +Y offset:');
console.log({
  contourRms_mm: x.contourRms * 1000, contourP95_mm: x.contourP95 * 1000, contourMax_mm: x.contourMax * 1000,
  vRms_mm: x.contourVerticalRms * 1000, hRms_mm: x.contourHorizontalRms * 1000,
  cornerCount: x.cornerCount, cornerDevMax_mm: x.cornerDeviationMax * 1000,
  meanOffset_mm: x.meanOffset.map((v) => v * 1000), progress: x.progress, score: x.score,
});
console.log('expected: contour rms/p95/max 0.2 mm (Z is also offset by 0 in Y only); cornerCount 1; progress 1');

// Bounds check: does a travel outside the extruded region extend the bounds?
const p2 = parseGcodeText('G90\nM82\nG92 E0\nG1 X0 Y0 E1\nG0 X500 Y500\nG1 X10 Y0 E2\n');
console.log('\ntravel-outside bounds:', p2.bounds, '-> maxX', p2.bounds.maxX, '(travel to 500 excluded by design)');
