// Reviewer scratch: is B3 ("Quality 100 on a third of the print") reachable
// through the shipped Upload path, end to end?
//
// Chain used (all shipped modules, read-only):
//   parseGcodeText(file)  -> reference segments  (inspectionToolsController.js)
//   MoveCommander(file)   -> per-step motor commands (workers/moveCommander.js)
//   QualityMonitor        -> score/progress       (quality-monitor.js)
//
// File: leg 1 is a G1 print, legs 2 and 3 are G0 lines carrying E.  RRF treats
// G0 exactly like G1 (GCodes2.cpp:228-229), but MoveCommander only recognises
// the literal token "G1".  So the reference has three legs and the machine
// prints one - a mismatch produced by the shipped app, not by a truncated file.
import { parseGcodeText } from '../../../../hp-sim-3d/app/inspectionToolsController.js';
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
import { runSim, simTrajectory } from '../session-2026-11-19-which-g-code-program-level-motion-semantics-does-the-browser-upl-1596692d/harness.mjs';

const text = [
  'M82',
  'G1 X50 Y0 E5 F600',
  'G0 X50 Y50 E10 F9000',
  'G0 X0 Y50 E15 F9000',
  '',
].join('\n');

const mm = 0.001;

// --- reference, exactly like inspectionToolsController.convertParsedSegmentsToSimulation
const parsed = parseGcodeText(text);
const reference = parsed.segments.map((s) => ({
  start: s.start.map((v) => v * mm),
  end: s.end.map((v) => v * mm),
  extrusion: s.extrusion,     // mm, unchanged
  length: s.length * mm,
}));

// --- simulation
const sim = await runSim(text);
const traj = await simTrajectory(sim);
const stepPosMm = [[0, 0, 0], ...traj.points];   // tip position at the start of each step

// --- quality monitor (stub HUD, same shape as the shipped test)
class Hud {
  constructor() { this.innerHTML = ''; this.classList = { add() {}, remove() {}, toggle() {}, contains: () => false }; this.style = { setProperty() {} }; }
  addEventListener() {} removeEventListener() {}
}
const hud = new Hud();
const m = new QualityMonitor({ hudElement: hud });
m.setReferenceSegments(reference);
sim.moves.forEach((move, i) => {
  m.recordExtrusion({ pos: stepPosMm[i].map((v) => v * mm), length: move.E });
});
m.runFinalCheck();
const x = m.getMetrics();

console.log('reference segments:', reference.length,
  ' total E(mm):', reference.reduce((s, r) => s + r.extrusion, 0));
console.log('reference travel(mm):', reference.reduce((s, r) => s + r.length, 0).toFixed(3));
console.log('emitted moves:', sim.moves.length, ' deposited E(mm):', sim.extrusionTotal);
console.log('sim travel(mm):', traj.pathLength.toFixed(3), ' end:', traj.end.map((v) => v.toFixed(2)).join(','));
console.log('progress %:', (x.progress * 100).toFixed(1),
  ' corners assessed:', x.cornerCount, ' score:', x.score);
