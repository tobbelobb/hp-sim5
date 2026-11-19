// Probe 21: truncated print (only the first leg extruded). Does the score see
// the corners that were never reached?
import { QualityMonitor } from './harness.mjs';
function segment(a, b) { return { start: a, end: b, extrusion: Math.hypot(b[0]-a[0], b[1]-a[1], b[2]-a[2]) }; }
const MM = 0.001;
const ref = [
  segment([0, 0, 0], [50 * MM, 0, 0]),
  segment([50 * MM, 0, 0], [50 * MM, 50 * MM, 0]),
  segment([50 * MM, 50 * MM, 0], [0, 50 * MM, 0]),
];
function hudField(html, label) {
  const m = html.match(new RegExp(`${label.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}</span><span>([^<]*)</span>`));
  return m ? m[1] : '(not rendered)';
}
function stub() { const c = new Set(['sim-hidden']); return { innerHTML: '', textContent: '', style: { setProperty() {} }, classList: { add: (...n) => n.forEach((x) => c.add(x)), remove: (...n) => n.forEach((x) => c.delete(x)), contains: (x) => c.has(x), toggle: (x, on) => (on ? c.add(x) : c.delete(x)) }, addEventListener() {}, removeEventListener() {} }; }
const hud = stub();
const m = new QualityMonitor({ hudElement: hud });
m.setReferenceSegments(ref);
// print only the first leg, 0.1 mm steps, perfect
const seg = ref[0];
const n = Math.round(seg.extrusion / 0.0001);
for (let i = 0; i < n; i += 1) {
  m.recordExtrusion({ pos: [seg.end[0] * (i / n), 0, 0], length: seg.extrusion / n });
}
m.runFinalCheck();
m.refreshHud(true);
const x = m.getMetrics();
console.log({
  progress: x.progress,
  score: x.score,
  cornerCount: x.cornerCount,
  shapeP95_mm: x.shapeP95 * 1000,
  contourMax_mm: x.contourMax * 1000,
});
console.log('HUD Quality:', (hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, '--'])[1]);
console.log('HUD Progress:', hudField(hud.innerHTML, 'Progress'));
console.log('HUD corner line:', hudField(hud.innerHTML, 'Corner deviation avg / max (0)'));
console.log('HUD deductions:', hudField(hud.innerHTML, 'Score deductions'));
