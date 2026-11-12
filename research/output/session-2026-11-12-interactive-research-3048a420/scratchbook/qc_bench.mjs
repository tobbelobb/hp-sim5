// Replays recorded extrusion samples of a full logo into QualityMonitor and
// times live processing and the queued (Finish ASAP) final drain.
import fs from 'node:fs';
import { parseGcodeText } from './gcode_parse_copy.mjs';
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
const root = new URL('../../../../', import.meta.url);
const parsed = parseGcodeText(fs.readFileSync(new URL('public/gcode/Hangprinter_logo6.gcode', root), 'utf8'));
const segments = parsed.segments.map(s => ({ start: s.start.map(v => v * .001), end: s.end.map(v => v * .001), extrusion: s.extrusion }));
const samples = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const mode = process.argv[3] || 'both';
function run(queued) {
  const m = new QualityMonitor();
  m.setReferenceSegments(segments);
  if (queued) m.setEnabled(false);
  const t0 = performance.now();
  for (const [x, y, z, length] of samples) m.recordExtrusion({ pos: [x, y, z], length });
  const t1 = performance.now();
  if (queued) m.setEnabled(true);
  m.runFinalCheck();
  const t2 = performance.now();
  return { record_ms: +(t1 - t0).toFixed(1), final_ms: +(t2 - t1).toFixed(1), metrics: m.getMetrics() };
}
console.log('segments', segments.length, 'samples', samples.length);
for (const q of mode === 'both' ? [false, true] : [mode === 'queued']) {
  const r = run(q);
  console.log(q ? 'disabled(ASAP or toggle off)' : 'live', 'record', r.record_ms, 'ms final', r.final_ms, 'ms');
  if (process.argv[4]) console.log(JSON.stringify(r.metrics));
}
