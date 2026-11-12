import fs from 'node:fs';
import { parseGcodeText } from './gcode_parse_copy.mjs';
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
const parsed = parseGcodeText(fs.readFileSync(new URL('../../../../public/gcode/Hangprinter_logo6.gcode', import.meta.url), 'utf8'));
const segments = parsed.segments.map(s => ({ start: s.start.map(v => v * .001), end: s.end.map(v => v * .001), extrusion: s.extrusion }));
const samples = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const m = new QualityMonitor(); m.setReferenceSegments(segments);
let big = [];
samples.forEach(([x, y, z, length, step], i) => {
  const before = m.trackingHistogram.max;
  const cursor = m.extrusionCursor;
  m.recordExtrusion({ pos: [x, y, z], length });
  if (m.trackingHistogram.max > before && m.trackingHistogram.max > 0.01) big.push({ i, step, length, cursor, total: m.totalExtrusion, entry: m.entryIndex, nEntries: m.entries.length, pos: [x, y, z].map(v => +(v * 1000).toFixed(1)), max: m.trackingHistogram.max });
});
console.log(big.slice(-5));
const tail = samples.slice(-5).map(s => s.map(v => +v.toFixed(4)));
console.log('last samples', tail, 'total sample E', samples.reduce((a, s) => a + s[3], 0), 'ref E', m.totalExtrusion);
