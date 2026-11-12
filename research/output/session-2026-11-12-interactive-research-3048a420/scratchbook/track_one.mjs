import fs from 'node:fs';
import { parseGcodeText } from './gcode_parse_copy.mjs';
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
const parsed = parseGcodeText(fs.readFileSync(new URL('../../../../public/gcode/Hangprinter_logo6.gcode', import.meta.url), 'utf8'));
const segments = parsed.segments.map(s => ({ start: s.start.map(v => v * .001), end: s.end.map(v => v * .001), extrusion: s.extrusion }));
const samples = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const I = Number(process.argv[3]);
const m = new QualityMonitor(); m.setReferenceSegments(segments);
const f = v => v.map(x => +(x * 1000).toFixed(2));
for (let i = 0; i < I + 4; i++) {
  const [x, y, z, length, step] = samples[i];
  if (i >= I - 4) {
    const c = m.extrusionCursor; const it = m._locateIntended(length); m.extrusionCursor = c;
    console.log(i, step, 'pos', f([x, y, z]), 'len', length.toFixed(5), 'cursor', c.toFixed(4), 'entry', it.index, 'intended', f([it.x, it.y, it.z]), 'E0', it.entry.extrusionStart.toFixed(4), 'Ee', it.entry.extrusion.toFixed(4), 'segLen', (it.entry.length * 1000).toFixed(2));
  }
  m.recordExtrusion({ pos: [x, y, z], length });
}
for (let k = 13866; k <= 13871; k++) { const e = m.entries[k]; console.log(k, f(e.start), f(e.end), e.extrusionStart.toFixed(4), e.extrusion.toFixed(4)); }
