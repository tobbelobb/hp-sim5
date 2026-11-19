// Negative results: inputs that look risky but are handled identically by the
// shipped worker and by RRF/Klipper semantics.  Development evidence for
// report.md; also feeds measurements.json.
import { measure, pct } from './harness.mjs';

const BASE = 'M82\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n';

const cases = {
  'explicit G21 (mm)': 'M82\nG21\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n',
  'explicit G90 (absolute)': 'M82\nG90\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n',
  'CRLF line endings, no final newline': 'M82\r\nG1 X10 Y0 E1 F600\r\nG1 X20 Y0 E2 F600',
  'lowercase': 'm82\ng1 x10 y0 e1 f600\ng1 x20 y0 e2 f600\n',
  'tab separators': 'M82\nG1\tX10\tY0\tE1\tF600\nG1\tX20\tY0\tE2\tF600\n',
  'checksum without line number': 'M82\nG1 X10 Y0 E1*45 F600\nG1 X20 Y0 E2 F600*46\n',
  'semicolon comments': 'M82 ; absolute E\nG1 X10 Y0 E1 F600 ; first\nG1 X20 Y0 E2 F600 ; second\n',
  'parenthesised inline comment': 'M82\nG1 X10 Y0 E1 F600 (first)\nG1 X20 Y0 E2 F600(second)\n',
  'feedrate-only G1 line': 'M82\nG1 F600\nG1 X10 Y0 E1\nG1 X20 Y0 E2\n',
  'multiple commands on one line (invalid)': 'M82 G1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n',
  'blank lines and comments only': 'M82\n\n;\n   \nG1 X10 Y0 E1 F600\n\nG1 X20 Y0 E2 F600\n',
};

const rows = [];
for (const [name, text] of Object.entries(cases)) {
  const m = await measure(text);
  const travelErr = pct(m.commanded.travelMm, m.sim.pathLengthMm);
  const extErr = pct(m.commanded.extrusionMm, m.sim.extrusionMm);
  const timeErr = pct(m.commanded.timeSec, m.sim.timeSec);
  const ok = Math.abs(travelErr) < 1 && Math.abs(extErr) < 1 && Math.abs(timeErr) < 2;
  console.log(`${ok ? 'MATCHES' : 'DIFFERS'}  ${name}`);
  console.log(`  commanded travel ${m.commanded.travelMm.toFixed(3)} mm, sim ${m.sim.pathLengthMm.toFixed(3)} mm (${travelErr.toFixed(2)}%)`);
  console.log(`  commanded ext ${m.commanded.extrusionMm.toFixed(3)} mm, sim ${m.sim.extrusionMm.toFixed(3)} mm (${extErr.toFixed(2)}%)`);
  console.log(`  commanded time ${m.commanded.timeSec.toFixed(3)} s, sim ${m.sim.timeSec.toFixed(3)} s (${timeErr.toFixed(2)}%)`);
  rows.push({ name, ok, travelErrPct: travelErr, extErrPct: extErr, timeErrPct: timeErr });
}

const bad = rows.filter((r) => !r.ok);
console.log(`\n${rows.length - bad.length}/${rows.length} negative controls match`);
