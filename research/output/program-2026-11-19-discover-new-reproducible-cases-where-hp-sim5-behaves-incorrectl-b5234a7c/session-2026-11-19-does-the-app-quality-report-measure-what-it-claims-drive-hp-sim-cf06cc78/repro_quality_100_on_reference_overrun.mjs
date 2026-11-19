// repro_quality_100_on_reference_overrun.mjs
//
// The score's corner and ringing terms are hard-wired to 0 when no corner was
// assessed (`cornerError: m.cornerCount > 0 ? ... : 0`, quality-monitor.js:859-860),
// and a corner is only assessed if extrusion samples exist both before and after
// it (`_finalizeCorner`, :785-788). Corners the print never reached are therefore
// scored as perfect, and the headline score carries no progress term.
//
// Reachability: the card needs the reference path to describe more extrusion than
// the machine executes. It is not produced by an ordinary abort of a print whose
// reference and executed paths agree. One concrete route is a reference the parser
// treats as motion while the command layer skips it -- here `G0 ... E...`:
// parseGcodeText accepts G0 as a moving, extruding line, workers/moveCommander.js
// ignores G0 entirely. (That G0/G91 semantics gap is a separate, already-reported
// issue; it is used here only to supply the reference/executed mismatch. There is
// no claim that this is common.)
//
// Result: reference 150 mm / 15 mm E, executed 50 mm / 5 mm E -> the HUD shows
// Quality 100, "Score deductions: none", and Progress 33.3 %. The Progress field
// does disclose the overrun, so this is classified "misleading", not "wrong".
//
// Exit 1 = the headline quality number is misleading for the input.
// Run: timeout 30 node repro_quality_100_on_reference_overrun.mjs
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
import { parseGcodeText } from '../../../../hp-sim-3d/app/inspectionToolsController.js';

const MM = 0.001;
const gcode = [
  'G21', 'G90', 'M82', 'G92 E0',
  'G1 X50 Y0 E5',    // executed by the command layer
  'G0 X50 Y50 E10',  // reference only: moveCommander.js ignores G0
  'G0 X0 Y50 E15',   // reference only
  '',
].join('\n');

const parsed = parseGcodeText(gcode);
const ref = parsed.segments.map((s) => ({
  start: s.start.map((v) => v * 0.001), end: s.end.map((v) => v * 0.001), extrusion: s.extrusion,
}));

function stubHud() {
  const classes = new Set(['sim-hidden']);
  return {
    innerHTML: '', textContent: '', style: { setProperty() {} },
    classList: { add: (...c) => c.forEach((x) => classes.add(x)), remove: (...c) => c.forEach((x) => classes.delete(x)),
      contains: (c) => classes.has(c), toggle: (c, on) => (on ? classes.add(c) : classes.delete(c)) },
    addEventListener() {}, removeEventListener() {},
  };
}

const hud = stubHud();
const m = new QualityMonitor({ hudElement: hud });
m.setReferenceSegments(ref);

// What the command layer actually executes: the G1 line only, 0.1 mm per sample.
const executed = ref[0];
const n = Math.round(executed.extrusion / 0.0001) * 10; // 0.01 mm steps for this short leg
for (let i = 0; i < n; i += 1) {
  const f = i / n;
  m.recordExtrusion({
    pos: [executed.start[0] + f * (executed.end[0] - executed.start[0]),
      executed.start[1] + f * (executed.end[1] - executed.start[1]), 0],
    length: executed.extrusion / n,
  });
}
m.runFinalCheck();
m.refreshHud(true);

const x = m.getMetrics();
const field = (label) => {
  const r = hud.innerHTML.match(new RegExp(`${label.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}</span><span>([^<]*)</span>`));
  return r ? r[1] : '(not rendered)';
};
const refLengthMm = parsed.segments.reduce((a, s) => a + s.length, 0);
const refExtrusionMm = parsed.segments.reduce((a, s) => a + s.extrusion, 0);
const execLengthMm = parsed.segments[0].length;
const execExtrusionMm = parsed.segments[0].extrusion;
console.log(`reference parsed from G-code: ${parsed.segments.length} segments, ` +
  `${refLengthMm} mm path, ${refExtrusionMm} mm E`);
console.log(`executed by the command layer: ${execLengthMm} mm path, ${execExtrusionMm} mm E`);
console.log(`HUD "Quality"                 : ${(hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, '--'])[1]}`);
console.log(`HUD "Score deductions"        : ${field('Score deductions')}`);
console.log(`HUD "Progress"                : ${field('Progress')}`);
console.log(`HUD "Corner deviation avg / max (0)": ${field('Corner deviation avg / max (0)')}`);
console.log(`\nprogress = ${(x.progress * 100).toFixed(1)} %, corners assessed = ${x.cornerCount} of ` +
  `${Math.max(0, ref.length - 1)} in the reference, score = ${x.score}`);

if (x.score >= 99.999 && x.progress < 0.5) {
  console.log('\nRESULT: MISLEADING. The reference describes 3x the executed path, no corner was reached, ' +
    'and the card still headlines 100 with no deductions (Progress 33.3 % is shown below it).');
  process.exit(1);
}
console.log('\nRESULT: score reflects the missing material/corners');
