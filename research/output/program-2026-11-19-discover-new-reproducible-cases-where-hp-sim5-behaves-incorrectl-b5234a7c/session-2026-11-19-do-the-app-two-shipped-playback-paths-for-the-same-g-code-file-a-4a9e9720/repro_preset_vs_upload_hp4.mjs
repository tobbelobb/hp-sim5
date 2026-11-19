// Reproducer / regression guard for the ONE comparison that should be equal:
// the same G-code file played by the "Print Squares" preset (compiled .can via
// integrations/rrf/rrfCanPlayer.js) and by the Upload path
// (hp-sim-3d/app/workers/moveCommander.js) on HP4 without line layering.
//
// Tolerances (see report.md):
//   * motor / line-payout travel   <= 0.1 %   (measured 0.0089..0.023 %)
//   * toolpath deviation from the G-code polyline, preset path
//                                  <= 0.6 mm  (measured 0.470 mm)
//   * upload path deviation        <= 1e-6 mm (measured 6e-12)
//   * IKK residual of the preset plan (line offsets that no effector position
//     can explain) must stay <= 1 mm; it is the flex-compensation signature.
// Exit 1 if the paths disagree beyond these bounds (i.e. a new bug appeared).
import fs from 'node:fs';
import { parseMachine, gcodePolyline } from './lib/rrfModel.mjs';
import { playCan, playUpload } from './lib/players.mjs';
import { createAccumulator, pathStats, lineError } from './lib/paths.mjs';

const ROOT = new URL('../../../../', import.meta.url);
const read = (p) => fs.readFileSync(new URL(p, ROOT), 'utf8');
const configText = read('RRF/run/vsd/sys/config_hp4.g');
const gcodeText = read('public/gcode/draw_squares_bigger.gcode');
const machine = parseMachine(configText);
const polyline = gcodePolyline(gcodeText);

const canAcc = createAccumulator(machine);
await playCan(new URL('public/RRF_CAN_commands/draw_squares_bigger_hp4_no_buildup.can', ROOT), configText,
  (c) => canAcc.push(c));
const can = canAcc.finish();
const upAcc = createAccumulator(machine);
await playUpload(gcodeText, configText, (c) => upAcc.push(c));
const up = upAcc.finish();

const canPath = pathStats(can, polyline);
const upPath = pathStats(up, polyline);
const canErr = lineError(machine, can, polyline);
const relAngle = machine.axes.map((a, k) => 100 * (can.absAngle[k] - up.absAngle[k]) / up.absAngle[k]);
const relLine = machine.axes.map((a, k) => 100 * (can.absLine[k] - up.absLine[k]) / up.absLine[k]);
const ikRms = Math.sqrt(can.residual.reduce((s, v) => s + v * v, 0) / can.n);
const maxRel = Math.max(...relLine.map(Math.abs));

console.log(`preset .can moves ${can.n}, upload moves ${up.n}`);
console.log(`line-payout travel (mm): CAN ${Array.from(can.absLine).map((v) => v.toFixed(2)).join(' ')}`);
console.log(`                         UPL ${Array.from(up.absLine).map((v) => v.toFixed(2)).join(' ')}`);
console.log(`relative difference (%): ${relLine.map((v) => v.toFixed(4)).join(' ')}  (max |.| = ${maxRel.toFixed(4)} %)`);
console.log(`toolpath deviation from G-code (mm): CAN max ${canPath.maxD.toFixed(4)} rms ${canPath.rms.toFixed(4)}`);
console.log(`                                     UPL max ${upPath.maxD.toExponential(2)} rms ${upPath.rms.toExponential(2)}`);
console.log(`line error vs ideal geometry (mm):   CAN rms ${canErr.rms.map((v) => v.toFixed(3)).join(' ')}  (all <= 0)`);
console.log(`4-line consistency residual (mm):    CAN rms ${ikRms.toFixed(4)}   UPL ${Math.max(...up.residual).toExponential(2)}`);

let ok = true;
const check = (name, actual, limit) => {
  const pass = actual <= limit;
  if (!pass) ok = false;
  console.log(`  ${pass ? 'ok  ' : 'FAIL'} ${name}: ${actual.toFixed(6)} <= ${limit}`);
};
check('max |line-payout travel difference| (%)', maxRel, 0.1);
check('preset toolpath deviation (mm)', canPath.maxD, 0.6);
check('upload toolpath deviation (mm)', upPath.maxD, 1e-6);
check('preset 4-line consistency residual rms (mm)', ikRms, 1.0);

if (!ok) {
  console.error('\nFAIL: the two shipped playback paths disagree beyond the documented envelope.');
  process.exit(1);
}
console.log('\nPASS: preset and upload paths agree; residual is the documented flex-compensation offset.');
