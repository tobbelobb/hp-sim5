// Sanity: the exact RRF map above must agree with the app's own forward model
// (hp-sim-3d/app/workers/kinematics.js) and round-trip through its inverse.
import fs from 'node:fs';
import { parseMachine, idealLines, positionFromLines, stepsFromLine, lineFromSteps } from './lib/rrfModel.mjs';
import { pos_to_motor_pos_samples_deg } from '../../../../hp-sim-3d/app/workers/kinematics.js';

const ROOT = new URL('../../../../', import.meta.url);
const config = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const machine = parseMachine(config);
console.log('axes', machine.axes, 'Q', machine.Q, 'R', machine.R, 'U', machine.U,
  'O', machine.O, 'H', machine.H, 'L', machine.L, 'flex', machine.flexEnabled);
console.log('anchors', JSON.stringify(machine.anchors));

const appsDeg = (p) => pos_to_motor_pos_samples_deg(
  machine.anchors, [p], 20, false, machine.Q, machine.R,
  { U: machine.U, O: machine.O, H: machine.H, L: machine.L })[0];

let worst = 0;
for (const p of [[0, 0, 0], [200, 200, 0], [-500, 500, 100], [600, -600, 0], [0, 0, 400]]) {
  const line = idealLines(machine, p);
  const mine = line.map((l, i) => stepsFromLine(machine, i, l) * 2 * Math.PI / 3200 * 180 / Math.PI);
  const app = appsDeg(p);
  const d = Math.max(...mine.map((v, i) => Math.abs(v - app[i])));
  worst = Math.max(worst, d);
  const back = positionFromLines(machine, line);
  console.log(JSON.stringify(p), 'app-vs-rrf max deg diff', d.toExponential(2),
    'line->pos residual', back.residualRms.toExponential(2), 'pos', back.p.map((v) => +v.toFixed(9)));
}
// round trip angle->line->angle
for (const l of [-100, -42, 0, 42, 100]) {
  const s = stepsFromLine(machine, 0, l);
  console.log('roundtrip line', l, '->', lineFromSteps(machine, 0, s).toFixed(12));
}
console.log('worst app-vs-rrf degrees', worst.toExponential(3));
