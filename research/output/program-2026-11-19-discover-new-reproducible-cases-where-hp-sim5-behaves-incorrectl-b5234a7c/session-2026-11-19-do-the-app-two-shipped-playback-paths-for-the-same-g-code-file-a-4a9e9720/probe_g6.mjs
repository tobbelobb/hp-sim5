// G6 in the upload path: does the commanded line payout survive the trailing
// 'Add to reference'?
//
// World semantics (authoritative): src/wasm/cable_solver/src/engine.rs:1568
//   let target = e[107] - e[108];          // commandedAngle - deltaAngle
//   hp-sim-3d/app/remoteSpoolSystem.js:267-289
//   'Move'             -> commandedAngle = v            (target -> v - delta)
//   'Add to reference' -> deltaAngle += d               (target -> target - d)
//
// moveCommander.js:353-411 (G6) emits 175 interpolated 'Move' commands that
// ramp the target by +delta, then one 'Add to reference' carrying the same
// +delta, which subtracts it again.
import fs from 'node:fs';
import { playUpload } from './lib/players.mjs';
import { parseMachine, idealLines, stepsFromLine, STEPS_PER_REV } from './lib/rrfModel.mjs';

const ROOT = new URL('../../../../', import.meta.url);
const read = (p) => fs.readFileSync(new URL(p, ROOT), 'utf8');

const configs = {
  'hp4 (control, Q=0)': 'RRF/run/vsd/sys/config_hp4.g',
  'hp4 w_line_layers (Q=0.6366)': 'RRF/run/vsd/sys/config_hp4_w_line_layers.g',
};
const gcode = read('public/gcode/tighten.gcode');
console.log('G-code:', JSON.stringify(gcode));

for (const [label, cfgPath] of Object.entries(configs)) {
  const configText = read(cfgPath);
  const machine = parseMachine(configText);
  const { commands } = await playUpload(gcode, configText);
  const na = machine.axes.length;
  const commanded = new Float64Array(na);
  const delta = new Float64Array(na);
  const target = () => machine.axes.map((_, k) => commanded[k] - delta[k]);
  const t0 = target();
  let peak = t0.map((v) => v);
  const trace = [];
  for (const c of commands) {
    if (c.type === 'Move') {
      for (let k = 0; k < na; k += 1) if (Number.isFinite(c[machine.axes[k]])) commanded[k] = c[machine.axes[k]];
    } else if (c.type === 'Add to reference') {
      for (let k = 0; k < na; k += 1) if (Number.isFinite(c[machine.axes[k]])) delta[k] += c[machine.axes[k]];
    }
    const t = target();
    for (let k = 0; k < na; k += 1) {
      if (Math.abs(t[k] - t0[k]) > Math.abs(peak[k] - t0[k])) peak[k] = t[k];
    }
    trace.push({ type: c.type, t: t.slice() });
  }
  const tEnd = target();
  const toDeg = (v) => v * 180 / Math.PI;
  console.log('\n===', label, 'Q', machine.Q, 'R', machine.R[0]);
  console.log('commands:', commands.length,
    commands.reduce((a, c) => (a[c.type] = (a[c.type] || 0) + 1, a), {}));
  console.log('  axis   intended line dl (mm)   exact angle dl (deg)  worker angle dl (deg)  NET target dl applied (deg)  peak (deg)');
  for (let k = 0; k < na; k += 1) {
    const lineDelta = k < 3 ? -42 : 0;              // G6 A-42 B-42 C-42
    const exactSteps = stepsFromLine(machine, k, lineDelta) - stepsFromLine(machine, k, 0);
    const exactDeg = toDeg(exactSteps * 2 * Math.PI / STEPS_PER_REV);
    // linearised formula the worker uses
    const workerDeg = toDeg(lineDelta / machine.R[k] * machine.U[k] * machine.H[k] / machine.L[k]);
    console.log(`  ${machine.axes[k]}      ${String(lineDelta).padStart(8)}            ${exactDeg.toFixed(5).padStart(12)}          ${workerDeg.toFixed(5).padStart(12)}          ${toDeg(tEnd[k] - t0[k]).toFixed(9).padStart(14)}       ${toDeg(peak[k] - t0[k]).toFixed(4).padStart(10)}`);
  }
  const last = commands.at(-1);
  console.log('  final command:', JSON.stringify(last));
  console.log('  deltaAngle after:', JSON.stringify(machine.axes.map((a, k) => +delta[k].toFixed(6))));
  console.log('  commandedAngle after:', JSON.stringify(machine.axes.map((a, k) => +commanded[k].toFixed(6))));
}
