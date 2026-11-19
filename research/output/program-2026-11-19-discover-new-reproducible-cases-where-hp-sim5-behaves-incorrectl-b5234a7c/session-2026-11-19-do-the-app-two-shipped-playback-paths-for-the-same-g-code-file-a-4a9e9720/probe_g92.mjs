// Secondary observation: G92 X<nonzero> in the upload path emits only an
// 'Add to reference' (moveCommander.js:332-351), i.e. it shifts deltaAngle
// without issuing the matching Move, so the world's target
// (commandedAngle - deltaAngle) jumps by -delta at the G92 line.
//
// Sign convention used below: target jump J = -Add. A positive J on B means
// B's line is paid *out* by J*R/U mm (B's anchor is at +x, so the tool is
// pushed towards -x). The independent review measured the tool on the same
// world: terminal `G92 X5` -> -5.000 mm (closed loop) / -0.606 mm (open loop).
import fs from 'node:fs';
globalThis.self = { addEventListener() {} };
globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
const configText = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');

async function run(gcode) {
  const c = new MoveCommander();
  c.configure(configText);
  c.setAsapMode(true);
  const cmds = [];
  c.sendCommand = async (x) => { cmds.push(x); };
  await c.run(new ReadableStream({ start(ctl) { ctl.enqueue(new TextEncoder().encode(gcode)); ctl.close(); } }));
  return { cmds, c };
}

const axes = ['A', 'B', 'C', 'D'];
for (const gcode of ['G92 X5\n', 'G1 X10 Y0 Z0\nG92 X5\n']) {
  const { cmds, c } = await run(gcode);
  const commanded = Object.fromEntries(axes.map((a) => [a, 0]));
  const delta = Object.fromEntries(axes.map((a) => [a, 0]));
  const t0 = Object.fromEntries(axes.map((a) => [a, commanded[a] - delta[a]]));
  let lastAdd = null;
  for (const cmd of cmds) {
    if (cmd.type === 'Move') for (const a of axes) if (Number.isFinite(cmd[a])) commanded[a] = cmd[a];
    if (cmd.type === 'Add to reference') {
      lastAdd = Object.fromEntries(axes.map((a) => [a, +(cmd[a] ?? 0).toFixed(6)]));
      for (const a of axes) if (Number.isFinite(cmd[a])) delta[a] += cmd[a];
    }
  }
  const radToMm = (a, v) => v * c.spool_radius_mm[axes.indexOf(a)] / c.spoolParams.U[axes.indexOf(a)];
  console.log(`\n${JSON.stringify(gcode.trim())}: ${cmds.filter((x) => x.type === 'Move').length} Move, ` +
    `${cmds.filter((x) => x.type === 'Add to reference').length} Add to reference`);
  console.log('  Add to reference (rad)  :', JSON.stringify(lastAdd));
  const jumpRad = Object.fromEntries(axes.map((a) => [a, commanded[a] - delta[a] - t0[a]]));
  const jumpLine = Object.fromEntries(axes.map((a) => [a, radToMm(a, jumpRad[a])]));
  const round = (o, d) => Object.fromEntries(Object.entries(o).map(([k, v]) => [k, Number(v.toFixed(d))]));
  console.log('  target jump J = -Add    :', JSON.stringify(round(jumpRad, 6)), '(must be 0 on every axis)');
  console.log('  implied line change (mm):', JSON.stringify(round(jumpLine, 4)));
  console.log('  => B line +4.28 mm, C line -4.29 mm: the tool is pushed towards -x');
}
