// C2: does the one-step G92 target jump produce any physical effect?
// Compare the physical trajectory of the G92 spelling with the equivalent
// plain file.  Fresh world per case; long settle.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { ExtruderComponent } from '../../../../hp-sim-3d/app/hangprinter_extruder.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
const config = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');
async function emit(gcode) {
  const c = new MoveCommander(); c.configure(config); c.setAsapMode(true);
  const cmds = []; c.sendCommand = async (x) => cmds.push(x);
  await c.run(new ReadableStream({ start(ctl) { ctl.enqueue(new TextEncoder().encode(gcode)); ctl.close(); } }));
  return cmds;
}

async function caseRun(label, gcode, settle = 2000) {
  const { world, remote, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, requireRrfTimestep: false });
  const axes = ['A', 'B', 'C', 'D'];
  const ext = () => world.getComponent([...world.query([ExtruderComponent])][0], ExtruderComponent);
  const tip = () => { const e = ext(); const p = e?.tipPos || e?.centerPos; return p ? [p.x, p.y, p.z] : null; };
  for (let i = 0; i < 300; i++) world.update(dt);
  const p0 = tip();
  const cmds = await emit(gcode);
  let maxD = 0;
  for (const c of cmds) {
    remote.addCommand(c); world.update(dt);
    const p = tip();
    if (p) maxD = Math.max(maxD, Math.hypot(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]));
  }
  for (let i = 0; i < settle; i++) world.update(dt);
  const p1 = tip();
  const mm = (v) => (v * 1000);
  console.log(label.padEnd(26), 'cmds', String(cmds.length).padStart(4),
    'peak|tip|', mm(maxD).toFixed(3).padStart(8), 'mm  final', p1.map((v) => mm(v).toFixed(2)).join(',').padStart(20));
  return p1;
}

const a = await caseRun('G1 X5 (equivalent)', 'G1 X5 Y0 Z0 F600\n');
const b = await caseRun('G92 X5 + G1 X10', 'G92 X5\nG1 X10 Y0 Z0 F600\n');
const c = await caseRun('G92 X5 only', 'G92 X5\n');
console.log('A vs B final difference (mm):', Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) * 1000);
