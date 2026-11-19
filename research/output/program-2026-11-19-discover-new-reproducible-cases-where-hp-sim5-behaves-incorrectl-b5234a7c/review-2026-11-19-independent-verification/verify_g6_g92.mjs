// Independent reviewer check for C1/C2.  Drives the shipped MoveCommander and
// the shipped headless HP4 world (real RemoteSpoolSystem).  One process.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { RigidBodyComponent, RigidBodyMemberComponent, PositionComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
const config = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');

async function emit(gcode) {
  const c = new MoveCommander();
  c.configure(config);
  c.setAsapMode(true);
  const cmds = [];
  c.sendCommand = async (x) => { cmds.push(x); };
  await c.run(new ReadableStream({ start(ctl) { ctl.enqueue(new TextEncoder().encode(gcode)); ctl.close(); } }));
  return cmds;
}

const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');
const { world, remote, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, requireRrfTimestep: false });
const axes = ['A', 'B', 'C', 'D'];
const stepper = (a) => world.getComponent(remote.axisToEntity[a][0], StepperMotorComponent);
const target = (a) => stepper(a).commandedAngle - stepper(a).deltaAngle;
const targets = () => Object.fromEntries(axes.map((a) => [a, target(a)]));
const resetSteppers = () => { for (const a of axes) { stepper(a).commandedAngle = 0; stepper(a).deltaAngle = 0; } };
const effector = () => {
  for (const id of world.query([RigidBodyComponent])) {
    const rb = world.getComponent(id, RigidBodyComponent);
    const m = rb?.members?.map((x) => world.getComponent(x, RigidBodyMemberComponent)).find(Boolean);
    if (!m || !(m.physicalMass > 0)) continue;
    const p = world.getComponent(m.bodyEntity ?? id, PositionComponent);
    if (p?.value) return p.value;
  }
  return null;
};
for (let i = 0; i < 120; i++) world.update(dt);

// ---------------- C1: G6 stream, with and without the trailing Add
const g6 = await emit('G6 A-42 B-42 C-42 F5000\n');
const lastMove = [...g6].reverse().find((c) => c.type === 'Move');
const lastAdd = [...g6].reverse().find((c) => c.type === 'Add to reference');
console.log('stream:', g6.length, 'commands,', g6.filter((c) => c.type === 'Move').length, 'Move,',
  g6.filter((c) => c.type === 'Add to reference').length, 'Add');
console.log('last Move (rad):', axes.map((a) => (+lastMove[a]).toFixed(9)).join(' '));
console.log('Add      (rad):', axes.map((a) => (+lastAdd[a]).toFixed(9)).join(' '));

function runStream(cmds) {
  resetSteppers();
  const t0 = targets();
  let peak = { ...t0 };
  for (const c of cmds) {
    remote.addCommand(c);
    world.update(dt);
    const t = targets();
    for (const a of axes) if (Math.abs(t[a] - t0[a]) > Math.abs(peak[a] - t0[a])) peak[a] = t[a];
  }
  const t1 = targets();
  return { net: axes.map((a) => t1[a] - t0[a]), peak: axes.map((a) => peak[a] - t0[a]) };
}
const deg = (r) => (r * 180) / Math.PI;
const withAdd = runStream(g6);
const withoutAdd = runStream(g6.filter((c) => c !== lastAdd));
for (const [label, r] of [['full stream  ', withAdd], ['Move stream only', withoutAdd]]) {
  console.log(label, 'net(deg)', r.net.map((v) => deg(v).toFixed(4)).join(' '),
    ' peak(deg)', r.peak.map((v) => deg(v).toFixed(4)).join(' '));
}
const linePerRad = 30 / 2;  // R / (U * H/L), hp4
console.log('=> net line payout full:', (deg(withAdd.net[0]) * Math.PI / 180 * linePerRad).toFixed(4),
  'mm; Move-only:', (deg(withoutAdd.net[0]) * Math.PI / 180 * linePerRad).toFixed(4), 'mm');

// ---------------- C2: G92 transient vs terminal
for (let i = 0; i < 200; i++) world.update(dt);
const p0 = effector();
const jumpTarget = {};
const cmds = await emit('G92 X5\nG1 X10 Y0 Z0 F600\n');
resetSteppers();
const t0 = targets();
let maxDisp = 0, maxJump = 0;
for (const c of cmds) {
  remote.addCommand(c);
  world.update(dt);
  const t = targets();
  const jump = Math.max(...axes.map((a) => Math.abs(t[a] - t0[a])));
  if (jump > maxJump) maxJump = jump;
  const p = effector();
  if (p && p0) maxDisp = Math.max(maxDisp, Math.hypot(p.x - p0.x, p.y - p0.y, p.z - p0.z));
}
console.log('G92 X5 then G1: max |target jump| =', maxJump.toFixed(6), 'rad (',
  (maxJump * linePerRad).toFixed(3), 'mm of line); max effector excursion =', (maxDisp * 1000).toFixed(3), 'mm over',
  cmds.length, 'steps');
console.log('first Add:', JSON.stringify(cmds.find((c) => c.type === 'Add to reference')));

// terminal G92 (what probe_world.mjs does)
const cmds2 = await emit('G92 X5\n');
resetSteppers();
const t2 = targets();
remote.addCommand(cmds2[0]);
world.update(dt);
const after = targets();
console.log('terminal G92 X5: target jump B =', (after.B - t2.B).toFixed(6), 'rad (=',
  ((after.B - t2.B) * linePerRad).toFixed(3), 'mm of B line)');
