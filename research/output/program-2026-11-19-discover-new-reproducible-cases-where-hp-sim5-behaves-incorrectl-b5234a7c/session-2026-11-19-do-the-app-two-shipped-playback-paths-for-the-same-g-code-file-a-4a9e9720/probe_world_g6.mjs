// End-to-end cross-check of the G6 finding inside the shipped world:
// run public/gcode/tighten.gcode through MoveCommander, feed every emitted
// command to a headless HP4 world's own RemoteSpoolSystem (one command per
// physics step, exactly as the app does), and read back the stepper targets.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));

const configText = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const gcode = fs.readFileSync(new URL('public/gcode/tighten.gcode', ROOT), 'utf8');
const commander = new MoveCommander();
commander.configure(configText);
commander.setAsapMode(true);
const commands = [];
commander.sendCommand = async (c) => { commands.push(c); };
await commander.run(new ReadableStream({
  start(ctl) { ctl.enqueue(new TextEncoder().encode(gcode)); ctl.close(); },
}));

const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');
const { world, remote, dt } = createHeadlessWorld(scene, {
  lineLayeringEnabled: false, requireRrfTimestep: false,
});
const axes = ['A', 'B', 'C', 'D'];
const target = (a) => {
  const s = world.getComponent(remote.axisToEntity[a][0], StepperMotorComponent);
  return s.commandedAngle - s.deltaAngle;
};
for (let i = 0; i < 20; i += 1) world.update(dt);
const t0 = Object.fromEntries(axes.map((a) => [a, target(a)]));
let peak = { ...t0 };
for (const c of commands) {
  remote.addCommand(c);
  world.update(dt);
  for (const a of axes) {
    const d = target(a) - t0[a];
    if (Math.abs(d) > Math.abs(peak[a] - t0[a])) peak[a] = target(a);
  }
}
for (let i = 0; i < 200; i += 1) world.update(dt);   // let the tail settle
const end = Object.fromEntries(axes.map((a) => [a, target(a)]));
const deg = (v) => v * 180 / Math.PI;
console.log('G-code:', gcode.trim(), '| commands fed:', commands.length);
console.log('axis  net target change (deg)  peak excursion (deg)');
for (const a of axes) {
  console.log(`  ${a}   ${deg(end[a] - t0[a]).toFixed(5).padStart(12)}        ${deg(peak[a] - t0[a]).toFixed(3).padStart(12)}`);
}
console.log('expected: -160.428 deg on A,B,C (= -42 mm of line)');
