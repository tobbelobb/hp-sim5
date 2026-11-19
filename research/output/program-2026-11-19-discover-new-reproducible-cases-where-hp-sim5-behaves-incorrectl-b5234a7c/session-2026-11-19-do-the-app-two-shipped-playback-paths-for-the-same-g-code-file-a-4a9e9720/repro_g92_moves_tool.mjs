// Reproducer (G92): a terminal `G92 X5` must not move the machine, but the
// Upload path emits only an 'Add to reference' (moveCommander.js:332-351)
// with no matching Move, and the world's target is
// commandedAngle - deltaAngle (src/wasm/cable_solver/src/engine.rs:1568).
// The target therefore jumps by -delta at the G92 line and the shipped HP4
// world physically rotates the motors.
//
// Assertions (run against the shipped headless HP4 world, shipped motor mode):
//   1. |target jump| at the G92 line must be 0 -> it is not (exit 1)
//   2. |motor rotation| measured on the EncoderComponent after the G92 must be
//      0 -> it is not
//
// HP4 ships open loop (`createHeadlessWorld` default closedLoopMotorsEnabled =
// false), so the realised motion is smaller than the commanded 5 mm; the size
// is motor-mode dependent. The independent review measured, on the same world:
//   terminal `G92 X5`            -> -5.000 mm tool (closed loop), -0.606 mm (open loop)
//   `G92 X5` then `G1 X10`       -> +4.983 mm (closed loop, correct +4.998), +0.538 mm (open loop)
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { EncoderComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));

const configText = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');

async function uploadStream(gcode) {
  const commander = new MoveCommander();
  commander.configure(configText);
  commander.setAsapMode(true);
  const commands = [];
  commander.sendCommand = async (c) => { commands.push(c); };
  await commander.run(new ReadableStream({
    start(ctl) { ctl.enqueue(new TextEncoder().encode(gcode)); ctl.close(); },
  }));
  return { commander, commands };
}

const { world, remote, dt } = createHeadlessWorld(scene, {
  lineLayeringEnabled: false, requireRrfTimestep: false,
});
const axes = ['A', 'B', 'C', 'D'];
const stepper = (a) => world.getComponent(remote.axisToEntity[a][0], StepperMotorComponent);
const encoder = (a) => world.getComponent(remote.axisToEntity[a][0], EncoderComponent);
const target = (a) => stepper(a).commandedAngle - stepper(a).deltaAngle;
const settle = (n) => { for (let i = 0; i < n; i += 1) world.update(dt); };
const deg = (v) => v * 180 / Math.PI;

console.log(`world: headless HP4, dt=${dt}s, axes ${axes.join('/')}, open-loop (as shipped)`);

async function runCase(label, gcode, steps, { motorMustStayStill = true } = {}) {
  const { commander, commands } = await uploadStream(gcode);
  settle(40);
  const t0 = Object.fromEntries(axes.map((a) => [a, target(a)]));
  const e0 = Object.fromEntries(axes.map((a) => [a, encoder(a)?.angle ?? NaN]));
  let jump = null;
  for (const c of commands) {
    remote.addCommand(c);
    world.update(dt);
    if (c.type === 'Add to reference') {
      jump = Object.fromEntries(axes.map((a) => [a, target(a) - t0[a]]));
    }
  }
  settle(steps);
  const tEnd = Object.fromEntries(axes.map((a) => [a, target(a)]));
  const eEnd = Object.fromEntries(axes.map((a) => [a, encoder(a)?.angle ?? NaN]));
  const lineMm = (a, v) => v * commander.spool_radius_mm[axes.indexOf(a)] / commander.spoolParams.U[axes.indexOf(a)];

  console.log(`\n${label}: ${JSON.stringify(gcode.trim())} -> ` +
    `${commands.filter((c) => c.type === 'Move').length} Move, ` +
    `${commands.filter((c) => c.type === 'Add to reference').length} Add to reference`);
  console.log('  axis | target jump at G92 (deg) | target@line (mm) | motor turned (deg) | motor line (mm)');
  for (const a of axes) {
    console.log(`    ${a}  | ${deg(jump?.[a] ?? 0).toFixed(4).padStart(12)} | ${lineMm(a, jump?.[a] ?? 0).toFixed(4).padStart(9)} | ` +
      `${deg(eEnd[a] - e0[a]).toFixed(4).padStart(11)} | ${lineMm(a, eEnd[a] - e0[a]).toFixed(4).padStart(9)}`);
  }
  const maxJump = Math.max(...axes.map((a) => Math.abs(jump?.[a] ?? 0)));
  const maxMotor = Math.max(...axes.map((a) => Math.abs(eEnd[a] - e0[a])));
  console.log(`  max |target jump| = ${deg(maxJump).toFixed(4)} deg (must be 0); ` +
    `max |motor rotation| = ${deg(maxMotor).toFixed(4)} deg ` +
    (motorMustStayStill ? '(must be 0)' : '(includes the legitimate G1 move - not asserted)'));
  return { maxJump, maxMotor, netTarget: Object.fromEntries(axes.map((a) => [a, tEnd[a] - t0[a]])) };
}

const terminal = await runCase('terminal G92', 'G92 X5\n', 400);
// the second case also contains a legitimate X=5 -> X=10 move, so only the
// target jump at the G92 line is asserted there
const withMove = await runCase('G92 then G1 X10', 'G92 X5\nG1 X10 Y0 Z0 F3000\n', 2000,
  { motorMustStayStill: false });

const ok = terminal.maxJump < 1e-9 && terminal.maxMotor < 1e-9 && withMove.maxJump < 1e-9;
if (!ok) {
  console.error('\nFAIL: G92 X<nonzero> moves the machine.');
  console.error('The Upload path emits only an Add to reference without the matching Move ' +
    '(moveCommander.js:332-351), so target = commandedAngle - deltaAngle jumps by -delta.');
  process.exit(1);
}
console.log('\nPASS: G92 X<nonzero> leaves the machine still.');
