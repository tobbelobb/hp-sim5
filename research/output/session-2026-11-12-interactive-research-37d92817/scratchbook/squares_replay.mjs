// Replay the browser "Print Squares" preset headlessly (HP4, unlayered by default,
// closed-loop by default) and dump per-step effector/motor/cable state as JSON.
// usage: node squares_replay.mjs <label> [seconds] [closed=1] [layered=0] [substeps=1] [can override] [gcodeSpeedScale]
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { SpoolStateComponent } from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import { EncoderComponent, PositionComponent, OrientationComponent, AngularVelocityComponent, RigidBodyComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
const root = new URL('../../../../', import.meta.url);
const [label, secondsText = '60', closedText = '1', layeredText = '0', substepsText = '1', canOverride] = process.argv.slice(2);
const closed = closedText === '1', layered = layeredText === '1';
const source = canOverride ?? `public/RRF_CAN_commands/draw_squares_bigger_hp4_${layered ? 'w_line_layers' : 'no_buildup'}.can`;
const config = `RRF/run/vsd/sys/${layered ? 'config_hp4_w_line_layers.g' : 'config_hp4.g'}`;
const maxSteps = Math.round(Number(secondsText) / .002);
globalThis.self = { addEventListener() {} };
globalThis.postMessage = m => { if (m.type === 'error') throw new Error(m.message); };
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const player = new RrfCanPlayer();
player.setDt(.002); player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(new URL(config, root), 'utf8')));
const commands = [];
player.sendCommand = async c => { if (commands.length < maxSteps) commands.push(c); };
await player.run(Readable.toWeb(fs.createReadStream(new URL(source, root))), FileFormat.RRF_CAN_BINARY);
const steps = commands.length;
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(new URL(process.env.SCENE ?? 'public/usd_scenes/hp4_rigid_body.usda', root), 'utf8'),
  { lineLayeringEnabled: layered, closedLoopMotorsEnabled: closed, physicsSubsteps: Number(substepsText) });
const motors = world.query([StepperMotorComponent]).map(id => ({ id, axis: world.getComponent(id, SpoolStateComponent).axis }))
  .sort((a, b) => a.axis.localeCompare(b.axis));
const body = world.query([RigidBodyComponent])[0];
const paths = world.query([CablePathComponent]);
let tip = null;
remote.onExtrusion = e => { tip = e.pos; };
remote.commands = commands.slice();
const out = { label, source, config, closed, layered, substeps: Number(substepsText), dt, steps, axes: motors.map(m => m.axis),
  backend: world.getResource('physicsEngine'), t: [], pos: [], quat: [], omega: [], tip: [], E: [], phase: [], servoErr: [], servoTau: [], slips: [], tension: [] };
for (let step = 1; step <= steps; step++) {
  const cmd = commands[step - 1];
  tip = null;
  world.update(dt);
  const p = world.getComponent(body, PositionComponent).pos;
  const q = world.getComponent(body, OrientationComponent).q ?? world.getComponent(body, OrientationComponent).orientation;
  const w = world.getComponent(body, AngularVelocityComponent).omega;
  out.t.push(+(step * dt).toFixed(4));
  out.pos.push([p.x, p.y, p.z]);
  out.quat.push(q ? [q.x, q.y, q.z, q.w] : null);
  out.omega.push([w.x, w.y, w.z]);
  out.tip.push(tip);
  out.E.push(cmd?.E ?? 0);
  out.phase.push(motors.map(m => world.getComponent(m.id, EncoderComponent).angle - world.getComponent(m.id, StepperMotorComponent).commandedAngle));
  out.servoErr.push(motors.map(m => world.getComponent(m.id, StepperMotorComponent).servoError));
  out.servoTau.push(motors.map(m => world.getComponent(m.id, StepperMotorComponent).servoIntegralTorque));
  out.slips.push(motors.map(m => world.getComponent(m.id, StepperMotorComponent).missedSteps));
  out.tension.push(paths.map(pid => {
    const js = world.getComponent(pid, CablePathComponent).jointEntities;
    return world.getComponent(js[js.length - 1], CableJointComponent).constraintForceMagnitude;
  }));
  if (world.getResource('errorState')?.hasError) throw new Error(`error at ${step}`);
}
fs.writeFileSync(new URL(`${label}.json`, import.meta.url), JSON.stringify(out));
console.log(JSON.stringify({ label, steps, backend: out.backend, finalSlips: out.slips.at(-1), axes: out.axes }));
world.getResource('cableSolverBackend')?.dispose?.();
