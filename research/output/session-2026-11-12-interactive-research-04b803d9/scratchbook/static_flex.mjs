// Static flex consistency test at the HP4 home pose (closed loop, native engine).
// usage: node static_flex.mjs <lpA> <lpB> <lpC> <lpD>   (effector-distance mm, firmware linePos)
// Ramps each spool to angle lp*MA/r over 1 s, holds 4 s, prints effector displacement and line tensions.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { PositionComponent, RigidBodyComponent, SceneEntityInfoComponent, EncoderComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { SpoolStateComponent } from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
globalThis.self = { addEventListener() {} };
globalThis.postMessage = m => { if (m.type === 'error') throw new Error(m.message); };
const root = new URL('../../../../', import.meta.url);
const lp = process.argv.slice(2, 6).map(Number);
const MA = [2, 2, 2, 4], r = 0.03;
const target = lp.map((v, i) => v * 1e-3 * MA[i] / r);
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(new URL(process.env.SCENE ?? 'public/usd_scenes/hp4_rigid_body.usda', root), 'utf8'),
  { lineLayeringEnabled: false, closedLoopMotorsEnabled: true, physicsSubsteps: Number(process.env.SUBSTEPS ?? 1) });
const body = world.query([RigidBodyComponent])[0];
const paths = world.query([CablePathComponent]);
const tension = () => Object.fromEntries(paths.map(pid => {
  const js = world.getComponent(pid, CablePathComponent).jointEntities;
  return [world.getComponent(pid, SceneEntityInfoComponent).name.replace('CablePath', ''),
    js.map(j => +world.getComponent(j, CableJointComponent).constraintForceMagnitude.toFixed(3))];
}));
const pos = () => { const p = world.getComponent(body, PositionComponent).pos; return [p.x, p.y, p.z]; };
const ramp = 500, hold = 2000;
const cmds = [];
for (let i = 1; i <= ramp + hold; i++) {
  const s = Math.min(1, i / ramp);
  cmds.push({ type: 'Move', A: target[0] * s, B: target[1] * s, C: target[2] * s, D: target[3] * s });
}
for (let i = 0; i < 500; i++) world.update(dt); // settle at home first
const p0 = pos();
const motors = world.query([StepperMotorComponent]).map(id => ({ id, axis: world.getComponent(id, SpoolStateComponent).axis })).sort((a, b) => a.axis.localeCompare(b.axis));
const enc = () => motors.map(m => world.getComponent(m.id, EncoderComponent).angle);
const e0 = enc();
const geo = () => Object.fromEntries(paths.map(pid => {
  const js = world.getComponent(pid, CablePathComponent).jointEntities.map(j => world.getComponent(j, CableJointComponent));
  return [world.getComponent(pid, SceneEntityInfoComponent).name.replace('CablePath', ''),
    { rest: js.map(j => j.restLength), len: js.map(j => j.attachmentPointA_world.distanceTo(j.attachmentPointB_world)) }];
}));
const g0 = geo();
remote.commands = cmds;
for (let i = 0; i < cmds.length; i++) world.update(dt);
const p1 = pos();
const g1 = geo();
if (process.env.GEO) for (const k of Object.keys(g0)) {
  const s = a => a.reduce((x, y) => x + y, 0);
  console.log(k, 'dRest_mm', g1[k].rest.map((v, i) => +((v - g0[k].rest[i]) * 1e3).toFixed(4)).join(' '), '| dLen_mm', g1[k].len.map((v, i) => +((v - g0[k].len[i]) * 1e3).toFixed(4)).join(' '),
    '| sum len m', s(g1[k].len).toFixed(4), 'stretch mm before', ((s(g0[k].len) - s(g0[k].rest)) * 1e3).toFixed(4), 'after', ((s(g1[k].len) - s(g1[k].rest)) * 1e3).toFixed(4));
}
console.log(JSON.stringify({ lp, target_rad: target.map(v => +v.toFixed(5)), enc_minus_target_rad: enc().map((v, i) => +((v - e0[i]) - target[i]).toFixed(5)), servoErr: motors.map(m => +world.getComponent(m.id, StepperMotorComponent).servoError.toFixed(5)), disp_mm: p1.map((v, i) => +((v - p0[i]) * 1e3).toFixed(4)), tension: tension() }));
world.getResource('cableSolverBackend')?.dispose?.();
