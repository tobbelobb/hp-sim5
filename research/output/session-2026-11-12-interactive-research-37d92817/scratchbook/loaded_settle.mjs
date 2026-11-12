// Load the HP4 loaded-stability fixture, run N steps at given substeps, print z peak-to-peak per window.
// usage: node loaded_settle.mjs <substeps> <steps> [holdingTorque override for scene+fixture]
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { PositionComponent, AngularVelocityComponent, RigidBodyComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
const root = new URL('../../../../', import.meta.url);
const [subText = '1', stepsText = '20000', tau] = process.argv.slice(2);
let scene = fs.readFileSync(new URL('public/usd_scenes/hp4_rigid_body.usda', root), 'utf8');
const saved = JSON.parse(fs.readFileSync(new URL('tests/fixtures/hp4_loaded_stability.json', root)),
  (k, v) => ['NaN', 'Infinity', '-Infinity'].includes(v) ? Number(v) : v);
if (tau) { scene = scene.replace(/holdingTorque = [0-9.]+/g, `holdingTorque = ${tau}`); for (const i of [109, 2509, 4749, 6829]) saved.regions[0][i] = Number(tau); }
const { world, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, closedLoopMotorsEnabled: true });
world.setResource('physicsSubsteps', Number(subText));
world.update(dt);
world.getResource('cableSolverBackend').regions.forEach((r, i) => r.data.set(saved.regions[i]));
const body = world.query([RigidBodyComponent])[0];
const motors = world.query([StepperMotorComponent]);
let lo = Infinity, hi = -Infinity, w = 0, zs = [];
for (let s = 1; s <= Number(stepsText); s++) {
  world.update(dt);
  const z = world.getComponent(body, PositionComponent).pos.z; zs.push(z);
  lo = Math.min(lo, z); hi = Math.max(hi, z); w = Math.max(w, world.getComponent(body, AngularVelocityComponent).omega.length());
  if (s % 2500 === 0) { console.log(`steps ${s - 2499}-${s}: z p2p ${((hi - lo) * 1e6).toFixed(3)} um, max omega ${w.toExponential(2)}, servoErr ${motors.map(m => world.getComponent(m, StepperMotorComponent).servoError.toExponential(1)).join(' ')}`); lo = Infinity; hi = -Infinity; w = 0; }
}
if (process.env.TRACE) for (let k = 0; k < 8; k++) { world.update(dt); console.log(motors.map(m => { const c = world.getComponent(m, StepperMotorComponent); return `${c.servoError.toExponential(3)}/${c.servoIntegralTorque.toFixed(4)}`; }).join("  "), world.getComponent(body, PositionComponent).pos.z.toFixed(8)); }
// dominant period from zero crossings of the last 1000 steps
const tail = zs.slice(-1000), mean = tail.reduce((a, b) => a + b) / tail.length;
let crossings = 0; for (let i = 1; i < tail.length; i++) if ((tail[i - 1] - mean) * (tail[i] - mean) < 0) crossings++;
console.log(`last 2 s: ~${(crossings / 2 / 2).toFixed(1)} Hz`);
world.getResource('cableSolverBackend')?.dispose?.();
