// Closed/open-loop hand-drag probe with holding-torque override (copy of motor_drag.test.mjs drag()).
// usage: node drag_probe.mjs <scene> <closed 0|1> <torque> [substeps]
import fs from 'node:fs';
const R = '../../../../';
const { default: Vector3 } = await import(R + 'src/js/cable_joints_3d/vector3.js');
const { PositionComponent, RigidBodyComponent, EncoderComponent } = await import(R + 'src/js/cable_joints_3d/ecs.js');
const { CableJointComponent, CableLinkComponent, CablePathComponent } = await import(R + 'src/js/cable_joints_3d/cable_joints_core.js');
const { StepperMotorComponent } = await import(R + 'hp-sim-3d/app/hangprinter_stepper_motor.js');
const { HAND_CORD_STIFFNESS_PER_UNIT_LENGTH } = await import(R + 'hp-sim-3d/app/hangprinter_input.js');
const { createHeadlessWorld } = await import(R + 'hp-sim-3d/app/headlessWorld.js');
const { getMachineMotorDiagnostics, resetMachineMotorDiagnostics } = await import(R + 'hp-sim-3d/app/motor-diagnostics.js');
const [scene, closedText, tau, sub = '1'] = process.argv.slice(2);
const closed = closedText === '1';
const source = fs.readFileSync(new URL(R + `public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8')
  .replace(/holdingTorque = [0-9.]+/g, `holdingTorque = ${tau}`);
const { world, dt } = createHeadlessWorld(source, { closedLoopMotorsEnabled: closed, requireRrfTimestep: false });
world.setResource('physicsSubsteps', Number(sub));
const body = world.query([RigidBodyComponent])[0];
const motors = world.query([StepperMotorComponent]);
const encoders = () => motors.map(m => world.getComponent(m, EncoderComponent).angle);
const position = () => world.getComponent(body, PositionComponent).pos.clone();
for (let i = 0; i < 500; i++) world.update(dt);
resetMachineMotorDiagnostics(world);
const start = position(), before = encoders();
const pointer = world.createEntity();
world.addComponent(pointer, new PositionComponent(start.x, start.y, start.z));
world.addComponent(pointer, new CableLinkComponent(start.x, start.y, start.z, null, new Vector3(0, 0, 1)));
const joint = world.createEntity();
world.addComponent(joint, CableJointComponent.fromWorld(body, pointer, 0.1, start.clone(), start.clone()));
const path = world.createEntity();
world.addComponent(path, new CablePathComponent(world, [joint], ['attachment', 'attachment'], [true, true], HAND_CORD_STIFFNESS_PER_UNIT_LENGTH));
let peakErr = 0;
for (let i = 0; i < 1000; i++) {
  const p = world.getComponent(pointer, PositionComponent).pos; const f = (i + 1) / 1000;
  p.x = start.x + f * 0.5; world.update(dt);
}
const dragged = position().distanceTo(start);
world.destroyEntity(path); world.destroyEntity(joint); world.destroyEntity(pointer);
let maxTail = 0, lastPos = position();
for (let i = 0; i < 2500; i++) { world.update(dt); if (i >= 2000) { maxTail = Math.max(maxTail, position().distanceTo(lastPos)); } lastPos = position(); }
const cycle = 2 * Math.PI / world.getComponent(motors[0], StepperMotorComponent).numPolePairs;
const shifts = encoders().map((a, k) => (a - before[k]) / cycle);
console.log(JSON.stringify({ scene, closed, tau, sub, dragged: +dragged.toFixed(4), back: +position().distanceTo(start).toExponential(2),
  shiftsCycles: shifts.map(s => +s.toFixed(3)), missed: getMachineMotorDiagnostics(world).totalMissedSteps, tailStepMotion: +maxTail.toExponential(2) }));
world.getResource('cableSolverBackend')?.dispose();
