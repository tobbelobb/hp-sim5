// Cross-check the command-level claim against the shipped world:
// feed one command stream into the real RemoteSpoolSystem of a headless HP4
// world and read back (a) the stepper target commandedAngle - deltaAngle and
// (b) the effector position from the physics.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { RigidBodyComponent, RigidBodyMemberComponent, PositionComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

const ROOT = new URL('../../../../', import.meta.url);
const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');
const { world, remote, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, requireRrfTimestep: false });
console.log('world built, dt', dt, 'axes', Object.keys(remote.axisToEntity || {}));

const stepperOf = (axis) => world.getComponent(remote.axisToEntity[axis][0], StepperMotorComponent);
const target = (axis) => {
  const s = stepperOf(axis);
  return s.commandedAngle - s.deltaAngle;
};
// effector = the single free rigid body (the mover)
const bodyIds = [...world.query([RigidBodyComponent])];
console.log('rigid bodies', bodyIds.length);
function effectorPos() {
  // the mover is the only rigid body whose members include a non-zero mass
  for (const id of bodyIds) {
    const rb = world.getComponent(id, RigidBodyComponent);
    const member = rb?.members?.map((m) => world.getComponent(m, RigidBodyMemberComponent)).find(Boolean);
    if (!member || !(member.physicalMass > 0)) continue;
    const pos = world.getComponent(member.bodyEntity ?? id, PositionComponent);
    if (pos?.value) return pos.value;
  }
  return null;
}
const show = (label) => {
  const p = effectorPos();
  console.log(label,
    'targets', ['A', 'B', 'C', 'D'].map((a) => target(a).toFixed(6)).join(' '),
    'eff', p ? [p.x ?? p[0], p.y ?? p[1], p.z ?? p[2]].map((v) => +v.toFixed(4)) : null);
};
for (let i = 0; i < 20; i += 1) world.update(dt);
show('start');

const stream = [
  { type: 'Move', A: 0, B: 0, C: 0, D: 0 },
  { type: 'Add to reference', A: 0.000434, B: -0.285475, C: 0, D: 0 }, // what G92 X5 emits on hp4
];
for (const c of stream) remote.addCommand(c);
for (let i = 0; i < 400; i += 1) world.update(dt);
show('after G92 X5');
