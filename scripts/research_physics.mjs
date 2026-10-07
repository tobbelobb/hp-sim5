// Fixed-step production JS physics, without a browser or rendering.
// Python owns collection/lifecycle and uses these detached states only for telemetry.
import fs from 'node:fs';
import readline from 'node:readline';
import { World, PositionComponent, OrientationComponent, VelocityComponent, AngularVelocityComponent,
  EncoderComponent, RigidBodyMemberComponent, SceneEntityInfoComponent } from '../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent } from '../src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '../src/js/usd/stage.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../hp-sim-3d/app/scene/machineScenePipeline.js';
import { registerSimulationSystems } from '../hp-sim-3d/app/simulationSystems.js';
import { RemoteSpoolSystem } from '../hp-sim-3d/app/remoteSpoolSystem.js';
import { StepperMotorComponent } from '../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { ExtruderComponent } from '../hp-sim-3d/app/hangprinter_extruder.js';

// Keep stdout strictly a request/reply protocol, including scene-load warnings.
console.log = console.warn = (...args) => console.error(...args);
const world = new World();
const checked = validateMachineSceneSpec(readMachineSceneSpec(
  parseStage(OpenText(fs.readFileSync(process.argv[2], 'utf8'))), '/World/HangprinterScene'));
if (!checked.valid) throw new Error(checked.warnings.join('\n'));
applyEntityPlan(world, buildEntityPlan(checked));
registerSimulationSystems(world);
const remote = world.systems.find(system => system instanceof RemoteSpoolSystem);
remote._ensureAxisMapping(world);
const dt = world.getResource('dt');
if (dt !== .002) throw new Error('RRF requires 0.002 s physics steps');
let step = 0;
let physicsWallS = 0;
const xyz = v => [v.x, v.y, v.z];
const xyzw = q => [q.x, q.y, q.z, q.w];
const fields = [
  [PositionComponent, c => ({pos: xyz(c.pos)})],
  [OrientationComponent, c => ({quaternion: xyzw(c.quaternion)})],
  [VelocityComponent, c => ({vel: xyz(c.vel)})],
  [AngularVelocityComponent, c => ({omega: xyz(c.omega)})],
  [EncoderComponent, c => ({angle: c.angle})],
  [RigidBodyMemberComponent, c => ({local_position: xyz(c.localPosition), local_orientation: xyzw(c.localOrientation)})],
  [StepperMotorComponent, c => ({commanded_angle: c.commandedAngle, delta_angle: c.deltaAngle,
    torque_mode: c.torqueMode, target_torque: c.targetTorque, missed_steps: c.missedSteps,
    current_missed_steps: c.currentMissedSteps, missed_step_encoder_offset: c.missedStepEncoderOffset ?? null})],
  [CableJointComponent, c => ({rest_length: c.restLength, attachment_point_a_world: xyz(c.attachmentPointA_world),
    attachment_point_b_world: xyz(c.attachmentPointB_world), constraint_lambda: c.constraintLambda,
    constraint_force: xyz(c.constraintForce), constraint_force_magnitude: c.constraintForceMagnitude,
    transferred_constraint_force_magnitude: c.transferredConstraintForceMagnitude})],
  [CablePathComponent, c => ({stored: c.stored})],
  [ExtruderComponent, c => ({extrusions: c.extrusions.map(r => ({...r, pos: xyz(r.pos)})),
    ...Object.fromEntries(['machineEffectorCenters', 'machineCenters', 'machineTips', 'machineColdEnds'].map(key => [
      key.replace(/[A-Z]/g, s => '_' + s.toLowerCase()),
      Object.fromEntries(Object.entries(c[key]).map(([name, v]) => [name, xyz(v)])),
    ]))})],
];
function state() {
  return {step, physics_wall_s: physicsWallS, queue_length: remote.getQueueLength(),
    identities: world.query([SceneEntityInfoComponent]).map(id => [id, world.getComponent(id, SceneEntityInfoComponent).name]),
    components: fields.map(([type, read]) => [type.name, world.query([type]).map(id => [id, read(world.getComponent(id, type))])])};
}
for await (const line of readline.createInterface({input: process.stdin})) {
  try {
    const request = JSON.parse(line);
    if (request.steps != null && (!Number.isInteger(request.steps) || request.steps < 0 || request.steps > 50)) {
      throw new Error('Worker advances are bounded to 50 fixed steps');
    }
    if (request.clear_commands) remote.clearCommandQueue();
    for (const command of request.commands ?? []) remote.addCommand(command);
    const started = performance.now();
    for (let i = 0; i < (request.steps ?? 0); i++) { world.update(dt); step++; }
    physicsWallS += (performance.now() - started) / 1000;
    process.stdout.write(JSON.stringify(state(), (key, value) => {
      if (typeof value === 'number' && !Number.isFinite(value)) throw new Error(`Non-finite physics observation: ${key}`);
      return value;
    }) + '\n');
  } catch (error) { process.stdout.write(JSON.stringify({error: error.message}) + '\n'); }
}
