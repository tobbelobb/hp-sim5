// Probe for repro_length_telemetry.py.
//
// Builds the production headless world on slideprinter.usda with the checked-in
// Wasm solver (so no build is needed), drives one position-mode move and one
// torque-mode switch, and prints, for the SAME world state:
//   * flight: the production JS flight-recorder cable lengths
//             (hp-sim-3d/app/flightRecorderSnapshot.js), and
//   * state:  the production worker state schema read by scripts/research_physics.mjs,
//             which Python's hp_sim5_research.mirror consumes.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = process.env.HP_SIM5_ROOT || path.resolve(here, '../../../..');

const { PositionComponent, OrientationComponent, VelocityComponent, AngularVelocityComponent,
  EncoderComponent, RigidBodyMemberComponent, SceneEntityInfoComponent, RadiusComponent,
  MachineTagComponent, RenderableComponent } = await import(path.join(root, 'src/js/cable_joints_3d/ecs.js'));
const { CableJointComponent, CablePathComponent } = await import(path.join(root, 'src/js/cable_joints_3d/cable_joints_core.js'));
const { SpoolStateComponent } = await import(path.join(root, 'hp-sim-3d/app/hangprinter_spools.js'));
const { createHeadlessWorld } = await import(path.join(root, 'hp-sim-3d/app/headlessWorld.js'));
const { StepperMotorComponent } = await import(path.join(root, 'hp-sim-3d/app/hangprinter_stepper_motor.js'));
const { ExtruderComponent } = await import(path.join(root, 'hp-sim-3d/app/hangprinter_extruder.js'));
const { captureFlightRecorderSnapshot } = await import(path.join(root, 'hp-sim-3d/app/flightRecorderSnapshot.js'));
const { WasmPhysicsEngine } = await import(path.join(root, 'src/js/cable_joints_3d/wasm_physics_engine.js'));

const wasm = new WebAssembly.Module(fs.readFileSync(path.join(root, 'public/wasm/cable_solver.wasm')));
const backend = new WasmPhysicsEngine(wasm);
const { world, remote, dt } = createHeadlessWorld(
  fs.readFileSync(path.join(root, 'public/usd_scenes/slideprinter.usda'), 'utf8'),
  { lineLayeringEnabled: false, cableSolverBackend: backend });
world.systems.find(s => s.constructor.name === 'ExtruderSystem')?.update(world, 0);

const axes = Object.keys(remote.axisToEntity);
const xyz = v => [v.x, v.y, v.z];
const xyzw = q => [q.x, q.y, q.z, q.w];
// Field list copied from the production worker, scripts/research_physics.mjs.
const fields = [
  [PositionComponent, c => ({pos: xyz(c.pos)})],
  [OrientationComponent, c => ({quaternion: xyzw(c.quaternion)})],
  [VelocityComponent, c => ({vel: xyz(c.vel)})],
  [AngularVelocityComponent, c => ({omega: xyz(c.omega)})],
  [EncoderComponent, c => ({angle: c.angle})],
  [RigidBodyMemberComponent, c => ({body_entity: c.bodyEntity, local_position: xyz(c.localPosition), local_orientation: xyzw(c.localOrientation)})],
  [StepperMotorComponent, c => ({commanded_angle: c.commandedAngle, delta_angle: c.deltaAngle,
    torque_mode: c.torqueMode, target_torque: c.targetTorque, missed_steps: c.missedSteps,
    current_missed_steps: c.currentMissedSteps, missed_step_encoder_offset: c.missedStepEncoderOffset ?? null})],
  [CableJointComponent, c => ({rest_length: c.restLength, attachment_point_a_world: xyz(c.attachmentPointA_world),
    attachment_point_b_world: xyz(c.attachmentPointB_world), constraint_lambda: c.constraintLambda,
    constraint_force: xyz(c.constraintForce), constraint_force_magnitude: c.constraintForceMagnitude,
    transferred_constraint_force_magnitude: c.transferredConstraintForceMagnitude})],
  [CablePathComponent, c => ({stored: c.stored, cable_half_width: c.cableHalfWidth, link_types: c.linkTypes, joint_entities: c.jointEntities})],
  [SpoolStateComponent, c => ({axis: c.axis})],
  [RadiusComponent, c => ({radius: c.radius})],
  [MachineTagComponent, c => ({id: c.id})],
  [SceneEntityInfoComponent, c => ({name: c.name, tags: c.tags})],
  [RenderableComponent, c => ({shape: c.shape, color: c.color})],
  [ExtruderComponent, c => ({extrusions: c.extrusions.map(r => ({...r, pos: xyz(r.pos)})),
    ...Object.fromEntries(['machineEffectorCenters', 'machineCenters', 'machineTips', 'machineColdEnds'].map(key => [
      key.replace(/[A-Z]/g, s => '_' + s.toLowerCase()),
      Object.fromEntries(Object.entries(c[key]).map(([name, v]) => [name, xyz(v)])),
    ]))})],
];
const state = step => ({
  step, dt_s: dt, scene_generation: world.getResource('sceneGeneration') ?? null,
  layering_flags: Object.fromEntries(['enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps']
    .map(k => [k, world.getResource(k)])),
  components: fields.map(([type, read]) => [type.name, world.query([type]).map(id => [id, read(world.getComponent(id, type))])]),
});
const flight = () => captureFlightRecorderSnapshot(world, { geometryDetail: 'fast' })
  .cables.map(c => ({ name: c.name, lengths: c.lengths }));

const out = { dt, axes, samples: [] };
out.samples.push({ label: 'step0', flight: flight(), state: state(0) });

// Position mode: 0.5 rad on the first motor. Open-loop detent slip means the
// rotor lags the commanded angle, so the JS world reports a real length error.
const move = { type: 'Move' };
axes.forEach(a => { move[a] = 0; });
move[axes[0]] = 0.5;
remote.addCommand(move);
for (let i = 1; i <= 4; i += 1) world.update(dt);
out.samples.push({ label: 'position-move', flight: flight(), state: state(4) });

// Torque mode: no commanded angle exists for this axis.
remote.addCommand({ type: 'SetTorqueMode', axis: axes[2], torqueNm: 0.02 });
for (let i = 1; i <= 2; i += 1) world.update(dt);
out.samples.push({ label: 'torque-mode', flight: flight(), state: state(6) });

process.stdout.write(JSON.stringify(out) + '\n');
