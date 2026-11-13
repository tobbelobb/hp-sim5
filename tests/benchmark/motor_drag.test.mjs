import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import { World, PositionComponent, RigidBodyComponent, EncoderComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CableLinkComponent, CablePathComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { StepperMotorComponent } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { setMotorStrength } from '../../hp-sim-3d/app/motor-strength.js';
import { GRAB_SPRING_STIFFNESS, updateGrabSpring } from '../../hp-sim-3d/app/hangprinter_input.js';
import { registerSimulationSystems } from '../../hp-sim-3d/app/simulationSystems.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { FULL_STEPS_PER_SLIP, getMachineMotorDiagnostics, resetMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';

// Grab the effector with the app's hand cord, drag it, let go. Open-loop
// steppers skip whole electrical cycles and stay there; closed-loop servos
// yield while overpowered and return to their targets.
function drag(scene, closed, pull, strength = 1, cableSolverBackend) {
  const source = fs.readFileSync(new URL(`../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
  const { world, dt } = createHeadlessWorld(source, { closedLoopMotorsEnabled: closed, requireRrfTimestep: false, cableSolverBackend });
  try {
    setMotorStrength(world, strength);
    const body = world.query([RigidBodyComponent])[0];
    const motors = world.query([StepperMotorComponent]);
    const encoders = () => motors.map(m => world.getComponent(m, EncoderComponent).angle);
    const position = () => world.getComponent(body, PositionComponent).pos.clone();
    for (let i = 0; i < 500; i++) world.update(dt);
    // Like starting a job: count from the settled machine.
    resetMachineMotorDiagnostics(world);
    const start = position(), before = encoders();
    const pointer = world.createEntity();
    world.addComponent(pointer, new PositionComponent(start.x, start.y, start.z));
    world.addComponent(pointer, new CableLinkComponent(start.x, start.y, start.z, null, new Vector3(0, 0, 1)));
    const joint = world.createEntity();
    world.addComponent(joint, CableJointComponent.fromWorld(body, pointer, 0.1, start.clone(), start.clone()));
    const path = world.createEntity();
    world.addComponent(path, new CablePathComponent(world, [joint], ['attachment', 'attachment'], [true, true],
      GRAB_SPRING_STIFFNESS * 0.1));
    let peakForce = 0;
    for (let i = 0; i < 1000; i++) {
      const p = world.getComponent(pointer, PositionComponent).pos;
      const f = (i + 1) / 1000;
      p.x = start.x + f * pull[0]; p.y = start.y + f * pull[1]; p.z = start.z + f * pull[2];
      updateGrabSpring(world, { jointE: joint, pathE: path, ptrE: pointer });
      world.update(dt);
      peakForce = Math.max(peakForce, world.getComponent(joint, CableJointComponent).constraintForceMagnitude);
    }
    const dragged = position().distanceTo(start);
    world.destroyEntity(path); world.destroyEntity(joint); world.destroyEntity(pointer);
    for (let i = 0; i < 2500; i++) world.update(dt);
    const shifts = encoders().map((angle, k) => angle - before[k]);
    return { peakForce, dragged, back: position().distanceTo(start), shifts, diagnostics: getMachineMotorDiagnostics(world),
      cycle: 2 * Math.PI / world.getComponent(motors[0], StepperMotorComponent).numPolePairs };
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
}

for (const scene of ['hp4', 'slideprinter']) {
  test(`${scene}: dragging makes open-loop motors skip steps`, () => {
    const { dragged, back, shifts, cycle, diagnostics } = drag(scene, false, [0.5, 0, 0]);
    assert(dragged > 0.2, `effector followed the hand only ${dragged} m`);
    const slipped = shifts.filter(s => Math.abs(s) > cycle / 2);
    assert(slipped.length > 0, `no motor skipped: ${shifts}`);
    // Skipped steps land in another detent, up to the load's deflection.
    for (const s of slipped) assert(Math.abs(s / cycle - Math.round(s / cycle)) < 0.1, `fractional slip ${s / cycle}`);
    assert(back > 1e-4, 'skipped steps should leave the effector displaced');
    // The HUD counts every slip the encoders saw, in full steps.
    const slips = shifts.reduce((sum, s) => sum + Math.abs(Math.round(s / cycle)), 0);
    assert.equal(diagnostics.totalMissedSteps, slips * FULL_STEPS_PER_SLIP);
  });
  test(`${scene}: closed-loop motors flex while dragged and return`, () => {
    const { dragged, shifts, diagnostics } = drag(scene, true, [0.5, 0, 0]);
    assert(dragged > 0.2, `servos never yielded: ${dragged} m`);
    for (const s of shifts) assert(Math.abs(s) < 0.02, `servo did not return: ${shifts}`);
    assert.equal(diagnostics.totalMissedSteps, 0, 'a servo that yields and returns loses no steps');
  });
}

// The old elastic grab asymptoted to 100 N, even when pulled arbitrarily far.
// High-torque motors must still yield, and expose the actual pull to Show Forces.
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
for (const [backend, createBackend] of [
  ['native', () => undefined],
  ['browser Wasm', () => new WasmPhysicsEngine(module)],
]) test(`dragging overpowers 4 Nm motors and reports tension above the old 100 N ceiling: ${backend}`, () => {
  const { dragged, peakForce, diagnostics } = drag('hp4', false, [0.5, 0, 0], 4, createBackend());
  assert(Number.isFinite(dragged) && dragged > 0.2, `strong motors resisted the grab: ${dragged} m`);
  assert(Number.isFinite(peakForce) && peakForce > 100, `grab tension was limited or invalid: ${peakForce} N`);
  assert(diagnostics.totalMissedSteps > 0, 'strong motors never skipped steps');
});

for (const [backend, createBackend] of [
  ['native', () => undefined],
  ['browser Wasm', () => new WasmPhysicsEngine(module)],
]) test(`grab force grows gently with extension without saturating: ${backend}`, () => {
  const world = new World();
  world.setResource('dt', 0.002);
  const body = world.createEntity();
  const pointer = world.createEntity();
  for (const id of [body, pointer]) {
    world.addComponent(id, new PositionComponent());
    world.addComponent(id, new CableLinkComponent());
  }
  const joint = world.createEntity();
  world.addComponent(joint, CableJointComponent.fromWorld(body, pointer, 0.1, new Vector3(), new Vector3()));
  const path = world.createEntity();
  world.addComponent(path, new CablePathComponent(world, [joint], ['attachment', 'attachment'], [true, true], 100));
  registerSimulationSystems(world, { cableSolverBackend: createBackend() });
  try {
    // Both ends stay fixed so the measured force is the hand spring's pull,
    // unaffected by effector motion, motor strength or cable geometry.
    for (const [distance, expected] of [[0.09, 0], [0.11, 10], [0.12, 20], [0.2, 100], [0.5, 400]]) {
      world.getComponent(pointer, PositionComponent).pos.x = distance;
      updateGrabSpring(world, { jointE: joint, pathE: path, ptrE: pointer });
      world.update(0.002);
      const force = world.getComponent(joint, CableJointComponent).constraintForceMagnitude;
      assert(Math.abs(force - expected) < 0.01, `at ${distance} m: ${force} N, expected ${expected} N`);
    }
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});
