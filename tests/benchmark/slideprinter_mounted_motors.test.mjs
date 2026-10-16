import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { StepperMotorComponent, getRigidBodyMemberSpoolFrame } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { SpoolStateComponent, getSpoolRotationAngle } from '../../hp-sim-3d/app/hangprinter_spools.js';
import { EncoderComponent, OrientationComponent, RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';

const scene = fs.readFileSync(new URL('../../public/usd_scenes/slideprinter_single_pinholes_rigid_body.usda', import.meta.url), 'utf8');
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

// Protect the mounted rotor DOFs themselves. Backend parity alone cannot show
// that a rotor is still free to turn relative to its moving host body.
// These are support checks, not a conservation test of the simplified hinge:
// cable pose corrections still skip member angular-velocity reconstruction.
for (const closedLoopMotorsEnabled of [false, true]) {
  for (const lineLayeringEnabled of [false, true]) {
    for (const physicsSubsteps of [1, 4, 16]) {
      test(`Slideprinter onboard motors: closed ${closedLoopMotorsEnabled}, layering ${lineLayeringEnabled}, substeps ${physicsSubsteps}`, () => {
        const backend = new WasmPhysicsEngine(module);
        try {
          const { world, remote, dt } = createHeadlessWorld(scene, {
            closedLoopMotorsEnabled, lineLayeringEnabled, physicsSubsteps, cableSolverBackend: backend,
          });
          const motors = world.query([StepperMotorComponent]);
          const bodies = world.query([RigidBodyComponent]);
          assert.equal(motors.length, 3);
          assert.equal(bodies.length, 1);
          remote.commands = Array.from({ length: 1500 }, (_, i) => {
            const phase = Math.sin(i * dt * 2 * Math.PI);
            return { type: 'Move', A: .03 * phase, B: -.015 * phase, C: .01 * phase };
          });
          let maxRelativeAngle = 0, maxBodyAngle = 0;
          for (let step = 0; step < 1500; step++) {
            world.update(dt);
            for (const id of motors) {
              const frame = getRigidBodyMemberSpoolFrame(world, id, world.getComponent(id, SpoolStateComponent));
              assert(frame);
              assert.equal(frame.member.bodyEntity, bodies[0]);
              const angle = getSpoolRotationAngle(frame.localSpoolState, frame.member.localOrientation);
              const encoder = world.getComponent(id, EncoderComponent).angle;
              const error = Math.atan2(Math.sin(encoder - angle), Math.cos(encoder - angle));
              assert(Number.isFinite(angle) && Number.isFinite(encoder));
              assert(Math.abs(error) < 1e-10, 'encoder must measure rotation relative to the body');
              maxRelativeAngle = Math.max(maxRelativeAngle, Math.abs(angle));
            }
            const q = world.getComponent(bodies[0], OrientationComponent).quaternion;
            maxBodyAngle = Math.max(maxBodyAngle, 2 * Math.atan2(Math.hypot(q.x, q.y, q.z), Math.abs(q.w)));
          }
          assert(maxRelativeAngle > .02, 'rotors must turn within the body');
          assert(maxBodyAngle > .001, 'exercise a moving and rotating host body');
          assert.equal(remote.getQueueLength(), 0);
          assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps, 0);
        } finally {
          backend.dispose();
        }
      });
    }
  }
}
