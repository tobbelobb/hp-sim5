import fs from 'node:fs';
import { createHeadlessWorld } from '../../../hp-sim-3d/app/headlessWorld.js';
import { setKinematicMotorMode } from '../../../hp-sim-3d/app/hangprinter_runtime.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { WasmPhysicsEngine } from '../../../src/js/cable_joints_3d/wasm_physics_engine.js';
import {
  SceneEntityInfoComponent,
  EncoderComponent,
  AngularVelocityComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';

const scenePath = new URL('../../../public/usd_scenes/copacabana.usda', import.meta.url);
const wasmPath = new URL('../../../public/wasm/cable_solver.wasm', import.meta.url);
const sceneText = fs.readFileSync(scenePath, 'utf8');
const wasmModule = new WebAssembly.Module(fs.readFileSync(wasmPath));

describe('kinematic motor mode for encoder replay', () => {
  test('tracks commanded spool angles exactly without motor dynamics and restores cleanly', () => {
    const backend = new WasmPhysicsEngine(wasmModule);
    try {
      const { world, remote, dt } = createHeadlessWorld(sceneText, {
        cableSolverBackend: backend,
        lineLayeringEnabled: false,
        closedLoopMotorsEnabled: false,
        kinematicMotors: true,
      });
      const named = new Map(
        world.query([SceneEntityInfoComponent]).map((id) => [
          world.getComponent(id, SceneEntityInfoComponent).name,
          id,
        ]),
      );
      const axes = Array.from('ABCDIJLO');
      const spools = axes.map((axis) => {
        const id = named.get(`Spool${axis}`);
        return {
          axis,
          stepper: world.getComponent(id, StepperMotorComponent),
          encoder: world.getComponent(id, EncoderComponent),
          angular: world.getComponent(id, AngularVelocityComponent),
        };
      });

      // Step large rapid multi-turn commands across all 8 axes under cable tension.
      for (let step = 1; step <= 40; step++) {
        const cmd = { type: 'Move' };
        for (let i = 0; i < axes.length; i++) {
          cmd[axes[i]] = (i % 2 === 0 ? 1 : -1) * 0.12 * step;
        }
        remote.addCommand(cmd);
        world.update(dt);
        for (const { stepper, encoder, angular } of spools) {
          const target = stepper.commandedAngle - stepper.deltaAngle;
          expect(Math.abs(encoder.angle - target)).toBeLessThan(1e-9);
          expect(angular.omega.length()).toBe(0);
          expect(stepper.missedSteps).toBe(0);
        }
      }

      // Holding still under full cable tension must not backdrive the kinematic spools.
      for (let step = 0; step < 20; step++) {
        world.update(dt);
        for (const { stepper, encoder } of spools) {
          const target = stepper.commandedAngle - stepper.deltaAngle;
          expect(Math.abs(encoder.angle - target)).toBeLessThan(1e-9);
        }
      }

      // Disabling kinematic mode restores dynamic motor inertia and finite rotor lag on a sudden jump.
      setKinematicMotorMode(world, false);
      const jump = { type: 'Move', A: spools[0].stepper.commandedAngle + 0.5 };
      remote.addCommand(jump);
      world.update(dt);
      const dynamicErr = Math.abs(
        spools[0].encoder.angle - (spools[0].stepper.commandedAngle - spools[0].stepper.deltaAngle),
      );
      expect(dynamicErr).toBeGreaterThan(1e-3);
    } finally {
      backend.dispose();
    }
  });
});
