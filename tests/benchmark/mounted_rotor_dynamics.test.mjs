import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { runFixture } from '../parity3d/oracle.mjs';
import { World } from '../../src/js/cable_joints_3d/ecs.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import { registerSimulationSystems } from '../../hp-sim-3d/app/simulationSystems.js';
import { WasmCableSolver } from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { NativePhysicsEngine } from '../../src/js/cable_joints_3d/native_physics_engine.js';

const fixture = JSON.parse(fs.readFileSync(new URL('../fixtures/python_3d_parity/mounted_rotor_momentum.json', import.meta.url)));
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));
const world = new World();
registerSimulationSystems(world, { cableSolverBackend: null });
const systems = world.systems.map(system => system.constructor.name);
const near = (actual, expected, tolerance = 1e-12) => assert(Math.abs(actual - expected) < tolerance, `${actual} != ${expected}`);

for (const backend of ['JavaScript', 'Wasm cable kernel', 'Wasm physics', 'native physics']) {
  for (const configuration of ['planar', 'tilted', 'tensor', 'passive', 'fixed']) {
    const tilted = configuration === 'tilted';
    test(`${backend}: mounted rotors conserve momentum and coast, ${configuration}`, () => {
      const input = structuredClone(fixture);
      const tensor = configuration === 'tensor';
      const fixed = configuration === 'fixed';
      if (fixed) {
        input.entities[0].components.MomentOfInertiaComponent = [0];
        input.entities[0].components.MassComponent = [0];
      }
      if (configuration === 'passive') delete input.entities.find(e => e.name === 'passive').components.StepperMotorComponent;
      if (tensor) input.entities[0].components.MomentOfInertiaComponent = [[[.8, .02, .1], [.02, .9, .05], [.1, .05, 1]]];
      const rotation = tilted ? new Quaternion().setFromAxisAngle(new Vector3(1, 2, 3).normalize(), .8) : new Quaternion();
      const axis = rotation.transformVector(new Vector3(0, 0, 1));
      const rotate = values => {
        const v = rotation.transformVector(new Vector3(...values));
        return [v.x, v.y, v.z];
      };
      input.resources.defaultPlaneNormal = [axis.x, axis.y, axis.z];
      const q = [rotation.x, rotation.y, rotation.z, rotation.w];
      input.initializeRigidBodies = ['body'];
      input.initialSet = [];
      for (const entity of input.entities) {
        const c = entity.components;
        for (const key of ['PositionComponent', 'PrevFinalPosComponent']) if (c[key]) c[key] = rotate(c[key]);
        for (const key of ['OrientationComponent', 'PrevFinalOrientationComponent']) if (c[key]) c[key] = q;
        if (c.SpoolStateComponent) c.SpoolStateComponent[2] = q;
        if (c.CableLinkComponent) {
          const p = rotate(c.CableLinkComponent.slice(0, 3));
          c.CableLinkComponent.splice(0, 3, ...p);
          input.initialSet.push([entity.name, 'CableLinkComponent', 'prevCableAttachmentTimeOrientation', q]);
        }
        if (c.CableJointComponent) for (const i of [3, 4]) c.CableJointComponent[i] = rotate(c.CableJointComponent[i]);
      }
      const engine = backend === 'native physics' ? new NativePhysicsEngine()
        : backend === 'Wasm physics' ? new WasmPhysicsEngine(module)
        : backend === 'Wasm cable kernel' ? new WasmCableSolver(module) : null;
      if (backend === 'Wasm cable kernel') { input.pipeline = false; input.systems = systems; }
      try {
        const snapshots = runFixture(input, { cableSolverBackend: engine }).snapshots;
        // Carrier inertia 0.7, active rotor 0.1, passive rotor 0.2.
        // Friction shares the 1 mm extension equally between the two segments.
        // J M^-1 J^T = 1/m + r²/Ir + r²/Icarrier; alpha = 250.
        // Eliminate x/y analytically for the non-diagonal carrier tensor.
        const xFactor = tensor ? -(.9 * .1 - .02 * .05) / (.8 * .9 - .02**2) : 0;
        const yFactor = tensor ? -(.8 * .05 - .02 * .1) / (.8 * .9 - .02**2) : 0;
        const carrierInertia = fixed ? Infinity : .7 + .1 * xFactor + .05 * yFactor;
        const lambda = -.0005 / ((fixed ? 0 : 1) + .1**2 / .1 + .1**2 / carrierInertia + 250);
        const bodyAngle = lambda * .1 / carrierInertia;
        const activeAngle = -lambda * .1 * (1 / .1 + 1 / carrierInertia);
        const passiveAngle = -bodyAngle;
        const omega = (snapshot, name) => {
          const values = snapshot.entities[name].AngularVelocityComponent.omega;
          return new Vector3(...values).dot(axis);
        };
        const first = snapshots[1];
        near(first.entities.outer.CableJointComponent.constraintLambda, lambda);
        near(first.entities.spool.EncoderComponent.angle, activeAngle);
        near(first.entities.passive.EncoderComponent.angle, passiveAngle);
        for (const [index, snapshot] of snapshots.entries()) {
          if (!index) continue;
          const body = omega(snapshot, 'body'), active = omega(snapshot, 'spool'), passive = omega(snapshot, 'passive');
          near(body, bodyAngle / .002, tensor ? 5e-10 : 1e-12);
          near(active, activeAngle / .002, tensor ? 5e-10 : 1e-12);
          near(passive, passiveAngle / .002, tensor ? 5e-10 : 1e-12);
          if (tensor) {
            const [x, y, z] = snapshot.entities.body.AngularVelocityComponent.omega;
            near(x, xFactor * bodyAngle / .002, 5e-10);
            near(y, yFactor * bodyAngle / .002, 5e-10);
            near(.1 * x + .05 * y + z + .1 * active + .2 * passive, 0, 5e-10);
          } else if (!fixed) near(body + .1 * active + .2 * passive, 0); // no external moment
          near(body + passive, 0, tensor ? 5e-10 : 1e-12); // unloaded rotor's absolute velocity
          near(snapshot.entities.spool.EncoderComponent.angle, index * activeAngle);
          near(snapshot.entities.passive.EncoderComponent.angle, index * passiveAngle);
          const pose = snapshot.entities.body.OrientationComponent.quaternion;
          const delta = new Quaternion(...pose).multiply(rotation.clone().conjugate()).normalize();
          near(2 * Math.atan2(new Vector3(delta.x, delta.y, delta.z).dot(axis), delta.w), index * bodyAngle);
        }
      } finally { engine?.dispose?.(); }
    });
  }
}
