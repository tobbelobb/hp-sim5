jest.mock('../../../src/js/cable_joints_3d/render_system_3d.js', () => ({
  RenderSystem3D: jest.fn(),
}));
jest.mock('../../../hp-sim-3d/app/runner.js', () => ({ runGame: jest.fn() }));

import { readFileSync } from 'node:fs';
import { World, HybridKnotAngleComponent, RigidBodyComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { registerSimulationSystems } from '../../../hp-sim-3d/app/simulationSystems.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';
import { setSolverIterations } from '../../../hp-sim-3d/app/solver-iterations.js';
import { setPhysicsSubsteps } from '../../../hp-sim-3d/app/physics-substeps.js';
import { LINE_LAYERING_TOGGLE_KEYS, setLineLayeringFeatureFlags } from '../../../hp-sim-3d/app/line-layering-flags.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { createMachineSceneController } from '../../../hp-sim-3d/app/machineSceneController.js';
import { createSceneChangeQueue } from '../../../hp-sim-3d/app/sceneController.js';
import { setupScene } from '../../../hp-sim-3d/app/setupScene.js';

const minimalSource = () => readFileSync('tests/fixtures/engine_parity/usd_scene_minimal.usda', 'utf8');

test.each([true, false])('an HP3 plan keeps its baked geometry on reset (layered=%s)', layered => {
  const source = readFileSync('public/usd_scenes/hp3.usda', 'utf8');
  const { source: baked } = bakeCableSceneUsdaSource(source, layered
    ? {} : { cablePathHalfWidthOverride: 0, deriveAll: true });
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(OpenText(baked)), '/World/HangprinterScene'));
  expect(checked.valid).toBe(true);
  const plan = buildEntityPlan(checked);
  const world = new World();
  const read = Type => world.query([Type]).map(id => world.getComponent(id, Type));

  setLineLayeringFeatureFlags(world, layered);
  applyEntityPlan(world, plan);
  const knots = read(HybridKnotAngleComponent);
  const joints = read(CableJointComponent);
  const paths = read(CablePathComponent);
  expect(knots.length).toBeGreaterThan(0);
  expect(paths.map(path => path.cableHalfWidth)).toEqual(Array(paths.length).fill(layered ? .001 : 0));

  setLineLayeringFeatureFlags(world, !layered);
  applyEntityPlan(world, plan);
  expect(read(HybridKnotAngleComponent)).toEqual(knots);
  expect(read(CableJointComponent)).toEqual(joints);
  expect(read(CablePathComponent)).toEqual(paths);
  for (const key of LINE_LAYERING_TOGGLE_KEYS) {
    expect(world.getResource(key)).toBe(!layered);
  }
});

test('scene changes retain per-machine iterations and overrides survive resets', () => {
  const source = name => readFileSync(`public/usd_scenes/${name}.usda`, 'utf8');
  const world = new World();
  const load = (name, append = false) => applyScene(world, source(name), '/World/HangprinterScene', { append, namespace: name });
  load('hp3');
  expect(world.getResource('physicsSubsteps')).toBe(1);
  load('hp4', true);
  expect(world.getResource('physicsSubsteps')).toBe(1);
  expect(world.getResource('solverIterations')).toBe(10);
  load('hp3');
  expect(world.getResource('physicsSubsteps')).toBe(1);
  load('hp4');
  load('hp3', true);
  expect(world.getResource('physicsSubsteps')).toBe(1);
  expect(world.getResource('solverIterations')).toBe(10);
  const counts = () => world.query([CablePathComponent]).map(id => world.getComponent(id, CablePathComponent).solverIterations);
  expect(new Set(counts())).toEqual(new Set([1, 10]));
  setSolverIterations(world, 8);
  expect(counts().every(count => count === 8)).toBe(true);
  setPhysicsSubsteps(world, 2);
  load('hp4');
  expect(world.getResource('physicsSubsteps')).toBe(2);
  load('hp3');
  expect(world.getResource('physicsSubsteps')).toBe(2);
  expect(counts().every(count => count === 8)).toBe(true);
  setSolverIterations(world, null);
  expect(counts().every(count => count === 1)).toBe(true);
  load('hp4');
  expect(counts().every(count => count === 10)).toBe(true);
});

function applyScene(world, source, root = '/World/Scene', options = {}) {
  const stage = OpenText(bakeCableSceneUsdaSource(source).source);
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), root, options));
  applyEntityPlan(world, buildEntityPlan(checked, options));
  return world;
}

test('the authored scene builder retains zero cable stiffness', () => {
  const world = applyScene(new World(), minimalSource());
  const paths = world.query([CablePathComponent]);
  expect(paths).toHaveLength(1);
  const path = world.getComponent(paths[0], CablePathComponent);
  expect(path.stiffnessPerUnitLength).toBe(0);
  expect(path.compliancePerUnitLength).toBe(Infinity);
});

test('the scene builder passes an authored cogging torque to the stepper, default otherwise', () => {
  const stepper = source => {
    const world = applyScene(new World(), source);
    return world.getComponent(world.query([StepperMotorComponent])[0], StepperMotorComponent);
  };
  expect(stepper(minimalSource()).coggingTorque).toBeUndefined();
  const authored = minimalSource().replace('double stepper:maxSpeedRad = 70',
    'double stepper:maxSpeedRad = 70\n            double stepper:coggingTorque = 0.014');
  expect(stepper(authored).coggingTorque).toBe(0.014);
});

test.each([
  ['double physics:mass = -1', /SpoolA.*missing required attributes: physics:mass/],
  ['token[] ecs:tags = ["Spool"]', /J0.*physics:body0 target \/World\/Scene\/SpoolA did not resolve/],
  ['rel physics:body0 = </World/Scene/SpoolA>', /J0.*requires exactly one physics:body0 target/],
  [/custom double restLength = [^\n]+/, /J0.*missing required attributes: restLength/],
  ['</World/Scene/J0>', /Path.*cablePath:joints target .* did not resolve to a CableJoint/, '</World/Scene/MissingJoint>'],
])('setup rejects invalid authored data for %s before replacing the world', (attribute, message, replacement = '') => {
  const source = bakeCableSceneUsdaSource(minimalSource()).source.replace(attribute, replacement);
  const world = applyScene(new World(), minimalSource());
  const generation = world.getResource('sceneGeneration');
  expect(() => setupScene(world, OpenText(source), null, { scenePrimPath: '/World/Scene' })).toThrow(message);
  expect(world.getResource('sceneGeneration')).toBe(generation);
});

test('an uploaded USDA with an unresolved cable body leaves the active machine registered', async () => {
  const world = applyScene(new World(), minimalSource());
  const structureVersion = world.structureVersion;
  const activeMachine = { id: 'machine-0' };
  const state = { machines: [activeMachine], machineIdCounter: 1, simDtSec: .002 };
  const onError = jest.fn();
  const controller = createMachineSceneController({
    world, state, dom: { canvas: {} }, catalog: new Map(),
    sceneQueue: createSceneChangeQueue({ onError }), featureFlags: { lineLayeringEnabled: true },
  });
  const source = minimalSource().replace('token[] ecs:tags = ["Spool"]', '');
  await expect(controller.addUploadedMachine({ name: 'bad.usda', text: async () => source }))
    .rejects.toThrow(/bad\.usda.*J0.*physics:body0 target \/World\/Scene\/SpoolA did not resolve/);
  expect(onError).toHaveBeenCalledTimes(1);
  expect(state.machines).toEqual([activeMachine]);
  expect(state.machineIdCounter).toBe(1);
  expect(world.structureVersion).toBe(structureVersion);
});

test('the legacy rigid-group spelling resolves its member relationships', () => {
  const source = minimalSource().replace('def RigidBody', 'def RigidGroup')
    .replace('rigidBody:members', 'rigidGroup:members');
  const world = applyScene(new World(), source);
  expect(world.query([RigidBodyComponent])).toHaveLength(1);
  const body = world.getComponent(world.query([RigidBodyComponent])[0], RigidBodyComponent);
  expect(body.members).toHaveLength(3);
});

test.each([false, true])('paused scene load resets entity loads only when replacing (append=%s)', append => {
  const source = name => readFileSync(`public/usd_scenes/${name}.usda`, 'utf8');
  const world = applyScene(new World(), source('hp4'), '/World/HangprinterScene', { namespace: 'old' });
  registerSimulationSystems(world);
  const remote = world.systems.find(system => system instanceof RemoteSpoolSystem);
  remote.commands = [{ type: 'SetTorqueMode', axis: 'D', torqueNm: -.01 }];
  world.update(.002);
  const keys = ['torqueModeCableLoadTorques', 'torqueModeCableLoadStiffnesses', 'torqueModeCableLoadDampings'];
  const loads = keys.map(key => world.getResource(key));
  expect(loads.every(map => map.size > 0)).toBe(true); // actual solver loads, not seeded stand-ins
  const colors = world.getResource('machineColors');
  const systems = world.systems.slice();
  remote.commands = [{ type: 'Move', A: .001 }];
  world.getResource('pauseState').paused = true;
  applyScene(world, source('hp3'), '/World/HangprinterScene', { namespace: 'new', append });
  expect(world.getResource('machineColors')).toBe(colors);
  expect([...colors.keys()]).toEqual(append ? ['old', 'new'] : ['new']);
  expect(world.systems).toEqual(systems);
  expect(remote.axisToEntity).toEqual({});
  expect(remote.getQueueLength()).toBe(1);
  world.update(.002); // paused systems cannot overwrite a stale load map
  keys.forEach((key, index) => {
    if (append) expect(world.getResource(key)).toBe(loads[index]);
    else expect(world.getResource(key).size).toBe(0);
  });
});
