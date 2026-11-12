import { readFileSync } from 'node:fs';
import { World, RigidBodyComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { registerSimulationSystems } from '../../../hp-sim-3d/app/simulationSystems.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';
import { setSolverIterations } from '../../../hp-sim-3d/app/solver-iterations.js';
import { setPhysicsSubsteps } from '../../../hp-sim-3d/app/physics-substeps.js';

const minimalSource = () => readFileSync('tests/fixtures/engine_parity/usd_scene_minimal.usda', 'utf8');

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
