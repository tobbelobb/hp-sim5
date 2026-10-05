import { readFileSync } from 'node:fs';
import { World, RigidBodyComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { registerSimulationSystems } from '../../../hp-sim-3d/app/simulationSystems.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';

const minimalSource = () => readFileSync('tests/fixtures/python_3d_parity/usd_scene_minimal.usda', 'utf8');

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
  expect(path.spring_constant).toBe(0);
  expect(path.compliance).toBe(Infinity);
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
  const source = name => readFileSync(`public/usd_scenes/${name}_rigid_body.usda`, 'utf8');
  const world = applyScene(new World(), source('hp4'), '/World/HangprinterScene', { namespace: 'old' });
  registerSimulationSystems(world);
  const remote = world.systems.find(system => system instanceof RemoteSpoolSystem);
  remote.commands = [{ type: 'SetTorqueMode', axis: 'D', torqueNm: -.01 }];
  world.update(.002);
  const keys = ['torqueModeCableLoadTorques', 'torqueModeCableLoadStiffnesses', 'torqueModeCableLoadDampings'];
  const loads = keys.map(key => world.getResource(key));
  expect(loads.every(map => map.size > 0)).toBe(true); // actual solver loads, not seeded stand-ins
  const systems = world.systems.slice();
  remote.commands = [{ type: 'Move', A: .001 }];
  world.getResource('pauseState').paused = true;
  applyScene(world, source('hp3'), '/World/HangprinterScene', { namespace: 'new', append });
  expect(world.systems).toEqual(systems);
  expect(remote.axisToEntity).toEqual({});
  expect(remote.getQueueLength()).toBe(1);
  world.update(.002); // paused systems cannot overwrite a stale load map
  keys.forEach((key, index) => {
    if (append) expect(world.getResource(key)).toBe(loads[index]);
    else expect(world.getResource(key).size).toBe(0);
  });
});
