import { readFileSync } from 'node:fs';
import { World, RigidBodyComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';

const minimalSource = () => readFileSync('tests/fixtures/python_3d_parity/usd_scene_minimal.usda', 'utf8');

function buildWorld(source) {
  const stage = OpenText(bakeCableSceneUsdaSource(source).source);
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), '/World/Scene'));
  const world = new World();
  applyEntityPlan(world, buildEntityPlan(checked));
  return world;
}

test('the authored scene builder retains zero cable stiffness', () => {
  const world = buildWorld(minimalSource());
  const paths = world.query([CablePathComponent]);
  expect(paths).toHaveLength(1);
  const path = world.getComponent(paths[0], CablePathComponent);
  expect(path.spring_constant).toBe(0);
  expect(path.compliance).toBe(Infinity);
});

test('the legacy rigid-group spelling resolves its member relationships', () => {
  const source = minimalSource().replace('def RigidBody', 'def RigidGroup')
    .replace('rigidBody:members', 'rigidGroup:members');
  const world = buildWorld(source);
  expect(world.query([RigidBodyComponent])).toHaveLength(1);
  const body = world.getComponent(world.query([RigidBodyComponent])[0], RigidBodyComponent);
  expect(body.members).toHaveLength(3);
});
