import { readFileSync } from 'node:fs';
import { World } from '../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';

test('the authored scene builder retains zero cable stiffness', () => {
  const source = readFileSync('tests/fixtures/python_3d_parity/usd_scene_minimal.usda', 'utf8');
  const stage = OpenText(bakeCableSceneUsdaSource(source).source);
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), '/World/Scene'));
  const world = new World();
  applyEntityPlan(world, buildEntityPlan(checked));
  const paths = world.query([CablePathComponent]);
  expect(paths).toHaveLength(1);
  const path = world.getComponent(paths[0], CablePathComponent);
  expect(path.spring_constant).toBe(0);
  expect(path.compliance).toBe(Infinity);
});
