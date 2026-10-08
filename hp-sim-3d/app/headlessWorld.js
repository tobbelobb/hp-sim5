import { World } from '../../src/js/cable_joints_3d/ecs.js';
import { OpenText } from '../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from './scene/machineScenePipeline.js';
import { registerSimulationSystems } from './simulationSystems.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';

// The same scene construction and system order used by the production app.
export function createHeadlessWorld(sceneText) {
  const sceneName = sceneText.match(/defaultPrim\s*=\s*"([^"]+)"/)?.[1];
  if (!sceneName) throw new Error('Scene has no defaultPrim');
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(OpenText(bakeCableSceneUsdaSource(sceneText).source)), `/World/${sceneName}`));
  if (!checked.valid) throw new Error(checked.warnings.join('\n'));
  const world = new World();
  applyEntityPlan(world, buildEntityPlan(checked));
  registerSimulationSystems(world);
  const remote = world.systems.find(system => system instanceof RemoteSpoolSystem);
  remote._ensureAxisMapping(world);
  const dt = world.getResource('dt');
  if (dt !== .002) throw new Error('RRF requires 0.002 s physics steps');
  return { world, remote, dt };
}
