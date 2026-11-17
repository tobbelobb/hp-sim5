// Diagnostic probe: load every catalog scene through the production pipeline and
// report entity/joint/path counts plus unresolved cable joints and duplicate names.
import { readFileSync } from 'node:fs';
import { World } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/cable_joints_core.js';
import { OpenText } from '/home/torbjorn/repos/hp-sim5/src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '/home/torbjorn/repos/hp-sim5/src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '/home/torbjorn/repos/hp-sim5/hp-sim-3d/app/scene/machineScenePipeline.js';
import { getChildren, getAttribute } from '/home/torbjorn/repos/hp-sim5/src/js/usd/stage.js';

const root = '/home/torbjorn/repos/hp-sim5';
const catalog = JSON.parse(readFileSync(`${root}/machines/catalog.json`, 'utf8'));

function load(scenePath) {
  const source = readFileSync(`${root}/${scenePath}`, 'utf8');
  const baked = bakeCableSceneUsdaSource(source);
  const stage = OpenText(baked.source);
  const defaultAssignment = (stage.ast?.descriptor?.assignments ?? []).find((a) => a.identifier === 'defaultPrim');
  const defaultPrim = defaultAssignment?.value;
  const sceneRootPath = defaultPrim && stage.GetPrimAtPath(`/World/${defaultPrim}`) ? `/World/${defaultPrim}` : `/${defaultPrim}`;
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), sceneRootPath, {}));
  const world = new World();
  applyEntityPlan(world, buildEntityPlan(checked, {}));
  // Count authored names under scene root and duplicate leaves.
  const rootPrim = stage.GetPrimAtPath(sceneRootPath);
  const names = getChildren(rootPrim).map((p) => p.name);
  const dup = names.filter((n, i) => names.indexOf(n) !== i);
  return { world, baked, stage, dup, sceneRootPath };
}

for (const scenePath of Object.keys(catalog.scenes)) {
  const { world, baked, dup } = load(scenePath);
  const paths = world.query([CablePathComponent]);
  const authoredPaths = baked.resolvedPaths.length;
  const issues = [];
  for (const id of paths) {
    const p = world.getComponent(id, CablePathComponent);
    if (!Array.isArray(p.jointEntities)) continue;
    const expected = p.linkTypes ? p.linkTypes.length - 1 : null;
    if (expected !== null && p.jointEntities.length !== expected) {
      issues.push(`path ${id}: ${p.jointEntities.length} joints vs linkTypes-1=${expected}`);
    }
  }
  console.log(JSON.stringify({
    scene: scenePath,
    authoredPaths,
    worldPaths: paths.length,
    duplicatePrimNames: dup,
    issues,
  }));
}
