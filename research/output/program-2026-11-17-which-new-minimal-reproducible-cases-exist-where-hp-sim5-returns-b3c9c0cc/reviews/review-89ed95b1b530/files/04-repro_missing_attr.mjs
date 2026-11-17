// Remove one optional-looking spool attribute (physics:angularVelocity) from
// the shipped HP5 scene and load it through the production headless pipeline.
// Expected: the loader rejects the scene, or keeps the spool.
// Actual:   one console warning; the spool is skipped; two cable joints keep
//           entityA = undefined and their spool-end attachment is placed at the
//           spool-local origin (2.5 m from the real spool).  The first physics
//           step then throws an unrelated-looking "Missing cable endpoint".
import { readFileSync } from 'node:fs';
import { createHeadlessWorld } from '/home/torbjorn/repos/hp-sim5/hp-sim-3d/app/headlessWorld.js';
import { WasmPhysicsEngine } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/wasm_physics_engine.js';
import { CableJointComponent, CablePathComponent } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent, SceneEntityInfoComponent } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';

const ROOT = '/home/torbjorn/repos/hp-sim5';
const module = new WebAssembly.Module(readFileSync(`${ROOT}/public/wasm/cable_solver.wasm`));

const pristine = readFileSync(`${ROOT}/public/usd_scenes/hp5.usda`, 'utf8');
const spoolC = pristine.indexOf('def Circle "SpoolC"');
const blockEnd = pristine.indexOf('\n        }', spoolC);
const block = pristine.slice(spoolC, blockEnd);
const stripped = block.replace(/\n\s*float3 physics:angularVelocity = \([^)]*\)/, '');
if (stripped === block) throw new Error('probe expected to find physics:angularVelocity in SpoolC');
const patched = pristine.slice(0, spoolC) + stripped + pristine.slice(blockEnd);

const backend = new WasmPhysicsEngine(module);
let world;
let dt;
try {
  ({ world, dt } = createHeadlessWorld(patched, { physicsSubsteps: 1, cableSolverBackend: backend }));

  const byName = new Map();
  for (const id of world.query([SceneEntityInfoComponent])) {
    byName.set(world.getComponent(id, SceneEntityInfoComponent).name, id);
  }
  const spoolCId = byName.get('SpoolC');
  console.log('SpoolC entity:', spoolCId);

  const paths = world.query([CablePathComponent]);
  for (const id of paths) {
    const info = world.getComponent(id, SceneEntityInfoComponent);
    if (info.name !== 'CablePathCL') continue;
    const path = world.getComponent(id, CablePathComponent);
    console.log(`CablePathCL joints=${path.jointEntities.length} linkTypes=${path.linkTypes.length} `
                + `stored=${JSON.stringify(path.stored)} totalRestLength=${path.totalRestLength.toFixed(6)}`);
  }

  const orphans = world.query([CableJointComponent])
    .map((id) => ({ info: world.getComponent(id, SceneEntityInfoComponent), j: world.getComponent(id, CableJointComponent) }))
    .filter(({ j }) => j.entityA === undefined || j.entityB === undefined);
  const spoolCIntended = [0.0, 2.5, -0.18];
  let maxDisplacement = 0;
  for (const o of orphans) {
    const a = o.j.attachmentPointA_world;
    const displacement = Math.hypot(a.x - spoolCIntended[0], a.y - spoolCIntended[1], a.z - spoolCIntended[2]);
    maxDisplacement = Math.max(maxDisplacement, displacement);
    console.log(`orphan ${o.info.name}: entityA=${o.j.entityA} attachmentA=(${a.x.toFixed(4)}, ${a.y.toFixed(4)}, ${a.z.toFixed(4)}) `
                + `distance_from_spoolC_centre=${displacement.toFixed(4)} m`);
  }

  let stepError = null;
  try {
    world.update(dt);
  } catch (error) {
    stepError = error.message;
  }
  console.log('first physics step error:', stepError);

  const spoolSkipped = spoolCId === undefined;
  const dangling = orphans.length > 0;
  maybeReport(spoolSkipped, dangling, maxDisplacement, stepError);
} finally {
  backend.dispose?.();
}

function maybeReport(spoolSkipped, dangling, maxDisplacement, stepError) {
  let failed = false;
  if (spoolSkipped) {
    console.log('BUG: SpoolC was skipped by the loader');
    failed = true;
  }
  if (dangling) {
    console.log(`BUG: ${dangling ? 'cable joints' : ''} reference a nonexistent body; max cable-end displacement `
                + `${maxDisplacement.toFixed(3)} m`);
    failed = true;
  }
  if (stepError) {
    console.log(`BUG: scene loaded without error but the first step threw: ${stepError}`);
    failed = true;
  }
  if (failed) {
    console.log('FAIL: a missing non-geometric attribute silently removed a motor and left dangling joints');
    process.exit(1);
  }
  console.log('PASS');
}
