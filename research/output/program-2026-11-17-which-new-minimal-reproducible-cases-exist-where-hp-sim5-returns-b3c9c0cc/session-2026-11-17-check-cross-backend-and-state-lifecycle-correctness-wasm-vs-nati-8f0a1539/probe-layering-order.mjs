// Diagnostic: initial hybrid-knot angles depend on the *current* layering
// resource, not on the layering option that produced the plan being applied.
//
// The production toggle order is: set the flag, then re-bake and re-apply the
// scene (featureFlagsController.setLineLayeringEnabledState -> onLineLayeringChanged
// -> rebuildScene -> refreshBakedStage). This script asserts that order restores
// a fresh world exactly, and reports what a stale flag at apply time costs.
//
// Exit code 1 only if the production order fails to reproduce a fresh world.
import { state, firstExactDifference, sceneText, makeWorld, planFor, rebuild } from './lib.mjs';
import { setLineLayeringFeatureFlags } from '../../../../hp-sim-3d/app/line-layering-flags.js';
import { CablePathComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

const STEPS = 500;
const hp3 = sceneText('hp3');
const layeredPlan = planFor(hp3);
const unlayeredPlan = planFor(hp3, { lineLayeringEnabled: false });

function step(instance, n) { for (let i = 0; i < n; i++) instance.world.update(instance.dt); }

function maxPositionDiff(a, b) {
  let max = 0;
  for (const id of a.world.query([PositionComponent])) {
    const pa = a.world.getComponent(id, PositionComponent)?.pos;
    const pb = b.world.getComponent(id, PositionComponent)?.pos;
    if (pa && pb) max = Math.max(max, Math.hypot(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z));
  }
  return max;
}

function freshLayered(steps = STEPS) {
  const world = makeWorld(hp3);
  step(world, steps);
  return world;
}

function rebuiltLayered({ staleFlag, steps = STEPS }) {
  const world = makeWorld(hp3, { lineLayeringEnabled: false });
  step(world, 200);
  if (!staleFlag) setLineLayeringFeatureFlags(world.world, true);
  world.world.getResource('cableSolverBackend')?.invalidate?.();
  rebuild(world.world, layeredPlan);
  if (staleFlag) setLineLayeringFeatureFlags(world.world, true);
  step(world, steps);
  return world;
}

const widthProbe = makeWorld(hp3);
const widths = [...new Set(widthProbe.world.query([CablePathComponent])
  .map(id => widthProbe.world.getComponent(id, CablePathComponent).cableHalfWidth))];
widthProbe.world.getResource('cableSolverBackend')?.dispose?.();
console.log(`hp3 cableHalfWidth (layered plan): ${JSON.stringify(widths)}`);

const entityOnly = list => list.filter(([key]) => !key.startsWith('r.'));
const reference = freshLayered();
const production = rebuiltLayered({ staleFlag: false });
const productionDiff = firstExactDifference(entityOnly(state(reference.world)), entityOnly(state(production.world)));
if (productionDiff) {
  console.error(`FAIL production toggle order does not reproduce a fresh layered world: ${JSON.stringify(productionDiff)}`);
  process.exit(1);
}
console.log(`ok   production order (flag set before apply): exactly equal after ${STEPS} steps, max position difference 0 m`);

const stale = rebuiltLayered({ staleFlag: true });
const staleDiff = firstExactDifference(entityOnly(state(reference.world)), entityOnly(state(stale.world)));
console.log(`info stale flag at apply time: first difference ${JSON.stringify(staleDiff)}`);
console.log(`info stale flag at apply time: max body position difference ${maxPositionDiff(reference, stale).toExponential(3)} m`);
console.log(`info knot angle that differs: ${staleDiff?.key}`);

const longReference = freshLayered(2000);
const longStale = rebuiltLayered({ staleFlag: true, steps: 2000 });
console.log(`info after 2000 steps: max body position difference ${maxPositionDiff(longReference, longStale).toExponential(3)} m`);

for (const world of [reference, production, stale, longReference, longStale]) {
  world.world.getResource('cableSolverBackend')?.dispose?.();
}
console.log('PASS layering order: production toggle order is exact; stale-flag ordering is a measurable API hazard only');
