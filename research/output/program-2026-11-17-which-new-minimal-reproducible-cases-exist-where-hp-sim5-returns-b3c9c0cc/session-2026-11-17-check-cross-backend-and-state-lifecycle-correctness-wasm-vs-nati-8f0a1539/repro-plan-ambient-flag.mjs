// Finding 2b: applying the same scene plan must produce the same world.
//
// A CablePathComponent's initial hybrid-knot angle is derived from
// path.stored using `layeringEnabled(world) ? cableHalfWidth : 0` at apply time.
// The plan that produced `stored` is not consulted. So the identical layered
// plan applied to two worlds with different ambient `enableLayering` resources
// yields different initial physical state, and the difference does not decay
// (7.2 cm body position after 500 steps).
//
// The production toggle path sets the flag before re-baking and re-applying, so
// it is not user-visible from the UI. This repro fails whenever plan application
// is not self-contained, which is the invariant a reset/rebuild API needs.
//
// Exit code 1 with the measured knot-angle and position differences.
import { state, firstExactDifference, sceneText, makeWorld, planFor, rebuild } from './lib.mjs';
import { setLineLayeringFeatureFlags } from '../../../../hp-sim-3d/app/line-layering-flags.js';
import { PositionComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

const STEPS = 500;
const hp3 = sceneText('hp3');
const layeredPlan = planFor(hp3); // half width 0.001 m in its CablePathComponents
const entityOnly = list => list.filter(([key]) => !key.startsWith('r.'));
const step = (instance, n) => { for (let i = 0; i < n; i++) instance.world.update(instance.dt); };
const maxPositionDiff = (a, b) => {
  let max = 0;
  for (const id of a.world.query([PositionComponent])) {
    const pa = a.world.getComponent(id, PositionComponent)?.pos;
    const pb = b.world.getComponent(id, PositionComponent)?.pos;
    if (pa && pb) max = Math.max(max, Math.hypot(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z));
  }
  return max;
};

// Both worlds end with the identical layered plan applied and the identical
// resource value (enableLayering = true). They differ only in the order:
// flag-before-apply (production) versus apply-before-flag.
const flagFirst = makeWorld(hp3);
step(flagFirst, 200);
setLineLayeringFeatureFlags(flagFirst.world, true);
flagFirst.world.getResource('cableSolverBackend')?.invalidate?.();
rebuild(flagFirst.world, layeredPlan);

const applyFirst = makeWorld(hp3, { lineLayeringEnabled: false });
step(applyFirst, 200);
applyFirst.world.getResource('cableSolverBackend')?.invalidate?.();
rebuild(applyFirst.world, layeredPlan);
setLineLayeringFeatureFlags(applyFirst.world, true);

step(flagFirst, STEPS);
step(applyFirst, STEPS);

const diff = firstExactDifference(entityOnly(state(flagFirst.world)), entityOnly(state(applyFirst.world)));
const position = maxPositionDiff(flagFirst, applyFirst);
console.log(`same layered plan and same final enableLayering=true; only apply/flag order differs:`);
console.log(`  first entity difference: ${JSON.stringify(diff)}`);
console.log(`  max body position difference after ${STEPS} steps: ${position.toExponential(3)} m`);

for (const world of [flagFirst, applyFirst]) world.world.getResource('cableSolverBackend')?.dispose?.();

if (diff) {
  console.error('FAIL scene application depends on the ambient enableLayering resource at apply time: ' +
    `${diff.key} differs by ${Math.abs((diff.left ?? 0) - (diff.right ?? 0)).toExponential(3)}, ` +
    `body position by ${position.toFixed(4)} m after ${STEPS} steps`);
  process.exit(1);
}
console.log('PASS scene plan application is self-contained');
