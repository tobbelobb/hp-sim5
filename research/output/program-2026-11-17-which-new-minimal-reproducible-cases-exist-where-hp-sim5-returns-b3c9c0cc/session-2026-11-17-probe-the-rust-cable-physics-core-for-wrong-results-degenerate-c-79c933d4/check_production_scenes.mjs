// Negative-result checks on the repo's real headless scenes: the Rust core must
// keep every pose/encoder/path field finite, keep tension non-negative and
// conserve cable material (sum of segment rest lengths + stored wrap arc =
// CablePathComponent.totalRestLength) for every step.
//
// Exits nonzero only if an invariant is violated.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent, OrientationComponent, EncoderComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

const SCENES = ['hp3', 'hp4', 'hp4_worn_bearings', 'hp5', 'slideprinter_single_pinholes'];
const STEPS = Number(process.argv[2] ?? 300);
let failures = 0;
for (const scene of SCENES) {
  for (const layering of [true, false]) {
    const src = fs.readFileSync(new URL(`../../../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
    const { world, dt } = createHeadlessWorld(src, { lineLayeringEnabled: layering });
    const pathIds = world.query([CablePathComponent]);
    let maxMat = 0, minT = 0, maxT = 0, nonFinite = 0;
    for (let step = 0; step < STEPS; step++) {
      world.update(dt);
      for (const id of world.query([PositionComponent])) {
        const p = world.getComponent(id, PositionComponent).pos;
        if (![p.x, p.y, p.z].every(Number.isFinite)) nonFinite++;
      }
      for (const id of world.query([OrientationComponent])) {
        const q = world.getComponent(id, OrientationComponent).quaternion;
        if (![q.x, q.y, q.z, q.w].every(Number.isFinite)) nonFinite++;
      }
      for (const id of world.query([EncoderComponent])) {
        if (!Number.isFinite(world.getComponent(id, EncoderComponent).angle)) nonFinite++;
      }
      for (const id of pathIds) {
        const p = world.getComponent(id, CablePathComponent);
        if (!p.stored.every(Number.isFinite)) nonFinite++;
        if (p.stored.length !== p.linkTypes.length || p.cw.length !== p.linkTypes.length) nonFinite++;
        let sum = p.stored.reduce((a, b) => a + b, 0);
        for (const j of p.jointEntities) {
          const c = world.getComponent(j, CableJointComponent);
          if (![c.restLength, c.constraintLambda, c.constraintForceMagnitude].every(Number.isFinite)) nonFinite++;
          if (![c.constraintForce.x, c.constraintForce.y, c.constraintForce.z].every(Number.isFinite)) nonFinite++;
          if (c.constraintForceMagnitude < 0) minT = -1;
          maxT = Math.max(maxT, c.constraintForceMagnitude);
          sum += c.restLength;
        }
        maxMat = Math.max(maxMat, Math.abs(sum - p.totalRestLength)
          / (1 + Math.abs(p.totalRestLength)));
      }
    }
    const ok = nonFinite === 0 && minT >= 0 && maxMat < 1e-9;
    if (!ok) failures++;
    console.log(`${ok ? 'PASS' : 'FAIL'}  ${scene} layering=${layering}: steps=${STEPS}`
      + ` nonFinite=${nonFinite} minTension=${minT.toFixed(3)} maxTension=${maxT.toFixed(3)}`
      + ` maxRelativeMaterialDrift=${maxMat.toExponential(2)} (tol 1e-9)`);
  }
}
console.log(failures ? `\n${failures} scene invariant failure(s)` : '\nall scene invariants hold');
process.exit(failures ? 1 : 0);
