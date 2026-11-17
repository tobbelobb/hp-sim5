// Exploratory: production-scene invariants (finite, material conservation,
// non-negative tension) for the native Rust core.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent, OrientationComponent, EncoderComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

const scene = process.argv[2] ?? 'hp3';
const layering = (process.argv[3] ?? 'true') === 'true';
const src = fs.readFileSync(new URL(`../../../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
const { world, dt } = createHeadlessWorld(src, { lineLayeringEnabled: layering });
const pathIds = world.query([CablePathComponent]);
let worst = { material: 0, tension: 0, nan: 0 };
for (let step = 0; step < 300; step++) {
  world.update(dt);
  for (const id of pathIds) {
    const p = world.getComponent(id, CablePathComponent);
    const total = p.totalRestLength;
    const sum = p.stored.reduce((a, b) => a + b, 0)
      + p.jointEntities.reduce((a, j) => a + world.getComponent(j, CableJointComponent).restLength, 0);
    worst.material = Math.max(worst.material, Math.abs(sum - total));
    for (const j of p.jointEntities) {
      const c = world.getComponent(j, CableJointComponent);
      worst.tension = Math.min(worst.tension, c.constraintForceMagnitude);
      if (![c.restLength, c.constraintForceMagnitude, c.constraintLambda, ...p.stored].every(Number.isFinite)) worst.nan++;
      for (const k of ['attachmentPointA_world', 'attachmentPointB_world', 'constraintForce']) {
        if (![c[k].x, c[k].y, c[k].z].every(Number.isFinite)) worst.nan++;
      }
    }
    for (const k of ['linkTypes', 'cw', 'stored']) if (!p[k].every(x => typeof x === 'string' || Number.isFinite(x) || typeof x === 'boolean')) worst.nan++;
  }
}
console.log(scene, 'layering', layering, JSON.stringify(worst));
let bad = 0;
for (const id of world.query([PositionComponent])) {
  const p = world.getComponent(id, PositionComponent).pos;
  if (![p.x, p.y, p.z].every(Number.isFinite)) bad++;
}
for (const id of world.query([OrientationComponent])) {
  const q = world.getComponent(id, OrientationComponent).quaternion;
  if (![q.x, q.y, q.z, q.w].every(Number.isFinite)) bad++;
}
for (const id of world.query([EncoderComponent])) {
  if (!Number.isFinite(world.getComponent(id, EncoderComponent).angle)) bad++;
}
console.log('entities with non-finite pose/encoder:', bad);
