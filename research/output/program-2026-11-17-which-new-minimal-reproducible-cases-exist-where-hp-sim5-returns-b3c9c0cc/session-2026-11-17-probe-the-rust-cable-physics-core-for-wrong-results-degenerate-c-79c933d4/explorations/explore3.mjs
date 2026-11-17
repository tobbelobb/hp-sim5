import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
const scene = process.argv[2] ?? 'hp4';
const src = fs.readFileSync(new URL(`../../../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
const { world, dt, remote } = createHeadlessWorld(src, { lineLayeringEnabled: true });
const pathIds = world.query([CablePathComponent]);
const ids = world.query([PositionComponent]);
const p0 = ids.map(id => world.getComponent(id, PositionComponent).pos).map(p => [p.x, p.y, p.z]);
for (let i = 0; i < 300; i++) world.update(dt);
const p1 = ids.map(id => world.getComponent(id, PositionComponent).pos).map(p => [p.x, p.y, p.z]);
let move = 0, at = -1;
ids.forEach((id, k) => { const d = Math.hypot(p1[k][0] - p0[k][0], p1[k][1] - p0[k][1], p1[k][2] - p0[k][2]); if (d > move) { move = d; at = k; } });
const tensions = pathIds.flatMap(id => world.getComponent(id, CablePathComponent).jointEntities
  .map(j => world.getComponent(j, CableJointComponent).constraintForceMagnitude));
console.log(scene, 'dt', dt, 'max move', move.toFixed(6), 'at entity', ids[at],
  'max tension', Math.max(...tensions).toFixed(3), 'min tension', Math.min(...tensions).toFixed(3),
  'rests', pathIds.map(id => world.getComponent(id, CablePathComponent).jointEntities.map(j => world.getComponent(j, CableJointComponent).restLength.toFixed(4)).join(',')));
