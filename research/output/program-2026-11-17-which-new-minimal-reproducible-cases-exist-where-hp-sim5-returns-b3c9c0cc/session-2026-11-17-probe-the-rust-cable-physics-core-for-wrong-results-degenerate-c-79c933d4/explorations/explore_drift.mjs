// Exploratory: does an uncommanded scene creep, or settle?
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../../hp-sim-3d/app/headlessWorld.js';
import { PositionComponent, VelocityComponent } from '../../../../../src/js/cable_joints_3d/ecs.js';

const scene = process.argv[2] ?? 'hp3';
const src = fs.readFileSync(new URL(`../../../../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
const { world, dt } = createHeadlessWorld(src, { lineLayeringEnabled: true });
const ids = world.query([PositionComponent]);
const p0 = ids.map(id => world.getComponent(id, PositionComponent).pos).map(p => [p.x, p.y, p.z]);
const vids = world.query([VelocityComponent]);
const maxSpeed = () => Math.max(...vids.map(id => {
  const v = world.getComponent(id, VelocityComponent).vel;
  return Math.hypot(v.x, v.y, v.z);
}));
const maxDisp = () => Math.max(...ids.map((id, k) => {
  const p = world.getComponent(id, PositionComponent).pos;
  return Math.hypot(p.x - p0[k][0], p.y - p0[k][1], p.z - p0[k][2]);
}));
let t = 0;
for (let block = 0; block < 6; block++) {
  for (let i = 0; i < 1000; i++) { world.update(dt); t += dt; }
  console.log(scene, 't', t.toFixed(2), 'maxDisp_m', maxDisp().toExponential(3), 'maxSpeed_m_s', maxSpeed().toExponential(3));
}
