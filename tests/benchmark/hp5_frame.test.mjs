import fs from 'node:fs';
import test from 'node:test';
import assert from 'node:assert/strict';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { RigidBodyComponent, PositionComponent, RenderableComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { collectRigidGroupEdges } from '../../src/js/cable_joints_3d/render_system_3d.js';

for (const [name, count] of [['hp5', 4], ['hp5_pulley_reduced', 44]]) {
  test(`${name} renders a square with bearings connected only to their corner`, () => {
    const scene = fs.readFileSync(`public/usd_scenes/${name}.usda`, 'utf8');
    const { world } = createHeadlessWorld(scene, { lineLayeringEnabled: false });
    try {
      const body = world.getComponent(world.query([RigidBodyComponent])[0], RigidBodyComponent);
      assert.equal(body.members.length, count);
      const edges = collectRigidGroupEdges(count, body.renderSegments);
      assert.equal(edges.length, count);
      assert.deepEqual(edges.slice(0, 4), [[0,1], [1,2], [2,3], [3,0]]);
      for (const [corner, member] of edges.slice(4)) {
        assert(corner < 4);
        assert(member >= 4);
        const p = world.getComponent(body.members[member], PositionComponent).pos;
        const q = world.getComponent(body.members[corner], PositionComponent).pos;
        assert(Math.hypot(p.x-q.x, p.y-q.y) < .05);
        assert(world.getComponent(body.members[member], RenderableComponent));
      }
    } finally {
      world.getResource('cableSolverBackend')?.dispose();
    }
  });
}
