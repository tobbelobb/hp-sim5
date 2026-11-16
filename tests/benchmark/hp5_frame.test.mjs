import fs from 'node:fs';
import test from 'node:test';
import assert from 'node:assert/strict';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { collectRigidGroupEdges } from '../../src/js/cable_joints_3d/render_system_3d.js';
import {
  parseRenderPolyline,
  resolveRenderPolylineMembers,
} from '../../hp-sim-3d/app/scene/usdValueReaders.js';

const CORNERS = [
  '/World/HangprinterScene/CornerSW',
  '/World/HangprinterScene/CornerSE',
  '/World/HangprinterScene/CornerNE',
  '/World/HangprinterScene/CornerNW',
];

for (const [name, count] of [['hp5', 4], ['hp5_pulley_reduced', 44]]) {
  test(`${name} renders only the square between the four corners`, () => {
    const scene = fs.readFileSync(`public/usd_scenes/${name}.usda`, 'utf8');
    const { world } = createHeadlessWorld(scene, { lineLayeringEnabled: false });
    try {
      const body = world.getComponent(world.query([RigidBodyComponent])[0], RigidBodyComponent);
      assert.equal(body.members.length, count);
      const edges = collectRigidGroupEdges(count, body.renderSegments);
      assert.deepEqual(edges, [[0, 1], [1, 2], [2, 3], [3, 0]]);
    } finally {
      world.getResource('cableSolverBackend')?.dispose();
    }
  });
}

test('render polyline accepts numbers, prim paths and mixed nested runs', () => {
  const inline = '[</World/A>, </World/B>, 1]';
  assert.deepEqual(parseRenderPolyline(inline), [['/World/A', '/World/B', 1]]);
  assert.deepEqual(parseRenderPolyline('[0,1,2,0]'), [[0, 1, 2, 0]]);
  assert.deepEqual(parseRenderPolyline('[[0,1], [</World/A>, </World/B>]]'), [
    [0, 1],
    ['/World/A', '/World/B'],
  ]);
  assert.deepEqual(parseRenderPolyline(['</World/A>', '</World/B>']), [
    ['/World/A', '/World/B'],
  ]);

  const members = ['/World/P', '/World/A', '/World/B'];
  assert.deepEqual(
    resolveRenderPolylineMembers([['/World/A', '/World/B'], [1, '/World/P']], members),
    [[1, 2], [1, 0]],
  );
  assert.equal(resolveRenderPolylineMembers([['/World/Missing']], members), null);
});
