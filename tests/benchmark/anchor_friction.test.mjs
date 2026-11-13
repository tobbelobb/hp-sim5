import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { CoefficientOfFrictionComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';

// Lines thread through Anchor entities, so friction authored on their material
// must reach the cable solver like a Pinhole's does.
test('Anchor entities carry their material friction', () => {
  const source = fs.readFileSync(new URL('../../public/usd_scenes/hp3.usda', import.meta.url), 'utf8');
  const anchors = source.split('ecs:tags = ["Anchor"]').length - 1;
  const authored = source.replace(
    /(def Material "AnchorMaterial" \([^{]*\{)/,
    '$1\n            float physics:staticFriction = 0.125\n            float physics:dynamicFriction = 0.125',
  );
  assert.notEqual(authored, source);
  const count = (text) => {
    const { world } = createHeadlessWorld(text, { requireRrfTimestep: false });
    return world.query([CoefficientOfFrictionComponent])
      .filter(e => world.getComponent(e, CoefficientOfFrictionComponent).mu === 0.125).length;
  };
  assert.equal(count(source), 0);
  assert.equal(count(authored), anchors);
});
