import {
  World,
  PositionComponent,
  OrientationComponent
} from '../../../src/js/cable_joints/ecs.js';

import { CableLinkComponent } from '../../../src/js/cable_joints/cable_joints_core.js';
import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints/cable_attachment_cache_system.js';

describe('CableAttachmentCacheSystem (2D)', () => {
  test('caches position and orientation onto CableLinkComponent', () => {
    const world = new World();
    const entity = world.createEntity();
    const system = new CableAttachmentCacheSystem();

    world.addComponent(entity, new PositionComponent(1, 2));
    world.addComponent(entity, new OrientationComponent(0.5));
    world.addComponent(entity, new CableLinkComponent());

    system.update(world, 0.016);

    const link = world.getComponent(entity, CableLinkComponent);
    expect(link.prevCableAttachmentTimePos.x).toBeCloseTo(1);
    expect(link.prevCableAttachmentTimePos.y).toBeCloseTo(2);
    expect(link.prevCableAttachmentTimeAngle).toBeCloseTo(0.5);
  });

  test('caches position even if OrientationComponent missing', () => {
    const world = new World();
    const entity = world.createEntity();
    const system = new CableAttachmentCacheSystem();

    world.addComponent(entity, new PositionComponent(-3, 4));
    world.addComponent(entity, new CableLinkComponent());

    system.update(world, 0.016);

    const link = world.getComponent(entity, CableLinkComponent);
    expect(link.prevCableAttachmentTimePos.x).toBeCloseTo(-3);
    expect(link.prevCableAttachmentTimePos.y).toBeCloseTo(4);
  });
});
