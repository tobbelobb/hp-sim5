import {
  World,
  PositionComponent,
  OrientationComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

import { CableLinkComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints_3d/cable_attachment_cache_system.js';

describe('CableAttachmentCacheSystem (3D)', () => {
  test('caches position and orientation onto CableLinkComponent', () => {
    const world = new World();
    const entity = world.createEntity();
    const system = new CableAttachmentCacheSystem();

    world.addComponent(entity, new PositionComponent(1, 2, 3));
    world.addComponent(entity, new OrientationComponent(0.1, 0.2, 0.3, 0.9));
    world.addComponent(entity, new CableLinkComponent());

    system.update(world, 0.016);

    const link = world.getComponent(entity, CableLinkComponent);
    expect(link.prevCableAttachmentTimePos.x).toBeCloseTo(1);
    expect(link.prevCableAttachmentTimePos.y).toBeCloseTo(2);
    expect(link.prevCableAttachmentTimePos.z).toBeCloseTo(3);
    expect(link.prevCableAttachmentTimeOrientation.x).toBeCloseTo(0.1);
    expect(link.prevCableAttachmentTimeOrientation.y).toBeCloseTo(0.2);
    expect(link.prevCableAttachmentTimeOrientation.z).toBeCloseTo(0.3);
    expect(link.prevCableAttachmentTimeOrientation.w).toBeCloseTo(0.9);
  });

  test('caches position even if OrientationComponent missing', () => {
    const world = new World();
    const entity = world.createEntity();
    const system = new CableAttachmentCacheSystem();

    world.addComponent(entity, new PositionComponent(-3, 4, -5));
    world.addComponent(entity, new CableLinkComponent());

    system.update(world, 0.016);

    const link = world.getComponent(entity, CableLinkComponent);
    expect(link.prevCableAttachmentTimePos.x).toBeCloseTo(-3);
    expect(link.prevCableAttachmentTimePos.y).toBeCloseTo(4);
    expect(link.prevCableAttachmentTimePos.z).toBeCloseTo(-5);
  });

  test('caches rigid-body member local orientation separately from world orientation', () => {
    const world = new World();
    const entity = world.createEntity();
    const system = new CableAttachmentCacheSystem();
    const localOrientation = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 7);

    world.addComponent(entity, new PositionComponent(0, 0, 0));
    world.addComponent(entity, new OrientationComponent(0.2, 0.3, 0.4, 0.8));
    world.addComponent(entity, new CableLinkComponent());
    world.addComponent(
      entity,
      new RigidBodyMemberComponent(
        123,
        new Vector3(0, 0, 0),
        localOrientation,
      ),
    );

    system.update(world, 0.016);

    const link = world.getComponent(entity, CableLinkComponent);
    expect(link.prevCableAttachmentTimeOrientation.x).toBeCloseTo(0.2);
    expect(link.prevCableAttachmentTimeOrientation.y).toBeCloseTo(0.3);
    expect(link.prevCableAttachmentTimeOrientation.z).toBeCloseTo(0.4);
    expect(link.prevCableAttachmentTimeOrientation.w).toBeCloseTo(0.8);
    expect(link.prevCableAttachmentTimeLocalOrientation.x).toBeCloseTo(localOrientation.x);
    expect(link.prevCableAttachmentTimeLocalOrientation.y).toBeCloseTo(localOrientation.y);
    expect(link.prevCableAttachmentTimeLocalOrientation.z).toBeCloseTo(localOrientation.z);
    expect(link.prevCableAttachmentTimeLocalOrientation.w).toBeCloseTo(localOrientation.w);
  });
});
