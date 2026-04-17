import {
  World,
  PositionComponent,
  OrientationComponent,
  RigidBodyComponent,
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
    const expectedOrientation = new Quaternion(0.1, 0.2, 0.3, 0.9).normalize();
    expect(link.prevCableAttachmentTimePos.x).toBeCloseTo(1);
    expect(link.prevCableAttachmentTimePos.y).toBeCloseTo(2);
    expect(link.prevCableAttachmentTimePos.z).toBeCloseTo(3);
    expect(link.prevCableAttachmentTimeOrientation.x).toBeCloseTo(expectedOrientation.x);
    expect(link.prevCableAttachmentTimeOrientation.y).toBeCloseTo(expectedOrientation.y);
    expect(link.prevCableAttachmentTimeOrientation.z).toBeCloseTo(expectedOrientation.z);
    expect(link.prevCableAttachmentTimeOrientation.w).toBeCloseTo(expectedOrientation.w);
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
    const expectedWorldOrientation = new Quaternion(0.2, 0.3, 0.4, 0.8).normalize();
    expect(link.prevCableAttachmentTimeOrientation.x).toBeCloseTo(expectedWorldOrientation.x);
    expect(link.prevCableAttachmentTimeOrientation.y).toBeCloseTo(expectedWorldOrientation.y);
    expect(link.prevCableAttachmentTimeOrientation.z).toBeCloseTo(expectedWorldOrientation.z);
    expect(link.prevCableAttachmentTimeOrientation.w).toBeCloseTo(expectedWorldOrientation.w);
    expect(link.prevCableAttachmentTimeLocalOrientation.x).toBeCloseTo(localOrientation.x);
    expect(link.prevCableAttachmentTimeLocalOrientation.y).toBeCloseTo(localOrientation.y);
    expect(link.prevCableAttachmentTimeLocalOrientation.z).toBeCloseTo(localOrientation.z);
    expect(link.prevCableAttachmentTimeLocalOrientation.w).toBeCloseTo(localOrientation.w);
  });

  test('caches rigid-body member world pose from the host body, not stale member components', () => {
    const world = new World();
    const body = world.createEntity();
    const member = world.createEntity();
    const system = new CableAttachmentCacheSystem();

    const bodyOrientation = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
    const localPosition = new Vector3(1, 0, 0);
    const localOrientation = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 7);

    world.addComponent(body, new PositionComponent(0, 0, 0));
    world.addComponent(body, new OrientationComponent());
    world.getComponent(body, OrientationComponent).quaternion.set(bodyOrientation);
    world.addComponent(body, new RigidBodyComponent([member]));

    world.addComponent(member, new PositionComponent(1, 0, 0));
    world.addComponent(member, new OrientationComponent());
    world.addComponent(member, new CableLinkComponent());
    world.addComponent(
      member,
      new RigidBodyMemberComponent(
        body,
        localPosition,
        localOrientation,
      ),
    );

    system.update(world, 0.016);

    const link = world.getComponent(member, CableLinkComponent);
    const expectedWorldPos = bodyOrientation.transformVector(localPosition);
    const expectedWorldOrientation = new Quaternion()
      .multiplyQuaternions(bodyOrientation, localOrientation)
      .normalize();

    expect(link.prevCableAttachmentTimePos.x).toBeCloseTo(expectedWorldPos.x);
    expect(link.prevCableAttachmentTimePos.y).toBeCloseTo(expectedWorldPos.y);
    expect(link.prevCableAttachmentTimePos.z).toBeCloseTo(expectedWorldPos.z);
    expect(link.prevCableAttachmentTimeOrientation.x).toBeCloseTo(expectedWorldOrientation.x);
    expect(link.prevCableAttachmentTimeOrientation.y).toBeCloseTo(expectedWorldOrientation.y);
    expect(link.prevCableAttachmentTimeOrientation.z).toBeCloseTo(expectedWorldOrientation.z);
    expect(link.prevCableAttachmentTimeOrientation.w).toBeCloseTo(expectedWorldOrientation.w);
    expect(link.prevCableAttachmentTimeLocalOrientation.x).toBeCloseTo(localOrientation.x);
    expect(link.prevCableAttachmentTimeLocalOrientation.y).toBeCloseTo(localOrientation.y);
    expect(link.prevCableAttachmentTimeLocalOrientation.z).toBeCloseTo(localOrientation.z);
    expect(link.prevCableAttachmentTimeLocalOrientation.w).toBeCloseTo(localOrientation.w);
  });
});
