import {
  World,
  PositionComponent,
  OrientationComponent,
  MassComponent,
  MomentOfInertiaComponent,
  DistanceConstraintComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { XPBDDistanceConstraintSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';
import { getEntityWorldPosition } from '../../../src/js/cable_joints_3d/rigid_bodies.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

describe('XPBDDistanceConstraintSystem rigid-body endpoint mapping', () => {
  test('maps external member constraints onto the host rigid body', () => {
    const world = new World();

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(2.0));
    world.addComponent(body, new MomentOfInertiaComponent(1.0));

    const member = world.createEntity();
    world.addComponent(member, new PositionComponent(1.0, 0.0, 0.0));
    world.addComponent(member, new MassComponent(0.0));
    world.addComponent(
      member,
      new RigidBodyMemberComponent(
        body,
        new Vector3(1.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([member]));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(3.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const constraintEntity = world.createEntity();
    world.addComponent(
      constraintEntity,
      new DistanceConstraintComponent(member, anchor, 1.0, 0.0),
    );

    const system = new XPBDDistanceConstraintSystem();
    system.update(world, 1.0);

    expect(world.getComponent(body, PositionComponent).pos.x).toBeCloseTo(1.0, 12);
    expect(getEntityWorldPosition(world, member).x).toBeCloseTo(2.0, 12);
  });

  test('uses host body rotation when a constrained member is offset from the center of mass', () => {
    const world = new World();

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(2.0));
    world.addComponent(body, new MomentOfInertiaComponent(1.0));

    const member = world.createEntity();
    world.addComponent(member, new PositionComponent(1.0, 0.0, 0.0));
    world.addComponent(member, new MassComponent(0.0));
    world.addComponent(
      member,
      new RigidBodyMemberComponent(
        body,
        new Vector3(1.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([member]));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(2.0, 1.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const constraintEntity = world.createEntity();
    world.addComponent(
      constraintEntity,
      new DistanceConstraintComponent(member, anchor, 0.2, 0.0),
    );

    const initialDistance = getEntityWorldPosition(world, member).distanceTo(
      world.getComponent(anchor, PositionComponent).pos,
    );

    const system = new XPBDDistanceConstraintSystem();
    system.update(world, 1.0);

    const bodyPosition = world.getComponent(body, PositionComponent).pos;
    const bodyOrientation = world.getComponent(body, OrientationComponent).quaternion;
    const finalDistance = getEntityWorldPosition(world, member).distanceTo(
      world.getComponent(anchor, PositionComponent).pos,
    );

    expect(bodyPosition.x).toBeGreaterThan(0.0);
    expect(bodyPosition.y).toBeGreaterThan(0.0);
    expect(Math.abs(bodyOrientation.z) + Math.abs(bodyOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(finalDistance).toBeLessThan(initialDistance);
  });
});
