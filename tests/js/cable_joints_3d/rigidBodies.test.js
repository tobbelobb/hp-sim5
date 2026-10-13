import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import {
  World, PositionComponent, OrientationComponent, RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  getEntityWorldPosition, getEntityWorldOrientation,
  computeWorldAttachment, computeLocalAttachment,
  createAttachmentTransformScratch, resolveRigidBodySolverEndpoint,
} from '../../../src/js/cable_joints_3d/rigid_bodies.js';

describe('rigid-body transform destinations', () => {
  function makeWorld(parentQuaternion = new Quaternion(0.2, -0.3, 0.4, 0.5)) {
    const world = new World();
    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(1.1, -2.2, 3.3));
    world.addComponent(body, new OrientationComponent());
    world.getComponent(body, OrientationComponent).quaternion.set(parentQuaternion);
    const member = world.createEntity();
    world.addComponent(member, new PositionComponent(99, 99, 99));
    world.addComponent(member, new OrientationComponent());
    world.addComponent(member, new RigidBodyMemberComponent(
      body, new Vector3(0.3, -0.2, 0.1), new Quaternion(-0.1, 0.3, 0.2, 0.8),
    ));
    return { world, body, member };
  }

  test.each([new Quaternion(0.2, -0.3, 0.4, 0.5), new Quaternion(0, 0, 0, 0)])(
    'composes live member poses using raw parent position rotation and normalized orientation (%p)', q => {
      const { world, body, member } = makeWorld(q);
      const bodyPosition = world.getComponent(body, PositionComponent).pos;
      const memberComponent = world.getComponent(member, RigidBodyMemberComponent);
      const before = JSON.stringify([bodyPosition, q, memberComponent]);
      const expectedPosition = bodyPosition.clone().add(q.transformVector(memberComponent.localPosition));
      const expectedOrientation = new Quaternion()
        .multiplyQuaternions(q, memberComponent.localOrientation).normalize();
      const outputPosition = new Vector3();
      const outputOrientation = new Quaternion();

      expect(getEntityWorldPosition(world, member, outputPosition)).toBe(outputPosition);
      expect(outputPosition).toEqual(expectedPosition);
      expect(getEntityWorldOrientation(world, member, outputOrientation)).toBe(outputOrientation);
      expect(outputOrientation).toEqual(expectedOrientation);
      expect(JSON.stringify([bodyPosition, q, memberComponent])).toBe(before);
      const detachedPosition = getEntityWorldPosition(world, member);
      const detachedOrientation = getEntityWorldOrientation(world, member);
      expect(detachedPosition).not.toBe(getEntityWorldPosition(world, member));
      expect(detachedOrientation).not.toBe(getEntityWorldOrientation(world, member));
      detachedPosition.scale(0);
      detachedOrientation.conjugate();
      expect(getEntityWorldPosition(world, member)).toEqual(expectedPosition);
      expect(getEntityWorldOrientation(world, member)).toEqual(expectedOrientation);
    },
  );

  test('reuses pose scratch while keeping endpoint results and retained local coordinates independent', () => {
    const { world, body, member } = makeWorld();
    const scratch = createAttachmentTransformScratch();
    const localA = new Vector3(0.4, -0.5, 0.6);
    const localB = new Vector3(-0.2, 0.7, 0.3);
    const position = getEntityWorldPosition(world, member);
    const orientation = getEntityWorldOrientation(world, member);
    const expectedA = position.clone().add(orientation.transformVector(localA));
    const expectedB = position.clone().add(orientation.transformVector(localB));
    const outputA = new Vector3();
    const outputB = new Vector3();
    const componentsBefore = JSON.stringify([...world.query([PositionComponent])].map(id => [
      world.getComponent(id, PositionComponent), world.getComponent(id, OrientationComponent),
      world.getComponent(id, RigidBodyMemberComponent),
    ]));

    expect(computeWorldAttachment(world, member, localA, outputA, scratch)).toBe(outputA);
    computeWorldAttachment(world, member, localB, outputB, scratch);
    expect(outputA).toEqual(expectedA);
    expect(outputB).toEqual(expectedB);
    const restoredA = computeLocalAttachment(world, member, outputA, new Vector3(), scratch);
    const restoredB = computeLocalAttachment(world, member, outputB, new Vector3(), scratch);
    ['x', 'y', 'z'].forEach(axis => {
      expect(restoredA[axis]).toBeCloseTo(localA[axis], 14);
      expect(restoredB[axis]).toBeCloseTo(localB[axis], 14);
    });
    expect(restoredA).not.toBe(restoredB);
    const alias = localA.clone();
    expect(computeWorldAttachment(world, member, alias, alias, scratch)).toBe(alias);
    expect(alias).toEqual(expectedA);
    expect(computeLocalAttachment(world, member, alias, alias, scratch)).toBe(alias);
    expect(alias).toEqual(restoredA);

    const mapped = resolveRigidBodySolverEndpoint(world, member, body, outputA, scratch);
    const retained = JSON.stringify(mapped);
    computeWorldAttachment(world, body, localB, outputB, scratch);
    computeLocalAttachment(world, member, outputB, new Vector3(), scratch);
    expect(JSON.stringify(mapped)).toBe(retained);
    expect(mapped.localPoint).not.toBe(scratch.position);
    expect(mapped.worldPoint).not.toBe(outputA);
    expect(JSON.stringify([...world.query([PositionComponent])].map(id => [
      world.getComponent(id, PositionComponent), world.getComponent(id, OrientationComponent),
      world.getComponent(id, RigidBodyMemberComponent),
    ]))).toBe(componentsBefore);
  });

  test('preserves absent and invalid pose fallbacks without leaking old scratch values', () => {
    const { world, body, member } = makeWorld();
    const scratch = createAttachmentTransformScratch();
    const output = new Vector3();
    const point = new Vector3(0.4, -0.5, 0.6);
    computeWorldAttachment(world, member, point, output, scratch);
    const empty = world.createEntity();
    expect(getEntityWorldPosition(world, empty, output)).toBeNull();
    expect(getEntityWorldOrientation(world, empty, scratch.orientation)).toBeNull();
    expect(computeWorldAttachment(world, empty, point, output, scratch)).toBe(output);
    expect(output).toEqual(point);
    expect(computeLocalAttachment(world, empty, point, output, scratch)).toEqual(point);
    world.getComponent(body, OrientationComponent).quaternion.w = NaN;
    const position = world.getComponent(body, PositionComponent).pos;
    expect(getEntityWorldOrientation(world, body, scratch.orientation)).toBeNull();
    expect(computeWorldAttachment(world, body, point, output, scratch)).toEqual(position.clone().add(point));
    expect(computeLocalAttachment(world, body, point, output, scratch)).toEqual(point.clone().subtract(position));
    expect(computeWorldAttachment(world, body, null, output, scratch)).toBeNull();
    expect(computeLocalAttachment(world, body, null, output, scratch)).toBeNull();
    const detached = computeWorldAttachment(world, empty, point);
    expect(detached).not.toBe(point);
    expect(detached).not.toBe(computeWorldAttachment(world, empty, point));
  });
});
