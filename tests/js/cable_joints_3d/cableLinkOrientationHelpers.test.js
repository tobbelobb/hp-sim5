import {
  World,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  _deltaAngleForEntity,
  _orientationAngleForEntity,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

function rotationX(angle) {
  return new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), angle);
}

function rotationY(angle) {
  return new Quaternion().setFromAxisAngle(new Vector3(0.0, 1.0, 0.0), angle);
}

describe('local-axis cable link orientation helpers', () => {
  test('use rigid-body-member local orientation for local-axis delta and absolute angles', () => {
    const world = new World();
    const body = world.createEntity();
    const spool = world.createEntity();
    const axisLocal = new Vector3(0.0, 0.0, 1.0);

    world.addComponent(spool, new CableLinkComponent(0.0, 0.0, 0.0, null, null, axisLocal));
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(0.0, 0.0, 0.0),
        new Quaternion(),
      ),
    );

    const prevWorld = new Quaternion().multiplyQuaternions(rotationY(1.0), rotationX(0.7)).normalize();
    const currWorld = new Quaternion().multiplyQuaternions(rotationX(0.7), rotationY(1.0)).normalize();
    const prevLocal = new Quaternion();
    const currLocal = new Quaternion();

    const worldFrameDelta = _deltaAngleForEntity(world, spool, prevWorld, currWorld);
    const localFrameDelta = _deltaAngleForEntity(
      world,
      spool,
      prevWorld,
      currWorld,
      prevLocal,
      currLocal,
    );
    const worldFrameAngle = _orientationAngleForEntity(world, spool, currWorld);
    const localFrameAngle = _orientationAngleForEntity(world, spool, currWorld, currLocal);

    expect(Math.abs(worldFrameDelta)).toBeGreaterThan(0.5);
    expect(localFrameDelta).toBeCloseTo(0.0, 12);
    expect(Math.abs(worldFrameAngle)).toBeGreaterThan(0.3);
    expect(localFrameAngle).toBeCloseTo(0.0, 12);
  });
});
