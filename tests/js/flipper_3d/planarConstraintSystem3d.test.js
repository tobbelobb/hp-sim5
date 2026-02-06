import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  AngularVelocityComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import { PlanarConstraintSystem3D } from '../../../examples/js/flipper_3d/flipper_common_3d.js';

function makeTiltedQuaternion() {
  const qx = new OrientationComponent().quaternion;
  const qy = new OrientationComponent().quaternion;
  const qz = new OrientationComponent().quaternion;

  qx.setFromAxisAngle(new Vector3(1, 0, 0), 0.35);
  qy.setFromAxisAngle(new Vector3(0, 1, 0), -0.28);
  qz.setFromAxisAngle(new Vector3(0, 0, 1), 0.6);

  const combined = qz.clone();
  combined.multiplyQuaternions(combined, qy).multiplyQuaternions(combined, qx).normalize();
  return combined;
}

describe('PlanarConstraintSystem3D', () => {
  test('projects point/velocity/angular state onto XY plane and locks orientation to Z axis', () => {
    const world = new World();
    const entity = world.createEntity();

    world.addComponent(entity, new PositionComponent(1.5, -0.8, 0.91));
    world.addComponent(entity, new PrevFinalPosComponent(2.0, 1.0, -0.41));
    world.addComponent(entity, new VelocityComponent(0.2, -0.5, 1.7));
    world.addComponent(entity, new AngularVelocityComponent(2.1, -4.0, 3.3));

    const orientation = new OrientationComponent();
    orientation.quaternion.set(makeTiltedQuaternion());
    world.addComponent(entity, orientation);

    const prevOrientation = new PrevFinalOrientationComponent();
    prevOrientation.quaternion.set(makeTiltedQuaternion());
    world.addComponent(entity, prevOrientation);

    const system = new PlanarConstraintSystem3D(new Vector3(0, 0, 1), 0.0);
    system.update(world, 1 / 120);

    const pos = world.getComponent(entity, PositionComponent).pos;
    const prevPos = world.getComponent(entity, PrevFinalPosComponent).pos;
    const vel = world.getComponent(entity, VelocityComponent).vel;
    const omega = world.getComponent(entity, AngularVelocityComponent).omega;

    expect(pos.z).toBeCloseTo(0.0, 12);
    expect(prevPos.z).toBeCloseTo(0.0, 12);
    expect(vel.z).toBeCloseTo(0.0, 12);
    expect(omega.x).toBeCloseTo(0.0, 12);
    expect(omega.y).toBeCloseTo(0.0, 12);
    expect(omega.z).toBeCloseTo(3.3, 12);

    const rotatedX = world.getComponent(entity, OrientationComponent).quaternion.transformVector(new Vector3(1, 0, 0));
    const rotatedPrevX = world.getComponent(entity, PrevFinalOrientationComponent).quaternion.transformVector(new Vector3(1, 0, 0));

    expect(rotatedX.z).toBeCloseTo(0.0, 12);
    expect(rotatedPrevX.z).toBeCloseTo(0.0, 12);
  });

  test('supports arbitrary plane offsets', () => {
    const world = new World();
    const entity = world.createEntity();

    world.addComponent(entity, new PositionComponent(0, 0, 8.2));
    world.addComponent(entity, new PrevFinalPosComponent(0, 0, 4.3));
    world.addComponent(entity, new VelocityComponent(1, 1, -9));

    const system = new PlanarConstraintSystem3D(new Vector3(0, 0, 1), 2.5);
    system.update(world, 1 / 120);

    const pos = world.getComponent(entity, PositionComponent).pos;
    const prevPos = world.getComponent(entity, PrevFinalPosComponent).pos;
    const vel = world.getComponent(entity, VelocityComponent).vel;

    expect(pos.z).toBeCloseTo(2.5, 12);
    expect(prevPos.z).toBeCloseTo(2.5, 12);
    expect(vel.z).toBeCloseTo(0.0, 12);
  });
});
