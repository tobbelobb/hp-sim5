import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
  PositionComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  SpoolStateComponent,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
import { RigidBodySyncSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';

describe('spool axis handling in RigidBodySyncSystem', () => {
  test('removes off-axis tilt for standalone spools while preserving twist', () => {
    const world = new World();
    const spoolEntity = world.createEntity();
    const swing = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 5);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);
    const orientation = new Quaternion().multiplyQuaternions(swing, twist).normalize();

    world.addComponent(spoolEntity, new SpoolStateComponent('A'));
    world.addComponent(
      spoolEntity,
      new OrientationComponent(orientation.x, orientation.y, orientation.z, orientation.w),
    );
    world.addComponent(spoolEntity, new AngularVelocityComponent(1.0, -2.0, 3.0));

    const system = new RigidBodySyncSystem();
    system.update(world, 1 / 500);

    const spoolState = world.getComponent(spoolEntity, SpoolStateComponent);
    const updatedOrientation = world.getComponent(spoolEntity, OrientationComponent);
    const angularVelocity = world.getComponent(spoolEntity, AngularVelocityComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrientation.quaternion);

    expect(worldAxis.x).toBeCloseTo(0.0, 12);
    expect(worldAxis.y).toBeCloseTo(0.0, 12);
    expect(worldAxis.z).toBeCloseTo(1.0, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrientation.quaternion)).toBeCloseTo(Math.PI / 3, 12);
    expect(angularVelocity.omega.x).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.y).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.z).toBeCloseTo(3.0, 12);
  });

  test('removes off-axis spool tilt while preserving twist and axis-aligned angular velocity', () => {
    const world = new World();
    const bodyEntity = world.createEntity();
    const spoolEntity = world.createEntity();
    const orientation = new OrientationComponent();
    const swing = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 5);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);
    orientation.quaternion.multiplyQuaternions(swing, twist).normalize();

    world.addComponent(bodyEntity, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(bodyEntity, new OrientationComponent());
    world.addComponent(bodyEntity, new RigidBodyComponent([spoolEntity]));

    world.addComponent(spoolEntity, new SpoolStateComponent('A'));
    world.addComponent(spoolEntity, orientation);
    world.addComponent(spoolEntity, new AngularVelocityComponent(1.0, -2.0, 3.0));
    world.addComponent(
      spoolEntity,
      new RigidBodyMemberComponent(
        bodyEntity,
        new Vector3(0.0, 0.0, 0.0),
        orientation.quaternion.clone(),
      ),
    );

    const system = new RigidBodySyncSystem();
    system.update(world, 1 / 500);

    const spoolState = world.getComponent(spoolEntity, SpoolStateComponent);
    const updatedOrientation = world.getComponent(spoolEntity, OrientationComponent);
    const angularVelocity = world.getComponent(spoolEntity, AngularVelocityComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrientation.quaternion);

    expect(worldAxis.x).toBeCloseTo(0.0, 12);
    expect(worldAxis.y).toBeCloseTo(0.0, 12);
    expect(worldAxis.z).toBeCloseTo(1.0, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrientation.quaternion)).toBeCloseTo(Math.PI / 3, 12);
    expect(angularVelocity.omega.x).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.y).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.z).toBeCloseTo(3.0, 12);
  });

  test('keeps a spool aligned with its host rigid body rotation', () => {
    const world = new World();
    const bodyEntity = world.createEntity();
    const spoolEntity = world.createEntity();
    const rigidTilt = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 5);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);

    world.addComponent(bodyEntity, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(bodyEntity, new OrientationComponent(rigidTilt.x, rigidTilt.y, rigidTilt.z, rigidTilt.w));
    world.addComponent(bodyEntity, new RigidBodyComponent([spoolEntity]));

    const memberOrientation = new Quaternion().multiplyQuaternions(rigidTilt, twist).normalize();
    world.addComponent(spoolEntity, new SpoolStateComponent('A'));
    world.addComponent(
      spoolEntity,
      new OrientationComponent(memberOrientation.x, memberOrientation.y, memberOrientation.z, memberOrientation.w),
    );
    world.addComponent(spoolEntity, new AngularVelocityComponent(0.0, 0.0, 3.0));
    world.addComponent(
      spoolEntity,
      new RigidBodyMemberComponent(
        bodyEntity,
        new Vector3(0.0, 0.0, 0.0),
        twist.clone(),
      ),
    );

    const system = new RigidBodySyncSystem();
    system.update(world, 1 / 500);

    const spoolState = world.getComponent(spoolEntity, SpoolStateComponent);
    const updatedOrientation = world.getComponent(spoolEntity, OrientationComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrientation.quaternion);
    const expectedWorldAxis = rigidTilt.transformVector(new Vector3(0.0, 0.0, 1.0)).normalize();

    expect(worldAxis.x).toBeCloseTo(expectedWorldAxis.x, 12);
    expect(worldAxis.y).toBeCloseTo(expectedWorldAxis.y, 12);
    expect(worldAxis.z).toBeCloseTo(expectedWorldAxis.z, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrientation.quaternion)).toBeCloseTo(Math.PI / 3, 12);
  });
});
