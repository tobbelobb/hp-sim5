import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  SpoolStateComponent,
  SpoolAxisConstraintSystem,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
  rotateSpoolReferenceOrientation,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('SpoolAxisConstraintSystem', () => {
  test('removes off-axis spool tilt while preserving twist and on-axis spin', () => {
    const world = new World();
    const spoolEntity = world.createEntity();
    const orientation = new OrientationComponent();
    const swing = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 5);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);
    orientation.quaternion.multiplyQuaternions(swing, twist).normalize();

    world.addComponent(spoolEntity, new SpoolStateComponent('A'));
    world.addComponent(spoolEntity, orientation);
    world.addComponent(spoolEntity, new AngularVelocityComponent(1.0, -2.0, 3.0));

    const system = new SpoolAxisConstraintSystem();
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

  test('preserves a rigid-group-updated spool axis', () => {
    const world = new World();
    const spoolEntity = world.createEntity();
    const orientation = new OrientationComponent();
    const spoolState = new SpoolStateComponent('A');
    const rigidTilt = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 5);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);
    rotateSpoolReferenceOrientation(spoolState, rigidTilt);
    orientation.quaternion.multiplyQuaternions(spoolState.referenceOrientation, twist).normalize();

    world.addComponent(spoolEntity, spoolState);
    world.addComponent(spoolEntity, orientation);
    world.addComponent(spoolEntity, new AngularVelocityComponent(0.0, 0.0, 3.0));

    const system = new SpoolAxisConstraintSystem();
    system.update(world, 1 / 500);

    const updatedOrientation = world.getComponent(spoolEntity, OrientationComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrientation.quaternion);
    const expectedWorldAxis = rigidTilt.transformVector(new Vector3(0.0, 0.0, 1.0)).normalize();

    expect(worldAxis.x).toBeCloseTo(expectedWorldAxis.x, 12);
    expect(worldAxis.y).toBeCloseTo(expectedWorldAxis.y, 12);
    expect(worldAxis.z).toBeCloseTo(expectedWorldAxis.z, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrientation.quaternion)).toBeCloseTo(Math.PI / 3, 12);
  });

  test('strips solver swing while preserving a previously rotated reference axis', () => {
    const world = new World();
    const spoolEntity = world.createEntity();
    const orientation = new OrientationComponent();
    const spoolState = new SpoolStateComponent('A');
    const rigidTilt = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 5);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);
    rotateSpoolReferenceOrientation(spoolState, rigidTilt);
    orientation.quaternion.multiplyQuaternions(spoolState.referenceOrientation, twist).normalize();

    world.addComponent(spoolEntity, spoolState);
    world.addComponent(spoolEntity, orientation);
    world.addComponent(spoolEntity, new AngularVelocityComponent(1.5, -0.75, 2.0));

    const solverSwingLocal = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 7);
    const twistOnly = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 3);
    orientation.quaternion.multiplyQuaternions(
      spoolState.referenceOrientation,
      new Quaternion().multiplyQuaternions(solverSwingLocal, twistOnly).normalize(),
    ).normalize();

    const system = new SpoolAxisConstraintSystem();
    system.update(world, 1 / 500);

    const updatedOrientation = world.getComponent(spoolEntity, OrientationComponent);
    const angularVelocity = world.getComponent(spoolEntity, AngularVelocityComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrientation.quaternion);
    const expectedWorldAxis = rigidTilt.transformVector(new Vector3(0.0, 0.0, 1.0)).normalize();

    expect(worldAxis.x).toBeCloseTo(expectedWorldAxis.x, 12);
    expect(worldAxis.y).toBeCloseTo(expectedWorldAxis.y, 12);
    expect(worldAxis.z).toBeCloseTo(expectedWorldAxis.z, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrientation.quaternion)).toBeCloseTo(Math.PI / 3, 6);
    expect(angularVelocity.omega.cross(worldAxis).length()).toBeCloseTo(0.0, 12);
  });
});
