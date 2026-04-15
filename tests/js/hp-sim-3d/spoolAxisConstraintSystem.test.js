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
});
