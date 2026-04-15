import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';

import {
  World,
  PositionComponent,
  MassComponent,
  OrientationComponent,
  RigidGroupComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';

import { RigidGroupSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';
import {
  SpoolStateComponent,
  getSpoolWorldAxis,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

function expectVectorClose(actual, expected, digits = 8) {
  expect(actual.x).toBeCloseTo(expected.x, digits);
  expect(actual.y).toBeCloseTo(expected.y, digits);
  expect(actual.z).toBeCloseTo(expected.z, digits);
}

describe('RigidGroupSystem (3D)', () => {
  test('preserves rigid tilt without collapsing members onto a shared z', () => {
    const world = new World();
    const restPositions = [
      new Vector3(-1.0, 0.0, 0.0),
      new Vector3(1.0, 0.0, 0.0),
      new Vector3(0.0, 1.0, 0.0),
    ];

    const members = restPositions.map((position) => {
      const entityId = world.createEntity();
      world.addComponent(entityId, new PositionComponent(position.x, position.y, position.z));
      world.addComponent(entityId, new MassComponent(1.0));
      world.addComponent(entityId, new OrientationComponent());
      return entityId;
    });

    const groupId = world.createEntity();
    world.addComponent(groupId, new RigidGroupComponent(members, 1.0));

    const system = new RigidGroupSystem();
    system.update(world, 1 / 120);

    const restCom = restPositions.reduce((sum, position) => sum.add(position), new Vector3(0, 0, 0)).scale(1.0 / restPositions.length);
    const tilt = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 3).normalize();
    const translatedCom = new Vector3(0.25, -0.4, 0.5);
    const expectedPositions = restPositions.map((position) => {
      const restRel = position.clone().subtract(restCom);
      return tilt.transformVector(restRel).add(translatedCom);
    });

    members.forEach((entityId, index) => {
      world.getComponent(entityId, PositionComponent).pos.set(expectedPositions[index]);
    });

    system.update(world, 1 / 120);

    const actualPositions = members.map((entityId) => world.getComponent(entityId, PositionComponent).pos.clone());
    for (let index = 0; index < actualPositions.length; index += 1) {
      expectVectorClose(actualPositions[index], expectedPositions[index]);
    }

    expect(actualPositions[0].z).toBeLessThan(actualPositions[2].z - 0.3);
    expect(actualPositions[1].z).toBeLessThan(actualPositions[2].z - 0.3);

    const distancePairs = [
      [0, 1],
      [1, 2],
      [0, 2],
    ];
    for (const [a, b] of distancePairs) {
      expect(actualPositions[a].distanceTo(actualPositions[b])).toBeCloseTo(restPositions[a].distanceTo(restPositions[b]), 8);
    }

    const rotatedZ = world.getComponent(members[0], OrientationComponent).quaternion.transformVector(new Vector3(0, 0, 1));
    const expectedZ = tilt.transformVector(new Vector3(0, 0, 1));
    expectVectorClose(rotatedZ, expectedZ);
  });

  test('tracks planar group angle for stepper control compatibility', () => {
    const world = new World();
    const restPositions = [
      new Vector3(-1.0, 0.0, 0.0),
      new Vector3(1.0, 0.0, 0.0),
      new Vector3(0.0, 1.0, 0.0),
    ];

    const members = restPositions.map((position) => {
      const entityId = world.createEntity();
      world.addComponent(entityId, new PositionComponent(position.x, position.y, position.z));
      world.addComponent(entityId, new MassComponent(1.0));
      world.addComponent(entityId, new OrientationComponent());
      return entityId;
    });

    const groupId = world.createEntity();
    world.addComponent(groupId, new RigidGroupComponent(members, 1.0));

    const system = new RigidGroupSystem();
    system.update(world, 1 / 120);

    const yaw = Math.PI / 4.0;
    const rotation = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), yaw).normalize();
    const restCom = restPositions.reduce((sum, position) => sum.add(position), new Vector3(0, 0, 0)).scale(1.0 / restPositions.length);
    const translatedCom = new Vector3(0.3, -0.2, 0.4);
    members.forEach((entityId, index) => {
      const rotatedPos = rotation.transformVector(restPositions[index].clone().subtract(restCom)).add(translatedCom);
      world.getComponent(entityId, PositionComponent).pos.set(rotatedPos);
    });

    system.update(world, 1 / 120);

    const group = world.getComponent(groupId, RigidGroupComponent);
    expect(group.prevAngle).toBeCloseTo(yaw, 8);
  });

  test('rotates spool reference orientations with the rigid group delta', () => {
    const world = new World();
    const restPositions = [
      new Vector3(-1.0, 0.0, 0.0),
      new Vector3(1.0, 0.0, 0.0),
      new Vector3(0.0, 1.0, 0.0),
    ];

    const members = restPositions.map((position, index) => {
      const entityId = world.createEntity();
      world.addComponent(entityId, new PositionComponent(position.x, position.y, position.z));
      world.addComponent(entityId, new MassComponent(1.0));
      world.addComponent(entityId, new OrientationComponent());
      if (index === 0) {
        world.addComponent(entityId, new SpoolStateComponent('A'));
      }
      return entityId;
    });

    const groupId = world.createEntity();
    world.addComponent(groupId, new RigidGroupComponent(members, 1.0));

    const system = new RigidGroupSystem();
    system.update(world, 1 / 120);

    const restCom = restPositions.reduce((sum, position) => sum.add(position), new Vector3(0, 0, 0)).scale(1.0 / restPositions.length);
    const tilt = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 3).normalize();
    const translatedCom = new Vector3(0.25, -0.4, 0.5);
    const expectedPositions = restPositions.map((position) => {
      const restRel = position.clone().subtract(restCom);
      return tilt.transformVector(restRel).add(translatedCom);
    });

    members.forEach((entityId, index) => {
      world.getComponent(entityId, PositionComponent).pos.set(expectedPositions[index]);
    });

    system.update(world, 1 / 120);

    const spoolEntity = members[0];
    const spoolState = world.getComponent(spoolEntity, SpoolStateComponent);
    const orientation = world.getComponent(spoolEntity, OrientationComponent);
    const spoolAxis = getSpoolWorldAxis(spoolState, orientation.quaternion);
    const expectedAxis = tilt.transformVector(new Vector3(0, 0, 1));

    expectVectorClose(spoolAxis, expectedAxis.normalize());
  });
});
