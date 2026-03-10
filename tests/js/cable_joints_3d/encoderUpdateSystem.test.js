import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  EncoderComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  PrevFinalOrientationSystem,
  EncoderUpdateSystem,
} from '../../../src/js/cable_joints_3d/commonSystems.js';

function setPlanarAngle(quaternion, angleRad) {
  quaternion.setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), angleRad);
}

describe('EncoderUpdateSystem', () => {
  test('accumulates full-turn history across wrapped planar orientations', () => {
    const world = new World();
    const entityId = world.createEntity();
    const orientation = new OrientationComponent();
    const prevOrientation = new PrevFinalOrientationComponent();
    const encoder = new EncoderComponent();

    world.addComponent(entityId, orientation);
    world.addComponent(entityId, prevOrientation);
    world.addComponent(entityId, encoder);
    world.setResource('defaultPlaneNormal', new Vector3(0.0, 0.0, 1.0));

    const prevSystem = new PrevFinalOrientationSystem();
    const encoderSystem = new EncoderUpdateSystem();
    const targetAngles = [
      (170.0 * Math.PI) / 180.0,
      (340.0 * Math.PI) / 180.0,
      (510.0 * Math.PI) / 180.0,
    ];

    for (const targetAngle of targetAngles) {
      prevSystem.update(world, 1 / 500);
      setPlanarAngle(orientation.quaternion, targetAngle);
      encoderSystem.update(world, 1 / 500);
    }

    expect(encoder.angle).toBeCloseTo((510.0 * Math.PI) / 180.0, 6);
  });

  test('respects a custom encoder axis', () => {
    const world = new World();
    const entityId = world.createEntity();
    const orientation = new OrientationComponent();
    const prevOrientation = new PrevFinalOrientationComponent();
    const encoder = new EncoderComponent(0.0, 1.0, 0.0, 0.0);

    world.addComponent(entityId, orientation);
    world.addComponent(entityId, prevOrientation);
    world.addComponent(entityId, encoder);
    world.setResource('defaultPlaneNormal', new Vector3(0.0, 0.0, 1.0));

    const prevSystem = new PrevFinalOrientationSystem();
    const encoderSystem = new EncoderUpdateSystem();

    prevSystem.update(world, 1 / 500);
    orientation.quaternion.setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 4);
    encoderSystem.update(world, 1 / 500);

    expect(encoder.angle).toBeCloseTo(Math.PI / 4, 6);
  });
});
