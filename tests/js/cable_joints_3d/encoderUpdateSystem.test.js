import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  EncoderComponent,
  OrientationComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
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
    const encoder = new EncoderComponent();

    world.addComponent(entityId, orientation);
    world.addComponent(entityId, encoder);
    world.setResource('defaultPlaneNormal', new Vector3(0.0, 0.0, 1.0));

    const encoderSystem = new EncoderUpdateSystem();
    const targetAngles = [
      (170.0 * Math.PI) / 180.0,
      (340.0 * Math.PI) / 180.0,
      (510.0 * Math.PI) / 180.0,
    ];

    for (const targetAngle of targetAngles) {
      setPlanarAngle(orientation.quaternion, targetAngle);
      encoderSystem.update(world, 1 / 500);
    }

    expect(encoder.angle).toBeCloseTo((510.0 * Math.PI) / 180.0, 6);
  });

  test('respects a custom encoder axis', () => {
    const world = new World();
    const entityId = world.createEntity();
    const orientation = new OrientationComponent();
    const encoder = new EncoderComponent(0.0, 1.0, 0.0, 0.0);

    world.addComponent(entityId, orientation);
    world.addComponent(entityId, encoder);
    world.setResource('defaultPlaneNormal', new Vector3(0.0, 0.0, 1.0));

    const encoderSystem = new EncoderUpdateSystem();

    orientation.quaternion.setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 4);
    encoderSystem.update(world, 1 / 500);

    expect(encoder.angle).toBeCloseTo(Math.PI / 4, 6);
  });

  test('works without PrevFinalOrientationComponent being present', () => {
    const world = new World();
    const entityId = world.createEntity();
    const orientation = new OrientationComponent();
    const encoder = new EncoderComponent();

    world.addComponent(entityId, orientation);
    world.addComponent(entityId, encoder);
    world.setResource('defaultPlaneNormal', new Vector3(0.0, 0.0, 1.0));

    const encoderSystem = new EncoderUpdateSystem();
    setPlanarAngle(orientation.quaternion, (225.0 * Math.PI) / 180.0);
    encoderSystem.update(world, 1 / 500);

    expect(encoder.angle).toBeCloseTo((-135.0 * Math.PI) / 180.0, 6);
  });
});
