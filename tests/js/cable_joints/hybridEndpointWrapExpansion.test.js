import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';
import {
  OverlayRadiusAndCircleSectorSystem,
  OverlayRadiusComponent,
  CircleSectorComponent,
} from '../../../example_apps/js/flipper/flipper_common.js';

function createEndpointWrapWorld(storedLength) {
  const world = new World();

  const endpointId = world.createEntity();
  world.addComponent(endpointId, new PositionComponent(0.0, 0.0));
  world.addComponent(endpointId, new RadiusComponent(1.0));
  world.addComponent(endpointId, new CableLinkComponent(0.0, 0.0));

  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(3.0, 0.0));
  world.addComponent(anchorId, new RadiusComponent(0.1));
  world.addComponent(anchorId, new CableLinkComponent(3.0, 0.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      endpointId,
      anchorId,
      0.0,
      new Vector2(1.0, 0.0),
      new Vector2(2.9, 0.0)
    )
  );

  const pathId = world.createEntity();
  world.addComponent(
    pathId,
    new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [true, true],
      1e4,
      [storedLength, 0.0],
      0.1
    )
  );

  return { world, endpointId };
}

describe('OverlayRadiusAndCircleSectorSystem endpoint layering', () => {
  test('removes overlay/sector components when no stored wrap exists', () => {
    const { world, endpointId } = createEndpointWrapWorld(0.0);
    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(world, 0.016);

    expect(world.hasComponent(endpointId, OverlayRadiusComponent)).toBe(false);
    expect(world.hasComponent(endpointId, CircleSectorComponent)).toBe(false);
  });
});
