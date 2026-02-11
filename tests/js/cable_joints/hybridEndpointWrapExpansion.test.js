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
  getCompositeSupportToward,
} from '../../../examples/js/flipper/flipper_common.js';

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
  test('creates partial layer circle sector and directional radius response', () => {
    const { world, endpointId } = createEndpointWrapWorld(2.0);
    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(world, 0.016);

    expect(world.hasComponent(endpointId, OverlayRadiusComponent)).toBe(false);
    expect(world.hasComponent(endpointId, CircleSectorComponent)).toBe(true);

    const supportEast = getCompositeSupportToward(world, endpointId, new Vector2(1.0, 0.0));
    const supportNorth = getCompositeSupportToward(world, endpointId, new Vector2(0.0, 1.0));
    expect(supportEast.projection).toBeCloseTo(1.2, 6);
    expect(supportNorth.projection).toBeCloseTo(1.0, 6);
  });

  test('combines full-layer overlay with partial-layer sector', () => {
    const baseLayerCircumference = 2.0 * Math.PI * 1.1;
    const { world, endpointId } = createEndpointWrapWorld(baseLayerCircumference + 1.0);
    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(world, 0.016);

    const overlay = world.getComponent(endpointId, OverlayRadiusComponent);
    const sector = world.getComponent(endpointId, CircleSectorComponent);
    expect(overlay).toBeTruthy();
    expect(overlay.radius).toBeCloseTo(1.2, 6);
    expect(sector).toBeTruthy();
    expect(sector.radius).toBeGreaterThan(1.2);

    const supportEast = getCompositeSupportToward(world, endpointId, new Vector2(1.0, 0.0));
    const supportNorth = getCompositeSupportToward(world, endpointId, new Vector2(0.0, 1.0));
    expect(supportEast.projection).toBeGreaterThan(1.2);
    expect(supportNorth.projection).toBeCloseTo(1.2, 6);
  });

  test('removes overlay/sector components when no stored wrap exists', () => {
    const { world, endpointId } = createEndpointWrapWorld(0.0);
    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(world, 0.016);

    expect(world.hasComponent(endpointId, OverlayRadiusComponent)).toBe(false);
    expect(world.hasComponent(endpointId, CircleSectorComponent)).toBe(false);
  });
});
