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
  getHybridEndpointWrapExpansion,
} from '../../../src/js/cable_joints/cable_joints_core.js';

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
      [true, true], // endpoint effective direction is CCW from attachment
      1e4,
      [storedLength, 0.0],
      0.1
    )
  );

  return { world, endpointId };
}

describe('getHybridEndpointWrapExpansion', () => {
  test('returns zero outside first partial wrap and 2w inside it', () => {
    const { world, endpointId } = createEndpointWrapWorld(2.0);
    const pointNorth = new Vector2(0.0, 1.0);
    const pointSouth = new Vector2(0.0, -1.0);

    const northExpansion = getHybridEndpointWrapExpansion(world, endpointId, pointNorth);
    const southExpansion = getHybridEndpointWrapExpansion(world, endpointId, pointSouth);

    expect(northExpansion).toBeCloseTo(0.2, 6);
    expect(southExpansion).toBeCloseTo(0.0, 9);
  });

  test('ramps expansion smoothly across partial-wrap boundaries', () => {
    const { world, endpointId } = createEndpointWrapWorld(2.0);
    const pointNearStart = new Vector2(Math.cos(0.05), Math.sin(0.05));
    const pointInterior = new Vector2(Math.cos(Math.PI * 0.5), Math.sin(Math.PI * 0.5));
    const pointNearEnd = new Vector2(Math.cos(1.75), Math.sin(1.75));

    const nearStart = getHybridEndpointWrapExpansion(world, endpointId, pointNearStart);
    const interior = getHybridEndpointWrapExpansion(world, endpointId, pointInterior);
    const nearEnd = getHybridEndpointWrapExpansion(world, endpointId, pointNearEnd);

    expect(interior).toBeCloseTo(0.2, 6);
    expect(nearStart).toBeGreaterThan(0.0);
    expect(nearStart).toBeLessThan(interior);
    expect(nearEnd).toBeGreaterThan(0.0);
    expect(nearEnd).toBeLessThan(interior);
  });

  test('returns full-layer expansion globally and larger expansion on partial second layer', () => {
    const baseLayerCircumference = 2.0 * Math.PI * 1.1;
    const { world, endpointId } = createEndpointWrapWorld(baseLayerCircumference + 2.2);
    const pointNorth = new Vector2(0.0, 1.0);
    const pointSouth = new Vector2(0.0, -1.0);

    const northExpansion = getHybridEndpointWrapExpansion(world, endpointId, pointNorth);
    const southExpansion = getHybridEndpointWrapExpansion(world, endpointId, pointSouth);

    expect(southExpansion).toBeCloseTo(0.2, 6);
    expect(northExpansion).toBeGreaterThan(southExpansion);
    expect(northExpansion).toBeLessThanOrEqual(0.4 + 1e-6);
  });
});
