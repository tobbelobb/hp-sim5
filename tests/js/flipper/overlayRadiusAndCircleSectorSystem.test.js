import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  OverlayRadiusAndCircleSectorSystem,
  CircleSectorComponent,
  OverlayRadiusComponent,
} from '../../../examples/js/flipper/flipper_common.js';
import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';

function makeEndpointWrapWorld({
  storedLength,
  rawRadius = 1.0,
  halfWidth = 0.1,
  cw = false,
} = {}) {
  const world = new World();
  world.setResource('enableLayering', true);
  world.setResource('layeringCollisionOverlayRadius', true);
  world.setResource('layeringCollisionCircleSectors', true);

  const wrappedId = world.createEntity();
  world.addComponent(wrappedId, new PositionComponent(0.0, 0.0));
  world.addComponent(wrappedId, new RadiusComponent(rawRadius));

  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(3.0, 0.0));
  world.addComponent(anchorId, new RadiusComponent(rawRadius));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    CableJointComponent.fromWorld(
      wrappedId,
      anchorId,
      1.0,
      new Vector2(rawRadius, 0.0),
      new Vector2(2.0, 0.0)
    )
  );

  const pathId = world.createEntity();
  const path = new CablePathComponent(
    world,
    [jointId],
    ['hybrid', 'attachment'],
    [cw, false],
    1e6,
    [storedLength, 0.0],
    halfWidth
  );
  world.addComponent(pathId, path);

  return { world, wrappedId };
}

describe('OverlayRadiusAndCircleSectorSystem radius ramp', () => {
  test('keeps sector radius continuous when crossing a full-wrap boundary', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const firstLayerRadius = rawRadius + halfWidth;
    const firstLayerCircumference = 2.0 * Math.PI * firstLayerRadius;
    const epsilon = 1e-4;

    const before = makeEndpointWrapWorld({
      storedLength: firstLayerCircumference - epsilon,
      rawRadius,
      halfWidth
    });
    const after = makeEndpointWrapWorld({
      storedLength: firstLayerCircumference + epsilon,
      rawRadius,
      halfWidth
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(before.world, 0.016);
    system.update(after.world, 0.016);

    const beforeSector = before.world.getComponent(before.wrappedId, CircleSectorComponent);
    const afterSector = after.world.getComponent(after.wrappedId, CircleSectorComponent);
    expect(beforeSector).toBeTruthy();
    expect(afterSector).toBeTruthy();

    // Crossing into the next layer should not cause a full +2*halfWidth radius jump.
    const radiusDelta = Math.abs(afterSector.radius - beforeSector.radius);
    expect(radiusDelta).toBeLessThan(0.01);
    expect(afterSector.radius).toBeLessThan(rawRadius + 0.15);

    // Overlay still steps when a complete layer is finished.
    const overlay = after.world.getComponent(after.wrappedId, OverlayRadiusComponent);
    expect(overlay).toBeTruthy();
    expect(overlay.radius).toBeCloseTo(rawRadius + 2.0 * halfWidth, 9);
  });

  test('reaches full new-layer sector radius after the ramp span', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const step = 2.0 * halfWidth;
    const firstLayerRadius = rawRadius + halfWidth;
    const secondLayerRadius = firstLayerRadius + step;
    const firstLayerCircumference = 2.0 * Math.PI * firstLayerRadius;
    const rampSpan = (2.0 * Math.PI) / 100.0;
    const partialLengthPastRamp = secondLayerRadius * rampSpan * 2.0;

    const worldState = makeEndpointWrapWorld({
      storedLength: firstLayerCircumference + partialLengthPastRamp,
      rawRadius,
      halfWidth
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(worldState.world, 0.016);

    const sector = worldState.world.getComponent(worldState.wrappedId, CircleSectorComponent);
    expect(sector).toBeTruthy();
    expect(sector.radius).toBeCloseTo(secondLayerRadius, 9);
  });
});
