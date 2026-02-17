import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  OverlayRadiusAndCircleSectorSystem,
  CircleSectorComponent,
  CircleSectorsComponent,
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
  overlayRampEnabled = true,
} = {}) {
  const world = new World();
  world.setResource('enableLayering', true);
  world.setResource('layeringCollisionOverlayRadius', true);
  world.setResource('layeringCollisionOverlayRamp', overlayRampEnabled);
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

const TEST_LAYER_RADIUS_RAMP_ANGLE = (2.0 * Math.PI) / 5.0;

function _rampAlpha(span) {
  const spanValue = Number.isFinite(span) ? Math.max(0.0, span) : 0.0;
  if (!(TEST_LAYER_RADIUS_RAMP_ANGLE > 1e-9)) {
    return 1.0;
  }
  return Math.max(0.0, Math.min(1.0, spanValue / TEST_LAYER_RADIUS_RAMP_ANGLE));
}

function _closingBlend(span) {
  const spanValue = Number.isFinite(span) ? Math.max(0.0, span) : 0.0;
  if (!(TEST_LAYER_RADIUS_RAMP_ANGLE > 1e-9)) {
    return 0.0;
  }
  const remaining = Math.max(0.0, (2.0 * Math.PI) - spanValue);
  if (remaining >= TEST_LAYER_RADIUS_RAMP_ANGLE) {
    return 0.0;
  }
  return 1.0 - (remaining / TEST_LAYER_RADIUS_RAMP_ANGLE);
}

function _expectedSectorRadius(rawRadius, halfWidth, fullLayers, span) {
  const step = 2.0 * halfWidth;
  const start = rawRadius + (step * fullLayers);
  return start + (step * _rampAlpha(span));
}

function _expectedOverlayRadius(rawRadius, halfWidth, fullLayers, span) {
  const step = 2.0 * halfWidth;
  const start = rawRadius + (step * fullLayers);
  return start + (step * _closingBlend(span));
}

describe('OverlayRadiusAndCircleSectorSystem radius ramp', () => {
  test('treats numeric clockwise flags as clockwise sectors', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const firstLayerRadius = rawRadius + halfWidth;
    const firstLayerCircumference = 2.0 * Math.PI * firstLayerRadius;
    const worldState = makeEndpointWrapWorld({
      // Ensure overlay envelope exceeds raw radius so a sector is emitted.
      storedLength: firstLayerCircumference * 0.99,
      rawRadius,
      halfWidth,
      cw: 1
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(worldState.world, 0.016);

    const sector = worldState.world.getComponent(worldState.wrappedId, CircleSectorComponent);
    expect(sector).toBeTruthy();
    expect(sector.cw).toBe(true);
  });

  test('keeps sector radius continuous when crossing a full-wrap boundary', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const fullLayerSupportRadius = rawRadius + 2.0 * halfWidth;
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
    expect(beforeSector.radius).toBeCloseTo(fullLayerSupportRadius, 3);

    // Crossing into the next layer should remain continuous.
    const radiusDelta = Math.abs(afterSector.radius - beforeSector.radius);
    expect(radiusDelta).toBeLessThan(0.01);
    expect(afterSector.radius).toBeGreaterThanOrEqual(fullLayerSupportRadius);

    // Overlay still steps when a complete layer is finished.
    const overlay = after.world.getComponent(after.wrappedId, OverlayRadiusComponent);
    expect(overlay).toBeTruthy();
    expect(overlay.radius).toBeCloseTo(fullLayerSupportRadius, 9);
  });

  test('higher partial layers are not clamped to previous overlay envelope', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const step = 2.0 * halfWidth;
    const firstLayerRadius = rawRadius + halfWidth;
    const secondLayerRadius = firstLayerRadius + step;
    const firstLayerCircumference = 2.0 * Math.PI * firstLayerRadius;
    const rampSpan = (2.0 * Math.PI) / 25.0;
    const partialLengthPastRamp = secondLayerRadius * rampSpan * 2.0;
    const span = partialLengthPastRamp / secondLayerRadius;

    const worldState = makeEndpointWrapWorld({
      storedLength: firstLayerCircumference + partialLengthPastRamp,
      rawRadius,
      halfWidth
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(worldState.world, 0.016);

    const sector = worldState.world.getComponent(worldState.wrappedId, CircleSectorComponent);
    const overlay = worldState.world.getComponent(worldState.wrappedId, OverlayRadiusComponent);
    expect(sector).toBeTruthy();
    expect(overlay).toBeTruthy();
    expect(overlay.radius).toBeCloseTo(_expectedOverlayRadius(rawRadius, halfWidth, 1, span), 9);
    expect(sector.radius).toBeGreaterThan(overlay.radius + 1e-6);
    expect(sector.radius).toBeCloseTo(_expectedSectorRadius(rawRadius, halfWidth, 1, span), 9);
  });

  test('keeps first-layer partial sector support even before overlay ramp starts', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const firstLayerRadius = rawRadius + halfWidth;
    const spanJustBeforeOverlayRamp = (2.0 * Math.PI) - TEST_LAYER_RADIUS_RAMP_ANGLE - 1e-4;
    const storedLength = firstLayerRadius * spanJustBeforeOverlayRamp;

    const worldState = makeEndpointWrapWorld({
      storedLength,
      rawRadius,
      halfWidth,
      overlayRampEnabled: true
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(worldState.world, 0.016);

    const overlay = worldState.world.getComponent(worldState.wrappedId, OverlayRadiusComponent);
    const sector = worldState.world.getComponent(worldState.wrappedId, CircleSectorComponent);
    expect(overlay).toBeFalsy();
    expect(sector).toBeTruthy();
    expect(sector.radius).toBeCloseTo(_expectedSectorRadius(rawRadius, halfWidth, 0, spanJustBeforeOverlayRamp), 9);
  });

  test('retains multiple simultaneous sectors on one entity when multiple paths wrap it', () => {
    const world = new World();
    world.setResource('enableLayering', true);
    world.setResource('layeringCollisionOverlayRadius', true);
    world.setResource('layeringCollisionOverlayRamp', true);
    world.setResource('layeringCollisionCircleSectors', true);

    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const wrappedId = world.createEntity();
    world.addComponent(wrappedId, new PositionComponent(0.0, 0.0));
    world.addComponent(wrappedId, new RadiusComponent(rawRadius));

    const rightAnchorId = world.createEntity();
    world.addComponent(rightAnchorId, new PositionComponent(3.0, 0.0));
    world.addComponent(rightAnchorId, new RadiusComponent(rawRadius));
    const topAnchorId = world.createEntity();
    world.addComponent(topAnchorId, new PositionComponent(0.0, 3.0));
    world.addComponent(topAnchorId, new RadiusComponent(rawRadius));

    const jointRight = world.createEntity();
    world.addComponent(
      jointRight,
      CableJointComponent.fromWorld(
        wrappedId,
        rightAnchorId,
        1.0,
        new Vector2(rawRadius, 0.0),
        new Vector2(2.0, 0.0)
      )
    );
    const jointTop = world.createEntity();
    world.addComponent(
      jointTop,
      CableJointComponent.fromWorld(
        wrappedId,
        topAnchorId,
        1.0,
        new Vector2(0.0, rawRadius),
        new Vector2(0.0, 2.0)
      )
    );

    const quarterTurnStored = (rawRadius + halfWidth) * (Math.PI / 2.0);
    const pathA = world.createEntity();
    world.addComponent(
      pathA,
      new CablePathComponent(
        world,
        [jointRight],
        ['hybrid', 'attachment'],
        [false, false],
        1e6,
        [quarterTurnStored, 0.0],
        halfWidth
      )
    );
    const pathB = world.createEntity();
    world.addComponent(
      pathB,
      new CablePathComponent(
        world,
        [jointTop],
        ['hybrid', 'attachment'],
        [true, false],
        1e6,
        [quarterTurnStored, 0.0],
        halfWidth
      )
    );

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(world, 0.016);

    const sectors = world.getComponent(wrappedId, CircleSectorsComponent);
    expect(sectors).toBeTruthy();
    expect(Array.isArray(sectors.sectors)).toBe(true);
    expect(sectors.sectors.length).toBeGreaterThanOrEqual(2);

    const single = world.getComponent(wrappedId, CircleSectorComponent);
    expect(single).toBeTruthy();
    const maxMulti = Math.max(...sectors.sectors.map((sector) => sector.radius));
    expect(single.radius).toBeCloseTo(maxMulti, 9);
  });

  test('overlay ramp can be toggled via layeringCollisionOverlayRamp', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const step = 2.0 * halfWidth;
    const firstLayerRadius = rawRadius + halfWidth;
    const secondLayerRadius = firstLayerRadius + step;
    const firstLayerCircumference = 2.0 * Math.PI * firstLayerRadius;
    const rampSpan = (2.0 * Math.PI) / 25.0;
    const nearClosureSpan = (2.0 * Math.PI) - (0.5 * rampSpan);
    const storedNearClosure = firstLayerCircumference + (secondLayerRadius * nearClosureSpan);

    const withRamp = makeEndpointWrapWorld({
      storedLength: storedNearClosure,
      rawRadius,
      halfWidth,
      overlayRampEnabled: true
    });
    const withoutRamp = makeEndpointWrapWorld({
      storedLength: storedNearClosure,
      rawRadius,
      halfWidth,
      overlayRampEnabled: false
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(withRamp.world, 0.016);
    system.update(withoutRamp.world, 0.016);

    const withRampOverlay = withRamp.world.getComponent(withRamp.wrappedId, OverlayRadiusComponent);
    const withoutRampOverlay = withoutRamp.world.getComponent(withoutRamp.wrappedId, OverlayRadiusComponent);
    expect(withRampOverlay).toBeTruthy();
    expect(withoutRampOverlay).toBeTruthy();
    expect(withRampOverlay.radius).toBeGreaterThan(withoutRampOverlay.radius);
    expect(withoutRampOverlay.radius).toBeCloseTo(rawRadius + step, 9);

    const withRampSector = withRamp.world.getComponent(withRamp.wrappedId, CircleSectorComponent);
    const withoutRampSector = withoutRamp.world.getComponent(withoutRamp.wrappedId, CircleSectorComponent);
    expect(withRampSector).toBeTruthy();
    expect(withoutRampSector).toBeTruthy();
    expect(withRampSector.radius).toBeCloseTo(rawRadius + (2.0 * step), 9);
    expect(withoutRampSector.radius).toBeCloseTo(rawRadius + (2.0 * step), 9);
  });

  test('second-revolution partial sector radius exceeds first-revolution radius', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const step = 2.0 * halfWidth;
    const firstLayerRadius = rawRadius + halfWidth;
    const secondLayerRadius = firstLayerRadius + step;
    const rampSpan = (2.0 * Math.PI) / 25.0;
    const probeSpan = rampSpan * 2.0;
    const firstLayerStored = firstLayerRadius * probeSpan;
    const secondLayerStored = (2.0 * Math.PI * firstLayerRadius) + (secondLayerRadius * probeSpan);

    const first = makeEndpointWrapWorld({
      storedLength: firstLayerStored,
      rawRadius,
      halfWidth
    });
    const second = makeEndpointWrapWorld({
      storedLength: secondLayerStored,
      rawRadius,
      halfWidth
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(first.world, 0.016);
    system.update(second.world, 0.016);

    const firstSector = first.world.getComponent(first.wrappedId, CircleSectorComponent);
    const secondSector = second.world.getComponent(second.wrappedId, CircleSectorComponent);
    expect(firstSector).toBeTruthy();
    expect(secondSector).toBeTruthy();
    expect(firstSector.radius).toBeCloseTo(_expectedSectorRadius(rawRadius, halfWidth, 0, probeSpan), 9);
    expect(secondSector.radius).toBeCloseTo(_expectedSectorRadius(rawRadius, halfWidth, 1, probeSpan), 9);
    expect(secondSector.radius).toBeGreaterThan(firstSector.radius + 1e-6);
  });

  test('avoids large ball-ball support jumps around near-complete second-layer coverage', () => {
    const rawRadius = 1.0;
    const halfWidth = 0.1;
    const firstLayerRadius = rawRadius + halfWidth;
    const secondLayerRadius = firstLayerRadius + (2.0 * halfWidth);
    const firstLayerCircumference = 2.0 * Math.PI * firstLayerRadius;
    const secondLayerCircumference = 2.0 * Math.PI * secondLayerRadius;
    const coverageBefore = 0.899;
    const coverageAfter = 0.901;

    const before = makeEndpointWrapWorld({
      storedLength: firstLayerCircumference + (coverageBefore * secondLayerCircumference),
      rawRadius,
      halfWidth
    });
    const after = makeEndpointWrapWorld({
      storedLength: firstLayerCircumference + (coverageAfter * secondLayerCircumference),
      rawRadius,
      halfWidth
    });

    const system = new OverlayRadiusAndCircleSectorSystem();
    system.update(before.world, 0.016);
    system.update(after.world, 0.016);

    const beforeOverlay = before.world.getComponent(before.wrappedId, OverlayRadiusComponent);
    const afterOverlay = after.world.getComponent(after.wrappedId, OverlayRadiusComponent);
    expect(beforeOverlay).toBeTruthy();
    expect(afterOverlay).toBeTruthy();
    const spanBefore = coverageBefore * (2.0 * Math.PI);
    const spanAfter = coverageAfter * (2.0 * Math.PI);
    expect(beforeOverlay.radius).toBeCloseTo(_expectedOverlayRadius(rawRadius, halfWidth, 1, spanBefore), 9);
    expect(afterOverlay.radius).toBeCloseTo(_expectedOverlayRadius(rawRadius, halfWidth, 1, spanAfter), 9);
    expect(Math.abs(afterOverlay.radius - beforeOverlay.radius)).toBeLessThan(0.01);
  });
});
