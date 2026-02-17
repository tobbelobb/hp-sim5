import {
  World,
  PositionComponent,
  OrientationComponent,
  RadiusComponent
} from '../../../src/js/cable_joints/ecs.js';
import { tangentFromCircleToPoint } from '../../../src/js/cable_joints/geometry.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints/cable_joints_core.js';
import { CableFrictionSystem } from '../../../src/js/cable_joints/cable_friction_system.js';

function createHybridEndpointWorld() {
  const world = new World();
  world.setResource('enableLayering', true);

  const wheel = world.createEntity();
  world.addComponent(wheel, new PositionComponent(0.10, 0.12));
  world.addComponent(wheel, new OrientationComponent(0.0));
  world.addComponent(wheel, new RadiusComponent(0.05));
  world.addComponent(wheel, new CableLinkComponent(0.10, 0.12, 0.0));

  const anchor = world.createEntity();
  world.addComponent(anchor, new PositionComponent(0.36, 0.08));
  world.addComponent(anchor, new RadiusComponent(0.01));
  world.addComponent(anchor, new CableLinkComponent(0.36, 0.08, 0.0));

  const wheelPos = world.getComponent(wheel, PositionComponent).pos;
  const anchorPos = world.getComponent(anchor, PositionComponent).pos;
  const tangent = tangentFromCircleToPoint(anchorPos, wheelPos, 0.05, true);

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    CableJointComponent.fromWorld(
      wheel,
      anchor,
      Math.max(1e-9, tangent.a_circle.distanceTo(tangent.a_attach)),
      tangent.a_circle,
      tangent.a_attach
    )
  );

  const pathId = world.createEntity();
  const pathComp = new CablePathComponent(
    world,
    [jointId],
    ['hybrid', 'attachment'],
    [true, true],
    1200.0,
    [0.08, 0.0],
    0.005
  );
  world.addComponent(pathId, pathComp);

  return { world, wheel, pathComp };
}

describe('CableFrictionSystem post-solve attachment sync', () => {
  test('does not sync hybrid stored/cache when orientation changes after cache', () => {
    const { world, wheel, pathComp } = createHybridEndpointWorld();
    const system = new CableFrictionSystem();
    const wheelOrientation = world.getComponent(wheel, OrientationComponent);
    const wheelLink = world.getComponent(wheel, CableLinkComponent);
    const storedBefore = pathComp.stored[0];
    const cachedBefore = wheelLink.prevCableAttachmentTimeAngle;

    // Simulate post-solver correction after the pre-solver cache was written.
    // CableFrictionSystem no longer performs post-solve attachment sync.
    wheelOrientation.angle += 0.05;
    system.update(world, 1.0 / 500.0);

    expect(pathComp.stored[0]).toBeCloseTo(storedBefore, 12);
    expect(wheelLink.prevCableAttachmentTimeAngle).toBeCloseTo(cachedBefore, 12);
  });
});
