import Vector2 from '../cable_joints/vector2.js';

import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent
} from '../cable_joints/ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem
} from '../cable_joints/cable_joints_core.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel
} from '../cable_joints/geometry.js';


describe('CableAttachmentUpdateSystem', () => {
  test('Merge joints when positions opposite vertically, conserving rest length', () => {
    const world = new World();
    const center = new Vector2(0, 0);
    const radius = 1;
    const cw = true;

    // Create entities
    const point1 = world.createEntity();
    const wheel = world.createEntity();
    const point2 = world.createEntity();

    // Components: links and positions
    world.addComponent(point1, new CableLinkComponent());
    world.addComponent(wheel, new CableLinkComponent());
    world.addComponent(point2, new CableLinkComponent());
    world.addComponent(wheel, new PositionComponent(0, 0));
    world.addComponent(point1, new PositionComponent(0.9999, 2));
    world.addComponent(point2, new PositionComponent(1.0, -2));
    world.addComponent(wheel, new RadiusComponent(radius));

    // Initial tangents on the rolling link (wheel)
    const tp1 = tangentFromPointToCircle(
      world.getComponent(point1, PositionComponent).pos,
      center,
      radius,
      cw
    );
    const tp2 = tangentFromCircleToPoint(
      world.getComponent(point2, PositionComponent).pos,
      center,
      radius,
      cw
    );

    // Create cable joints with initial rest lengths
    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        point1,
        wheel,
        tp1.a_attach.clone().subtract(tp1.a_circle).length(),
        tp1.a_attach,
        tp1.a_circle
      )
    );
    const joint2 = world.createEntity();
    world.addComponent(
      joint2,
      new CableJointComponent(
        wheel,
        point2,
        tp2.a_attach.clone().subtract(tp2.a_circle).length(),
        tp2.a_circle,
        tp2.a_attach
      )
    );

    // Build cable path
    const cablePath = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint1, joint2],
      ['attachment', 'rolling', 'attachment'],
      [cw, cw, cw]
    );
    world.addComponent(cablePath, pathComp);

    // Record initial total rest length and joint count
    const initialTotalRest = pathComp.totalRestLength;
    expect(pathComp.jointEntities).toHaveLength(2);
    expect(pathComp.linkTypes).toHaveLength(3);
    expect(pathComp.stored[1]).toBeLessThan(radius);
    expect(pathComp.stored[1]).toBeGreaterThan(0.0);

    // Let point1 travel to the right
    world.getComponent(point1, PositionComponent).prevPos = new Vector2(1.001, 2);
    world.getComponent(point1, PositionComponent).pos = new Vector2(1.001, 2);

    // Run the attachment update to trigger merge
    const system = new CableAttachmentUpdateSystem();
    world.setResource('debugRenderPoints', {});
    system.update(world);

    // After merge, one joint should be removed
    expect(pathComp.jointEntities).toHaveLength(1);
    expect(pathComp.linkTypes).toHaveLength(2);
    // Total rest length should remain unchanged
    expect(pathComp.totalRestLength).toBeCloseTo(initialTotalRest);
  });
});
