const {
  Vector2,
  World,
  PositionComponent,
  RadiusComponent,
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  signedArcLengthOnWheel,
  CableAttachmentUpdateSystem
} = require('../cable_joints/cable_joints_core');

describe('CableAttachmentUpdateSystem', () => {
  test('initial stored length matches signedArcLength between tangent points for rolling link', () => {
    const world = new World();
    const center = new Vector2(0, 0);
    const radius = 1;
    const cw = false;

    // Create entities
    const ball1 = world.createEntity();
    const wheel = world.createEntity();
    const ball2 = world.createEntity();

    // Components: links and positions
    world.addComponent(ball1, new CableLinkComponent());
    world.addComponent(wheel, new CableLinkComponent());
    world.addComponent(ball2, new CableLinkComponent());
    world.addComponent(wheel, new PositionComponent(0, 0));
    world.addComponent(ball1, new PositionComponent(1, 0));
    world.addComponent(ball2, new PositionComponent(0, 1));
    world.addComponent(wheel, new RadiusComponent(radius));

    // Initial tangents on the rolling link (wheel)
    const tp1 = tangentFromPointToCircle(
      world.getComponent(ball1, PositionComponent).pos,
      center,
      radius,
      cw
    );
    const tp2 = tangentFromCircleToPoint(
      world.getComponent(ball2, PositionComponent).pos,
      center,
      radius,
      cw
    );

    // Create cable joints with initial rest lengths
    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        ball1,
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
        ball2,
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

    // Expected stored arc length on wheel
    const expected = signedArcLengthOnWheel(
      tp1.a_circle,
      tp2.a_circle,
      center,
      radius,
      cw,
      true
    );

    expect(pathComp.stored[1]).toBeCloseTo(expected);
  });
});
