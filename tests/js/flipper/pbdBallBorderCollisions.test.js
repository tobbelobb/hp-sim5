import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';
import {
  BallTagComponent,
  BorderComponent,
  PBDBallBorderCollisions,
} from '../../../examples/js/flipper/flipper_common.js';

function addEndpointHybridWrap(world, endpointId, storedLength, halfWidth = 0.1) {
  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(3.0, 0.0));
  world.addComponent(anchorId, new RadiusComponent(0.1));
  world.addComponent(anchorId, new CableLinkComponent(3.0, 0.0));

  const endpointPos = world.getComponent(endpointId, PositionComponent).pos;
  const endpointRadius = world.getComponent(endpointId, RadiusComponent).radius;
  const baseRadius = endpointRadius + halfWidth;
  const attachmentA = endpointPos.clone().add(new Vector2(0.0, baseRadius));
  const attachmentB = attachmentA.clone().add(new Vector2(0.8, 0.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      endpointId,
      anchorId,
      0.0,
      attachmentA,
      attachmentB
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
      halfWidth
    )
  );
}

describe('PBDBallBorderCollisions wrapped effective radius', () => {
  test('detects border contact when wrapped effective radius exceeds base radius', () => {
    const baseLayerCircumference = 2.0 * Math.PI * 1.1;
    const system = new PBDBallBorderCollisions();

    const makeWorld = (withWrap) => {
      const world = new World();

      const ballId = world.createEntity();
      world.addComponent(ballId, new BallTagComponent());
      world.addComponent(ballId, new PositionComponent(0.0, 1.15));
      world.addComponent(ballId, new RadiusComponent(1.0));
      world.addComponent(ballId, new MassComponent(1.0));
      world.addComponent(ballId, new CableLinkComponent(0.0, 1.15));

      const borderId = world.createEntity();
      world.addComponent(
        borderId,
        new BorderComponent([
          new Vector2(-2.0, 0.0),
          new Vector2(2.0, 0.0),
        ])
      );

      if (withWrap) {
        addEndpointHybridWrap(world, ballId, baseLayerCircumference);
      }
      return { world, ballId };
    };

    const noWrap = makeWorld(false);
    const withWrap = makeWorld(true);

    system.update(noWrap.world, 0.016);
    system.update(withWrap.world, 0.016);

    const noWrapContacts = noWrap.world.getResource('ball_border_contacts');
    const withWrapContacts = withWrap.world.getResource('ball_border_contacts');
    expect(noWrapContacts).toHaveLength(0);
    expect(withWrapContacts).toHaveLength(1);

    const yNoWrap = noWrap.world.getComponent(noWrap.ballId, PositionComponent).pos.y;
    const yWithWrap = withWrap.world.getComponent(withWrap.ballId, PositionComponent).pos.y;
    expect(yNoWrap).toBeCloseTo(1.15, 9);
    expect(yWithWrap).toBeGreaterThan(1.15);
    expect(yWithWrap).toBeCloseTo(1.2, 6);
  });
});
