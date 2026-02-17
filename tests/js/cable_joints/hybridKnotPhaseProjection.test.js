import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  _updateAttachmentPoints,
  _updateHybridLinkStates
} from '../../../src/js/cable_joints/cable_joints_core.js';

describe('hybrid knot phase projection', () => {
  test('non-layered hybrid transition does not collapse stored on the next attachment update', () => {
    const world = new World();
    world.setResource('enableLayering', true);

    const wheel = world.createEntity();
    world.addComponent(wheel, new PositionComponent(0.0, 0.0));
    world.addComponent(wheel, new RadiusComponent(1.0));
    world.addComponent(wheel, new CableLinkComponent(0.0, 0.0));
    world.addComponent(wheel, new OrientationComponent(0.0));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(0.0, 3.0));
    world.addComponent(anchor, new RadiusComponent(0.01));
    world.addComponent(anchor, new CableLinkComponent(0.0, 3.0));
    world.addComponent(anchor, new OrientationComponent(0.0));

    const jointId = world.createEntity();
    world.addComponent(
      jointId,
      new CableJointComponent(
        wheel,
        anchor,
        3.2,
        new Vector2(1.0, 0.0),
        new Vector2(0.0, 3.0)
      )
    );

    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [jointId],
      ['hybrid-attachment', 'attachment'],
      [false, false],
      1e6,
      null,
      0.0
    );
    world.addComponent(pathId, pathComp);

    _updateHybridLinkStates(world);
    expect(pathComp.linkTypes[0]).toBe('hybrid');
    expect(pathComp.stored[0]).toBeGreaterThan(1e-6);

    const storedAfterTransition = pathComp.stored[0];
    _updateAttachmentPoints(world);
    _updateAttachmentPoints(world);

    expect(pathComp.stored[0]).toBeCloseTo(storedAfterTransition, 8);
  });
});
