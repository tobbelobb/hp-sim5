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

function createHybridAttachmentEndpointWorld() {
  const world = new World();

  const wheel = world.createEntity();
  world.addComponent(wheel, new PositionComponent(0.0, 0.0));
  world.addComponent(wheel, new CableLinkComponent(0.0, 0.0));
  world.addComponent(wheel, new RadiusComponent(1.0));
  world.addComponent(wheel, new OrientationComponent(0.0));

  const anchor = world.createEntity();
  world.addComponent(anchor, new PositionComponent(0.0, 3.0));
  world.addComponent(anchor, new CableLinkComponent(0.0, 3.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      wheel,
      anchor,
      3.2,
      new Vector2(1.0, 0.0), // authored at raw body radius
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
    0.1
  );
  world.addComponent(pathId, pathComp);

  return { world, wheel, jointId, pathComp };
}

//describe('Hybrid attachment endpoint radius continuity', () => {
//  test('hybrid-attachment endpoint projects to rawRadius + halfWidth', () => {
//    const { world, wheel, jointId } = createHybridAttachmentEndpointWorld();
//
//    _updateAttachmentPoints(world);
//
//    const center = world.getComponent(wheel, PositionComponent).pos;
//    const attachment = world.getComponent(jointId, CableJointComponent).attachmentPointA_world;
//    expect(attachment.distanceTo(center)).toBeCloseTo(1.1, 8);
//  });
//
//  test('hybrid-attachment -> hybrid transition keeps endpoint radial distance continuous', () => {
//    const { world, wheel, jointId, pathComp } = createHybridAttachmentEndpointWorld();
//
//    _updateAttachmentPoints(world);
//
//    const center = world.getComponent(wheel, PositionComponent).pos;
//    const before = world
//      .getComponent(jointId, CableJointComponent)
//      .attachmentPointA_world
//      .distanceTo(center);
//
//    _updateHybridLinkStates(world);
//
//    const after = world
//      .getComponent(jointId, CableJointComponent)
//      .attachmentPointA_world
//      .distanceTo(center);
//
//    expect(pathComp.linkTypes[0]).toBe('hybrid');
//    expect(after).toBeCloseTo(before, 8);
//  });
//});
