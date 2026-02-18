import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent,
  HybridKnotAngleComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  _updateAttachmentPoints,
  _updateHybridLinkStates
} from '../../../src/js/cable_joints/cable_joints_core.js';
import { tangentFromCircleToPoint } from '../../../src/js/cable_joints/geometry.js';

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

  test('hybrid endpoint initialized at path creation gets knot projection updates', () => {
    const world = new World();
    world.setResource('enableLayering', true);

    const wheel = world.createEntity();
    world.addComponent(wheel, new PositionComponent(0.0, 0.0));
    world.addComponent(wheel, new RadiusComponent(1.0));
    world.addComponent(wheel, new CableLinkComponent(0.0, 0.0));
    world.addComponent(wheel, new OrientationComponent(0.0));

    const anchor = world.createEntity();
    const anchorPos = new Vector2(0.0, 3.0);
    world.addComponent(anchor, new PositionComponent(anchorPos.x, anchorPos.y));
    world.addComponent(anchor, new CableLinkComponent(anchorPos.x, anchorPos.y));
    world.addComponent(anchor, new OrientationComponent(0.0));

    const tangent = tangentFromCircleToPoint(anchorPos, new Vector2(0.0, 0.0), 1.0, false).a_circle;
    const jointId = world.createEntity();
    world.addComponent(
      jointId,
      new CableJointComponent(
        wheel,
        anchor,
        tangent.distanceTo(anchorPos),
        tangent.clone(),
        anchorPos.clone()
      )
    );

    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [true, false],
      1e6,
      [0.4, 0.0],
      0.0
    );
    world.addComponent(pathId, pathComp);

    expect(world.hasComponent(wheel, HybridKnotAngleComponent)).toBe(true);
    const initialStored = pathComp.stored[0];
    const joint = world.getComponent(jointId, CableJointComponent);
    const initialRestLength = joint.restLength;

    const wheelOrientation = world.getComponent(wheel, OrientationComponent);
    wheelOrientation.angle = 0.1;
    const wheelLink = world.getComponent(wheel, CableLinkComponent);
    wheelLink.prevCableAttachmentTimeAngle = 0.1;

    _updateAttachmentPoints(world);

    expect(pathComp.stored[0]).toBeCloseTo(initialStored - 0.1, 8);
    expect(joint.restLength).toBeCloseTo(initialRestLength + 0.1, 8);
  });

  test('shared hybrid endpoint keeps independent knot phases per cable path', () => {
    const world = new World();
    world.setResource('enableLayering', true);

    const wheel = world.createEntity();
    const wheelPos = new Vector2(0.0, 0.0);
    const wheelRadius = 1.0;
    world.addComponent(wheel, new PositionComponent(wheelPos.x, wheelPos.y));
    world.addComponent(wheel, new RadiusComponent(wheelRadius));
    world.addComponent(wheel, new CableLinkComponent(wheelPos.x, wheelPos.y));
    world.addComponent(wheel, new OrientationComponent(0.0));

    const anchorTop = world.createEntity();
    const anchorTopPos = new Vector2(0.0, 3.0);
    world.addComponent(anchorTop, new PositionComponent(anchorTopPos.x, anchorTopPos.y));
    world.addComponent(anchorTop, new CableLinkComponent(anchorTopPos.x, anchorTopPos.y));
    world.addComponent(anchorTop, new OrientationComponent(0.0));

    const anchorBottom = world.createEntity();
    const anchorBottomPos = new Vector2(0.0, -3.0);
    world.addComponent(anchorBottom, new PositionComponent(anchorBottomPos.x, anchorBottomPos.y));
    world.addComponent(anchorBottom, new CableLinkComponent(anchorBottomPos.x, anchorBottomPos.y));
    world.addComponent(anchorBottom, new OrientationComponent(0.0));

    const topJointId = world.createEntity();
    const topEffectiveCw = true;
    const topAttachment = tangentFromCircleToPoint(anchorTopPos, wheelPos, wheelRadius, topEffectiveCw).a_circle;
    world.addComponent(
      topJointId,
      new CableJointComponent(
        wheel,
        anchorTop,
        topAttachment.distanceTo(anchorTopPos),
        topAttachment.clone(),
        anchorTopPos.clone()
      )
    );

    const bottomJointId = world.createEntity();
    const bottomEffectiveCw = false;
    const bottomAttachment = tangentFromCircleToPoint(anchorBottomPos, wheelPos, wheelRadius, bottomEffectiveCw).a_circle;
    world.addComponent(
      bottomJointId,
      new CableJointComponent(
        wheel,
        anchorBottom,
        bottomAttachment.distanceTo(anchorBottomPos),
        bottomAttachment.clone(),
        anchorBottomPos.clone()
      )
    );

    const topPathId = world.createEntity();
    const topPath = new CablePathComponent(
      world,
      [topJointId],
      ['hybrid', 'attachment'],
      [false, false],
      1e6,
      [0.25, 0.0],
      0.0
    );
    world.addComponent(topPathId, topPath);

    const bottomPathId = world.createEntity();
    const bottomPath = new CablePathComponent(
      world,
      [bottomJointId],
      ['hybrid', 'attachment'],
      [true, false],
      1e6,
      [0.35, 0.0],
      0.0
    );
    world.addComponent(bottomPathId, bottomPath);

    _updateAttachmentPoints(world);

    const knotComp = world.getComponent(wheel, HybridKnotAngleComponent);
    expect(knotComp).toBeDefined();
    expect(Number.isFinite(knotComp.pathAngles[String(topPathId)])).toBe(true);
    expect(Number.isFinite(knotComp.pathAngles[String(bottomPathId)])).toBe(true);
    expect(knotComp.pathAngles[String(topPathId)]).not.toBeCloseTo(knotComp.pathAngles[String(bottomPathId)], 6);

    expect(topPath.stored[0]).toBeCloseTo(0.25, 8);
    expect(bottomPath.stored[0]).toBeCloseTo(0.35, 8);
  });
});
