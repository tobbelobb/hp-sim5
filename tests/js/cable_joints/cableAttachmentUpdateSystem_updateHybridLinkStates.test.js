import Vector2 from '../../../src/js/cable_joints/vector2.js';

import {
  World,
  PositionComponent,
  RadiusComponent
} from '../../../src/js/cable_joints/ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  _updateHybridLinkStates
} from '../../../src/js/cable_joints/cable_joints_core.js';

describe('_updateHybridLinkStates', () => {
  const addWheel = (world, pos, r = 1) => {
    const id = world.createEntity();
    world.addComponent(id, new PositionComponent(pos.x, pos.y));
    world.addComponent(id, new CableLinkComponent(pos.x, pos.y));
    world.addComponent(id, new RadiusComponent(r));
    return id;
  };

  const addAnchor = (world, pos) => {
    const id = world.createEntity();
    world.addComponent(id, new PositionComponent(pos.x, pos.y));
    world.addComponent(id, new CableLinkComponent(pos.x, pos.y));
    return id;
  };

  test('first link: hybrid -> hybrid-attachment when stored is negative', () => {
    const world  = new World();
    const wheel  = addWheel(world, new Vector2(0, 0), 1);
    const anchor = addAnchor(world, new Vector2(0, 3));

    const joint = world.createEntity();
    const initialRest = 3; // arbitrary
    world.addComponent(
      joint,
      new CableJointComponent(
        wheel, anchor, initialRest,
        new Vector2(1, 0), // attachment on wheel
        new Vector2(0, 3), // anchor point
      ),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['hybrid', 'attachment'], // first link hybrid
      [false, false],
    );
    world.addComponent(path, pathComp);
    // Feed out a bit of "negative" rope
    pathComp.stored[0] = -0.2;

    _updateHybridLinkStates(world);

    // link type changed
    expect(pathComp.linkTypes[0]).toBe('hybrid-attachment');
    // stored was clamped back to zero
    expect(pathComp.stored[0]).toBeCloseTo(0);
    // restLength shortened by exactly 0.2
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest - 0.2, 8);
  });

  test('first link: tiny negative stored stays hybrid (hysteresis)', () => {
    const world  = new World();
    const wheel  = addWheel(world, new Vector2(0, 0), 1);
    const anchor = addAnchor(world, new Vector2(0, 3));

    const joint = world.createEntity();
    const initialRest = 3;
    world.addComponent(
      joint,
      new CableJointComponent(
        wheel, anchor, initialRest,
        new Vector2(1, 0),
        new Vector2(0, 3),
      ),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['hybrid', 'attachment'],
      [false, false],
      1e6,
      null,
      0.01
    );
    world.addComponent(path, pathComp);
    pathComp.stored[0] = -0.001;

    _updateHybridLinkStates(world);

    expect(pathComp.linkTypes[0]).toBe('hybrid');
    expect(pathComp.stored[0]).toBeCloseTo(-0.001, 12);
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest, 12);
  });

  test('first link: hybrid -> hybrid-attachment uses layered radius for unwind rotation', () => {
    const world  = new World();
    const wheel  = addWheel(world, new Vector2(0, 0), 1.0);
    const anchor = addAnchor(world, new Vector2(0, 3));

    const joint = world.createEntity();
    const initialRest = 3.0;
    const attachmentBefore = new Vector2(1.0, 0.0);
    const anchorAttachment = new Vector2(0.0, 3.0);
    world.addComponent(
      joint,
      new CableJointComponent(
        wheel, anchor, initialRest,
        attachmentBefore.clone(),
        anchorAttachment.clone(),
      ),
    );

    const path = world.createEntity();
    const cableHalfWidth = 0.2;
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['hybrid', 'attachment'],
      [false, false],
      1e6,
      null,
      cableHalfWidth
    );
    world.addComponent(path, pathComp);

    const oldStored = -0.3;
    pathComp.stored[0] = oldStored;

    const expected = attachmentBefore.clone();
    const expectedRadius = 1.0 + cableHalfWidth;
    const expectedRotation = -oldStored / expectedRadius;
    expected.rotate(expectedRotation, new Vector2(0.0, 0.0), pathComp.cw[0]);

    _updateHybridLinkStates(world);

    expect(pathComp.linkTypes[0]).toBe('hybrid-attachment');
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest + oldStored, 12);
    expect(j.attachmentPointA_world.x).toBeCloseTo(expected.x, 12);
    expect(j.attachmentPointA_world.y).toBeCloseTo(expected.y, 12);
  });

  test('last link: hybrid -> hybrid-attachment when stored is negative', () => {
    const world  = new World();

    const anchor = addAnchor(world, new Vector2(-1, 0));
    const wheel  = addWheel(world, new Vector2(0, 0), 1);

    const joint = world.createEntity();
    const initialRest = 2;
    world.addComponent(
      joint,
      new CableJointComponent(
        anchor, wheel, initialRest,
        new Vector2(-1, 0),
        new Vector2(1, 0),
      ),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['attachment', 'hybrid'],
      [false, false],
    );
    world.addComponent(path, pathComp);
    pathComp.stored[1] = -0.15;

    _updateHybridLinkStates(world);

    expect(pathComp.linkTypes[1]).toBe('hybrid-attachment');
    expect(pathComp.stored[1]).toBeCloseTo(0);
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest - 0.15, 8);
  });

  test('first link: hybrid-attachment -> hybrid when rope wraps onto wheel again', () => {
    const world  = new World();

    // Geometry chosen so the rope must wrap CCW onto the wheel
    const wheel  = addWheel(world, new Vector2(0, 0), 1);
    const anchor = addAnchor(world, new Vector2(0, 3));

    const joint = world.createEntity();
    const initialRest = 3.2;
    world.addComponent(
      joint,
      new CableJointComponent(
        wheel, anchor, initialRest,
        new Vector2(1, 0), // start at 0 degrees
        new Vector2(0, 3), // anchor
      ),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['hybrid-attachment', 'attachment'],
      [false, false], // initial cw flag for hybrid-attachment is arbitrary
    );
    world.addComponent(path, pathComp);

    // Nothing in stored; hybrid-attachment means "ready to re-wrap"
    _updateHybridLinkStates(world);

    //  switched back to hybrid
    expect(pathComp.linkTypes[0]).toBe('hybrid');
    // index 0 stores raw cw; effective cw (used for circle->point) is inverted
    expect(pathComp.cw[0]).toBe(true);
    // some positive arc was added
    expect(pathComp.stored[0]).toBeGreaterThan(0);
    // restLength reduced by exactly that arc
    const arc = pathComp.stored[0];
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest - arc, 8);
  });

  test('last link: hybrid-attachment -> hybrid when rope wraps onto wheel again', () => {
    const world  = new World();

    const anchor = addAnchor(world, new Vector2(0, -3));
    const wheel  = addWheel(world, new Vector2(0, 0), 1);

    const joint = world.createEntity();
    const initialRest = 3.1;
    world.addComponent(
      joint,
      new CableJointComponent(
        anchor, wheel, initialRest,
        new Vector2(0, -3), // anchor point
        new Vector2(1, 0),  // start at 0 degrees
      ),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['attachment', 'hybrid-attachment'],
      [false, false],
    );
    world.addComponent(path, pathComp);

    _updateHybridLinkStates(world);

    // switched back to hybrid
    expect(pathComp.linkTypes[1]).toBe('hybrid');
    // last-link cw is not inverted
    expect(pathComp.cw[1]).toBe(false);
    // positive stored arc, restLength shorter by that arc
    expect(pathComp.stored[1]).toBeGreaterThan(0);
    const arc = pathComp.stored[1];
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest - arc, 8);
  });

  test('first link: tiny re-wrap arc stays hybrid-attachment (hysteresis)', () => {
    const baselineWorld = new World();
    const baselineWheel = addWheel(baselineWorld, new Vector2(0, 0), 1);
    const baselineAnchor = addAnchor(baselineWorld, new Vector2(0, 2));
    const baselineJoint = baselineWorld.createEntity();
    const initialRest = 3.0;
    const tinyArcAttach = new Vector2(0.8771471956617061, 0.4802216125319691);
    baselineWorld.addComponent(
      baselineJoint,
      new CableJointComponent(
        baselineWheel, baselineAnchor, initialRest,
        tinyArcAttach.clone(),
        new Vector2(0, 2),
      ),
    );
    const baselinePath = baselineWorld.createEntity();
    const baselinePathComp = new CablePathComponent(
      baselineWorld,
      [baselineJoint],
      ['hybrid-attachment', 'attachment'],
      [false, false],
      1e6,
      null,
      0.0
    );
    baselineWorld.addComponent(baselinePath, baselinePathComp);
    _updateHybridLinkStates(baselineWorld);
    expect(baselinePathComp.linkTypes[0]).toBe('hybrid');
    expect(baselinePathComp.stored[0]).toBeGreaterThan(1e-4);

    const world = new World();
    const wheel = addWheel(world, new Vector2(0, 0), 1);
    const anchor = addAnchor(world, new Vector2(0, 2));
    const joint = world.createEntity();
    world.addComponent(
      joint,
      new CableJointComponent(
        wheel, anchor, initialRest,
        tinyArcAttach.clone(),
        new Vector2(0, 2),
      ),
    );
    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['hybrid-attachment', 'attachment'],
      [false, false],
      1e6,
      null,
      0.1
    );
    world.addComponent(path, pathComp);

    _updateHybridLinkStates(world);

    expect(pathComp.linkTypes[0]).toBe('hybrid-attachment');
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 12);
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest, 12);
  });

  test('hybrid-attachment endpoint keeps cw stable for near-degenerate endpoint geometry', () => {
    const world  = new World();
    const wheel  = addWheel(world, new Vector2(0, 0), 1);
    const anchor = addAnchor(world, new Vector2(0.001, 0.0));

    const joint = world.createEntity();
    const initialRest = 1.0;
    world.addComponent(
      joint,
      new CableJointComponent(
        wheel, anchor, initialRest,
        new Vector2(1.0, 0.0),
        new Vector2(1.0005, 0.0),
      ),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['hybrid-attachment', 'attachment'],
      [true, false],
      1e6,
      null,
      0.01
    );
    world.addComponent(path, pathComp);

    _updateHybridLinkStates(world);

    expect(pathComp.linkTypes[0]).toBe('hybrid-attachment');
    expect(pathComp.cw[0]).toBe(true);
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 12);
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest, 12);
  });

});
