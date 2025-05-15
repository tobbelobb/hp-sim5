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
  _updateHybridLinkStates
} from '../cable_joints/cable_joints_core.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel
} from '../cable_joints/geometry.js';


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

  const EAST = new Vector2(1, 0);
  const WEST = new Vector2(-1, 0);

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
    // positive stored arc, restLength shorter by that arc
    expect(pathComp.stored[1]).toBeGreaterThan(0);
    const arc = pathComp.stored[1];
    const j = world.getComponent(joint, CableJointComponent);
    expect(j.restLength).toBeCloseTo(initialRest - arc, 8);
  });
});
