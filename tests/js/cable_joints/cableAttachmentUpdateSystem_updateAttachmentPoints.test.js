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
  _updateAttachmentPoints
} from '../../../src/js/cable_joints/cable_joints_core.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel
} from '../../../src/js/cable_joints/geometry.js';

const TEST_KNOT_SPAN = Math.PI / 30.0;
const TEST_EPSILON = 1e-9;

function _testLayerWrapParams(r0, dr, rampLength, layerIndex) {
  const twoPi = 2.0 * Math.PI;
  const rn = r0 + dr * layerIndex;
  let dPhiRamp = 0.0;
  if (rampLength > TEST_EPSILON) {
    dPhiRamp = rampLength / (rn + 0.5 * dr);
    if (dPhiRamp > twoPi) dPhiRamp = twoPi;
    if (dPhiRamp < 0.0) dPhiRamp = 0.0;
  }
  const phiConst = twoPi - dPhiRamp;
  const lConst = rn * phiConst;
  return { rn, dPhiRamp, phiConst, lConst, lWrap: lConst + dPhiRamp * (rn + 0.5 * dr) };
}

function _testStoredInWrapAtPhi(phi, dr, wrap) {
  const twoPi = 2.0 * Math.PI;
  const phiClamped = Math.max(0.0, Math.min(twoPi, phi));
  if (!(wrap.dPhiRamp > TEST_EPSILON) || phiClamped <= wrap.phiConst + TEST_EPSILON) {
    return wrap.rn * Math.min(wrap.phiConst, phiClamped);
  }
  const x = phiClamped - wrap.phiConst;
  const a = dr / (2.0 * wrap.dPhiRamp);
  return wrap.lConst + wrap.rn * x + a * x * x;
}

function _testThetaToStored(theta, baseRadius, halfWidth, rampLength) {
  if (!Number.isFinite(theta) || Math.abs(theta) <= TEST_EPSILON) {
    return 0.0;
  }
  if (theta < 0.0) {
    return -_testThetaToStored(-theta, baseRadius, halfWidth, rampLength);
  }

  const r0 = baseRadius + halfWidth;
  const dr = 2.0 * halfWidth;
  if (!(r0 > TEST_EPSILON) || !(dr > TEST_EPSILON)) {
    return Math.max(baseRadius, 0.0) * theta;
  }

  const twoPi = 2.0 * Math.PI;
  const maxLayers = 2048;
  let remainingTheta = theta;
  let stored = 0.0;
  let layer = 0;

  while (remainingTheta > twoPi + TEST_EPSILON && layer < maxLayers) {
    const wrap = _testLayerWrapParams(r0, dr, rampLength, layer);
    stored += wrap.lWrap;
    remainingTheta -= twoPi;
    layer++;
  }

  if (layer >= maxLayers) {
    const rn = r0 + dr * maxLayers;
    return stored + rn * remainingTheta;
  }

  const wrap = _testLayerWrapParams(r0, dr, rampLength, layer);
  return stored + _testStoredInWrapAtPhi(remainingTheta, dr, wrap);
}

describe('_updateAttachmentPoints', () => {
  test('Attachment to Rolling - Translation', () => {
    const world = new World();

    // Entities
    const attachPoint = world.createEntity();
    const rollingLink = world.createEntity();

    // Components
    const attachPos = new Vector2(0, 2);
    const rollingPos = new Vector2(0, 0);
    const rollingRadius = 1.0;
    const cw = true;

    world.addComponent(attachPoint, new PositionComponent(attachPos.x, attachPos.y));
    world.addComponent(attachPoint, new CableLinkComponent(attachPos.x, attachPos.y)); // prevPos = current pos initially

    world.addComponent(rollingLink, new PositionComponent(rollingPos.x, rollingPos.y));
    world.addComponent(rollingLink, new RadiusComponent(rollingRadius));
    world.addComponent(rollingLink, new CableLinkComponent(rollingPos.x, rollingPos.y)); // prevPos = current pos initially

    // Initial Tangent
    const initialTangent = tangentFromPointToCircle(attachPos, rollingPos, rollingRadius, cw);
    const initialAttachA = initialTangent.a_attach;
    const initialAttachB = initialTangent.a_circle;
    const initialRestLength = initialAttachA.distanceTo(initialAttachB);

    // Cable Joint
    const jointId = world.createEntity();
    const jointComp = new CableJointComponent(attachPoint, rollingLink, initialRestLength, initialAttachA.clone(), initialAttachB.clone());
    world.addComponent(jointId, jointComp);

    // Cable Path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointId], ['attachment', 'hybrid'], [cw, cw]);
    world.addComponent(pathId, pathComp);
    const initialStoredB = pathComp.stored[1];
    expect(initialStoredB).toBeCloseTo(0.0, 6);

    // --- Simulate Movement ---
    const moveVector = new Vector2(0.5, 0);
    const newAttachPos = attachPos.clone().add(moveVector);
    world.getComponent(attachPoint, PositionComponent).pos.set(newAttachPos); // Update current position

    _updateAttachmentPoints(world);

    // --- Assertions ---
    // Calculate expected new tangent points
    const expectedTangent = tangentFromPointToCircle(newAttachPos, rollingPos, rollingRadius, cw);
    const expectedAttachA = expectedTangent.a_attach;
    const expectedAttachB = expectedTangent.a_circle;

    // Check attachment points
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(expectedAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(expectedAttachA.y);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(expectedAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(expectedAttachB.y);

    // Check stored length change (sB)
    const expectedSB = signedArcLengthOnWheel(
        initialAttachB.clone().subtract(rollingPos), // prevAttachB relative to prevCenterB
        expectedAttachB.clone().subtract(rollingPos), // currentAttachB relative to currentCenterB
        new Vector2(0,0), // Center offset for calculation is zero
        rollingRadius,
        cw
    );
    expect(pathComp.stored[1]).toBeCloseTo(initialStoredB - expectedSB); // stored[B] -= sB

    // Check rest length change
    // restLength += sB
    expect(jointComp.restLength).toBeCloseTo(initialRestLength + expectedSB);

    // Total rest length of path should remain constant
    const finalTotalRestLength = jointComp.restLength + pathComp.stored[0] + pathComp.stored[1];
    expect(finalTotalRestLength).toBeCloseTo(pathComp.totalRestLength);

  });

  test('Rolling to Rolling (hybrid) - Translation', () => {
    const world = new World();

    // Entities
    const attach0 = world.createEntity();
    const rollingA = world.createEntity();
    const rollingB = world.createEntity();

    // Components
    const pos0 = new Vector2(-2.5, -2);
    const posA = new Vector2(-2.0, 0);
    const radiusA = 0.5;
    const cwA = true;
    const posB = new Vector2(2.0, 0);
    const radiusB = 0.5;
    const cwB = true;

    world.addComponent(attach0, new PositionComponent(pos0.x, pos0.y));
    world.addComponent(attach0, new CableLinkComponent(pos0.x, pos0.y)); // prevPos

    world.addComponent(rollingA, new PositionComponent(posA.x, posA.y));
    world.addComponent(rollingA, new RadiusComponent(radiusA));
    world.addComponent(rollingA, new CableLinkComponent(posA.x, posA.y)); // prevPos

    world.addComponent(rollingB, new PositionComponent(posB.x, posB.y));
    world.addComponent(rollingB, new RadiusComponent(radiusB));
    world.addComponent(rollingB, new CableLinkComponent(posB.x, posB.y)); // prevPos

    // Initial Tangents and joint attach->rolling
    const initialTangents_0 = tangentFromPointToCircle(pos0, posA, radiusA, cwA);
    const initialAttachA_0 = initialTangents_0.a_attach;
    const initialAttachB_0 = initialTangents_0.a_circle;
    const initialRestLength_0 = initialAttachA_0.distanceTo(initialAttachB_0);
    const jointId_0 = world.createEntity();
    const jointComp_0 = new CableJointComponent(attach0, rollingA, initialRestLength_0, initialAttachA_0, initialAttachB_0);
    world.addComponent(jointId_0, jointComp_0);

    // Initial Tangents Rolling links
    const initialTangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    const initialAttachA = initialTangents.a_circle;
    const initialAttachB = initialTangents.b_circle;
    const initialRestLength = initialAttachA.distanceTo(initialAttachB);

    // Cable Joint
    const jointId = world.createEntity();
    const jointComp = new CableJointComponent(rollingA, rollingB, initialRestLength, initialAttachA.clone(), initialAttachB.clone());
    world.addComponent(jointId, jointComp);
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(initialAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(initialAttachA.y);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(initialAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(initialAttachB.y);

    // Cable Path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointId_0, jointId], ['attachment', 'rolling', 'hybrid'], [true, cwA, cwB]);
    world.addComponent(pathId, pathComp);
    const initialStored0 = pathComp.stored[0];
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];

    // --- Simulate Movement ---
    const moveVectorA = new Vector2(0.0, 0.1); // Move A slightly up
    const newPosA = posA.clone().add(moveVectorA);
    world.getComponent(rollingA, PositionComponent).pos.set(newPosA);

    _updateAttachmentPoints(world);

    // --- Assertions ---
    // Calculate expected new tangent points
    const expectedTangents = tangentFromCircleToCircle(newPosA, radiusA, cwA, posB, radiusB, cwB);
    const expectedAttachA = expectedTangents.a_circle;
    const expectedAttachB = expectedTangents.b_circle;

    // Check attachment points
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(expectedAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(expectedAttachA.y);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(expectedAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(expectedAttachB.y);

    // Check stored length changes (sA, sB)
    const expectedSA = signedArcLengthOnWheel(
        initialAttachA.clone().subtract(posA),    // prevAttachA relative to prevCenterA
        expectedAttachA.clone().subtract(newPosA), // currentAttachA relative to currentCenterA
        new Vector2(0,0), radiusA, cwA
    );

    const expectedSB = signedArcLengthOnWheel(
        initialAttachB.clone().subtract(posB), // prevAttachB relative to prevCenterB
        expectedAttachB.clone().subtract(posB), // currentAttachB relative to currentCenterB
        new Vector2(0,0), radiusB, cwB
    );

    expect(pathComp.stored[1]).toBeCloseTo(initialStoredA + expectedSA); // stored[A] += sA
    expect(pathComp.stored[2]).toBeCloseTo(initialStoredB - expectedSB); // stored[B] -= sB

    // Check rest length change
    // restLength -= sA; restLength += sB;
    expect(jointComp.restLength).toBeCloseTo(initialRestLength - expectedSA + expectedSB);

    // Total rest length of path should remain constant
    const finalTotalRestLength = jointComp_0.restLength + jointComp.restLength + pathComp.stored[0] + pathComp.stored[1] + pathComp.stored[2];
     expect(finalTotalRestLength).toBeCloseTo(pathComp.totalRestLength);
  });

  test('hybrid-attachment -> rolling -> rolling -> hybrid with small rotations', () => {
    const world = new World();

    // Entities: hybrid-attachment start, two rolling links, hybrid end
    const r = 0.5;
    const startAttach = world.createEntity();
    const rollA = world.createEntity();
    const rollB = world.createEntity();
    const endHybrid = world.createEntity();
    const cwStart = true;
    const cwEnd = true;

    // Initial positions and parameters
    const posStart = new Vector2(-2.5, -2.0);
    const posA     = new Vector2(-2.0,  0.0);
    const posB     = new Vector2( 2.0,  0.0);
    const posEnd   = new Vector2( 2.5,  2.0);
    const cwA = true;
    const cwB = false;
    const smallRot = 0.1;

    // Setup positions, radii, orientations on all four entities
    [ [startAttach, posStart, r],
      [rollA, posA, r],
      [rollB, posB, r],
      [endHybrid, posEnd, r] ].forEach(([e, pos, r_], i) => {
      world.addComponent(e, new PositionComponent(pos.x, pos.y));
      world.addComponent(e, new OrientationComponent(0.0));
      world.addComponent(e, new RadiusComponent(r_));
      world.addComponent(e, new CableLinkComponent(pos.x, pos.y));
    });

    // Create joints: start->rollA, rollA->rollB, rollB->end
    const jointIds = [];
    const t0_tangent = tangentFromPointToCircle(posStart, posA, r, cwA);
    const t0_dir = t0_tangent.a_circle.clone().subtract(t0_tangent.a_attach).normalize();
    const t0_hybrid_attach = posStart.clone().add(t0_dir.scale(r));
    const t1 = tangentFromCircleToCircle(posA, r, cwA, posB, r, cwB);
    const t2 = tangentFromCircleToCircle(posB, r, cwB, posEnd, r, cwEnd);
    const tArgs = [
      [startAttach, rollA, t0_hybrid_attach, t0_tangent.a_circle],
      [rollA, rollB, t1.a_circle, t1.b_circle],
      [rollB, endHybrid, t2.a_circle, t2.b_circle],
    ];
    tArgs.forEach(([a, b, Apt, Bpt]) => {
      const id = world.createEntity();
      world.addComponent(id, new CableJointComponent(
        a, b, Apt.distanceTo(Bpt), Apt.clone(), Bpt.clone()
      ));
      jointIds.push(id);
    });
    const firstJoint = world.getComponent(jointIds[0], CableJointComponent);
    expect(firstJoint.attachmentPointA_world).toEqual(t0_hybrid_attach);
    expect(firstJoint.attachmentPointB_world).toEqual(t0_tangent.a_circle);

    // Build the cable path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      jointIds,
      ['hybrid-attachment', 'rolling', 'rolling', 'hybrid'],
      [cwStart, cwA, cwB, cwEnd]
    );
    world.addComponent(pathId, pathComp);
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];
    const initialStoredEnd = pathComp.stored[3];
    expect(initialStoredEnd).toBeCloseTo(0.0, 8);

    // Rotate both hybrid ends slightly before update
    world.getComponent(startAttach, OrientationComponent).angle += smallRot;
    world.getComponent(endHybrid, OrientationComponent).angle += -smallRot;

    // Run attachment update
    _updateAttachmentPoints(world);

    // Verity stored
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    expect(pathComp.stored[1]).toBeLessThan(initialStoredA);
    expect(pathComp.stored[2]).toBeCloseTo(initialStoredB, 8);
    expect(pathComp.stored[3]).toBeCloseTo(r*smallRot, 8);

    // Verify that the hybrid ends have moved from their original tangents
    expect(firstJoint.attachmentPointA_world).not.toEqual(t0_hybrid_attach);
    expect(firstJoint.attachmentPointA_world.x).toBeLessThan(t0_hybrid_attach.x);
    expect(firstJoint.attachmentPointA_world.y).toBeLessThan(t0_hybrid_attach.y);
    expect(firstJoint.attachmentPointA_world.distanceTo(t0_hybrid_attach)).toBeLessThan(r*smallRot);
    expect(firstJoint.attachmentPointA_world.distanceTo(posStart)).toBeCloseTo(r, 6);

    expect(firstJoint.attachmentPointB_world).not.toEqual(t0_tangent.a_circle);
    expect(firstJoint.attachmentPointB_world.x).toBeGreaterThan(t0_tangent.a_circle.x);
    expect(firstJoint.attachmentPointB_world.y).toBeGreaterThan(t0_tangent.a_circle.y);
    expect(firstJoint.attachmentPointB_world.distanceTo(t0_tangent.a_circle)).toBeLessThan(r*smallRot);
    expect(firstJoint.attachmentPointB_world.distanceTo(posA)).toBeCloseTo(r, 6);

    const lastJoint = world.getComponent(jointIds[jointIds.length - 1], CableJointComponent);
    expect(lastJoint.attachmentPointB_world).toEqual(t2.b_circle);
  });

  test('hybrid -> rolling -> rolling -> hybrid-attachment with small rotations', () => {
    const world = new World();

    // Entities: hybrid-attachment start, two rolling links, hybrid end
    const r = 0.5;
    const startHybrid = world.createEntity();
    const rollA = world.createEntity();
    const rollB = world.createEntity();
    const endHybridAttach = world.createEntity();
    const cwStart = false;
    const cwEnd = true;

    // Initial positions and parameters
    const posStart = new Vector2(-3.0, -2.0);
    const posA     = new Vector2(-2.0,  0.0);
    const posB     = new Vector2( 2.0,  0.0);
    const posEnd   = new Vector2( 2.5,  2.0);
    const cwA = true;
    const cwB = false;
    const smallRot = 0.1;

    // Setup positions, radii, orientations on all four entities
    [ [startHybrid, posStart, r],
      [rollA, posA, r],
      [rollB, posB, r],
      [endHybridAttach, posEnd, r] ].forEach(([e, pos, r_], i) => {
      world.addComponent(e, new PositionComponent(pos.x, pos.y));
      world.addComponent(e, new OrientationComponent(0.0));
      world.addComponent(e, new RadiusComponent(r_));
      world.addComponent(e, new CableLinkComponent(pos.x, pos.y));
    });

    // Create joints: start->rollA, rollA->rollB, rollB->end
    const jointIds = [];
    // NOTE: This !cwStart is the biggest gotcha in the whole code base.
    //       cw/ccw are treated differently for hybrid links at position A.
    const t0 = tangentFromCircleToCircle(posStart, r, !cwStart, posA, r, cwA); // !cwStart because of _effectiveCW
    const t1 = tangentFromCircleToCircle(posA, r, cwA, posB, r, cwB);
    const t2_tangent = tangentFromCircleToPoint(posEnd, posB, r, cwB);
    const t2_dir = t2_tangent.a_circle.clone().subtract(t2_tangent.a_attach).normalize();
    const t2_hybrid_attach = posEnd.clone().add(t2_dir.scale(r));
    const tArgs = [
      [startHybrid, rollA, t0.a_circle, t0.b_circle],
      [rollA, rollB, t1.a_circle, t1.b_circle],
      [rollB, endHybridAttach, t2_tangent.a_circle, t2_hybrid_attach],
    ];
    tArgs.forEach(([a, b, Apt, Bpt]) => {
      const id = world.createEntity();
      world.addComponent(id, new CableJointComponent(
        a, b, Apt.distanceTo(Bpt), Apt.clone(), Bpt.clone()
      ));
      jointIds.push(id);
    });
    const lastJoint = world.getComponent(jointIds[jointIds.length - 1], CableJointComponent);
    expect(lastJoint.attachmentPointA_world).toEqual(t2_tangent.a_circle);
    expect(lastJoint.attachmentPointB_world).toEqual(t2_hybrid_attach);

    // Build the cable path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      jointIds,
      ['hybrid', 'rolling', 'rolling', 'hybrid-attachment'],
      [cwStart, cwA, cwB, cwEnd]
    );
    expect(lastJoint.attachmentPointA_world).toEqual(t2_tangent.a_circle);
    expect(lastJoint.attachmentPointB_world).toEqual(t2_hybrid_attach);
    world.addComponent(pathId, pathComp);
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];
    const initialStoredEnd = pathComp.stored[3];
    expect(initialStoredEnd).toBeCloseTo(0.0, 8);

    // Rotate both hybrid ends slightly before update
    world.getComponent(startHybrid, OrientationComponent).angle += smallRot;
    world.getComponent(endHybridAttach, OrientationComponent).angle += -smallRot;

    // Run attachment update
    _updateAttachmentPoints(world);

    // Verity stored
    expect(pathComp.stored[0]).toBeGreaterThan(0.0, 8);
    expect(pathComp.stored[0]).toBeCloseTo(r*smallRot, 8);
    expect(pathComp.stored[1]).toBeCloseTo(initialStoredA, 8);
    expect(pathComp.stored[2]).toBeGreaterThan(initialStoredB, 8);

    // Verify that the hybrid ends have moved from their original tangents
    expect(lastJoint.attachmentPointB_world).not.toEqual(t2_hybrid_attach);
    expect(lastJoint.attachmentPointB_world.x).toBeLessThan(t2_hybrid_attach.x);
    expect(lastJoint.attachmentPointB_world.y).toBeGreaterThan(t2_hybrid_attach.y);
    expect(lastJoint.attachmentPointB_world.distanceTo(t2_hybrid_attach)).toBeLessThan(r*smallRot);
    expect(lastJoint.attachmentPointB_world.distanceTo(posEnd)).toBeCloseTo(r, 6);

    expect(lastJoint.attachmentPointA_world).not.toEqual(t2_tangent.a_circle);
    expect(lastJoint.attachmentPointA_world.x).toBeLessThan(t2_tangent.a_circle.x);
    expect(lastJoint.attachmentPointA_world.y).toBeGreaterThan(t2_tangent.a_circle.y);
    expect(lastJoint.attachmentPointA_world.distanceTo(t2_tangent.a_circle)).toBeLessThan(r*smallRot);
    expect(lastJoint.attachmentPointA_world.distanceTo(posB)).toBeCloseTo(r, 6);

    const firstJoint = world.getComponent(jointIds[0], CableJointComponent);
    expect(firstJoint.attachmentPointA_world).toEqual(t0.a_circle);
  });

  test('clampRestLength keeps hybrid attachment geometry aligned with unclamped update', () => {
    const buildWorld = (clampEnabled) => {
      const world = new World();
      const spool = world.createEntity();
      const anchor = world.createEntity();
      const spoolPos = new Vector2(0.0, 0.0);
      const anchorPos = new Vector2(0.0, 3.0);
      const spoolRadius = 1.0;
      const cwRaw = false; // _effectiveCW inverts index 0 to true
      const cwEffective = true;
      const deltaAngle = 0.2;

      world.addComponent(spool, new PositionComponent(spoolPos.x, spoolPos.y));
      world.addComponent(spool, new OrientationComponent(deltaAngle));
      world.addComponent(spool, new RadiusComponent(spoolRadius));
      world.addComponent(spool, new CableLinkComponent(spoolPos.x, spoolPos.y, 0.0));

      world.addComponent(anchor, new PositionComponent(anchorPos.x, anchorPos.y));
      world.addComponent(anchor, new CableLinkComponent(anchorPos.x, anchorPos.y, 0.0));

      const initialA = tangentFromCircleToPoint(anchorPos, spoolPos, spoolRadius, cwEffective).a_circle;
      const initialB = anchorPos.clone();
      const tinyRest = 1e-4;
      const jointId = world.createEntity();
      world.addComponent(
        jointId,
        new CableJointComponent(spool, anchor, tinyRest, initialA.clone(), initialB.clone())
      );

      const pathId = world.createEntity();
      const pathComp = new CablePathComponent(
        world,
        [jointId],
        ['hybrid', 'attachment'],
        [cwRaw, false]
      );
      world.addComponent(pathId, pathComp);
      world.removeComponent(spool, HybridKnotAngleComponent);
      world.setResource('layeringClampJointRestLength', clampEnabled);

      return { world, spool, jointId };
    };

    const clamped = buildWorld(true);
    const unclamped = buildWorld(false);
    _updateAttachmentPoints(clamped.world);
    _updateAttachmentPoints(unclamped.world);

    const clampedJoint = clamped.world.getComponent(clamped.jointId, CableJointComponent);
    const unclampedJoint = unclamped.world.getComponent(unclamped.jointId, CableJointComponent);

    expect(clampedJoint.attachmentPointA_world.x).toBeCloseTo(unclampedJoint.attachmentPointA_world.x, 12);
    expect(clampedJoint.attachmentPointA_world.y).toBeCloseTo(unclampedJoint.attachmentPointA_world.y, 12);
    expect(clampedJoint.attachmentPointB_world.x).toBeCloseTo(unclampedJoint.attachmentPointB_world.x, 12);
    expect(clampedJoint.attachmentPointB_world.y).toBeCloseTo(unclampedJoint.attachmentPointB_world.y, 12);

    const clampedRest = clampedJoint.restLength;
    const unclampedRest = unclampedJoint.restLength;
    expect(clampedRest).toBeGreaterThan(0.0);
    expect(unclampedRest).toBeLessThan(0.0);
  });

  test('hybrid layered rotation updates stored via theta->stored across wrap boundary', () => {
    const world = new World();

    const spool = world.createEntity();
    const anchor = world.createEntity();
    const spoolPos = new Vector2(0.0, 0.0);
    const anchorPos = new Vector2(0.0, 3.0);
    const baseRadius = 1.0;
    const halfWidth = 0.05;
    const firstLayerRadius = baseRadius + halfWidth;
    const cwRaw = false; // _effectiveCW at index 0 inverts this to true
    const cwEffective = true;
    const thetaBefore = (2.0 * Math.PI) - 0.12;
    const deltaAngle = 0.5;
    const rampLength = baseRadius * TEST_KNOT_SPAN;

    world.addComponent(spool, new PositionComponent(spoolPos.x, spoolPos.y));
    world.addComponent(spool, new OrientationComponent(deltaAngle));
    world.addComponent(spool, new RadiusComponent(baseRadius));
    world.addComponent(spool, new CableLinkComponent(spoolPos.x, spoolPos.y, 0.0));

    world.addComponent(anchor, new PositionComponent(anchorPos.x, anchorPos.y));
    world.addComponent(anchor, new CableLinkComponent(anchorPos.x, anchorPos.y, 0.0));

    const initialA = tangentFromCircleToPoint(anchorPos, spoolPos, firstLayerRadius, cwEffective).a_circle;
    const initialB = anchorPos.clone();
    const jointId = world.createEntity();
    world.addComponent(
      jointId,
      new CableJointComponent(spool, anchor, 10.0, initialA.clone(), initialB.clone())
    );

    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [cwRaw, false],
      1e6,
      null,
      halfWidth
    );

    const oldStored = pathComp.stored[0] ?? 0.0;
    const initialStored = _testThetaToStored(thetaBefore, baseRadius, halfWidth, rampLength);
    pathComp.stored[0] = initialStored;
    pathComp.totalRestLength += (initialStored - oldStored);

    world.addComponent(pathId, pathComp);
    world.removeComponent(spool, HybridKnotAngleComponent);
    world.setResource('layeringClampJointRestLength', false);

    _updateAttachmentPoints(world);

    const expectedDelta = _testThetaToStored(thetaBefore + deltaAngle, baseRadius, halfWidth, rampLength)
      - _testThetaToStored(thetaBefore, baseRadius, halfWidth, rampLength);
    expect(pathComp.stored[0]).toBeCloseTo(initialStored + expectedDelta, 8);

    // Ensure this is not just a constant-radius update.
    const linearDelta = deltaAngle * firstLayerRadius;
    expect(Math.abs(expectedDelta - linearDelta)).toBeGreaterThan(1e-4);
  });

  test('hybrid endpoint tangent uses layered rolling radius when stored exceeds one full wrap', () => {
    const world = new World();

    const spool = world.createEntity();
    const anchor = world.createEntity();
    const spoolPos = new Vector2(0.0, 0.0);
    const anchorPos = new Vector2(0.0, 3.0);
    const spoolRadius = 1.0;
    const halfWidth = 0.05;
    const baseRadius = spoolRadius;
    const firstLayerRadius = baseRadius + halfWidth;
    const fullWidth = 2.0 * halfWidth;

    world.addComponent(spool, new PositionComponent(spoolPos.x, spoolPos.y));
    world.addComponent(spool, new RadiusComponent(spoolRadius));
    world.addComponent(spool, new CableLinkComponent(spoolPos.x, spoolPos.y));
    world.addComponent(anchor, new PositionComponent(anchorPos.x, anchorPos.y));
    world.addComponent(anchor, new CableLinkComponent(anchorPos.x, anchorPos.y));

    const jointId = world.createEntity();
    world.addComponent(
      jointId,
      new CableJointComponent(
        spool,
        anchor,
        3.0,
        new Vector2(baseRadius, 0.0),
        anchorPos.clone()
      )
    );

    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [false, false],
      1e6,
      null,
      halfWidth
    );
    const oneFullWrap = 2.0 * Math.PI * firstLayerRadius;
    pathComp.stored[0] = oneFullWrap + 0.2;
    pathComp.totalRestLength += oneFullWrap + 0.2;
    world.addComponent(pathId, pathComp);

    _updateAttachmentPoints(world);

    const joint = world.getComponent(jointId, CableJointComponent);
    const expectedTopRadius = firstLayerRadius + fullWidth;
    expect(joint.attachmentPointA_world.distanceTo(spoolPos)).toBeCloseTo(expectedTopRadius, 6);
  });
});
