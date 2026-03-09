import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent
} from '../../../src/js/cable_joints_3d/ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

import {
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  tangentFromSphereToSphere,
  signedArcLengthOnWheel
} from '../../../src/js/cable_joints_3d/geometry3.js';


describe('CableAttachmentUpdateSystem (3D)', () => {
  test('increments the hybrid transition step and records phase summaries', () => {
    const world = new World();
    world.setResource('debugRenderPoints', {});
    world.setResource('cableHybridTransitionStep', 10);
    world.setResource('cableEventTrace', true);

    const system = new CableAttachmentUpdateSystem();
    system.update(world);

    expect(world.getResource('cableHybridTransitionStep')).toBe(11);
    expect(world.getResource('cableEventTraceBuffer')).toEqual([
      expect.objectContaining({ type: 'summary', phase: 'begin', step: 11 }),
      expect.objectContaining({ type: 'summary', phase: 'afterAttachment', step: 11 }),
      expect.objectContaining({ type: 'summary', phase: 'afterMerge', step: 11 }),
      expect.objectContaining({ type: 'summary', phase: 'afterSplit', step: 11 }),
      expect.objectContaining({ type: 'summary', phase: 'afterHybrid', step: 11 })
    ]);
  });

  test('Merge joints when positions opposite vertically, conserving rest length', () => {
    const world = new World();
    const center = new Vector3(0, 0, 0);
    const radius = 1;
    const cw = true;
    const planeNormal = new Vector3(0, 0, 1);

    // Create entities
    const point1 = world.createEntity();
    const wheel = world.createEntity();
    const point2 = world.createEntity();

    // Components: links and positions
    world.addComponent(point1, new CableLinkComponent(0.9999, 2, 0, null, planeNormal));
    world.addComponent(wheel, new CableLinkComponent(0, 0, 0, null, planeNormal));
    world.addComponent(point2, new CableLinkComponent(1.0, -2, 0, null, planeNormal));
    world.addComponent(wheel, new PositionComponent(0, 0, 0));
    world.addComponent(point1, new PositionComponent(0.9999, 2, 0));
    world.addComponent(point2, new PositionComponent(1.0, -2, 0));
    world.addComponent(wheel, new RadiusComponent(radius));
    world.addComponent(wheel, new OrientationComponent(0, 0, 0, 1));

    // Initial tangents on the rolling link (wheel)
    const tp1 = tangentFromPointToSphere(
      world.getComponent(point1, PositionComponent).pos,
      center,
      radius,
      planeNormal,
      cw
    );
    const tp2 = tangentFromSphereToPoint(
      world.getComponent(point2, PositionComponent).pos,
      center,
      radius,
      planeNormal,
      cw
    );

    // Create cable joints with initial rest lengths
    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        point1,
        wheel,
        tp1.a_attach.clone().subtract(tp1.a_sphere).length(),
        tp1.a_attach,
        tp1.a_sphere
      )
    );
    const joint2 = world.createEntity();
    world.addComponent(
      joint2,
      new CableJointComponent(
        wheel,
        point2,
        tp2.a_attach.clone().subtract(tp2.a_sphere).length(),
        tp2.a_sphere,
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
    world.getComponent(point1, PositionComponent).pos = new Vector3(1.001, 2, 0);

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
