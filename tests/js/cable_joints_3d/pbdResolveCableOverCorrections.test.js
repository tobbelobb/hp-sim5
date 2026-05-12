import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

import {
  World,
  PositionComponent,
  MassComponent,
  MomentOfInertiaComponent,
  OrientationComponent,
  RadiusComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent
} from '../../../src/js/cable_joints_3d/ecs.js';

import {
  CableJointComponent,
  CablePathComponent,
  CableLinkComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { PBDResolveCableOverCorrections } from '../../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';
import { _updateAttachmentPoints } from '../../../src/js/cable_joints_3d/cable_joints_core.js';

describe('PBDResolveCableOverCorrections (3D)', () => {
  test('test_resolve_over_correction_pushes_apart', () => {
    const world = new World();
    const dt = 1.0 / 60.0;
    world.setResource('dt', dt);

    // --- Setup Entities ---
    // Entity A: A static anchor point at the origin
    const entityA = world.createEntity();
    world.addComponent(entityA, new PositionComponent(0, 0, 0));
    world.addComponent(entityA, new MassComponent(-1.0));
    world.addComponent(entityA, new CableLinkComponent());

    // Entity B: A dynamic body, positioned so the cable is slack
    const entityB = world.createEntity();
    const initialPosB = new Vector3(8.0, 0.0, 0.0);
    world.addComponent(entityB, new PositionComponent(initialPosB.x, initialPosB.y, initialPosB.z));
    world.addComponent(entityB, new MassComponent(1.0));
    world.addComponent(entityB, new MomentOfInertiaComponent(1.0));
    world.addComponent(entityB, new OrientationComponent(0, 0, 0, 1));
    world.addComponent(entityB, new RadiusComponent(1.0));
    world.addComponent(entityB, new CableLinkComponent());

    // --- Setup Cable ---
    // A single joint with rest length 10.
    // The attachment on A is its center (0,0).
    // The attachment on B is its left-most point (-1,0) locally.
    // With B at (8,0), the world attachment point is (7,0).
    // The initial distance is 7, which is < rest_length, so it's slack.
    const jointEntity1 = world.createEntity();
    const joint1 = new CableJointComponent(
      entityA,
      entityB,
      10.0,
      new Vector3(0.0, 0.0, 0.0),
      new Vector3(7.0, 0.0, 0.0)
    );
    world.addComponent(jointEntity1, joint1);

    const jointEntity2 = world.createEntity();
    const joint2 = new CableJointComponent(
      entityB,
      entityA,
      10.0,
      new Vector3(7.0, 0.0, 0.0),
      new Vector3(0.0, 0.0, 0.0)
    );
    world.addComponent(jointEntity2, joint2);

    const pathEntity = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointEntity1, jointEntity2], ['attachment', 'rolling', 'attachment'], [false, false, false]);
    world.addComponent(pathEntity, pathComp);

    // --- Simulate Pre-Solver State ---
    // To trigger the over-correction logic, the system must believe the joint
    // *was* taut before the main solver ran. We simulate this by manually
    // setting the world attachment points to a stretched state and caching them.
    joint1.attachmentPointB_world.set(new Vector3(11.0, 0.0, 0.0)); // Stretched state
    joint2.attachmentPointA_world.set(new Vector3(11.0, 0.0, 0.0));

    const cacheSystem = new CableAttachmentCacheSystem();
    cacheSystem.update(world, dt);

    // --- Run the System Under Test ---
    // The system should now see a joint that was taut (from cache) but is
    // now slack (based on current body positions), and apply a push correction.
    const resolveSystem = new PBDResolveCableOverCorrections();
    resolveSystem.update(world, dt);

    // --- Assertions ---
    const finalPosCompB = world.getComponent(entityB, PositionComponent);
    const finalOrientationCompB = world.getComponent(entityB, OrientationComponent);

    // 1. The primary assertion: Entity B should have been pushed to the right.
    expect(finalPosCompB.pos.x).toBeGreaterThan(initialPosB.x);

    // 2. The distance between attachment points should have increased.
    const initialDistance = 7.0;
    // The new attachment point on B is its center + rotated local attachment.
    // Since angle is 0, it's (final_pos_x - 1, 0).
    const finalAttachmentB = finalPosCompB.pos.clone().add(new Vector3(-1.0, 0.0, 0.0));
    const finalDistance = finalAttachmentB.distanceTo(joint1.attachmentPointA_world);
    expect(finalDistance).toBeGreaterThan(initialDistance);

    // 3. In this symmetrical setup, there should be no vertical movement or rotation.
    expect(finalPosCompB.pos.y).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.x).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.y).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.z).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.w).toBeCloseTo(1.0);
  });

  test('test_resolve_over_correction_pushes_apart_multiple_paths', () => {
    const world = new World();
    const dt = 1.0 / 60.0;
    world.setResource('dt', dt);

    // --- Setup Entities ---
    // Entity A: A static anchor point at the origin
    const entityA = world.createEntity();
    world.addComponent(entityA, new PositionComponent(0, 0, 0));
    world.addComponent(entityA, new MassComponent(-1.0));
    world.addComponent(entityA, new CableLinkComponent());

    // Entity B: A dynamic body, positioned so the cable is slack
    const entityB = world.createEntity();
    const initialPosB = new Vector3(8.0, 0.0, 0.0);
    world.addComponent(entityB, new PositionComponent(initialPosB.x, initialPosB.y, initialPosB.z));
    world.addComponent(entityB, new MassComponent(1.0));
    world.addComponent(entityB, new MomentOfInertiaComponent(1.0));
    world.addComponent(entityB, new OrientationComponent(0, 0, 0, 1));
    world.addComponent(entityB, new RadiusComponent(1.0));
    world.addComponent(entityB, new CableLinkComponent());

    // --- Setup Cable ---
    // A single joint with rest length 10.
    // The attachment on A is its center (0,0).
    // The attachment on B is its left-most point (-1,0) locally.
    // With B at (8,0), the world attachment point is (7,0).
    // The initial distance is 7, which is < rest_length, so it's slack.
    const jointEntity1 = world.createEntity();
    const joint1 = new CableJointComponent(
      entityA,
      entityB,
      10.0,
      new Vector3(0.0, 0.0, 0.0),
      new Vector3(7.0, 0.0, 0.0)
    );
    world.addComponent(jointEntity1, joint1);

    const jointEntity2 = world.createEntity();
    const joint2 = new CableJointComponent(
      entityB,
      entityA,
      10.0,
      new Vector3(7.0, 0.0, 0.0),
      new Vector3(0.0, 0.0, 0.0)
    );
    world.addComponent(jointEntity2, joint2);

    const pathEntity1 = world.createEntity();
    const pathComp1 = new CablePathComponent(world, [jointEntity1], ['attachment', 'attachment'], [false, false]);
    world.addComponent(pathEntity1, pathComp1);

    const pathEntity2 = world.createEntity();
    const pathComp2 = new CablePathComponent(world, [jointEntity2], ['attachment', 'attachment'], [false, false]);
    world.addComponent(pathEntity2, pathComp2);

    // --- Simulate Pre-Solver State ---
    // To trigger the over-correction logic, the system must believe the joint
    // *was* taut before the main solver ran. We simulate this by manually
    // setting the world attachment points to a stretched state and caching them.
    joint1.attachmentPointB_world.set(new Vector3(11.0, 0.0, 0.0)); // Stretched state
    joint2.attachmentPointA_world.set(new Vector3(11.0, 0.0, 0.0));

    const cacheSystem = new CableAttachmentCacheSystem();
    cacheSystem.update(world, dt);

    // --- Run the System Under Test ---
    // The system should now see a joint that was taut (from cache) but is
    // now slack (based on current body positions), and apply a push correction.
    const resolveSystem = new PBDResolveCableOverCorrections();
    resolveSystem.update(world, dt);

    // --- Assertions ---
    const finalPosCompB = world.getComponent(entityB, PositionComponent);
    const finalOrientationCompB = world.getComponent(entityB, OrientationComponent);

    // 1. The primary assertion: Entity B should have been pushed to the right.
    expect(finalPosCompB.pos.x).toBeGreaterThan(initialPosB.x);

    // 2. The distance between attachment points should have increased.
    const initialDistance = 7.0;
    // The new attachment point on B is its center + rotated local attachment.
    // Since angle is 0, it's (final_pos_x - 1, 0).
    const finalAttachmentB = finalPosCompB.pos.clone().add(new Vector3(-1.0, 0.0, 0.0));
    const finalDistance = finalAttachmentB.distanceTo(joint1.attachmentPointA_world);
    expect(finalDistance).toBeGreaterThan(initialDistance);
    const finalDistance2 = finalAttachmentB.distanceTo(joint2.attachmentPointB_world);
    expect(finalDistance).toBeGreaterThan(initialDistance);

    // 3. In this symmetrical setup, there should be no vertical movement or rotation.
    expect(finalPosCompB.pos.y).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.x).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.y).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.z).toBeCloseTo(0.0);
    expect(finalOrientationCompB.quaternion.w).toBeCloseTo(1.0);
  });

  test('single over-corrected joint is ignored', () => {
    const world = new World();
    const dt = 1.0 / 60.0;
    world.setResource('dt', dt);

    const entityA = world.createEntity();
    world.addComponent(entityA, new PositionComponent(0, 0, 0));
    world.addComponent(entityA, new MassComponent(-1));
    world.addComponent(entityA, new CableLinkComponent());

    const entityB = world.createEntity();
    const initialPosB = new Vector3(8, 0, 0);
    world.addComponent(entityB, new PositionComponent(initialPosB.x, initialPosB.y, initialPosB.z));
    world.addComponent(entityB, new MassComponent(1));
    world.addComponent(entityB, new MomentOfInertiaComponent(1));
    world.addComponent(entityB, new OrientationComponent(0, 0, 0, 1));
    world.addComponent(entityB, new RadiusComponent(1));
    world.addComponent(entityB, new CableLinkComponent());

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      new CableJointComponent(
        entityA,
        entityB,
        10.0,
        new Vector3(0, 0, 0),
        new Vector3(7, 0, 0)
      )
    );

    const pathEntity = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointEntity], ['attachment', 'rolling'], [false, false]);
    world.addComponent(pathEntity, pathComp);

    const jointComp = world.getComponent(jointEntity, CableJointComponent);
    jointComp.attachmentPointA_world.set(new Vector3(0, 0, 0));
    jointComp.attachmentPointB_world.set(new Vector3(11, 0, 0));

    const cacheSystem = new CableAttachmentCacheSystem();
    cacheSystem.update(world, dt);

    const resolveSystem = new PBDResolveCableOverCorrections();
    resolveSystem.update(world, dt);

    const finalPosCompB = world.getComponent(entityB, PositionComponent);
    expect(finalPosCompB.pos.x).toBeCloseTo(initialPosB.x);
    expect(finalPosCompB.pos.y).toBeCloseTo(initialPosB.y);
  });
});
