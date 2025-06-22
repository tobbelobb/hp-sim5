import Vector2 from '../../../src/js/cable_joints/vector2.js';

import {
  World,
  PositionComponent,
  MassComponent,
  MomentOfInertiaComponent,
  OrientationComponent,
  RadiusComponent
} from '../../../src/js/cable_joints/ecs.js';

import {
  CableJointComponent,
  CablePathComponent,
  CableLinkComponent
} from '../../../src/js/cable_joints/cable_joints_core.js';

import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints/cable_attachment_cache_system.js';
import { PBDResolveCableOverCorrections } from '../../../src/js/cable_joints/pbdResolveCableOverCorrections.js';
import { _updateAttachmentPoints } from '../../../src/js/cable_joints/cable_joints_core.js';

describe('PBDResolveCableOverCorrections', () => {
  test('single over-corrected joint is ignored', () => {
    const world = new World();
    const dt = 1.0 / 60.0;
    world.setResource('dt', dt);

    const entityA = world.createEntity();
    world.addComponent(entityA, new PositionComponent(0, 0));
    world.addComponent(entityA, new MassComponent(-1));
    world.addComponent(entityA, new CableLinkComponent());

    const entityB = world.createEntity();
    const initialPosB = new Vector2(8, 0);
    world.addComponent(entityB, new PositionComponent(initialPosB.x, initialPosB.y));
    world.addComponent(entityB, new MassComponent(1));
    world.addComponent(entityB, new MomentOfInertiaComponent(1));
    world.addComponent(entityB, new OrientationComponent(0));
    world.addComponent(entityB, new RadiusComponent(1));
    world.addComponent(entityB, new CableLinkComponent());

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      new CableJointComponent(
        entityA,
        entityB,
        10.0,
        new Vector2(0, 0),
        new Vector2(7, 0)
      )
    );

    const pathEntity = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointEntity], ['attachment', 'rolling'], [false, false]);
    world.addComponent(pathEntity, pathComp);

    const jointComp = world.getComponent(jointEntity, CableJointComponent);
    jointComp.attachmentPointA_world.set(new Vector2(0, 0));
    jointComp.attachmentPointB_world.set(new Vector2(11, 0));

    const cacheSystem = new CableAttachmentCacheSystem();
    cacheSystem.update(world, dt);

    const resolveSystem = new PBDResolveCableOverCorrections();
    resolveSystem.update(world, dt);

    const finalPosCompB = world.getComponent(entityB, PositionComponent);
    expect(finalPosCompB.pos.x).toBeCloseTo(initialPosB.x);
    expect(finalPosCompB.pos.y).toBeCloseTo(initialPosB.y);
  });

  test('two over-corrected joints on one body push it apart', () => {
    const world = new World();
    const dt = 1.0 / 60.0;
    world.setResource('dt', dt);

    const entityA = world.createEntity();
    world.addComponent(entityA, new PositionComponent(0, 0));
    world.addComponent(entityA, new MassComponent(-1));
    world.addComponent(entityA, new CableLinkComponent());

    const entityB = world.createEntity();
    const initialPosB = new Vector2(8, 0);
    world.addComponent(entityB, new PositionComponent(initialPosB.x, initialPosB.y));
    world.addComponent(entityB, new MassComponent(1));
    world.addComponent(entityB, new MomentOfInertiaComponent(1));
    world.addComponent(entityB, new OrientationComponent(0));
    world.addComponent(entityB, new RadiusComponent(1));
    world.addComponent(entityB, new CableLinkComponent());

    const entityC = world.createEntity();
    world.addComponent(entityC, new PositionComponent(16, 0));
    world.addComponent(entityC, new MassComponent(-1));
    world.addComponent(entityC, new CableLinkComponent());

    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        entityA,
        entityB,
        10.0,
        new Vector2(0, 0),
        new Vector2(7, 0)
      )
    );

    const joint2 = world.createEntity();
    world.addComponent(
      joint2,
      new CableJointComponent(
        entityB,
        entityC,
        10.0,
        new Vector2(9, 0),
        new Vector2(16, 0)
      )
    );

    const pathEntity = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint1, joint2],
      ['fixed', 'rolling', 'fixed'],
      [false, false, false]
    );
    world.addComponent(pathEntity, pathComp);

    // Populate current attachment points and then set a stretched state
    _updateAttachmentPoints(world);

    const j1Comp = world.getComponent(joint1, CableJointComponent);
    j1Comp.attachmentPointA_world.set(new Vector2(0, 0));
    j1Comp.attachmentPointB_world.set(new Vector2(11, 0)); // Stretched: length is 11 > 10

    const j2Comp = world.getComponent(joint2, CableJointComponent);
    // Stretched: C is at (16,0), set attachment on B to (5,0). Length is |16-5|=11 > 10
    j2Comp.attachmentPointA_world.set(new Vector2(5, 0));
    j2Comp.attachmentPointB_world.set(new Vector2(16, 0));

    const cacheSystem = new CableAttachmentCacheSystem();
    cacheSystem.update(world, dt);

    const resolveSystem = new PBDResolveCableOverCorrections();
    resolveSystem.update(world, dt);

    const finalPosCompB = world.getComponent(entityB, PositionComponent);
    expect(finalPosCompB.pos.x).toBeGreaterThan(initialPosB.x);
  });
});
