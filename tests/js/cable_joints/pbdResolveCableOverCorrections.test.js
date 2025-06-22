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
  test('test_resolve_over_correction_pushes_apart', () => {

  });

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
});
