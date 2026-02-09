import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  MassComponent,
  GravityAffectedComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  PrevFinalOrientationComponent,
  RenderableComponent,
  RadiusComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem
} from '../../../src/js/cable_joints/commonSystems.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints/cable_joints_core.js';
import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints/cable_attachment_cache_system.js';
import { tangentFromPointToCircle } from '../../../src/js/cable_joints/geometry.js';

function addMirrorSpool(world, side) {
  const spoolRadius = 0.03;
  const spoolMass = 0.005;
  const spoolInertia = 10.0 * 0.5 * spoolMass * spoolRadius * spoolRadius;
  const x = (side === 'left') ? -0.25 : 0.25;
  const cw = (side === 'left') ? false : true;

  const spool = world.createEntity();
  world.addComponent(spool, new PositionComponent(x, -0.5));
  world.addComponent(spool, new PrevFinalPosComponent(x, -0.5));
  world.addComponent(spool, new VelocityComponent(0.0, 0.0));
  world.addComponent(spool, new RadiusComponent(spoolRadius));
  world.addComponent(spool, new MassComponent(spoolMass));
  world.addComponent(spool, new GravityAffectedComponent());
  world.addComponent(spool, new RenderableComponent('circle', '#a0a0a0'));
  world.addComponent(spool, new OrientationComponent(0.0));
  world.addComponent(spool, new PrevFinalOrientationComponent(0.0));
  world.addComponent(spool, new AngularVelocityComponent(0.0));
  world.addComponent(spool, new MomentOfInertiaComponent(spoolInertia));

  const anchor = world.createEntity();
  world.addComponent(anchor, new PositionComponent(x, 0.5));
  world.addComponent(anchor, new VelocityComponent(0.0, 0.0));
  world.addComponent(anchor, new RadiusComponent(0.01));
  world.addComponent(anchor, new MassComponent(-1.0));
  world.addComponent(anchor, new RenderableComponent('circle', '#aaaaaa'));

  const spoolPos = world.getComponent(spool, PositionComponent).pos;
  const anchorPos = world.getComponent(anchor, PositionComponent).pos;
  world.addComponent(spool, new CableLinkComponent(spoolPos.x, spoolPos.y));
  world.addComponent(anchor, new CableLinkComponent(anchorPos.x, anchorPos.y));

  const tang = tangentFromPointToCircle(anchorPos, spoolPos, spoolRadius, cw);
  const dist = tang.a_attach.clone().subtract(tang.a_circle).length();
  const joint = world.createEntity();
  if (side === 'left') {
    world.addComponent(joint, new CableJointComponent(anchor, spool, dist, tang.a_attach, tang.a_circle));
  } else {
    world.addComponent(joint, new CableJointComponent(spool, anchor, dist, tang.a_circle, tang.a_attach));
  }
  world.addComponent(joint, new RenderableComponent('line', '#AA3311'));

  const pathId = world.createEntity();
  let path;
  if (side === 'left') {
    path = new CablePathComponent(world, [joint], ['attachment', 'hybrid'], [true, cw]);
    path.stored[1] = 0.2;
  } else {
    path = new CablePathComponent(world, [joint], ['hybrid', 'attachment'], [cw, true]);
    path.stored[0] = 0.2;
  }
  path.totalRestLength += 0.2;
  world.addComponent(pathId, path);

  return {
    path,
    index: side === 'left' ? 1 : 0
  };
}

describe('Hybrid endpoint symmetry', () => {
  test('mirrored first/last hybrid endpoints evolve symmetrically', () => {
    const world = new World();
    world.setResource('gravity', new Vector2(0.0, -10.0));
    world.setResource('dt', 1.0 / 200.0);
    world.setResource('pauseState', { paused: false });
    world.setResource('debugRenderPoints', {});
    world.setResource('errorState', { hasError: false });

    const left = addMirrorSpool(world, 'left');
    const right = addMirrorSpool(world, 'right');

    world.registerSystem(new PrevFinalPosSystem());
    world.registerSystem(new PrevFinalOrientationSystem());
    world.registerSystem(new GravitySystem());
    world.registerSystem(new MovementSystem());
    world.registerSystem(new AngularMovementSystem());
    world.registerSystem(new CableAttachmentUpdateSystem());
    world.registerSystem(new CableAttachmentCacheSystem());
    world.registerSystem(new PBDCableConstraintSolver());
    world.registerSystem(new PBDVelocityUpdateSystem());
    world.registerSystem(new PBDAngularVelocityUpdateSystem());

    const dt = world.getResource('dt');
    let sawAttachmentZeroLeft = false;
    let sawAttachmentZeroRight = false;
    for (let step = 0; step < 1500; step++) {
      world.update(dt);

      const typeLeft = left.path.linkTypes[left.index];
      const typeRight = right.path.linkTypes[right.index];
      const storedLeft = left.path.stored[left.index];
      const storedRight = right.path.stored[right.index];

      expect(storedLeft).toBeCloseTo(storedRight, 6);
      expect(typeLeft).toBe(typeRight);
      expect(Boolean(left.path.cw[left.index])).toBe(!Boolean(right.path.cw[right.index]));

      if (typeLeft === 'hybrid-attachment' && Math.abs(storedLeft) <= 1e-9) {
        sawAttachmentZeroLeft = true;
      }
      if (typeRight === 'hybrid-attachment' && Math.abs(storedRight) <= 1e-9) {
        sawAttachmentZeroRight = true;
      }
    }

    expect(sawAttachmentZeroLeft).toBe(true);
    expect(sawAttachmentZeroRight).toBe(true);
  });
});
