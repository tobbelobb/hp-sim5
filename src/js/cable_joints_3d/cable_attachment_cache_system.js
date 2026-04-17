import {
  PositionComponent,
  OrientationComponent,
  RigidBodyMemberComponent,
} from './ecs.js';
import { CableLinkComponent } from './cable_joints_core.js';
import {
  getEntityWorldOrientation,
  getEntityWorldPosition,
} from './rigid_bodies.js';

function _storeCableLinkPoses(world) {
  const linkEntities = world.query([CableLinkComponent, PositionComponent]);
  for (const linkId of linkEntities) {
    const worldPos = getEntityWorldPosition(world, linkId);
    const worldOrientation = getEntityWorldOrientation(world, linkId);
    const rigidBodyMemberComp = world.getComponent(linkId, RigidBodyMemberComponent);
    const linkComp = world.getComponent(linkId, CableLinkComponent);
    if (worldPos) {
      linkComp.prevCableAttachmentTimePos.set(worldPos);
    }
    if (worldOrientation) {
      linkComp.prevCableAttachmentTimeOrientation.set(worldOrientation);
    }
    if (rigidBodyMemberComp?.localOrientation) {
      linkComp.prevCableAttachmentTimeLocalOrientation.set(rigidBodyMemberComp.localOrientation);
    } else if (worldOrientation) {
      linkComp.prevCableAttachmentTimeLocalOrientation.set(worldOrientation);
    }
  }
}

export class CableAttachmentCacheSystem {
  runInPause = false;
  update(world, dt) {
    _storeCableLinkPoses(world);
  }
}
