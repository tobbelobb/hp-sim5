import { CableLinkComponent, PositionComponent, OrientationComponent } from './ecs.js';

function _storeCableLinkPoses(world) {
  const linkEntities = world.query([CableLinkComponent, PositionComponent]);
  for (const linkId of linkEntities) {
    const posComp = world.getComponent(linkId, PositionComponent);
    const orientationComp = world.getComponent(linkId, OrientationComponent);
    const linkComp = world.getComponent(linkId, CableLinkComponent);
    linkComp.prevCableAttachmentTimePos.set(posComp.pos);
    if (orientationComp) {
      linkComp.prevCableAttachmentTimeAngle = orientationComp.angle;
    }
  }
}

export class CableAttachmentCacheSystem {
  runInPause = false;
  update(world, dt) {
    _storeCableLinkPoses(world);
  }
}
