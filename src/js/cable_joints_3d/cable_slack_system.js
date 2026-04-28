import {
  CablePathComponent,
  CableJointComponent
} from './cable_joints_core.js';

const EPSILON = 1e-9;

function _equalizePinholeTension(world) {
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (path.jointEntities.length < 2) continue;
    for (let i = 0; i < path.jointEntities.length - 1; i++) {
      if (path.linkTypes[i + 1] !== 'pinhole') continue;
      const j0 = world.getComponent(path.jointEntities[i], CableJointComponent);
      const j1 = world.getComponent(path.jointEntities[i + 1], CableJointComponent);
      const d0 = j0.attachmentPointA_world.distanceTo(j0.attachmentPointB_world);
      const d1 = j1.attachmentPointA_world.distanceTo(j1.attachmentPointB_world);
      const availableRestLength = j0.restLength + j1.restLength;
      const totalDist = d0 + d1;
      if (availableRestLength <= EPSILON || totalDist <= EPSILON) continue;

      // Frictionless pinholes slide cable until adjacent d/rest tension ratios match.
      j0.restLength = availableRestLength * d0 / totalDist;
      j1.restLength = availableRestLength - j0.restLength;
    }
  }
}

export class CableSlackSystem {
  runInPause = false;
  update(world, _dt) {
    _equalizePinholeTension(world);
  }
}
