import {
    CablePathComponent,
    CableJointComponent
} from './cable_joints_core.js';

// Adjust the number of slack iterations so the cost per second remains
// constant regardless of substep size. See `ai_docs/CableJoints/CableJoints.md`
// and `ai_docs/Smallsteps/Smallsteps.md` for details on the model and the
// rationale behind scaling work with the timestep.

const BASE_ITERATIONS = 4;
const TARGET_DT = 1 / 60;

function _slipSlack(world) {
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (path.jointEntities.length < 2) continue;
    for (let i = 0; i < path.jointEntities.length - 1; i++) {
      if (path.linkTypes[i + 1] === 'attachment') continue; // Nothing slides across an attachment
      const j0 = world.getComponent(path.jointEntities[i], CableJointComponent);
      const j1 = world.getComponent(path.jointEntities[i + 1], CableJointComponent);
      const d0 = j0.attachmentPointA_world.distanceTo(j0.attachmentPointB_world);
      const d1 = j1.attachmentPointA_world.distanceTo(j1.attachmentPointB_world);
      const l0 = j0.restLength;
      const l1 = j1.restLength;
      const slack0 = l0 - d0;  // >0 means slack, <0 means over-stretched
      const slack1 = l1 - d1;
      let slip = 0;
      if (slack0 > 0 && slack1 < 0) {
        slip = Math.min(slack0, -slack1);
        j0.restLength -= slip;
        j1.restLength += slip;
      }
      else if (slack1 > 0 && slack0 < 0) {
        slip = Math.min(slack1, -slack0);
        j1.restLength -= slip;
        j0.restLength += slip;
      }
    }
  }
}

export class CableSlackSystem {
    runInPause = false;
    update(world, dt) {
        // Scale iterations with dt so slack propagation work per second
        // matches the target budget (see Smallsteps.md).
        const iterations = Math.max(1, Math.floor(BASE_ITERATIONS * dt / TARGET_DT));
        for (let i = 0; i < iterations; i++) {
            _slipSlack(world);
        }
    }
}
