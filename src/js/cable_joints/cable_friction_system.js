import Vector2 from './vector2.js';
import {
    RadiusComponent,
    CoefficientOfFrictionComponent
} from './ecs.js';
import {
    CablePathComponent,
    CableJointComponent
} from './cable_joints_core.js';

// Base iteration count tuned for a 60 Hz frame time. When the simulation uses
// smaller timesteps, we reduce the per‑step iterations so that the total number
// of friction iterations per real‑time second stays roughly constant. See
// `ai_docs/CableJoints/CableJoints.md` for the friction model and
// `ai_docs/Smallsteps/Smallsteps.md` for why the work should scale with the
// timestep size.

const BASE_ITERATIONS = 4;
const TARGET_DT = 1 / 60;


function _evenOutTensionFriction(world) {
  const pathEntities = world.query([CablePathComponent]);
  const epsilon = 1e-9;

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (!path || path.jointEntities.length < 2) continue;

    for (let i = 0; i < path.jointEntities.length - 1; i++) {
      const j0_comp = world.getComponent(path.jointEntities[i], CableJointComponent);
      const j1_comp = world.getComponent(path.jointEntities[i + 1], CableJointComponent);

      if (!j0_comp || !j1_comp) continue;

      const d0 = j0_comp.attachmentPointA_world.distanceTo(j0_comp.attachmentPointB_world);
      const d1 = j1_comp.attachmentPointA_world.distanceTo(j1_comp.attachmentPointB_world);
      const l0_current = j0_comp.restLength;
      const l1_current = j1_comp.restLength;

      if (l0_current <= epsilon || l1_current <= epsilon) continue;

      const linkType = path.linkTypes[i + 1];
      let frictionActive = false;
      let frictionThreshold = 1.0; // Default for mu=0 or radius=0 or no wrap

      if (linkType === 'rolling' || linkType === 'pinhole') {
        const linkEntityId = j0_comp.entityB; // Assuming j0.entityB is the link
        const frictionComp = world.getComponent(linkEntityId, CoefficientOfFrictionComponent);
        const mu = frictionComp ? frictionComp.mu : 0.0;

        const radiusComp = world.getComponent(linkEntityId, RadiusComponent);
        const radius = radiusComp ? radiusComp.radius : 0.0;

        if (mu > epsilon) {
          const storedLengthOnLink = path.stored[i + 1];
          let wrapAngle = 0;
          if (linkType === 'rolling' && radius > epsilon && Math.abs(storedLengthOnLink) > epsilon) {
            wrapAngle = Math.abs(storedLengthOnLink / radius);
          } else if (linkType === 'pinhole') {
            const v0 = j0_comp.attachmentPointA_world.clone().subtract(j0_comp.attachmentPointB_world).normalize();
            const v1 = j1_comp.attachmentPointB_world.clone().subtract(j1_comp.attachmentPointA_world).normalize();
            const dot = v0.dot(v1);
            wrapAngle = Math.acos(Math.max(-1.0, Math.min(1.0, dot)));
          }
          if (wrapAngle > epsilon) {
            frictionActive = true;
            frictionThreshold = Math.exp(mu * wrapAngle);
          }
        }
      }

      if (!frictionActive) {
        if  (linkType !== 'attachment') {
          const availableRestLength = l0_current + l1_current;
          const totalDist = d0 + d1;
          if (totalDist > epsilon) {
            j0_comp.restLength = availableRestLength * d0 / totalDist;
            j1_comp.restLength = availableRestLength * d1 / totalDist;
          }
        }
        continue;
      }

      const tension0 = l0_current > epsilon ? d0 / l0_current : Infinity;
      const tension1 = l1_current > epsilon ? d1 / l1_current : Infinity;

      if (Math.abs(tension0 - tension1) < epsilon) continue;

      let T_high, L_high_current, D_high, is_j0_high;
      let T_low, L_low_current, D_low;

      if (tension0 > tension1) {
        [T_high, L_high_current, D_high, is_j0_high] = [tension0, l0_current, d0, true];
        [T_low, L_low_current, D_low] = [tension1, l1_current, d1];
      } else {
        [T_high, L_high_current, D_high, is_j0_high] = [tension1, l1_current, d1, false];
        [T_low, L_low_current, D_low] = [tension0, l0_current, d0];
      }

      if (T_high > T_low * frictionThreshold + epsilon) {
        const L_total = L_high_current + L_low_current;
        const denominator = D_high + D_low * frictionThreshold;
        if (denominator > epsilon) {
          let L_high_new = (D_high * L_total) / denominator;
          let L_low_new = L_total - L_high_new;

          if (L_high_new < 0) L_high_new = 0;
          if (L_low_new < 0) L_low_new = 0;
          const currentNewTotal = L_high_new + L_low_new;
          if (currentNewTotal > epsilon && Math.abs(currentNewTotal - L_total) > epsilon) {
             L_high_new = (L_high_new / currentNewTotal) * L_total;
             L_low_new = (L_low_new / currentNewTotal) * L_total;
          }

          if (is_j0_high) {
            j0_comp.restLength = L_high_new;
            j1_comp.restLength = L_low_new;
          } else {
            j1_comp.restLength = L_high_new;
            j0_comp.restLength = L_low_new;
          }
        }
      }
    }
  }
}

function _sanityCheck(world) {
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (!path || path.jointEntities.length < 1) continue;

    let totalCurrentRestLength = path.stored.reduce((acc, val) => acc + val, 0);
    for (const jointId of path.jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      totalCurrentRestLength += joint.restLength;
    }

    const error = path.totalRestLength - totalCurrentRestLength;
    if (Math.abs(error) > 1e-9) {
      console.warn(`Non-zero error for path ${pathId}: ${error}`);
    }
    if (path.stored.some(s => s < -1e-9)) {
      console.warn(`Negative stored lengths for path ${pathId}: ${path.stored}`);
    }
  }
}

export class CableFrictionSystem {
    runInPause = false;
    update(world, dt) {
        // Keep the total number of friction iterations per real-time second
        // constant as described in Smallsteps.md.
        const iterations = Math.max(1, Math.floor(BASE_ITERATIONS * dt / TARGET_DT));
        for (let i = 0; i < iterations; i++) {
            _evenOutTensionFriction(world);
        }
        _sanityCheck(world);
    }
}
