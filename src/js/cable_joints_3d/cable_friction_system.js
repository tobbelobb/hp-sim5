import {
  RadiusComponent,
  CoefficientOfFrictionComponent,
  layeringEnabled
} from './ecs.js';
import {
  CablePathComponent,
  CableJointComponent
} from './cable_joints_core.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';

// Base iteration count tuned for a 60 Hz frame time. When the simulation uses
// smaller timesteps, we reduce the per-step iterations so that the total number
// of friction iterations per real-time second stays roughly constant. See
// `ai_docs/CableJoints/CableJoints.md` for the friction model and
// `ai_docs/Smallsteps/Smallsteps.md` for why the work should scale with the
// timestep size.

const BASE_ITERATIONS = 4;
const TARGET_DT = 1 / 500;

function _hybridTransitionArcThreshold(path) {
  const halfWidth = Number.isFinite(path?.cableHalfWidth) ? Math.max(0.0, path.cableHalfWidth) : 0.0;
  if (!(halfWidth > 1e-9)) {
    return 1e-6;
  }
  return Math.max(1e-6, 0.25 * halfWidth);
}

function _redistributePairRestLengthByTensionRatio(
  jHighComp,
  jLowComp,
  dHigh,
  dLow,
  tensionRatio,
) {
  // The PBD cable solver uses one stiffness per joint, so equal force means
  // equal absolute extension, not equal strain.
  const totalRest = jHighComp.restLength + jLowComp.restLength;
  const totalDist = dHigh + dLow;
  if (!(totalRest > 0.0) || !(totalDist > 1e-9)) {
    return;
  }

  if (totalRest >= totalDist) {
    jHighComp.restLength = totalRest * (dHigh / totalDist);
    jLowComp.restLength = totalRest - jHighComp.restLength;
    return;
  }

  const ratio = Math.max(1.0, tensionRatio);
  let highRest = (dHigh - (ratio * dLow) + (ratio * totalRest)) / (1.0 + ratio);
  highRest = Math.max(0.0, Math.min(totalRest, highRest));
  jHighComp.restLength = highRest;
  jLowComp.restLength = totalRest - highRest;
}

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

      if (
        !Number.isFinite(l0_current) ||
        !Number.isFinite(l1_current) ||
        l0_current < 0.0 ||
        l1_current < 0.0
      ) {
        continue;
      }

      const linkType = path.linkTypes[i + 1];
      let frictionActive = false;
      let frictionThreshold = 1.0; // Default for mu=0 or radius=0 or no wrap

      if (linkType === 'rolling' || linkType === 'pinhole') {
        const linkEntityId = j0_comp.entityB; // Assuming j0.entityB is the link
        const frictionComp = world.getComponent(linkEntityId, CoefficientOfFrictionComponent);
        const mu = frictionComp ? frictionComp.mu : 0.0;

        const radiusComp = world.getComponent(linkEntityId, RadiusComponent);
        const radius = radiusComp ? radiusComp.radius : 0.0;
        const effectiveRadius = radius + (layeringEnabled(world) ? (path.cableHalfWidth ?? 0.0) : 0.0);

        const freeRollingLink = linkType === 'rolling' && world.getComponent(linkEntityId, SpoolStateComponent);

        if (!freeRollingLink && mu > epsilon) {
          const storedLengthOnLink = path.stored[i + 1];
          let wrapAngle = 0;
          if (linkType === 'rolling' && effectiveRadius > epsilon && Math.abs(storedLengthOnLink) > epsilon) {
            wrapAngle = Math.abs(storedLengthOnLink / effectiveRadius);
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
        if (linkType !== 'attachment') {
          _redistributePairRestLengthByTensionRatio(j0_comp, j1_comp, d0, d1, 1.0);
        }
        continue;
      }

      const tension0 = Math.max(0.0, d0 - l0_current);
      const tension1 = Math.max(0.0, d1 - l1_current);

      if (Math.abs(tension0 - tension1) < epsilon) continue;

      let T_high, D_high, is_j0_high;
      let T_low, D_low;

      if (tension0 > tension1) {
        [T_high, D_high, is_j0_high] = [tension0, d0, true];
        [T_low, D_low] = [tension1, d1];
      } else {
        [T_high, D_high, is_j0_high] = [tension1, d1, false];
        [T_low, D_low] = [tension0, d0];
      }

      if (T_high > T_low * frictionThreshold + epsilon) {
        if (is_j0_high) {
          _redistributePairRestLengthByTensionRatio(
            j0_comp,
            j1_comp,
            D_high,
            D_low,
            frictionThreshold,
          );
        } else {
          _redistributePairRestLengthByTensionRatio(
            j1_comp,
            j0_comp,
            D_high,
            D_low,
            frictionThreshold,
          );
        }
      }
    }
  }
}

function _sanityCheck(world) {
  const pathEntities = world.query([CablePathComponent]);
  const negativeStoredEpsilon = 1e-9;
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
    const hybridEndpointThreshold = _hybridTransitionArcThreshold(path);
    const hasStoredBelowThreshold = path.stored.some((stored, index) => {
      const isEndpoint = index === 0 || index === path.stored.length - 1;
      const usesHybridHysteresis = isEndpoint && path.linkTypes[index] === 'hybrid';
      const threshold = usesHybridHysteresis ? hybridEndpointThreshold : negativeStoredEpsilon;
      return stored < -threshold;
    });
    if (hasStoredBelowThreshold) {
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
    //_sanityCheck(world);
  }
}
