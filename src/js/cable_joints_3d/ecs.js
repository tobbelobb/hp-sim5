import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

// 3D analogs of 2D components live here (Vector3 + Quaternion).
// Reuse the 2D World implementation to avoid duplication.
export {
  World,
  RadiusComponent,
  MassComponent,
  // NOTE: This is still a scalar inertia from the 2D implementation.
  // In 3D we currently interpret it as an effective scalar inertia about
  // the constraint axis (or isotropic inertia for spheres). Replace with
  // a full 3x3 tensor when general 3D rigid bodies/groups are required.
  // If we want general 3D rigid bodies or rigid groups with anisotropic inertia (which PBDBodies explicitly assumes by using an inertia tensor),
  // then we will need a full 3×3 tensor, and we should use it in the angular part of the constraint  solve:
  // deltaAng = -lambda * I_world^-1 * gradAng, with I_world^-1 = R * I_body^-1 * R^T.
  // That becomes important once constraints induce off‑axis rotations, or when rigid groups combine bodies with different mass distributions.
  MomentOfInertiaComponent,
  RestitutionComponent,
  GravityAffectedComponent,
  SimulationErrorStateComponent,
  MachineTagComponent,
  CoefficientOfFrictionComponent,
  RenderableComponent,
  DistanceConstraintComponent,
  RigidGroupComponent,
} from '../cable_joints/ecs.js';

export {
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent
} from '../../../examples/js/flipper/flipper_common.js';

export class PositionComponent {
  constructor(x = 0, y = 0, z = 0) {
    this.pos = new Vector3(x, y, z);
  }
}

export class PrevFinalPosComponent {
  constructor(x = 0, y = 0, z = 0) {
    this.pos = new Vector3(x, y, z);
  }
}

export class VelocityComponent {
  constructor(x = 0, y = 0, z = 0) {
    this.vel = new Vector3(x, y, z);
  }
}

export class EncoderComponent {
    constructor(angle = 0.0, x = 0.0, y = 0.0, z = 1.0) {
        this.angle = angle; // Radians
        this.axis = new Vector3(x, y, z);
    }
}

export class OrientationComponent {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.quaternion = new Quaternion(x, y, z, w);
  }
}

export class PrevFinalOrientationComponent {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.quaternion = new Quaternion(x, y, z, w);
  }
}

export class HybridKnotAngleComponent {
  constructor(angle = 0.0) {
    this.angle = angle;
    this.pathAngles = {};
  }
}

export class AngularVelocityComponent {
    constructor(x = 0, y = 0, z = 0) {
        this.omega = new Vector3(x, y, z); // Axis = normalize(omega), speed = norm_L2(omega) [Radians per second]
    }
}

function _resourceBool(world, key, fallback = true) {
  const value = world?.getResource?.(key);
  return typeof value === 'boolean' ? value : fallback;
}

export function layeringEnabled(world) {
  return _resourceBool(world, 'enableLayering', true);
}
