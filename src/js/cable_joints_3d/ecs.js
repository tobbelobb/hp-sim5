import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';
export { MomentOfInertiaComponent } from './inertia_tensor.js';

// 3D analogs of 2D components live here (Vector3 + Quaternion).
// Reuse the 2D World implementation to avoid duplication.
export {
  World,
  RadiusComponent,
  MassComponent,
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
} from '../../../example_apps/js/flipper/flipper_common.js';

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

export class RigidBodyComponent {
  constructor(members = [], renderSegments = null) {
    this.members = Array.isArray(members) ? members.slice() : [];
    this.renderSegments = Array.isArray(renderSegments) ? renderSegments : null;
    this.syncedPosition = new Vector3(0.0, 0.0, 0.0);
    this.syncedOrientation = new Quaternion();
  }
}

export class RigidBodyMemberComponent {
  constructor(bodyEntity = null, localPosition = null, localOrientation = null, physicalMass = null) {
    this.bodyEntity = bodyEntity;
    this.localPosition = localPosition ? localPosition.clone() : new Vector3(0.0, 0.0, 0.0);
    this.localOrientation = localOrientation ? localOrientation.clone().normalize() : new Quaternion();
    this.physicalMass = Number.isFinite(physicalMass) ? physicalMass : null;
  }
}

function _resourceBool(world, key, fallback = true) {
  const value = world?.getResource?.(key);
  return typeof value === 'boolean' ? value : fallback;
}

export function layeringEnabled(world) {
  return _resourceBool(world, 'enableLayering', true);
}

