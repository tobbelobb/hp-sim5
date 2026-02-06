import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

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
} from '../cable_joints/ecs.js';

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

export class AngularVelocityComponent {
    constructor(x = 0, y = 0, z = 0) {
        this.omega = new Vector3(x, y, z); // Axis = normalize(omega), speed = norm_L2(omega) [Radians per second]
    }
}
