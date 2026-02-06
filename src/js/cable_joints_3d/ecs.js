import Vector3 from './vector3.js';

// Reuse the 2D World implementation to avoid duplication.
export { World } from '../cable_joints/ecs.js';

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

export class GravityAffectedComponent { }
