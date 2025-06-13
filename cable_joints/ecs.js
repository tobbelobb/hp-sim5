import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

export class World {
  constructor() {
    this.entities = new Map(); // entityId -> Set<ComponentClass>
    this.components = new Map(); // ComponentClass -> Map<entityId, ComponentInstance>
    this.nextEntityId = 0;
    this.systems = [];
    this.resources = {}; // Global data like gravity, dt
  }

  createEntity() {
    const id = this.nextEntityId++;
    this.entities.set(id, new Set());
    return id;
  }

  addComponent(entityId, component) {
    const componentClass = component.constructor;
    if (!this.components.has(componentClass)) {
      this.components.set(componentClass, new Map());
    }
    this.components.get(componentClass).set(entityId, component);
    if (this.entities.has(entityId)) {
      this.entities.get(entityId).add(componentClass);
    } else {
      console.warn(`Entity ${entityId} does not exist when adding ${componentClass.name}`);
    }
  }

  getComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    return componentMap ? componentMap.get(entityId) : undefined;
  }

  hasComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    return componentMap ? componentMap.has(entityId) : false;
  }

  removeComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    if (componentMap) {
      componentMap.delete(entityId);
    }
    const entityComponents = this.entities.get(entityId);
    if (entityComponents) {
      entityComponents.delete(componentClass);
    }
  }

  destroyEntity(entityId) {
    if (!this.entities.has(entityId)) return;
    for (const componentClass of this.entities.get(entityId)) {
      this.removeComponent(entityId, componentClass);
    }
    this.entities.delete(entityId);
  }

  query(componentClasses) {
    const entities = [];
    if (componentClasses.length === 0) return [];

    // Start with entities having the first component
    const firstComponentMap = this.components.get(componentClasses[0]);
    if (!firstComponentMap) return [];

    for (const entityId of firstComponentMap.keys()) {
      let hasAll = true;
      for (let i = 1; i < componentClasses.length; i++) {
        if (!this.hasComponent(entityId, componentClasses[i])) {
          hasAll = false;
          break;
        }
      }
      if (hasAll) {
        entities.push(entityId);
      }
    }
    return entities;
  }

  registerSystem(system) {
    this.systems.push(system);
  }

  setResource(name, value) {
    this.resources[name] = value;
  }

  getResource(name) {
    return this.resources[name];
  }

  clear() {
    this.entities.clear();
    this.components.clear();
    this.nextEntityId = 0;
    // Keep systems and resources
  }

  update(dt) {
    const pauseState = this.getResource('pauseState');
    const errorState = this.getResource('errorState');
    const isPaused = pauseState ? pauseState.paused : false;
    const hasError = errorState ? errorState.hasError : false;

    for (const system of this.systems) {
      // Skip systems if paused (and system doesn't run in pause) OR if there's an error
      if (system.update && ((!system.runInPause && isPaused) || hasError)) {
        continue; // Skip systems if paused or error occurred
      }
      if (system.update) {
        system.update(this, dt);
      }
    }
  }
}


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
export class PrevFinalOrientationComponent {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.orientation = new Quaternion(x, y, z, w);
  }
}
export class VelocityComponent {
  constructor(x = 0, y = 0, z = 0) {
    this.vel = new Vector3(x, y, z);
  }
}
export class RadiusComponent { constructor(radius = 0.1) { this.radius = radius; } }
export class MassComponent { constructor(mass = 1.0) { this.mass = mass; } }
export class RestitutionComponent { constructor(restitution = 0.5) { this.restitution = restitution; } }
export class GravityAffectedComponent { }
export class BallTagComponent { }
export class BorderComponent {
  constructor(points = []) {
    // In 3D, a border would likely be a mesh of faces (triangles)
    // For now, we'll keep it as points, but the logic will need to change.
    this.points = points.map(p => p.clone());
    this.faces = []; // e.g., [[0, 1, 2], [0, 2, 3]]
  }
}
export class FlipperTagComponent { }
export class FlipperStateComponent {
  // This component is highly 2D specific. Generalizing it requires
  // defining how a flipper works in 3D (e.g., rotation around an axis).
  // For now, we'll leave it, but it won't work with 3D systems.
  constructor(length, restAngle, maxRotation, angularVelocity) {
    this.length = length;
    this.restAngle = restAngle; // This would be a rest orientation (Quaternion)
    this.maxRotation = Math.abs(maxRotation);
    this.sign = Math.sign(maxRotation); // This would be a rotation axis (Vector3)
    this.angularVelocity = angularVelocity;
    // Dynamic state
    this.rotation = 0.0; // Current rotation from restAngle
    this.currentAngularVelocity = 0.0; // Velocity in the last frame
    this.pressed = false; // Was it activated?
  }
}
export class ObstacleTagComponent { }
export class ObstaclePushComponent { constructor(pushVel = 2.0) { this.pushVel = pushVel; } }
export class PauseStateComponent { constructor(paused = true) { this.paused = paused; } }
export class SimulationErrorStateComponent { constructor(hasError = false) { this.hasError = hasError; } }
export class OrientationComponent {
    constructor(x = 0, y = 0, z = 0, w = 1) {
        this.orientation = new Quaternion(x, y, z, w);
    }
}
export class AngularVelocityComponent {
    constructor(x = 0, y = 0, z = 0) {
        this.angularVelocity = new Vector3(x, y, z); // Radians per second around each axis
    }
}
export class MomentOfInertiaComponent {
    constructor(inertia = 1.0) {
        // For a sphere, inertia is a scalar. For general objects, it's a tensor.
        // We store the inverse inertia for calculations.
        // For a sphere: I = 2/5 * m * r^2. invI = 1/I.
        // For a general body, we'd store the inverse inertia tensor (a 3x3 matrix).
        // We'll simplify and assume a sphere-like object where inertia is the same
        // around all axes, so we can use a scalar inverse.
        this.inertia = inertia;
        this.invInertia = inertia > 0 ? 1.0 / inertia : 0.0;
    }
}
export class RenderableComponent {
  constructor(shape = 'sphere', color = '#888888') {
    this.shape = shape; // 'sphere', 'line', 'flipper', 'border'
    this.color = color;
  }
}
export class CoefficientOfFrictionComponent {
  constructor(mu = 0.1) { // Default static friction coefficient
    this.mu = mu; // Coefficient of static friction
  }
}
export class ScoredTagComponent { }

export class DistanceConstraintComponent {
  constructor(entityA, entityB, restLength, compliance = 0.0) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.restLength = restLength;
    this.compliance = compliance; // XPBD compliance (alpha / dt^2)
    this.lambda = 0.0; // Accumulated Lagrange multiplier
  }
}

export class FlipperTipComponent {
  constructor(flipperEntityId) {
    this.flipperEntityId = flipperEntityId;
  }
}
