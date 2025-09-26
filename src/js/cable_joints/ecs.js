import Vector2 from './vector2.js';

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
  constructor(x = 0, y = 0) {
    this.pos = new Vector2(x, y);
  }
}
export class PrevFinalPosComponent {
  constructor(x = 0, y = 0) {
    this.pos = new Vector2(x, y);
  }
}
export class PrevFinalOrientationComponent {
  constructor(angle = 0.0) {
    this.angle = angle; // Radians
  }
}
export class VelocityComponent { constructor(x = 0, y = 0) { this.vel = new Vector2(x, y); } }
export class RadiusComponent { constructor(radius = 0.1) { this.radius = radius; } }
export class MassComponent { constructor(mass = 1.0) { this.mass = mass; } }
export class RestitutionComponent { constructor(restitution = 0.5) { this.restitution = restitution; } }
export class GravityAffectedComponent { }
export class SimulationErrorStateComponent { constructor(hasError = false) { this.hasError = hasError; } }
export class OrientationComponent {
    constructor(angle = 0.0) {
        this.angle = angle; // Radians
    }
}
export class AngularVelocityComponent {
    constructor(velocity = 0.0) {
        this.angularVelocity = velocity; // Radians per second
    }
}
export class MomentOfInertiaComponent {
    constructor(inertia = 1.0) {
        this.inertia = inertia; // kg * m^2 (approximate for 2D)
        this.invInertia = inertia > 0 ? 1.0 / inertia : 0.0;
    }
}
export class RenderableComponent {
  constructor(shape = 'circle', color = '#888888') {
    this.shape = shape; // 'circle', 'line', 'flipper', 'border'
    this.color = color;
  }
}

export class MachineTagComponent {
  constructor(machineId = '') {
    this.id = machineId;
  }
}
export class CoefficientOfFrictionComponent {
  constructor(mu = 0.1) { // Default static friction coefficient
    this.mu = mu; // Coefficient of static friction
  }
}

export class DistanceConstraintComponent {
  constructor(entityA, entityB, restLength, compliance = 0.0) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.restLength = restLength;
    this.compliance = compliance; // XPBD compliance (alpha / dt^2)
    this.lambda = 0.0; // Accumulated Lagrange multiplier
  }
}

// Groups multiple bodies to move as a single rigid cluster using shape matching.
// - members: array of entityIds
// - restLocal: array of Vector2 offsets (initialized on first update)
// - stiffness: [0..1], how strongly to enforce rigidity (1 = hard constraint)
export class RigidGroupComponent {
  constructor(members = [], stiffness = 1.0) {
    this.members = members;
    this.restLocal = null; // filled with Vector2 per member
    this.stiffness = stiffness;
    this.prevAngle = 0.0; // last solved absolute group angle
  }
}
