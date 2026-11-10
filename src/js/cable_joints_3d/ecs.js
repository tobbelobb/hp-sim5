import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';
export { MomentOfInertiaComponent } from './inertia_tensor.js';

// Shared ECS used by the browser app, headless worlds and the Rust host adapter.
export class World {
  constructor() {
    this.entities = new Map(); // entityId -> Set<ComponentClass>
    this.components = new Map(); // ComponentClass -> Map<entityId, ComponentInstance>
    this.nextEntityId = 0;
    this.systems = [];
    this.resources = {}; // Global data like gravity, dt
    this.structureVersion = 0;
  }

  createEntity() {
    this.structureVersion += 1;
    const id = this.nextEntityId++;
    this.entities.set(id, new Set());
    return id;
  }

  addComponent(entityId, component) {
    this.structureVersion += 1;
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

  getComponentStore(componentClass) {
    return this.components.get(componentClass) || null;
  }

  hasComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    return componentMap ? componentMap.has(entityId) : false;
  }

  removeComponent(entityId, componentClass) {
    this.structureVersion += 1;
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
    if (!Array.isArray(componentClasses) || componentClasses.length === 0) {
      return [];
    }

    const stores = [];
    for (const componentClass of componentClasses) {
      const store = this.components.get(componentClass);
      if (!store) {
        return [];
      }
      stores.push(store);
    }

    if (stores.length === 1) {
      return Array.from(stores[0].keys());
    }

    stores.sort((a, b) => a.size - b.size);
    const [primary, ...others] = stores;
    const result = [];

    outer: for (const entityId of primary.keys()) {
      for (let i = 0; i < others.length; i += 1) {
        if (!others[i].has(entityId)) {
          continue outer;
        }
      }
      result.push(entityId);
    }
    return result;
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
    this.structureVersion += 1;
    this.entities.clear();
    this.components.clear();
    this.nextEntityId = 0;
    // Keep systems and resources
  }

  update(dt) {
    const pauseState = this.getResource('pauseState');
    const errorState = this.getResource('errorState');
    const perf = this.getResource('performanceMonitor');

    const isPaused = pauseState ? pauseState.paused : false;
    const hasError = errorState ? errorState.hasError : false;

    perf?.beginStep?.();

    try {
      for (let index = 0; index < this.systems.length;) {
        const start = index++;
        const physics = this.systems[start].physicsSubstep === true;
        while (physics && this.systems[index]?.physicsSubstep === true) index++;
        const count = physics
          ? (this.systems[start].physicsSubsteps?.(this) ?? this.getResource('physicsSubsteps') ?? 1) : 1;
        const savedDt = this.getResource('dt');
        const stepDt = dt / count;
        try {
          if (physics) this.setResource('dt', stepDt);
          for (let substep = 0; substep < count; substep++) {
            for (let current = start; current < index; current++) {
              const system = this.systems[current];
              if (!system.update || (!system.runInPause && isPaused) || hasError) continue;
              const name = system.constructor?.name || 'AnonymousSystem';
              if (perf?.enabled) {
                perf.measureSystem(name, () => system.update(this, stepDt));
              } else {
                system.update(this, stepDt);
              }
            }
          }
        } finally {
          if (physics) this.setResource('dt', savedDt);
        }
      }
    } finally {
      perf?.endStep?.();
    }
  }
}

export class RadiusComponent { constructor(radius = 0.1) { this.radius = radius; } }
export class MassComponent { constructor(mass = 1.0) { this.mass = mass; } }
export class RestitutionComponent { constructor(restitution = 0.5) { this.restitution = restitution; } }
export class GravityAffectedComponent { }
export class SimulationErrorStateComponent { constructor(hasError = false) { this.hasError = hasError; } }
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

export class RenderableComponent {
  constructor(shape = 'circle', color = '#888888', options = {}) {
    this.shape = shape; // 'circle', 'line', 'flipper', 'border'
    this.color = color;
    this.height = Number.isFinite(options?.height) && options.height > 0 ? options.height : null;
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

export class RigidGroupComponent {
  constructor(members = [], stiffness = 1.0, renderSegments = null) {
    this.members = members;
    this.restLocal = null;
    this.stiffness = stiffness;
    this.prevAngle = 0.0;
    this.renderSegments = Array.isArray(renderSegments) ? renderSegments : null;
  }
}

export class BallTagComponent { }
export class ObstacleTagComponent { }
export class ObstaclePushComponent { constructor(pushVel = 2.0) { this.pushVel = pushVel; } }

export class PositionComponent {
  constructor(x = 0, y = 0, z = 0) {
    this.pos = new Vector3(x, y, z);
  }
}

// Authored scene identity, independent of rendering and physics components.
export class SceneEntityInfoComponent {
  constructor(name, tags = []) {
    this.name = name;
    this.tags = tags;
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
