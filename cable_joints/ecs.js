const { Vector2 } = require('./vector2');


class World {
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
    canvas.focus();
  }
}


class PositionComponent {
  constructor(x = 0, y = 0) {
    this.pos = new Vector2(x, y);
  }
}
class PrevFinalPosComponent {
  constructor(x = 0, y = 0) {
    this.pos = new Vector2(x, y);
  }
}
class VelocityComponent { constructor(x = 0, y = 0) { this.vel = new Vector2(x, y); } }
class RadiusComponent { constructor(radius = 0.1) { this.radius = radius; } }
class MassComponent { constructor(mass = 1.0) { this.mass = mass; } }
class RestitutionComponent { constructor(restitution = 0.5) { this.restitution = restitution; } }
class GravityAffectedComponent { }
class BallTagComponent { }
class BorderComponent { constructor(points = []) { this.points = points.map(p => p.clone()); } }
class FlipperTagComponent { }
class FlipperStateComponent {
  constructor(length, restAngle, maxRotation, angularVelocity) {
    this.length = length;
    this.restAngle = restAngle;
    this.maxRotation = Math.abs(maxRotation);
    this.sign = Math.sign(maxRotation); // Direction it rotates
    this.angularVelocity = angularVelocity;
    // Dynamic state
    this.rotation = 0.0; // Current rotation from restAngle
    this.currentAngularVelocity = 0.0; // Velocity in the last frame
    this.pressed = false; // Was it activated?
  }
}
class ObstacleTagComponent { }
class PauseStateComponent { constructor(paused = true) { this.paused = paused; } }
class SimulationErrorStateComponent { constructor(hasError = false) { this.hasError = hasError; } }
class OrientationComponent {
    constructor(angle = 0.0) {
        this.angle = angle; // Radians
    }
}
class AngularVelocityComponent {
    constructor(velocity = 0.0) {
        this.angularVelocity = velocity; // Radians per second
    }
}
class MomentOfInertiaComponent {
    constructor(inertia = 1.0) {
        this.inertia = inertia; // kg * m^2 (approximate for 2D)
        this.invInertia = inertia > 0 ? 1.0 / inertia : 0.0;
    }
}
class RenderableComponent {
  constructor(shape = 'circle', color = '#888888') {
    this.shape = shape; // 'circle', 'flipper', 'border'
    this.color = color;
  }
}

module.exports = {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  GravityAffectedComponent,
  BallTagComponent,
  BorderComponent,
  FlipperTagComponent,
  FlipperStateComponent,
  ObstacleTagComponent,
  PauseStateComponent,
  SimulationErrorStateComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RenderableComponent
};
