import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

import {
  PositionComponent,
  RadiusComponent,
  VelocityComponent,
  PrevFinalPosComponent,
  RenderableComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidGroupComponent,
  MachineTagComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

export {
  ExtruderComponent,
  SpoolTagComponent,
  SpoolStateComponent,
  StepperMotorComponent,
  RemoteSpoolSystem,
  setStepperTorqueMode,
  setStepperPositionMode,
  isStepperInTorqueMode,
  getStepperTorque,
} from '../slideprinter/slideprinter_common.js';

import {
  ExtruderComponent,
  SpoolTagComponent,
  SpoolStateComponent,
  StepperMotorComponent,
} from '../slideprinter/slideprinter_common.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0.0, 0.0, 1.0);

function normalizeAngle(angle) {
  let normalized = angle;
  while (normalized > Math.PI) normalized -= 2.0 * Math.PI;
  while (normalized < -Math.PI) normalized += 2.0 * Math.PI;
  return normalized;
}

function getPlanarAngle(quaternion) {
  if (!quaternion || typeof quaternion.transformVector !== 'function') {
    return 0.0;
  }
  const axis = quaternion.transformVector(new Vector3(1.0, 0.0, 0.0));
  return Math.atan2(axis.y, axis.x);
}

export class ExtruderSystem {
  update(world, dt) {
    let extruderComp = null;
    for (const entityId of world.query([ExtruderComponent])) {
      extruderComp = world.getComponent(entityId, ExtruderComponent);
      break;
    }

    if (!extruderComp) {
      return;
    }

    const spoolEntities = world.query([SpoolTagComponent, PositionComponent]);
    const sumByMachine = {};
    const countByMachine = {};

    for (const entityId of spoolEntities) {
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (!pos) {
        continue;
      }
      const machineTag = world.getComponent(entityId, MachineTagComponent);
      const machineId = machineTag?.id || 'default';
      let sum = sumByMachine[machineId];
      if (!sum) {
        sum = new Vector3();
        sumByMachine[machineId] = sum;
        countByMachine[machineId] = 0;
      }
      sum.add(pos);
      countByMachine[machineId] += 1;
    }

    const centerSources = extruderComp.centerSources && typeof extruderComp.centerSources === 'object'
      ? extruderComp.centerSources
      : {};
    const sourceMachineIds = Object.keys(centerSources);

    const resolveAverage = (entityIds) => {
      if (!Array.isArray(entityIds) || entityIds.length === 0) {
        return null;
      }
      const sum = new Vector3();
      let count = 0;
      for (const entityId of entityIds) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        if (pos) {
          sum.add(pos);
          count += 1;
        }
      }
      if (count === 0) {
        return null;
      }
      return sum.clone().scale(1.0 / count);
    };

    const machineCenters = {};
    for (const machineId of sourceMachineIds) {
      const entityIds = centerSources[machineId];
      let center = resolveAverage(entityIds);
      if (!center && sumByMachine[machineId] && (countByMachine[machineId] ?? 0) > 0) {
        center = sumByMachine[machineId].clone().scale(1.0 / countByMachine[machineId]);
      }
      if (center) {
        machineCenters[machineId] = center;
      }
    }

    if (sourceMachineIds.length === 0) {
      for (const machineId of Object.keys(sumByMachine)) {
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          machineCenters[machineId] = sumByMachine[machineId].clone().scale(1.0 / count);
        }
      }
    } else {
      for (const machineId of Object.keys(sumByMachine)) {
        if (machineCenters[machineId]) {
          continue;
        }
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          machineCenters[machineId] = sumByMachine[machineId].clone().scale(1.0 / count);
        }
      }
    }

    const machineIds = Object.keys(machineCenters);
    if (machineIds.length > 0) {
      extruderComp.machineCenters = machineCenters;
      const preferredOrder = sourceMachineIds.length > 0 ? sourceMachineIds : machineIds;
      let chosenId = preferredOrder.find((id) => machineCenters[id]) || null;
      if (!chosenId) {
        chosenId = machineIds[0];
      }
      extruderComp.centerPos = machineCenters[chosenId].clone();
    }
  }
}

export class StepperMotorSystem {
  update(world, dt) {
    const query = [
      StepperMotorComponent,
      OrientationComponent,
      AngularVelocityComponent,
      MomentOfInertiaComponent,
    ];

    const groupAngleByMember = new Map();
    try {
      const groups = world.query([RigidGroupComponent]);
      for (const groupId of groups) {
        const group = world.getComponent(groupId, RigidGroupComponent);
        const angle = group?.prevAngle || 0.0;
        const members = group?.members || [];
        for (const memberId of members) {
          groupAngleByMember.set(memberId, angle);
        }
      }
    } catch (_err) {
      // Ignore if the slideprinter scene does not use rigid groups.
    }

    for (const entityId of world.query(query)) {
      const stepper = world.getComponent(entityId, StepperMotorComponent);
      const orient = world.getComponent(entityId, OrientationComponent);
      const angVel = world.getComponent(entityId, AngularVelocityComponent);
      const inertia = world.getComponent(entityId, MomentOfInertiaComponent);
      if (!stepper || !orient || !angVel || !inertia) {
        continue;
      }

      const currentAngle = getPlanarAngle(orient.quaternion);
      const omegaZ = angVel.omega?.z ?? 0.0;
      let totalTorque;

      if (stepper.torqueMode) {
        const maxSpeedRad = Math.max(1e-6, stepper.maxSpeedRad ?? 600);
        const droop = Math.max(0, Math.min(1, 1 - Math.abs(omegaZ) / maxSpeedRad));
        const electricalTorque = stepper.targetTorque * droop;
        const dampingTorque = -(stepper.holdingTorque / maxSpeedRad) * omegaZ;
        const windageCoeff = stepper.windageCoeff ?? (stepper.dampingCoeff * 1e-3);
        const windageTorque = -windageCoeff * omegaZ * Math.abs(omegaZ);

        const epsW = 1e-3;
        const smoothSign = omegaZ / (Math.abs(omegaZ) + epsW);
        const coulomb = stepper.coulombFriction ?? (0.002 * stepper.holdingTorque);
        const stiction = stepper.stictionTorque ?? (0.003 * stepper.holdingTorque);
        const stictionSpeed = stepper.stictionSpeed ?? 1.0;
        const stictionFactor = Math.exp(-Math.abs(omegaZ) / stictionSpeed);
        const frictionTorque = -(coulomb + stiction * stictionFactor) * smoothSign;

        const cogAmp = stepper.coggingTorque ?? (0.01 * stepper.holdingTorque);
        const cogFreq = stepper.coggingFreq ?? stepper.numPolePairs;
        const coggingTorque = -cogAmp * Math.sin(cogFreq * currentAngle);

        totalTorque =
          electricalTorque +
          dampingTorque +
          windageTorque +
          frictionTorque +
          coggingTorque;
      } else {
        const groupAngle = groupAngleByMember.get(entityId) || 0.0;
        const targetWorldAngle = groupAngle + (stepper.commandedAngle - stepper.deltaAngle);
        const error = normalizeAngle(currentAngle - targetWorldAngle);
        const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);
        const dampingTorque = -stepper.dampingCoeff * omegaZ;
        totalTorque = restoringTorque + dampingTorque;
      }

      const angularAcceleration = totalTorque / inertia.inertia;
      angVel.omega.z += angularAcceleration * dt;
    }
  }
}

export class RemoteInputSystem {
  runInPause = true;

  constructor(canvas, world, ws) {
    this.canvas = canvas;
    this.world = world;
    this.ws = ws;
    this.scaleMultiplier = 1.0;
    this.viewOffsetX = 0.0;
    this.viewOffsetY = 0.0;
    this.interactionMode = 'select';
    this.isPanning = false;
    this.panPointerId = null;
    this.panLastX = 0;
    this.panLastY = 0;
    this.isOrbiting = false;
    this.orbitPointerId = null;
    this.orbitLastX = 0;
    this.orbitLastY = 0;
    this.onViewChange = null;

    this.handlePointerDown = this.handlePointerDown.bind(this);
    this.handlePointerMove = this.handlePointerMove.bind(this);
    this.handlePointerUp = this.handlePointerUp.bind(this);

    document.addEventListener('pointerdown', this.handlePointerDown);
    document.addEventListener('pointermove', this.handlePointerMove);
    document.addEventListener('pointerup', this.handlePointerUp);
  }

  setInteractionMode(mode) {
    this.interactionMode = mode === 'pan' ? 'pan' : 'select';
    if (this.interactionMode !== 'pan') {
      this.isPanning = false;
      this.panPointerId = null;
    } else if (this.isOrbiting) {
      this.isOrbiting = false;
      this.orbitPointerId = null;
    }
  }

  getRenderSystem() {
    return this.world.getResource('renderSystem') || null;
  }

  projectClientToSim(clientX, clientY) {
    const renderSystem = this.getRenderSystem();
    if (renderSystem && typeof renderSystem.projectClientToSim === 'function') {
      const projected = renderSystem.projectClientToSim(clientX, clientY);
      if (projected && Number.isFinite(projected.x) && Number.isFinite(projected.y)) {
        return { x: projected.x, y: projected.y };
      }
    }
    return null;
  }

  setViewTransform({ scaleMultiplier, offsetX, offsetY }) {
    if (typeof scaleMultiplier === 'number') {
      this.scaleMultiplier = scaleMultiplier;
    }
    if (typeof offsetX === 'number') {
      this.viewOffsetX = offsetX;
    }
    if (typeof offsetY === 'number') {
      this.viewOffsetY = offsetY;
    }
  }

  setViewChangeListener(listener) {
    this.onViewChange = typeof listener === 'function' ? listener : null;
  }

  toSimCoords(canvasX, canvasY) {
    const projected = this.projectClientToSim(canvasX, canvasY);
    if (projected) {
      return projected;
    }
    const rect = this.canvas.getBoundingClientRect();
    const baseScale = this.canvas.height / this.world.getResource('simHeight');
    const scale = baseScale * this.scaleMultiplier;
    const pixelX = canvasX - rect.left;
    const pixelY = canvasY - rect.top;
    const simX = (pixelX - this.canvas.width / 2) / scale + this.viewOffsetX;
    const simY = (this.canvas.height / 2 - pixelY) / scale + this.viewOffsetY;
    return { x: simX, y: simY };
  }

  handlePointerDown(event) {
    event.preventDefault();
    const onCanvas = event.target === this.canvas;
    if (onCanvas && this.interactionMode === 'pan') {
      this.isPanning = true;
      this.panPointerId = event.pointerId;
      this.panLastX = event.clientX;
      this.panLastY = event.clientY;
      if (typeof this.canvas.setPointerCapture === 'function') {
        try {
          this.canvas.setPointerCapture(event.pointerId);
        } catch (_err) {
          // Ignore pointer capture failures.
        }
      }
      return;
    }

    if (!onCanvas) {
      return;
    }

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const { x, y } = this.toSimCoords(event.clientX, event.clientY);

      let isGrabClick = false;
      for (const spoolId of this.world.query([SpoolTagComponent, PositionComponent, RadiusComponent])) {
        const pos = this.world.getComponent(spoolId, PositionComponent)?.pos;
        const radius = this.world.getComponent(spoolId, RadiusComponent)?.radius;
        if (!pos || !Number.isFinite(radius)) {
          continue;
        }
        const dx = x - pos.x;
        const dy = y - pos.y;
        if (dx * dx + dy * dy <= radius * radius) {
          isGrabClick = true;
          break;
        }
      }

      const shouldOrbit = !isGrabClick
        && event.pointerType === 'mouse'
        && event.button === 0
        && this.interactionMode !== 'pan'
        && typeof this.getRenderSystem()?.rotateOrbitByPixels === 'function';

      if (shouldOrbit) {
        this.isOrbiting = true;
        this.orbitPointerId = event.pointerId;
        this.orbitLastX = event.clientX;
        this.orbitLastY = event.clientY;
        if (typeof this.canvas.setPointerCapture === 'function') {
          try {
            this.canvas.setPointerCapture(event.pointerId);
          } catch (_err) {
            // Ignore pointer capture failures.
          }
        }
        return;
      }

      if (isGrabClick) {
        const pauseState = this.world.getResource('pauseState');
        if (pauseState && pauseState.paused) {
          pauseState.paused = false;
          const pauseBtn = document.getElementById('pauseBtn');
          if (pauseBtn) pauseBtn.textContent = 'Pause';
          this.ws.send(JSON.stringify({ action: 'pause', paused: false }));
        }
      }

      this.ws.send(JSON.stringify({ action: 'input', type: 'pointerdown', x, y }));
    }

    if (typeof this.canvas.setPointerCapture === 'function') {
      try {
        this.canvas.setPointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore pointer capture failures.
      }
    }
  }

  handlePointerMove(event) {
    if (this.isOrbiting && this.orbitPointerId === event.pointerId) {
      event.preventDefault();
      const renderSystem = this.getRenderSystem();
      if (renderSystem && typeof renderSystem.rotateOrbitByPixels === 'function') {
        renderSystem.rotateOrbitByPixels(
          event.clientX - this.orbitLastX,
          event.clientY - this.orbitLastY
        );
      }
      this.orbitLastX = event.clientX;
      this.orbitLastY = event.clientY;
      return;
    }

    if (this.interactionMode === 'pan') {
      if (!this.isPanning || this.panPointerId !== event.pointerId) {
        return;
      }
      event.preventDefault();
      const renderSystem = this.getRenderSystem();
      const prevPoint = renderSystem?.projectClientToSim?.(this.panLastX, this.panLastY) ?? null;
      const nextPoint = renderSystem?.projectClientToSim?.(event.clientX, event.clientY) ?? null;
      let nextOffsetX = this.viewOffsetX;
      let nextOffsetY = this.viewOffsetY;
      if (
        prevPoint && nextPoint
        && Number.isFinite(prevPoint.x) && Number.isFinite(prevPoint.y)
        && Number.isFinite(nextPoint.x) && Number.isFinite(nextPoint.y)
      ) {
        nextOffsetX += prevPoint.x - nextPoint.x;
        nextOffsetY += prevPoint.y - nextPoint.y;
      } else {
        const baseScale = this.canvas.height / this.world.getResource('simHeight');
        const scale = baseScale * this.scaleMultiplier;
        if (scale <= 0) {
          return;
        }
        const deltaX = event.clientX - this.panLastX;
        const deltaY = event.clientY - this.panLastY;
        nextOffsetX -= deltaX / scale;
        nextOffsetY += deltaY / scale;
      }
      this.panLastX = event.clientX;
      this.panLastY = event.clientY;
      this.viewOffsetX = nextOffsetX;
      this.viewOffsetY = nextOffsetY;
      if (this.onViewChange) {
        this.onViewChange({
          scale: this.scaleMultiplier,
          offsetX: this.viewOffsetX,
          offsetY: this.viewOffsetY,
        });
      }
      return;
    }

    if (event.target !== this.canvas) {
      return;
    }

    event.preventDefault();
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const { x, y } = this.toSimCoords(event.clientX, event.clientY);
      this.ws.send(JSON.stringify({ action: 'input', type: 'pointermove', x, y }));
    }
  }

  handlePointerUp(event) {
    event.preventDefault();
    if (this.isOrbiting && this.orbitPointerId === event.pointerId) {
      this.isOrbiting = false;
      this.orbitPointerId = null;
      if (typeof this.canvas.releasePointerCapture === 'function') {
        try {
          this.canvas.releasePointerCapture(event.pointerId);
        } catch (_err) {
          // Ignore pointer release failures.
        }
      }
      return;
    }
    if (this.interactionMode === 'pan') {
      if (this.isPanning && this.panPointerId === event.pointerId) {
        this.isPanning = false;
        this.panPointerId = null;
        if (typeof this.canvas.releasePointerCapture === 'function') {
          try {
            this.canvas.releasePointerCapture(event.pointerId);
          } catch (_err) {
            // Ignore pointer release failures.
          }
        }
      }
      return;
    }

    if (event.target !== this.canvas) {
      return;
    }

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const { x, y } = this.toSimCoords(event.clientX, event.clientY);
      this.ws.send(JSON.stringify({ action: 'input', type: 'pointerup', x, y }));
    }
    if (typeof this.canvas.releasePointerCapture === 'function') {
      try {
        this.canvas.releasePointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore pointer release failures.
      }
    }
  }

  update(world, dt) {}
}

export class InputSystem {
  runInPause = true;

  constructor(canvas, world, pauseBtn) {
    this.canvas = canvas;
    this.world = world;
    this.pauseBtn = pauseBtn;
    this.clicks = [];
    this.releases = [];
    this.eventLog = [];
    this.frame = 0;
    this.grabSpring = null;
    this.scaleMultiplier = 1.0;
    this.viewOffsetX = 0.0;
    this.viewOffsetY = 0.0;
    this.interactionMode = 'select';
    this.isPanning = false;
    this.panPointerId = null;
    this.panLastX = 0;
    this.panLastY = 0;
    this.isOrbiting = false;
    this.orbitPointerId = null;
    this.orbitLastX = 0;
    this.orbitLastY = 0;
    this.onViewChange = null;
    this.canvas.setAttribute('tabindex', '0');
    this.canvas.style.outline = 'none';
    this.canvas.focus();
    this.canvas.style.touchAction = 'none';
    this.touchActionBeforeGrab = null;
    this.activeGrabPointerId = null;
    this.scrollBlockerAttached = false;
    this.touchMoveListenerOptions = { passive: false, capture: true };
    this.activePointers = new Map();
    this.pinchActive = false;
    this.pinchLastDistance = 0;
    this.globalTouchOverrides = null;
    this.preventScrollDuringGrab = (event) => {
      if (this.activeGrabPointerId !== null && event.cancelable) {
        event.preventDefault();
      }
    };
    document.addEventListener('pointerdown', this.handlePointerDown.bind(this));
    document.addEventListener('pointerup', this.handlePointerUp.bind(this));
    document.addEventListener('pointercancel', this.handlePointerCancel.bind(this));
    document.addEventListener('pointermove', this.handlePointerMove.bind(this));
  }

  setInteractionMode(mode) {
    this.interactionMode = mode === 'pan' ? 'pan' : 'select';
    if (this.interactionMode !== 'pan' && this.isPanning) {
      this.cancelPan();
    }
    if (this.interactionMode === 'pan' && this.isOrbiting) {
      this.cancelOrbit();
    }
  }

  getRenderSystem() {
    return this.world.getResource('renderSystem') || null;
  }

  projectClientToSim(clientX, clientY) {
    const renderSystem = this.getRenderSystem();
    if (renderSystem && typeof renderSystem.projectClientToSim === 'function') {
      const projected = renderSystem.projectClientToSim(clientX, clientY);
      if (projected && Number.isFinite(projected.x) && Number.isFinite(projected.y)) {
        return { x: projected.x, y: projected.y };
      }
    }

    const rect = this.canvas.getBoundingClientRect();
    const baseScale = this.canvas.height / this.world.getResource('simHeight');
    const scale = baseScale * this.scaleMultiplier;
    return {
      x: (clientX - rect.left - this.canvas.width / 2) / scale + this.viewOffsetX,
      y: (this.canvas.height / 2 - (clientY - rect.top)) / scale + this.viewOffsetY,
    };
  }

  setViewTransform({ scaleMultiplier, offsetX, offsetY }) {
    if (typeof scaleMultiplier === 'number') {
      this.scaleMultiplier = scaleMultiplier;
    }
    if (typeof offsetX === 'number') {
      this.viewOffsetX = offsetX;
    }
    if (typeof offsetY === 'number') {
      this.viewOffsetY = offsetY;
    }
  }

  setViewChangeListener(listener) {
    this.onViewChange = typeof listener === 'function' ? listener : null;
  }

  reset() {
    this.clicks = [];
    this.releases = [];
    this.eventLog = [];
    this.frame = 0;
    this.isPanning = false;
    this.panPointerId = null;
    this.isOrbiting = false;
    this.orbitPointerId = null;
    this.orbitLastX = 0;
    this.orbitLastY = 0;
    this.activeGrabPointerId = null;
    this.setTouchScrollBlockActive(false);
    this.activePointers.clear();
    this.pinchActive = false;
    this.pinchLastDistance = 0;
    if (this.touchActionBeforeGrab !== null) {
      this.canvas.style.touchAction = this.touchActionBeforeGrab;
      this.touchActionBeforeGrab = null;
    }
    if (this.grabSpring) {
      const { ptrE, jointE, pathE } = this.grabSpring;
      this.world.destroyEntity(pathE);
      this.world.destroyEntity(jointE);
      this.world.destroyEntity(ptrE);
      this.grabSpring = null;
    }
  }

  setTouchScrollBlockActive(active) {
    if (typeof window === 'undefined' || !('ontouchstart' in window)) {
      return;
    }
    if (active) {
      if (!this.scrollBlockerAttached) {
        document.addEventListener('touchmove', this.preventScrollDuringGrab, this.touchMoveListenerOptions);
        this.scrollBlockerAttached = true;
      }
      this.applyGlobalTouchOverrides(true);
    } else if (this.scrollBlockerAttached) {
      document.removeEventListener('touchmove', this.preventScrollDuringGrab, this.touchMoveListenerOptions);
      this.scrollBlockerAttached = false;
      this.applyGlobalTouchOverrides(false);
    } else if (this.globalTouchOverrides) {
      this.applyGlobalTouchOverrides(false);
    }
  }

  applyGlobalTouchOverrides(activate) {
    if (typeof document === 'undefined') {
      return;
    }
    const docEl = document.documentElement;
    const body = document.body;
    if (!docEl || !body) {
      return;
    }

    if (activate) {
      if (!this.globalTouchOverrides) {
        this.globalTouchOverrides = {
          bodyTouchAction: body.style.touchAction,
          bodyOverflow: body.style.overflow,
          bodyOverscroll: body.style.overscrollBehavior,
          docTouchAction: docEl.style.touchAction,
          docOverscroll: docEl.style.overscrollBehavior,
        };
      }
      body.style.touchAction = 'none';
      body.style.overflow = 'hidden';
      if (body.style.overscrollBehavior !== undefined) {
        body.style.overscrollBehavior = 'none';
      }
      docEl.style.touchAction = 'none';
      if (docEl.style.overscrollBehavior !== undefined) {
        docEl.style.overscrollBehavior = 'none';
      }
      return;
    }

    if (!this.globalTouchOverrides) {
      return;
    }
    body.style.touchAction = this.globalTouchOverrides.bodyTouchAction;
    body.style.overflow = this.globalTouchOverrides.bodyOverflow;
    if (body.style.overscrollBehavior !== undefined) {
      body.style.overscrollBehavior = this.globalTouchOverrides.bodyOverscroll;
    }
    docEl.style.touchAction = this.globalTouchOverrides.docTouchAction;
    if (docEl.style.overscrollBehavior !== undefined) {
      docEl.style.overscrollBehavior = this.globalTouchOverrides.docOverscroll;
    }
    this.globalTouchOverrides = null;
  }

  shouldStartAuxPan(event) {
    if (!event || event.pointerType !== 'mouse') {
      return false;
    }
    return event.button === 1;
  }

  beginPan(event) {
    this.isPanning = true;
    this.panPointerId = event.pointerId;
    this.panLastX = event.clientX;
    this.panLastY = event.clientY;
  }

  beginOrbit(event) {
    this.isOrbiting = true;
    this.orbitPointerId = event.pointerId;
    this.orbitLastX = event.clientX;
    this.orbitLastY = event.clientY;
  }

  cancelPan() {
    if (!this.isPanning) {
      return;
    }
    if (typeof this.canvas.releasePointerCapture === 'function' && this.panPointerId !== null) {
      try {
        this.canvas.releasePointerCapture(this.panPointerId);
      } catch (_err) {
        // Ignore pointer release failures.
      }
    }
    this.isPanning = false;
    this.panPointerId = null;
  }

  cancelOrbit() {
    if (!this.isOrbiting) {
      return;
    }
    if (typeof this.canvas.releasePointerCapture === 'function' && this.orbitPointerId !== null) {
      try {
        this.canvas.releasePointerCapture(this.orbitPointerId);
      } catch (_err) {
        // Ignore pointer release failures.
      }
    }
    this.isOrbiting = false;
    this.orbitPointerId = null;
  }

  trackPointerDown(event) {
    if (!event || event.pointerType !== 'touch') {
      return;
    }
    this.activePointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
    if (this.activePointers.size >= 2) {
      this.beginPinch();
    }
  }

  trackPointerMove(event) {
    if (event.pointerType === 'touch' && this.activePointers.has(event.pointerId)) {
      this.activePointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
    }
  }

  trackPointerEnd(event) {
    if (event.pointerType !== 'touch') {
      return;
    }
    this.activePointers.delete(event.pointerId);
    if (this.activePointers.size < 2) {
      this.endPinch();
    }
  }

  computePinchMetrics() {
    if (this.activePointers.size < 2) {
      return null;
    }
    const iterator = this.activePointers.values();
    const first = iterator.next().value;
    const second = iterator.next().value;
    if (!first || !second) {
      return null;
    }
    const dx = first.x - second.x;
    const dy = first.y - second.y;
    const distance = Math.hypot(dx, dy);
    if (!(distance > 0)) {
      return null;
    }
    return {
      distance,
      centerX: (first.x + second.x) * 0.5,
      centerY: (first.y + second.y) * 0.5,
    };
  }

  beginPinch() {
    if (this.pinchActive || this.activePointers.size < 2) {
      return;
    }
    const metrics = this.computePinchMetrics();
    if (!metrics) {
      return;
    }
    this.cancelPan();
    this.pinchActive = true;
    this.pinchLastDistance = metrics.distance;
  }

  updatePinchGesture() {
    if (!this.pinchActive) {
      return;
    }
    const metrics = this.computePinchMetrics();
    if (!metrics) {
      return;
    }
    const { distance, centerX, centerY } = metrics;
    if (!(distance > 0)) {
      return;
    }
    if (this.pinchLastDistance <= 0) {
      this.pinchLastDistance = distance;
      return;
    }
    const delta = distance / this.pinchLastDistance;
    if (!Number.isFinite(delta) || delta <= 0) {
      this.pinchLastDistance = distance;
      return;
    }
    const simHeight = this.world.getResource('simHeight');
    if (!Number.isFinite(simHeight) || simHeight === 0) {
      this.pinchLastDistance = distance;
      return;
    }
    const baseScale = this.canvas.height / simHeight;
    if (!Number.isFinite(baseScale) || baseScale <= 0) {
      this.pinchLastDistance = distance;
      return;
    }
    const rect = this.canvas.getBoundingClientRect();
    const prevScale = baseScale * this.scaleMultiplier;
    if (!(prevScale > 0)) {
      this.pinchLastDistance = distance;
      return;
    }
    const pixelX = centerX - rect.left;
    const pixelY = centerY - rect.top;
    const projected = this.projectClientToSim(centerX, centerY);
    const simX = projected?.x ?? ((pixelX - this.canvas.width / 2) / prevScale + this.viewOffsetX);
    const simY = projected?.y ?? ((this.canvas.height / 2 - pixelY) / prevScale + this.viewOffsetY);
    const nextScaleMultiplier = this.scaleMultiplier * delta;
    const nextScale = baseScale * nextScaleMultiplier;
    if (!(nextScale > 0)) {
      this.pinchLastDistance = distance;
      return;
    }
    const nextOffsetX = simX - (pixelX - this.canvas.width / 2) / nextScale;
    const nextOffsetY = simY - (this.canvas.height / 2 - pixelY) / nextScale;
    this.scaleMultiplier = nextScaleMultiplier;
    this.viewOffsetX = nextOffsetX;
    this.viewOffsetY = nextOffsetY;
    if (this.onViewChange) {
      this.onViewChange(
        {
          scale: nextScaleMultiplier,
          offsetX: nextOffsetX,
          offsetY: nextOffsetY,
        },
        { gesture: 'pinch' }
      );
    }
    this.pinchLastDistance = distance;
  }

  endPinch() {
    if (!this.pinchActive) {
      return;
    }
    this.pinchActive = false;
    this.pinchLastDistance = 0;
  }

  handlePointerDown(event) {
    if (event.target !== this.canvas) return;
    event.preventDefault();
    if (typeof this.canvas.setPointerCapture === 'function') {
      try {
        this.canvas.setPointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore browsers that disallow capture here.
      }
    }
    this.trackPointerDown(event);
    const pinchEngaged = this.pinchActive;
    const wantsAuxPan = this.shouldStartAuxPan(event);
    const usePanMode = this.interactionMode === 'pan' || wantsAuxPan;
    if ((event.pointerType === 'touch' || event.pointerType === 'pen') && this.interactionMode !== 'pan') {
      this.activeGrabPointerId = event.pointerId;
      this.setTouchScrollBlockActive(true);
    }
    if (pinchEngaged) {
      return;
    }
    if (usePanMode) {
      this.beginPan(event);
      return;
    }

    const { x: simX, y: simY } = this.projectClientToSim(event.clientX, event.clientY);

    const cmOnScreen = (event.pointerType === 'touch' || event.pointerType === 'pen') ? 1.5 : 1.0;
    const dpi = 96;
    const pixelsPerCm = dpi / 2.54;
    const extraPixels = cmOnScreen * pixelsPerCm;
    const effectiveScale = (this.canvas.height / this.world.getResource('simHeight')) * this.scaleMultiplier;
    const extraClickableRadius = extraPixels / effectiveScale;

    let closestBall = null;
    let closestDistSq = Infinity;
    for (const entityId of this.world.query([SpoolTagComponent, PositionComponent, RadiusComponent])) {
      const pos = this.world.getComponent(entityId, PositionComponent)?.pos;
      const radius = this.world.getComponent(entityId, RadiusComponent)?.radius;
      if (!pos || !Number.isFinite(radius)) {
        continue;
      }
      const dx = simX - pos.x;
      const dy = simY - pos.y;
      const distSq = dx * dx + dy * dy;
      const limit = radius + extraClickableRadius;
      if (distSq <= limit * limit && distSq < closestDistSq) {
        closestBall = entityId;
        closestDistSq = distSq;
      }
    }

    const shouldOrbit = closestBall === null
      && event.pointerType === 'mouse'
      && event.button === 0
      && this.interactionMode !== 'pan'
      && typeof this.getRenderSystem()?.rotateOrbitByPixels === 'function';

    if (shouldOrbit) {
      this.beginOrbit(event);
      return;
    }

    if (closestBall === null) {
      return;
    }

    if ((event.pointerType === 'touch' || event.pointerType === 'pen') && this.interactionMode !== 'pan') {
      if (this.touchActionBeforeGrab === null) {
        this.touchActionBeforeGrab = this.canvas.style.touchAction;
      }
      this.canvas.style.touchAction = 'none';
    }

    const ptrPos = new Vector3(simX, simY, 0.0);
    const ballPos = this.world.getComponent(closestBall, PositionComponent).pos.clone();

    const ptrE = this.world.createEntity();
    this.world.addComponent(ptrE, new PositionComponent(ptrPos.x, ptrPos.y, ptrPos.z));
    this.world.addComponent(ptrE, new CableLinkComponent(ptrPos.x, ptrPos.y, ptrPos.z, null, DEFAULT_PLANE_NORMAL));

    const jointE = this.world.createEntity();
    this.world.addComponent(
      jointE,
      CableJointComponent.fromWorld(this.world, closestBall, ptrE, 0.1, ballPos, ptrPos)
    );
    this.world.addComponent(jointE, new RenderableComponent('line', '#888888'));

    const pathE = this.world.createEntity();
    this.world.addComponent(
      pathE,
      new CablePathComponent(this.world, [jointE], ['attachment', 'attachment'], [true, true], 10.0)
    );

    this.grabSpring = { ptrE, jointE, pathE, ballE: closestBall };
    this.world.setResource('grabbedBall', closestBall);

    const pauseState = this.world.getResource('pauseState');
    if (pauseState) {
      pauseState.paused = false;
    }
    if (this.pauseBtn) {
      this.pauseBtn.textContent = 'Pause';
    }
    if (typeof this.canvas.setPointerCapture === 'function') {
      try {
        this.canvas.setPointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore browsers that disallow capture here.
      }
    }
  }

  handlePointerUp(event) {
    const wasPanPointer = this.isPanning && this.panPointerId === event.pointerId;
    const wasOrbitPointer = this.isOrbiting && this.orbitPointerId === event.pointerId;
    this.trackPointerEnd(event);
    if (wasPanPointer) {
      event.preventDefault();
      if (this.onViewChange) {
        this.onViewChange(
          {
            scale: this.scaleMultiplier,
            offsetX: this.viewOffsetX,
            offsetY: this.viewOffsetY,
          },
          { forceRedraw: true }
        );
      }
      this.cancelPan();
      return;
    }
    if (wasOrbitPointer) {
      event.preventDefault();
      this.cancelOrbit();
      return;
    }
    if (event.target !== this.canvas) {
      if (typeof this.canvas.releasePointerCapture === 'function') {
        try {
          this.canvas.releasePointerCapture(event.pointerId);
        } catch (_err) {
          // Ignore errors if capture was never set.
        }
      }
      return;
    }
    event.preventDefault();
    if (typeof this.canvas.releasePointerCapture === 'function') {
      try {
        this.canvas.releasePointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore errors if capture was never set.
      }
    }

    if (this.grabSpring) {
      const { ptrE, jointE, pathE, ballE } = this.grabSpring;

      const posComp = this.world.getComponent(ballE, PositionComponent);
      const velComp = this.world.getComponent(ballE, VelocityComponent);
      const prevFinalPosComp = this.world.getComponent(ballE, PrevFinalPosComponent);
      const dt = this.world.getResource('dt');

      if (velComp && posComp && prevFinalPosComp && dt > 1e-9) {
        velComp.vel.set(posComp.pos.clone().subtract(prevFinalPosComp.pos).scale(1.0 / dt));
      } else if (velComp) {
        velComp.vel.set(new Vector3(0.0, 0.0, 0.0));
      }

      this.world.destroyEntity(pathE);
      this.world.destroyEntity(jointE);
      this.world.destroyEntity(ptrE);
      this.grabSpring = null;
      this.world.setResource('grabbedBall', null);
      if (this.touchActionBeforeGrab !== null && this.interactionMode !== 'pan') {
        this.canvas.style.touchAction = this.touchActionBeforeGrab;
        this.touchActionBeforeGrab = null;
      }
    }
    if (this.activeGrabPointerId === event.pointerId) {
      this.activeGrabPointerId = null;
      this.setTouchScrollBlockActive(false);
    }
  }

  handlePointerCancel(event) {
    this.handlePointerUp(event);
  }

  update(world, dt) {}

  handlePointerMove(event) {
    if (event.pointerType === 'touch') {
      this.trackPointerMove(event);
    }
    const isPanPointer = this.isPanning && this.panPointerId === event.pointerId;
    const isOrbitPointer = this.isOrbiting && this.orbitPointerId === event.pointerId;
    if (this.pinchActive) {
      event.preventDefault();
      this.updatePinchGesture();
      return;
    }
    if (isPanPointer) {
      event.preventDefault();
      const renderSystem = this.getRenderSystem();
      const prevPoint = renderSystem?.projectClientToSim?.(this.panLastX, this.panLastY) ?? null;
      const nextPoint = renderSystem?.projectClientToSim?.(event.clientX, event.clientY) ?? null;
      let nextOffsetX = this.viewOffsetX;
      let nextOffsetY = this.viewOffsetY;
      if (
        prevPoint && nextPoint
        && Number.isFinite(prevPoint.x) && Number.isFinite(prevPoint.y)
        && Number.isFinite(nextPoint.x) && Number.isFinite(nextPoint.y)
      ) {
        nextOffsetX += prevPoint.x - nextPoint.x;
        nextOffsetY += prevPoint.y - nextPoint.y;
      } else {
        const baseScale = this.canvas.height / this.world.getResource('simHeight');
        const scale = baseScale * this.scaleMultiplier;
        if (scale <= 0) {
          return;
        }
        const deltaX = event.clientX - this.panLastX;
        const deltaY = event.clientY - this.panLastY;
        nextOffsetX -= deltaX / scale;
        nextOffsetY += deltaY / scale;
      }
      this.panLastX = event.clientX;
      this.panLastY = event.clientY;
      this.viewOffsetX = nextOffsetX;
      this.viewOffsetY = nextOffsetY;
      if (this.onViewChange) {
        this.onViewChange({
          scale: this.scaleMultiplier,
          offsetX: this.viewOffsetX,
          offsetY: this.viewOffsetY,
        });
      }
      return;
    }
    if (isOrbitPointer) {
      event.preventDefault();
      const renderSystem = this.getRenderSystem();
      if (renderSystem && typeof renderSystem.rotateOrbitByPixels === 'function') {
        renderSystem.rotateOrbitByPixels(
          event.clientX - this.orbitLastX,
          event.clientY - this.orbitLastY
        );
      }
      this.orbitLastX = event.clientX;
      this.orbitLastY = event.clientY;
      return;
    }
    if (event.target !== this.canvas || this.interactionMode === 'pan' || this.grabSpring === null) {
      return;
    }

    event.preventDefault();
    const { x: simX, y: simY } = this.projectClientToSim(event.clientX, event.clientY);
    const { ptrE } = this.grabSpring;
    const pos = this.world.getComponent(ptrE, PositionComponent)?.pos;
    if (pos) {
      pos.set(new Vector3(simX, simY, 0.0));
    }
  }
}
