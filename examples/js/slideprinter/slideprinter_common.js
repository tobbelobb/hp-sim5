import Vector2 from '../../../src/js/cable_joints/vector2.js';
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
} from '../../../src/js/cable_joints/ecs.js';
import {
    CableLinkComponent,
    CableJointComponent,
    CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';

const SIMULATION_PLAYBACK_RESOURCE = 'simulationPlayback';

export class ExtruderComponent {
    constructor() {
        // Array of { pos: [x,y,z], length, machineId, color }
        this.extrusions = [];
        this.centerPos = new Vector2(0.0, 0.0);
        this.machineCenters = {};
        this.centerSources = {};
    }
}

export class ExtruderSystem {
    update(world, dt) {
        let extruderComp = null;
        for (const e of world.query([ExtruderComponent])) {
            extruderComp = world.getComponent(e, ExtruderComponent);
            break;
        }

        if (!extruderComp) {
            return;
        }

        const spoolEntities = world.query([SpoolTagComponent, PositionComponent]);
        const sumByMachine = {};
        const countByMachine = {};
        for (const e of spoolEntities) {
            const pos = world.getComponent(e, PositionComponent)?.pos;
            if (!pos) {
                continue;
            }
            const machineTag = world.getComponent(e, MachineTagComponent);
            const machineId = machineTag?.id || 'default';
            let sum = sumByMachine[machineId];
            if (!sum) {
                sum = new Vector2();
                sumByMachine[machineId] = sum;
                countByMachine[machineId] = 0;
            }
            sum.add(pos);
            countByMachine[machineId] += 1;
        }

        const machineCenters = {};
        const centerSources = extruderComp.centerSources && typeof extruderComp.centerSources === 'object'
            ? extruderComp.centerSources
            : {};
        const sourceMachineIds = Object.keys(centerSources);

        const resolveAverage = (entityIds) => {
            if (!Array.isArray(entityIds) || entityIds.length === 0) {
                return null;
            }
            const sum = new Vector2();
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
            return sum.clone().scale(1 / count);
        };

        for (const machineId of sourceMachineIds) {
            const targetIds = centerSources[machineId];
            let center = resolveAverage(targetIds);
            if (!center && sumByMachine[machineId] && (countByMachine[machineId] ?? 0) > 0) {
                center = sumByMachine[machineId].clone().scale(1 / countByMachine[machineId]);
            }
            if (center) {
                machineCenters[machineId] = center;
            }
        }

        if (sourceMachineIds.length === 0) {
            for (const machineId of Object.keys(sumByMachine)) {
                const count = countByMachine[machineId] ?? 0;
                if (count > 0) {
                    machineCenters[machineId] = sumByMachine[machineId].clone().scale(1 / count);
                }
            }
        } else {
            for (const machineId of Object.keys(sumByMachine)) {
                if (machineCenters[machineId]) {
                    continue;
                }
                const count = countByMachine[machineId] ?? 0;
                if (count > 0) {
                    machineCenters[machineId] = sumByMachine[machineId].clone().scale(1 / count);
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

export class SpoolTagComponent {}

export class SpoolStateComponent {
    constructor(axis = null) {
        this.axis = axis;
    }
}

// Adapted from the Stepper Motor Model for Dynamic Simulation paper by Alexandru Morar.
// Parameters are typically authored in the USD scene and applied when the prim loads.
export class StepperMotorComponent {
    constructor(
        commandedAngle = 0.0,
        deltaAngle = 0.0,
        holdingTorque = 0.5, // Nm. Within the typical range for Nema 17 motors
        numPolePairs = 50,   // For a 1.8 deg/step motor
        dampingCoeff = 0.01  // Gotten by trial and error. For stepper inertia 5e-05
    ) {
        this.commandedAngle = commandedAngle;
        this.deltaAngle = deltaAngle;
        this.holdingTorque = holdingTorque;
        this.numPolePairs = numPolePairs;
        this.dampingCoeff = dampingCoeff;
    }
}

// Adapted from the Stepper Motor Model for Dynamic Simulation paper by Alexandru Morar
export class StepperMotorSystem {
    update(world, dt) {
        const query = [
            StepperMotorComponent,
            OrientationComponent,
            AngularVelocityComponent,
            MomentOfInertiaComponent,
        ];

        // Build a quick lookup: entityId -> group angle (if in a rigid group)
        const groupAngleByMember = new Map();
        try {
            const groups = world.query([RigidGroupComponent]);
            for (const gid of groups) {
                const group = world.getComponent(gid, RigidGroupComponent);
                const angle = group?.prevAngle || 0.0;
                const members = group?.members || [];
                for (const m of members) {
                    groupAngleByMember.set(m, angle);
                }
            }
        } catch(_e) {
            // RigidGroupComponent may not be present; ignore
        }
        for (const e of world.query(query)) {
            const stepper = world.getComponent(e, StepperMotorComponent);
            const orient = world.getComponent(e, OrientationComponent);
            const angVel = world.getComponent(e, AngularVelocityComponent);
            const inertia = world.getComponent(e, MomentOfInertiaComponent);

            // Calculate restoring torque based on angular error.
            // Make the stepper act in the group's local frame so rigid-group rotation
            // does not fight the motor controller. Target in world = groupAngle + commanded.
            const groupAngle = groupAngleByMember.get(e) || 0.0;
            const targetWorldAngle = groupAngle + (stepper.commandedAngle - stepper.deltaAngle);
            const error = orient.angle - targetWorldAngle;
            const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);

            // Add damping to help the motor settle
            const dampingTorque = -stepper.dampingCoeff * angVel.angularVelocity;

            const totalTorque = restoringTorque + dampingTorque;

            // Apply torque to angular velocity (F=ma -> a=F/m -> v=v+a*dt)
            const angularAcceleration = totalTorque / inertia.inertia;
            angVel.angularVelocity += angularAcceleration * dt;
            //orient.angle = stepper.commandedAngle;
        }
    }
}

export class RemoteSpoolSystem {
    constructor() {
        this._commands = [];
        this.commandHead = 0;
        this.commandCompactThreshold = 1024;
        Object.defineProperty(this, 'commands', {
            configurable: true,
            enumerable: true,
            get: () => this._commands,
            set: (value) => {
                if (Array.isArray(value)) {
                    this._commands = value;
                } else if (value && typeof value.length === 'number') {
                    this._commands = Array.from(value);
                } else {
                    this._commands = [];
                }
                this.commandHead = 0;
            },
        });
        this.axisToEntity = {};
        this.wasPaused = false;
        // Base queue watermarks. Actual watermarks scale with playback speed.
        this.baseHighWaterMark = 80;
        this.baseLowWaterMark = 40;
        this.highWaterMark = this.baseHighWaterMark;
        this.lowWaterMark = this.baseLowWaterMark;
        this.fastModeActive = false;
        this.history = [];
        this.onCommandExecuted = null;
        this.onExtrusion = null;
        this.playbackMode = 'linear';
        this.asapModeActive = false;
        this._worker = null;
        Object.defineProperty(this, 'worker', {
            configurable: true,
            enumerable: true,
            get: () => this._worker,
            set: (value) => {
                this._worker = value || null;
                if (!this._worker || typeof this._worker.postMessage !== 'function') {
                    return;
                }
                try {
                    this._worker.postMessage({
                        type: 'set_asap_mode',
                        enable: this.asapModeActive,
                    });
                } catch (err) {
                    console.warn('RemoteSpoolSystem: unable to sync ASAP mode on worker.', err);
                }
            },
        });
    }

    resetAxisMapping() {
        this.axisToEntity = {};
    }

    _ensureAxisMapping(world) {
        if (Object.keys(this.axisToEntity).length !== 0) {
            return;
        }
        const spoolEntities = world.query([SpoolTagComponent, SpoolStateComponent]);
        for (const e of spoolEntities) {
            const state = world.getComponent(e, SpoolStateComponent);
            if (!state?.axis) {
                continue;
            }
            if (!this.axisToEntity[state.axis]) {
                this.axisToEntity[state.axis] = [];
            }
            this.axisToEntity[state.axis].push(e);
        }
    }

    addCommand(command) {
        this._commands.push(command);
    }

    clearPlaybackState() {
        this.history = [];
        this._commands.length = 0;
        this.commandHead = 0;
    }

    getPlaybackState() {
        return {
            history: this.history.map((cmd) => ({ ...cmd })),
            queue: this._commands.slice(this.commandHead).map((cmd) => ({ ...cmd })),
        };
    }

    setPlaybackState(state = {}) {
        const nextHistory = Array.isArray(state.history) ? state.history : [];
        const nextQueue = Array.isArray(state.queue) ? state.queue : [];
        this.history = nextHistory.map((cmd) => ({ ...cmd }));
        this._commands = nextQueue.map((cmd) => ({ ...cmd }));
        this.commandHead = 0;
    }

    setCommandExecutedListener(listener) {
        this.onCommandExecuted = typeof listener === 'function' ? listener : null;
    }

    setExtrusionListener(listener) {
        this.onExtrusion = typeof listener === 'function' ? listener : null;
    }

    getQueueLength() {
        const available = this._commands.length - this.commandHead;
        return available > 0 ? available : 0;
    }

    clearCommandQueue() {
        this._commands.length = 0;
        this.commandHead = 0;
    }

    _compactCommands(force = false) {
        if (this.commandHead === 0) {
            return;
        }
        const remaining = this._commands.length - this.commandHead;
        if (!force) {
            const thresholdReached = this.commandHead >= this.commandCompactThreshold;
            const moreConsumedThanRemaining = this.commandHead >= remaining;
            if (!thresholdReached && !moreConsumedThanRemaining) {
                return;
            }
        }
        if (remaining > 0) {
            this._commands.splice(0, this.commandHead);
        } else {
            this._commands.length = 0;
        }
        this.commandHead = 0;
    }

    _processCommand(world, command, options = {}) {
        if (!command) {
            return;
        }
        const { recordHistory = true, emitEvents = true } = options;
        const recordedCommand = { ...command };
        if (recordHistory) {
            this.history.push(recordedCommand);
        }
        if (emitEvents && this.onCommandExecuted) {
            try {
                this.onCommandExecuted(recordedCommand);
            } catch (err) {
                console.warn('RemoteSpoolSystem: command listener threw', err);
            }
        }

        const commandType = command?.type || '';
        const touchedMachines = new Set();
        const colorByMachine = new Map();
        const globalMachineColors = world.getResource('machineColors');
        const machineStore = world.getComponentStore ? world.getComponentStore(MachineTagComponent) : null;
        const renderStore = world.getComponentStore ? world.getComponentStore(RenderableComponent) : null;
        const stepperStore = world.getComponentStore ? world.getComponentStore(StepperMotorComponent) : null;

        for (const axis of Object.keys(this.axisToEntity)) {
            const axisValue = command[axis];
            if (axisValue === undefined) {
                continue;
            }
            const mapping = this.axisToEntity[axis];
            if (!mapping || (Array.isArray(mapping) && mapping.length === 0)) {
                continue;
            }
            const entityIds = Array.isArray(mapping) ? mapping : [mapping];
            for (const entityId of entityIds) {
                if (entityId == null) {
                    continue;
                }
                const machineTag = machineStore ? machineStore.get(entityId) : world.getComponent(entityId, MachineTagComponent);
                const machineId = machineTag?.id || null;
                if (!machineId) {
                    continue;
                }
                touchedMachines.add(machineId);
                if (!colorByMachine.has(machineId)) {
                    let chosenColor = null;
                    if (globalMachineColors && typeof globalMachineColors.get === 'function') {
                        const record = globalMachineColors.get(machineId);
                        if (record && typeof record.extrusionColor === 'string' && record.extrusionColor.length > 0) {
                            chosenColor = record.extrusionColor;
                        }
                    }
                    if (!chosenColor) {
                        const renderComp = renderStore ? renderStore.get(entityId) : world.getComponent(entityId, RenderableComponent);
                        if (renderComp?.color) {
                            chosenColor = renderComp.color;
                        }
                    }
                    if (chosenColor) {
                        colorByMachine.set(machineId, chosenColor);
                    }
                }

                if (commandType === 'Move') {
                    const stepperComp = stepperStore ? stepperStore.get(entityId) : world.getComponent(entityId, StepperMotorComponent);
                    if (stepperComp != null) {
                        stepperComp.commandedAngle = axisValue;
                    }
                }
                if (commandType === 'Add to reference') {
                    const stepperComp = stepperStore ? stepperStore.get(entityId) : world.getComponent(entityId, StepperMotorComponent);
                    if (stepperComp) {
                        stepperComp.deltaAngle += axisValue;
                    }
                }
            }
        }

        const touchedList = Array.from(touchedMachines);
        if (recordHistory && touchedList.length > 0) {
            recordedCommand.__touchedMachines = touchedList;
        }

        if (command.E !== undefined && command.E > 0.0) {
            const extruderStore = world.getComponentStore ? world.getComponentStore(ExtruderComponent) : null;
            let extruderComp = null;
            if (extruderStore && extruderStore.size > 0) {
                const iter = extruderStore.values();
                const first = iter.next();
                if (!first.done) {
                    extruderComp = first.value;
                }
            } else {
                for (const e of world.query([ExtruderComponent])) {
                    extruderComp = world.getComponent(e, ExtruderComponent);
                    break;
                }
            }
            if (extruderComp != null) {
                const recordedMachines = Array.isArray(command.__touchedMachines)
                    ? command.__touchedMachines
                    : null;
                let machineIds = touchedMachines.size > 0 ? Array.from(touchedMachines) : [];
                if (machineIds.length === 0 && recordedMachines && recordedMachines.length > 0) {
                    machineIds = recordedMachines.slice();
                }

                const machineCenters = extruderComp.machineCenters || {};
                const centerKeys = Object.keys(machineCenters);

                if (machineIds.length === 0 && centerKeys.length === 1) {
                    machineIds = [centerKeys[0]];
                }

                for (const machineId of machineIds) {
                    if (!machineId) {
                        continue;
                    }
                    let center = machineCenters[machineId];
                    if (!center) {
                        if (centerKeys.length === 0 && extruderComp.centerPos) {
                            center = extruderComp.centerPos;
                        } else {
                            continue;
                        }
                    }
                    let color = colorByMachine.get(machineId) || null;
                    if (!color && globalMachineColors && typeof globalMachineColors.get === 'function') {
                        const record = globalMachineColors.get(machineId);
                        if (record && typeof record.extrusionColor === 'string' && record.extrusionColor.length > 0) {
                            color = record.extrusionColor;
                        }
                    }
                    if (!center) {
                        continue;
                    }
                    const extrusionEvent = {
                        pos: [center.x, center.y, 0],
                        length: command.E,
                        machineId,
                        color,
                    };
                    extruderComp.extrusions.push(extrusionEvent);
                    if (emitEvents && this.onExtrusion) {
                        try {
                            this.onExtrusion({ ...extrusionEvent });
                        } catch (err) {
                            console.warn('RemoteSpoolSystem: extrusion listener threw', err);
                        }
                    }
                }
            }
        }
    }

    replayHistory(world, commands, options = {}) {
        if (!Array.isArray(commands) || commands.length === 0) {
            return;
        }
        const { recordHistory = false, emitEvents = false } = options;
        this.resetAxisMapping();
        this._ensureAxisMapping(world);
        for (const command of commands) {
            this._processCommand(world, command, { recordHistory, emitEvents });
        }
    }

    update(world, dt) {
        // Scale queue targets with playback speed to avoid underflows at high speeds
        const playbackState = world.getResource(SIMULATION_PLAYBACK_RESOURCE) || null;
        const desiredPlaybackMode = playbackState?.mode === 'asap' ? 'asap' : 'linear';
        if (desiredPlaybackMode !== this.playbackMode) {
            this.playbackMode = desiredPlaybackMode;
            this.asapModeActive = desiredPlaybackMode === 'asap';
            if (this.worker) {
                try {
                    this.worker.postMessage({ type: 'set_asap_mode', enable: this.asapModeActive });
                } catch (err) {
                    console.warn('RemoteSpoolSystem: unable to toggle ASAP mode on worker.', err);
                }
                if (this.asapModeActive) {
                    try {
                        this.worker.postMessage({ type: 'set_fast_mode', enable: false });
                    } catch (_err) {
                        /* noop */
                    }
                    try {
                        this.worker.postMessage({ type: 'resume' });
                    } catch (_err) {
                        /* noop */
                    }
                }
            }
            if (!this.asapModeActive) {
                this.fastModeActive = false;
                this.wasPaused = false;
            }
        }

        const timeScale = Number(world.getResource('timeScale')) || 1;
        const scaleFactor = Math.max(1, timeScale);
        const targetHigh = Math.max(this.baseHighWaterMark, Math.ceil(this.baseHighWaterMark * scaleFactor));
        const targetLow = Math.max(this.baseLowWaterMark, Math.ceil(this.baseLowWaterMark * scaleFactor * 0.75));
        if (targetHigh !== this.highWaterMark || targetLow !== this.lowWaterMark) {
            this.highWaterMark = targetHigh;
            this.lowWaterMark = Math.min(targetHigh - 1, targetLow);
        }

        const queueSize = this.getQueueLength();
        if (this.worker && !this.asapModeActive) {
            // Engage fast mode when the queue runs low; disengage when recovered
            if (!this.fastModeActive && queueSize < this.lowWaterMark) {
                this.worker.postMessage({ type: 'set_fast_mode', enable: true });
                this.fastModeActive = true;
            } else if (this.fastModeActive && queueSize >= Math.max(this.lowWaterMark, Math.floor(this.highWaterMark * 0.7))) {
                this.worker.postMessage({ type: 'set_fast_mode', enable: false });
                this.fastModeActive = false;
            }
            if (queueSize > this.highWaterMark && !this.wasPaused) {
                this.worker.postMessage({ type: 'pause' });
                this.wasPaused = true;
            } else if (queueSize < this.lowWaterMark && this.wasPaused) {
                this.worker.postMessage({ type: 'resume' });
                this.wasPaused = false;
            }
        }
        if (this.worker && this.asapModeActive) {
            if (this.fastModeActive) {
                this.worker.postMessage({ type: 'set_fast_mode', enable: false });
                this.fastModeActive = false;
            }
            if (this.wasPaused) {
                this.worker.postMessage({ type: 'resume' });
                this.wasPaused = false;
            }
        }

        this._ensureAxisMapping(world);

        const command = this._commands[this.commandHead];
        if (command === undefined) {
            this._compactCommands(true);
            return;
        }
        this._commands[this.commandHead] = undefined;
        this.commandHead += 1;
        this._compactCommands();
        this._processCommand(world, command, { recordHistory: true, emitEvents: true });
    }
}

// --- System: Remote Input ---
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
         }
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
                 } catch (err) {
                     // ignore capture errors
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
                 const pos = this.world.getComponent(spoolId, PositionComponent).pos;
                 const radius = this.world.getComponent(spoolId, RadiusComponent).radius;
                 const dx = x - pos.x;
                 const dy = y - pos.y;
                 if (dx * dx + dy * dy <= radius * radius) {
                     isGrabClick = true;
                     break;
                 }
             }

             if (isGrabClick) {
                 const pauseState = this.world.getResource('pauseState');
                 if (pauseState && pauseState.paused) {
                     pauseState.paused = false;
                     const pauseBtn = document.getElementById("pauseBtn");
                     if (pauseBtn) pauseBtn.textContent = "Pause";
                     this.ws.send(JSON.stringify({ action: 'pause', paused: false }));
                 }
             }

            this.ws.send(JSON.stringify({ action: 'input', type: 'pointerdown', x, y }));
        }
        if (typeof this.canvas.setPointerCapture === 'function') {
            try {
                this.canvas.setPointerCapture(event.pointerId);
            } catch (err) {
                // ignore capture errors
            }
        }
    }

     handlePointerMove(event) {
         if (this.interactionMode === 'pan') {
             if (!this.isPanning || this.panPointerId !== event.pointerId) {
                 return;
             }
             event.preventDefault();
             const baseScale = this.canvas.height / this.world.getResource('simHeight');
             const scale = baseScale * this.scaleMultiplier;
             if (scale <= 0) {
                 return;
             }
             const deltaX = event.clientX - this.panLastX;
             const deltaY = event.clientY - this.panLastY;
             this.panLastX = event.clientX;
             this.panLastY = event.clientY;
             this.viewOffsetX -= deltaX / scale;
             this.viewOffsetY += deltaY / scale;
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
         if (this.interactionMode === 'pan') {
             if (this.isPanning && this.panPointerId === event.pointerId) {
                 this.isPanning = false;
                 this.panPointerId = null;
                 if (typeof this.canvas.releasePointerCapture === 'function') {
                     try {
                         this.canvas.releasePointerCapture(event.pointerId);
                     } catch (err) {
                         // ignore release errors
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
             } catch (err) {
                 // ignore release errors
             }
         }
     }

     update(world, dt) {
        // This system doesn't run per-step logic, only handles events.
     }
}

// --- System: Input --- (Simplified Click Handling)
export class InputSystem {
     runInPause = true; // Input should work even when paused to unpause/interact

     constructor(canvas, world, pauseBtn) {
         this.canvas = canvas;
         this.world = world;
         this.pauseBtn = pauseBtn;
         this.clicks = [];
         this.releases = [];
         this.eventLog = [];
         this.frame = 0;
         this.grabSpring = null; // { ptrE, jointE, pathE, ballE }
         this.scaleMultiplier = 1.0;
         this.viewOffsetX = 0.0;
         this.viewOffsetY = 0.0;
         this.interactionMode = 'select';
         this.isPanning = false;
         this.panPointerId = null;
         this.panLastX = 0;
         this.panLastY = 0;
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
         } else if (this.globalTouchOverrides) {
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

     cancelPan() {
         if (!this.isPanning) {
             return;
         }
         if (typeof this.canvas.releasePointerCapture === 'function' && this.panPointerId !== null) {
             try {
                 this.canvas.releasePointerCapture(this.panPointerId);
             } catch (_err) {
                 // Ignore release errors.
             }
        }
        this.isPanning = false;
        this.panPointerId = null;
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
         if (event.pointerType === 'touch') {
             this.activePointers.delete(event.pointerId);
             if (this.activePointers.size < 2) {
                 this.endPinch();
             }
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
         const centerX = (first.x + second.x) * 0.5;
         const centerY = (first.y + second.y) * 0.5;
         return { distance, centerX, centerY };
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
         const simX = (pixelX - this.canvas.width / 2) / prevScale + this.viewOffsetX;
         const simY = (this.canvas.height / 2 - pixelY) / prevScale + this.viewOffsetY;
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
             } catch (err) {
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
         const rect = this.canvas.getBoundingClientRect();
         const baseScale = this.canvas.height / this.world.getResource('simHeight');
         const scale = baseScale * this.scaleMultiplier;
         const pixelX = event.clientX - rect.left;
         const pixelY = event.clientY - rect.top;
         const simX = (pixelX - this.canvas.width / 2) / scale + this.viewOffsetX;
         const simY = (this.canvas.height / 2 - pixelY) / scale + this.viewOffsetY;
         const clickVec = new Vector2(simX, simY);

         // Make touch targeting a bit more forgiving
         const cmOnScreen = (event.pointerType === 'touch' || event.pointerType === 'pen') ? 1.5 : 1.0;
         const dpi = 96;
         const pixelsPerCm = dpi / 2.54;
         const extraPixels = cmOnScreen * pixelsPerCm;
         const effectiveScale = (this.canvas.height / this.world.getResource('simHeight')) * this.scaleMultiplier;
         const extraClickableRadius = extraPixels / effectiveScale;

         let closestBall = null;
         let closestDistSq = Infinity;
         for (const b of this.world.query([SpoolTagComponent, PositionComponent, RadiusComponent])) {
           const pos = this.world.getComponent(b, PositionComponent).pos;
           const r = this.world.getComponent(b, RadiusComponent).radius + extraClickableRadius;
           const distSq = clickVec.clone().subtract(pos).lengthSq();
           if (distSq <= r * r && distSq < closestDistSq) {
             closestBall = b;
             closestDistSq = distSq;
           }
         }

         if (closestBall !== null) {
           if ((event.pointerType === 'touch' || event.pointerType === 'pen') && this.interactionMode !== 'pan') {
             if (this.touchActionBeforeGrab === null) {
               this.touchActionBeforeGrab = this.canvas.style.touchAction;
             }
             this.canvas.style.touchAction = 'none';
           }
           const ptrE = this.world.createEntity();
           this.world.addComponent(ptrE, new PositionComponent(simX, simY));
           this.world.addComponent(ptrE, new CableLinkComponent(simX, simY));

           const ballPos = this.world.getComponent(closestBall, PositionComponent).pos.clone();
           const ptrPos  = new Vector2(simX, simY);
           const jointE = this.world.createEntity();
           this.world.addComponent(jointE,
             new CableJointComponent(
               closestBall, ptrE, 0.1,
               ballPos.clone().subtract(ballPos), // local attachment on ball is zero
               ptrPos.clone().subtract(ptrPos) // local attachment on pointer is zero
             )
           );
           this.world.addComponent(jointE, new RenderableComponent('line', '#888888'));

           const pathE = this.world.createEntity();
           const pathComp = new CablePathComponent(
             this.world, [ jointE ], ['attachment', 'attachment'], [ true, true ], 10.0
           );
           this.world.addComponent(pathE, pathComp);

           this.grabSpring = { ptrE, jointE, pathE, ballE: closestBall };
           this.world.setResource('grabbedBall', closestBall);

           const pauseState = this.world.getResource('pauseState');
           if (pauseState) {
             pauseState.paused = false;
           }
           if (this.pauseBtn) {
             this.pauseBtn.textContent = "Pause";
           }
           if (typeof this.canvas.setPointerCapture === 'function') {
             try {
               this.canvas.setPointerCapture(event.pointerId);
             } catch (err) {
               // Ignore browsers that disallow capture here.
             }
           }
           return;
         }
     }

     handlePointerUp(event) {
         const wasPanPointer = this.isPanning && this.panPointerId === event.pointerId;
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
            velComp.vel.set(posComp.pos.clone().subtract(prevFinalPosComp.pos).scale(1.0/dt));
          } else if (velComp) {
            velComp.vel.set(new Vector2(0,0));
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

     update(world, dt) {
        // This system doesn't run per-step logic, only handles events.
     }

     handlePointerMove(event) {
         if (event.pointerType === 'touch') {
             this.trackPointerMove(event);
         }
         const isPanPointer = this.isPanning && this.panPointerId === event.pointerId;
         if (this.pinchActive) {
             event.preventDefault();
             this.updatePinchGesture();
             return;
         }
         if (isPanPointer) {
             event.preventDefault();
             const baseScale = this.canvas.height / this.world.getResource('simHeight');
             const scale = baseScale * this.scaleMultiplier;
             if (scale <= 0) {
                 return;
             }
             const deltaX = event.clientX - this.panLastX;
             const deltaY = event.clientY - this.panLastY;
             this.panLastX = event.clientX;
             this.panLastY = event.clientY;
             this.viewOffsetX -= deltaX / scale;
             this.viewOffsetY += deltaY / scale;
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
         if (this.interactionMode === 'pan') {
             return;
         }
         event.preventDefault();
         if (this.grabSpring === null) {
             return;
         }
         const rect = this.canvas.getBoundingClientRect();
         const baseScale = this.canvas.height / this.world.getResource('simHeight');
         const scale = baseScale * this.scaleMultiplier;
         const pixelX = event.clientX - rect.left;
         const pixelY = event.clientY - rect.top;
         const simX = (pixelX - this.canvas.width / 2) / scale + this.viewOffsetX;
         const simY = (this.canvas.height / 2 - pixelY) / scale + this.viewOffsetY;
         const { ptrE } = this.grabSpring;
         const pos = this.world.getComponent(ptrE, PositionComponent).pos;
         pos.set(new Vector2(simX, simY));
     }
}
