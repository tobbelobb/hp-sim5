import {
  RenderableComponent,
  MachineTagComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import { ExtruderComponent } from './hangprinter_extruder.js';
import { SpoolTagComponent, SpoolStateComponent } from './hangprinter_spools.js';
import { StepperMotorComponent } from './hangprinter_stepper_motor.js';
import {
  setStepperTorqueMode,
  setStepperPositionMode,
} from './hangprinter_runtime.js';

const SIMULATION_PLAYBACK_RESOURCE = 'simulationPlayback';

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
    this.baseHighWaterMark = 320;
    this.baseLowWaterMark = 160;
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

    if (commandType === 'SetTorqueMode' || commandType === 'SetPositionMode') {
      const axis = command.axis;
      const mapping = this.axisToEntity[axis];
      const entityIds = Array.isArray(mapping) ? mapping : (mapping !== undefined ? [mapping] : []);
      for (const entityId of entityIds) {
        if (commandType === 'SetTorqueMode') {
          setStepperTorqueMode(world, entityId, command.torqueNm || 0);
        } else {
          setStepperPositionMode(world, entityId);
        }
      }
      return;
    }

    for (const axis of Object.keys(this.axisToEntity)) {
      const axisValue = command[axis] ?? (command.axes ? command.axes[axis] : undefined);
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
            if (stepperComp.torqueMode) {
              continue;
            }
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

        const machineTips = extruderComp.machineTips || {};
        const machineCenters = extruderComp.machineCenters || {};
        const tipKeys = Object.keys(machineTips);
        const centerKeys = Object.keys(machineCenters);

        if (machineIds.length === 0 && tipKeys.length === 1) {
          machineIds = [tipKeys[0]];
        } else if (machineIds.length === 0 && centerKeys.length === 1) {
          machineIds = [centerKeys[0]];
        }

        for (const machineId of machineIds) {
          if (!machineId) {
            continue;
          }
          let tip = machineTips[machineId];
          if (!tip) {
            tip = machineCenters[machineId];
          }
          if (!tip) {
            if (tipKeys.length === 0 && centerKeys.length === 0 && extruderComp.tipPos) {
              tip = extruderComp.tipPos;
            } else if (centerKeys.length === 0 && extruderComp.centerPos) {
              tip = extruderComp.centerPos;
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
          if (!tip) {
            continue;
          }
          const extrusionEvent = {
            pos: [tip.x, tip.y, Number.isFinite(tip.z) ? tip.z : 0],
            length: command.E,
            machineId,
            color,
          };
          extruderComp.extrusions.push(extrusionEvent);
          if (emitEvents && this.onExtrusion) {
            try {
              this.onExtrusion(extrusionEvent);
            } catch (err) {
              console.warn('RemoteSpoolSystem: extrusion listener threw', err);
            }
          }
        }
      }
    }
  }

  update(world, dt) {
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
