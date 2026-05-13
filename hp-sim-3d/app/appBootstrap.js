import { World } from '../../src/js/cable_joints_3d/ecs.js';
import { PerformanceMonitor } from './performance-monitor.js';
import { createAppState } from './appState.js';
import { HP4_USDA_KEY, createUsdaCatalog } from './machineCatalog.js';
import { buildUploadPresetConfig } from './uploadPresetConfig.js';
import { createSceneChangeQueue, createSimulationRuntime } from './sceneController.js';
import { createFeatureFlagsController } from './featureFlagsController.js';
import { createQualityController } from './qualityController.js';
import { createReferencePathController } from './referencePathController.js';
import { createViewController } from './viewController.js';
import { createWorkerController } from './workerController.js';
import { createMachineSceneController } from './machineSceneController.js';
import { createCommandJobController } from './commandJobController.js';
import { createExternalCommandController, normalizeWsUrl } from './externalCommandSocket.js';

function collectDomRefs(ownerDocument, canvasArg, controlsRootArg) {
  const canvas = canvasArg || ownerDocument.getElementById('myCanvas');
  const controlsRoot = controlsRootArg || ownerDocument.getElementById('controls');
  const simApp = canvas?.closest('.sim-app') || null;
  const simButtons = controlsRoot?.querySelector('.sim-buttons') || null;
  const machinesContainer = simButtons?.querySelector('.sim-machines') || null;
  const qualityToggleWrapper = ownerDocument.getElementById('qualityToggleWrapper');
  return {
    canvas,
    controlsRoot,
    simApp,
    simButtons,
    printLogoBtn: ownerDocument.getElementById('printLogoBtn'),
    printSquareBtn: ownerDocument.getElementById('printSquareBtn'),
    uploadBtn: ownerDocument.getElementById('uploadBtn'),
    gcodeInput: ownerDocument.getElementById('gcodeFile'),
    resetBtn: ownerDocument.getElementById('resetBtn'),
    pauseBtn: ownerDocument.getElementById('pauseBtn'),
    finishAsapBtn: ownerDocument.getElementById('finishAsapBtn'),
    positionTraceBtn: ownerDocument.getElementById('positionTraceBtn'),
    positionTraceClearBtn: ownerDocument.getElementById('positionTraceClearBtn'),
    measureBtn: ownerDocument.getElementById('measureBtn'),
    measureClearBtn: ownerDocument.getElementById('measureClearBtn'),
    zoomInBtn: ownerDocument.getElementById('zoomInBtn'),
    zoomOutBtn: ownerDocument.getElementById('zoomOutBtn'),
    panModeBtn: ownerDocument.getElementById('panModeBtn'),
    speedSlowerBtn: ownerDocument.getElementById('speedSlowerBtn'),
    speedFasterBtn: ownerDocument.getElementById('speedFasterBtn'),
    secondaryControls: ownerDocument.getElementById('simSecondaryControls'),
    fullscreenBtn: ownerDocument.getElementById('fullscreenBtn'),
    referenceToggleBtn: ownerDocument.getElementById('referenceToggleBtn'),
    speedStatusEl: ownerDocument.getElementById('speedStatus'),
    machinesContainer,
    machinesToggle: ownerDocument.getElementById('machinesToggle'),
    machinesMenu: ownerDocument.getElementById('machinesMenu'),
    presetMachinesList: ownerDocument.getElementById('presetMachinesList'),
    customMachinesSection: ownerDocument.getElementById('customMachinesSection'),
    customMachinesList: ownerDocument.getElementById('customMachinesList'),
    machinesRemoveAllBtn: ownerDocument.getElementById('machinesRemoveAllBtn'),
    printStatusEl: ownerDocument.getElementById('printStatus'),
    asapStatusEl: ownerDocument.getElementById('asapStatus'),
    qualityHudEl: ownerDocument.getElementById('qualityHud'),
    qualityHistoryHud: ownerDocument.getElementById('qualityHistoryHud'),
    qualityHistoryToggleBtn: ownerDocument.getElementById('qualityHistoryToggle'),
    qualityHistoryList: ownerDocument.getElementById('qualityHistoryList'),
    secondaryToggleBtn: ownerDocument.getElementById('secondaryToggleBtn'),
    qualityToggle: ownerDocument.getElementById('qualityToggle'),
    showForcesToggle: ownerDocument.getElementById('showForcesToggle'),
    lineLayeringToggle: ownerDocument.getElementById('lineLayeringToggle'),
    closedLoopMotorsToggle: ownerDocument.getElementById('closedLoopMotorsToggle'),
    qualityToggleLabel: qualityToggleWrapper?.querySelector('span') || null,
  };
}

function parseBooleanParam(params, key) {
  return ['1', 'true', 'yes'].includes((params?.get(key) || '').toLowerCase());
}

function createNoopLifecycle() {
  return {
    async loadDefaultScene() {},
    bindUi() {},
    start() {},
  };
}

export function createHpSimApp({
  document: ownerDocument = globalThis.document,
  window: ownerWindow = globalThis.window,
  canvas = null,
  controlsRoot = null,
} = {}) {
  if (!ownerDocument) {
    return createNoopLifecycle();
  }
  const dom = collectDomRefs(ownerDocument, canvas, controlsRoot);
  if (!dom.canvas || !dom.controlsRoot) {
    return createNoopLifecycle();
  }

  const urlParams = ownerWindow?.location ? new URLSearchParams(ownerWindow.location.search) : null;
  const state = createAppState({
    qualityEnabled: Boolean(dom.qualityToggle?.checked),
    showConstraintForces: Boolean(dom.showForcesToggle?.checked),
    lineLayeringEnabled: dom.lineLayeringToggle ? Boolean(dom.lineLayeringToggle.checked) : true,
    closedLoopMotorsEnabled: Boolean(dom.closedLoopMotorsToggle?.checked),
  });
  const world = new World();
  world.setResource('performanceMonitor', new PerformanceMonitor({
    enabled: parseBooleanParam(urlParams, 'perf'),
    logEverySteps: 3000,
    slowStepMs: 8.0,
  }));

  const catalog = createUsdaCatalog();
  const sceneQueue = createSceneChangeQueue();
  const runtime = createSimulationRuntime({ world, state });
  const controllers = {
    machines: null,
    commands: null,
    external: null,
  };

  const quality = createQualityController({
    document: ownerDocument,
    world,
    state,
    hudElement: dom.qualityHudEl,
    historyHud: dom.qualityHistoryHud,
    historyToggle: dom.qualityHistoryToggleBtn,
    historyList: dom.qualityHistoryList,
    qualityToggle: dom.qualityToggle,
    getMachines: () => controllers.machines.getMachines(),
    getMachineDisplayName: (machineOrId) => controllers.machines.getMachineDisplayName(machineOrId),
    getReferenceOverlayState: () => referencePaths.getState(),
  });
  const referencePaths = createReferencePathController({
    world,
    state,
    dom,
    quality,
    isMobileLayout: () => view.isMobileLayout(),
  });
  const featureFlags = createFeatureFlagsController({
    world,
    state,
    toggles: {
      showForcesToggle: dom.showForcesToggle,
      lineLayeringToggle: dom.lineLayeringToggle,
      closedLoopMotorsToggle: dom.closedLoopMotorsToggle,
    },
    onLineLayeringChanged: () => controllers.commands?.handleUserReset?.(),
  });
  const view = createViewController({
    document: ownerDocument,
    window: ownerWindow,
    world,
    state,
    dom,
    runtime,
    machines: {
      getMachines: () => controllers.machines.getMachines(),
      syncMachineMenuPlacement: () => controllers.machines.syncMachineMenuPlacement(),
    },
    referencePaths,
    getCommands: () => controllers.commands,
  });
  const workers = createWorkerController({
    state,
    getRemoteSystem: () => controllers.commands?.getRemoteSystem?.() || null,
    onWorkerDone: () => controllers.commands?.finishJob?.(),
    onWorkerError: () => controllers.commands?.handleWorkerError?.(),
    klipperPacerDiagnosticsEnabled: parseBooleanParam(urlParams, 'klipper_pacer_debug'),
  });
  controllers.machines = createMachineSceneController({
    document: ownerDocument,
    window: ownerWindow,
    world,
    state,
    dom,
    catalog,
    defaultSourceKey: HP4_USDA_KEY,
    sceneQueue,
    featureFlags,
    quality,
    runtime,
    view,
    getCommands: () => controllers.commands,
    getExternal: () => controllers.external,
  });
  controllers.commands = createCommandJobController({
    window: ownerWindow,
    world,
    state,
    dom,
    machines: controllers.machines,
    referencePaths,
    workers,
    quality,
    runtime,
    view,
    uploadPresetConfig: buildUploadPresetConfig(dom.gcodeInput),
  });
  controllers.external = createExternalCommandController({
    world,
    url: normalizeWsUrl(urlParams?.get('gcode_ws') || urlParams?.get('rrf_ws') || null),
    commands: controllers.commands,
    runtime,
    referencePaths,
  });

  let defaultScenePromise = null;
  let bound = false;
  let started = false;

  return {
    loadDefaultScene() {
      defaultScenePromise ||= controllers.machines.loadDefaultMachine();
      return defaultScenePromise;
    },
    bindUi() {
      if (bound) {
        return;
      }
      bound = true;
      quality.bindUi();
      featureFlags.bindUi();
      referencePaths.bindUi();
      view.bindUi();
      controllers.machines.bindUi();
      controllers.commands.bindUi();
    },
    async start() {
      if (started) {
        return;
      }
      started = true;
      defaultScenePromise ||= controllers.machines.loadDefaultMachine();
      await defaultScenePromise;
      runtime.start(() => controllers.machines.rebuildScene(), {
        onTimeScaleChange: (scale) => controllers.commands.handleRuntimeTimeScaleChange(scale),
      });
      runtime.reset({ autoPause: true });
      view.resetViewStateDefaults();
      view.setNavigationCursorActive(false);
      view.resetCanvasTapTracking();
      view.reapplyViewState({ clearExtrusions: true });
      view.setPanMode(false);
      view.syncCanvasDimensions();
      view.setSceneControlsEnabled(controllers.machines.getMachines().length > 0);
      controllers.external.connect();
    },
  };
}
