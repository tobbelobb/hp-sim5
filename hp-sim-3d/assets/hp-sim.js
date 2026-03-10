import { Open as UsdOpen, getAttribute } from '../../src/js/usd/stage.js';
import { World, OrientationComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { runGame } from '../../examples/js/slideprinter_3d/runner.js';
import { setupScene } from '../../examples/js/slideprinter_3d/setupScene.js';
import { RemoteSpoolSystem, InputSystem, ExtruderComponent } from '../../examples/js/slideprinter_3d/slideprinter_common.js';
import { detectFileFormat, FileFormat, isMcuFormat, isRrfFormat } from '../../examples/js/slideprinter/fileFormatUtils.js';
import { orientationToDegrees } from './encoder_angles.js';
import { _updateAttachmentPoints } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { QualityMonitor } from './quality-monitor.js';
import { cloneExtrusionList, restoreReplayExtrusions } from './replay_state.js';
import { setLineLayeringFeatureFlags } from './line-layering-flags.js';

const COMMAND_PRESET_VARIANTS = Object.freeze({
  hangprinterLogo: Object.freeze({
    default: Object.freeze({
      url: new URL('../../public/examples/mcu_commands/Hangprinter_logo6.serial', import.meta.url).href,
      format: FileFormat.MCU_SERIAL,
      referencePresetKey: 'hangprinterLogo',
    }),
    lineLayered: Object.freeze({
      url: new URL('../../public/examples/RRF_CAN_commands/Hangprinter_logo6_w_line_layers.can', import.meta.url).href,
      format: FileFormat.RRF_CAN_BINARY,
      referencePresetKey: 'hangprinterLogo',
    }),
  }),
  straightMoves: Object.freeze({
    default: Object.freeze({
      url: new URL('../../public/examples/mcu_commands/draw_squares.serial', import.meta.url).href,
      format: FileFormat.MCU_SERIAL,
      referencePresetKey: 'straightMoves',
    }),
    lineLayered: Object.freeze({
      url: new URL('../../public/examples/RRF_CAN_commands/draw_squares_bigger_w_line_layers.can', import.meta.url).href,
      format: FileFormat.RRF_CAN_BINARY,
      referencePresetKey: 'straightMovesBigger',
    }),
  }),
});

function resolvePresetCommand(presetKey, lineLayeringEnabled) {
  const variants = COMMAND_PRESET_VARIANTS[presetKey];
  if (variants === null || variants === undefined) {
    return null;
  }
  if (lineLayeringEnabled === true && variants.lineLayered?.url) {
    return variants.lineLayered;
  }
  if (variants.default?.url) {
    return variants.default;
  }
  if (variants.lineLayered?.url) {
    return variants.lineLayered;
  }
  return null;
}

const PRESET_GCODE_MAP = Object.freeze({
  hangprinterLogo: {
    url: new URL('../../public/examples/gcode/Hangprinter_logo6.gcode', import.meta.url).href,
    label: 'Hangprinter Logo (G-code)',
    color: '#ff7a18',
  },
  straightMoves: {
    url: new URL('../../public/examples/gcode/draw_squares.gcode', import.meta.url).href,
    label: 'Draw Squares (G-code)',
    color: '#00b2ff',
  },
  straightMovesBigger: {
    url: new URL('../../public/examples/gcode/draw_squares_bigger.gcode', import.meta.url).href,
    label: 'Draw Bigger Squares (G-code)',
    color: '#00b2ff',
  },
});

const DEFAULT_UPLOAD_PRESET_MATCHES = Object.freeze([
  { substring: 'Hangprinter_logo6', presetKey: 'hangprinterLogo' },
  { substring: 'draw_squares_bigger', presetKey: 'straightMovesBigger' },
  { substring: 'draw_squares', presetKey: 'straightMoves' },
]);
const DEFAULT_UPLOAD_PRESET_EXTENSIONS = Object.freeze(['.txt', '.serial', '.csv', '.can']);

function parseUploadPresetMappings(value) {
  if (typeof value !== 'string' || value.trim().length === 0) {
    return DEFAULT_UPLOAD_PRESET_MATCHES;
  }
  const entries = [];
  const normalized = value
    .split(',')
    .map((segment) => segment.trim())
    .filter((segment) => segment.length > 0);
  for (const segment of normalized) {
    const [name, key] = segment.split('=').map((entry) => entry.trim());
    if (name && key) {
      entries.push({ substring: name, presetKey: key });
    }
  }
  return entries.length > 0 ? entries : DEFAULT_UPLOAD_PRESET_MATCHES;
}

function parseUploadPresetExtensions(value) {
  if (typeof value !== 'string' || value.trim().length === 0) {
    return DEFAULT_UPLOAD_PRESET_EXTENSIONS;
  }
  const entries = value
    .split(',')
    .map((segment) => segment.trim().toLowerCase())
    .filter((segment) => segment.length > 0)
    .map((segment) => (segment.startsWith('.') ? segment : `.${segment}`));
  return entries.length > 0 ? entries : DEFAULT_UPLOAD_PRESET_EXTENSIONS;
}

function buildUploadPresetConfig(inputElement) {
  const dataset = inputElement?.dataset;
  const presets = parseUploadPresetMappings(dataset?.referencePresets);
  const extensions = parseUploadPresetExtensions(dataset?.referenceExtensions);
  return {
    presets,
    extensionSet: new Set(extensions),
  };
}

const GCODE_MM_TO_SIM_SCALE = 0.001;
const GCODE_EXTRUSION_EPSILON = 1e-6;
const GCODE_MOVE_EPSILON = 1e-9;
const GCODE_INLINE_COMMENT_RE = /\(.*?\)/g;
const referencePathCache = new Map();

const DEFAULT_PRESET_KEY = 'hangprinterLogo';
const DEFAULT_VIEW_SCALE = 0.6;
const MIN_VIEW_SCALE = 0.01;
const MAX_VIEW_SCALE = 200;
const ZOOM_FACTOR = 1.2;
const ZOOM_EPSILON = 1e-3;
const QUALITY_HISTORY_MAX_ENTRIES = 20;

const AVAILABLE_USDAS = Object.freeze([
  { file: 'slideprinter_multi_unit.usda', label: 'Slideprinter Multi Unit (default)' },
  { file: 'slideprinter.usda', label: 'Slideprinter Original' },
  { file: 'slideprinter_hexagon.usda', label: 'Slideprinter (hexagon)' },
  { file: 'slideprinter_hexagon_pure_distancejoints.usda', label: 'Slideprinter (hexagon, pure distance joints)' },
  { file: 'slideprinter_pure_distancejoints.usda', label: 'Slideprinter (distance joints)' },
  { file: 'slideprinter_single_pinholes.usda', label: 'Slideprinter (single pinholes)' },
]);

function initHpSim() {
  const canvas = document.getElementById('myCanvas');
  const controlsRoot = document.getElementById('controls');
  if (!canvas || !controlsRoot) {
    return;
  }

  const printLogoBtn = document.getElementById('printLogoBtn');
  const printSquareBtn = document.getElementById('printSquareBtn');
  const uploadBtn = document.getElementById('uploadBtn');
  const gcodeInput = document.getElementById('gcodeFile');
  const uploadPresetConfig = buildUploadPresetConfig(gcodeInput);
  const resetBtn = document.getElementById('resetBtn');
  const pauseBtn = document.getElementById('pauseBtn');
  const finishAsapBtn = document.getElementById('finishAsapBtn');
  const positionTraceBtn = document.getElementById('positionTraceBtn');
  const zoomInBtn = document.getElementById('zoomInBtn');
  const zoomOutBtn = document.getElementById('zoomOutBtn');
  const panModeBtn = document.getElementById('panModeBtn');
  const speedSlowerBtn = document.getElementById('speedSlowerBtn');
  const speedFasterBtn = document.getElementById('speedFasterBtn');
  const secondaryControls = document.getElementById('simSecondaryControls');
  const fullscreenBtn = document.getElementById('fullscreenBtn');
  const referenceToggleBtn = document.getElementById('referenceToggleBtn');
  const speedStatusEl = document.getElementById('speedStatus');
  const simApp = canvas.closest('.sim-app');
  const initialTouchAction = canvas ? canvas.style.touchAction || '' : '';
  const simButtons = controlsRoot.querySelector('.sim-buttons');
  const startButtons = simButtons ? Array.from(simButtons.querySelectorAll('.sim-start')) : [];
  const machinesContainer = simButtons ? simButtons.querySelector('.sim-machines') : null;
  const machinesToggle = document.getElementById('machinesToggle');
  const machinesMenu = document.getElementById('machinesMenu');
  const presetMachinesList = document.getElementById('presetMachinesList');
  const customMachinesSection = document.getElementById('customMachinesSection');
  const customMachinesList = document.getElementById('customMachinesList');
  const machinesRemoveAllBtn = document.getElementById('machinesRemoveAllBtn');
  const printStatusEl = document.getElementById('printStatus');
  const replayStatusEl = document.getElementById('replayStatus');
  const asapStatusEl = document.getElementById('asapStatus');
  const qualityHudEl = document.getElementById('qualityHud');
  const qualityHistoryHud = document.getElementById('qualityHistoryHud');
  const qualityHistoryToggleBtn = document.getElementById('qualityHistoryToggle');
  const qualityHistoryList = document.getElementById('qualityHistoryList');
  const secondaryToggleBtn = document.getElementById('secondaryToggleBtn');
  const qualityToggle = document.getElementById('qualityToggle');
  const lineLayeringToggle = document.getElementById('lineLayeringToggle');
  const qualityToggleWrapper = document.getElementById('qualityToggleWrapper');
  const qualityToggleLabel = qualityToggleWrapper ? qualityToggleWrapper.querySelector('span') : null;
  const supportsMatchMedia = typeof window.matchMedia === 'function';
  const mobileLayoutQuery = supportsMatchMedia ? window.matchMedia('(max-width: 600px)') : null;
  const isMobileLayout = () => (mobileLayoutQuery ? mobileLayoutQuery.matches : window.innerWidth <= 600);
  const QUALITY_LABELS = {
    desktop: 'Live Quality Checks',
    mobile: 'Live QC',
  };
  const REFERENCE_LABELS = {
    show: {
      desktop: 'Show Reference Path',
      mobile: 'Show Reference',
    },
    hide: {
      desktop: 'Hide Reference Path',
      mobile: 'Hide Reference',
    },
  };
  const MOBILE_SECONDARY_CONTROLS_TIMEOUT_MS = 3000;
  const MOBILE_SECONDARY_CONTROLS_INTERACTION_DELAY_MS = 150;
  const MOBILE_MACHINES_MENU_MARGIN_PX = 12;
  const MACHINE_MENU_HOVER_CLOSE_DELAY_MS = 3000;
  let secondaryControlsHideTimeout = null;
  let secondaryControlsInteractionEnableTimeout = null;
  let lastMobileLayoutMatches = isMobileLayout();
  let positionTraceRightClickCount = 0;
  let positionTraceFirstRightClickMs = 0;
  let positionTraceDoubleClickTimer = null;

  if (qualityToggle) {
    qualityToggle.checked = false;
  }
  if (lineLayeringToggle) {
    lineLayeringToggle.checked = true;
  }

  const usdaCatalog = new Map(
    AVAILABLE_USDAS.map((entry) => [
      entry.file,
      {
        ...entry,
        url: new URL(`../../examples/usd_scenes/${entry.file}`, import.meta.url).href,
        tintColor: null,
        tintColorLoaded: false,
        tintColorPromise: null,
      },
    ])
  );
  const defaultUsdaKey = 'slideprinter_multi_unit.usda';
  const presetOptionInputs = new Map();
  const presetOptionLabels = new Map();
  const presetOptionColorChips = new Map();
  let machineMenuOpen = false;
  let machineMenuHoverCloseTimeout = null;
  let machineMenuHoverTrackingEnabled = false;
  let machineMenuPointerInside = false;
  let machineMenuFocusInside = false;

  const world = new World();
  const machines = [];
  let machineIdCounter = 0;
  let klipperCommanderWorker = null;
  let rrfCommanderWorker = null;
  let moveCommanderWorker = null;
  let simDtSec = null;
  let stageReady = false;
  let currentPresetKey = DEFAULT_PRESET_KEY;
  let gameControls = null;
  let currentViewScale = DEFAULT_VIEW_SCALE;
  let currentViewOffsetX = 0;
  let currentViewOffsetY = 0;
  let panModeActive = false;
  let viewListenerSystem = null;
  let printActive = false;
  let secondaryControlsEverShown = false;
  let fullscreenActive = false;
  let currentTimeScale = 1.0;
  let speedStatusArmed = false;
  const machineQualityMonitors = new Map();
  let qualityEnabled = qualityToggle ? Boolean(qualityToggle.checked) : false;
  let lineLayeringEnabled = lineLayeringToggle ? Boolean(lineLayeringToggle.checked) : true;
  let secondaryControlsUserPreference = null;
  let secondaryControlsAutoActive = false;
  let jobSequenceCounter = 0;
  let activeJobId = null;
  let lastRecordedJobId = null;
  let currentJobDescriptor = null;
  const qualityHistoryRecords = [];
  let qualityHistoryExpanded = false;
  const urlParams =
    typeof window !== 'undefined' && window.location
      ? new URLSearchParams(window.location.search)
      : null;
  const externalWsParam = urlParams?.get('gcode_ws') || urlParams?.get('rrf_ws') || null;
  const externalWsUrl = normalizeWsUrl(externalWsParam);
  const externalCommandQueue = [];
  const EXTERNAL_QUEUE_LIMIT = 5000;
  let externalCommandSocket = null;
  let externalCommandSocketConnecting = false;
  let externalWsReconnectTimer = null;
  const EXTERNAL_WS_RECONNECT_INITIAL_DELAY_MS = 1000;
  const EXTERNAL_WS_RECONNECT_MAX_DELAY_MS = 5000;
  let externalWsReconnectDelayMs = EXTERNAL_WS_RECONNECT_INITIAL_DELAY_MS;

  function forEachQualityMonitor(callback) {
    if (typeof callback !== 'function') {
      return;
    }
    for (const entry of machineQualityMonitors.values()) {
      if (!entry?.monitor) {
        continue;
      }
      callback(entry.monitor, entry);
    }
  }

  function updateQualityHudVisibility() {
    if (!qualityHudEl) {
      return;
    }
    const hasVisibleMonitor = Array.from(machineQualityMonitors.values()).some((entry) => {
      const hudEl = entry?.monitor?.hudElement;
      return hudEl instanceof HTMLElement && !hudEl.classList.contains('sim-hidden');
    });
    if (hasVisibleMonitor) {
      qualityHudEl.classList.remove('sim-hidden');
    } else {
      qualityHudEl.classList.add('sim-hidden');
    }
  }

  function getJobLabel(descriptor) {
    if (!descriptor) {
      return 'Unknown Job';
    }
    if (typeof descriptor.label === 'string' && descriptor.label.trim().length > 0) {
      return descriptor.label;
    }
    if (typeof descriptor.name === 'string' && descriptor.name.trim().length > 0) {
      return descriptor.name;
    }
    if (typeof descriptor.key === 'string' && descriptor.key.trim().length > 0) {
      return descriptor.key;
    }
    return 'Program';
  }

  function beginNewJob(descriptor = null) {
    jobSequenceCounter += 1;
    activeJobId = jobSequenceCounter;
    currentJobDescriptor = descriptor ? { ...descriptor } : null;
  }

  function resetJobTracking() {
    activeJobId = null;
    currentJobDescriptor = null;
  }

  function updateQualityHistoryUI() {
    if (!qualityHistoryHud || !qualityHistoryToggleBtn || !qualityHistoryList) {
      return;
    }
    if (qualityHistoryRecords.length === 0) {
      qualityHistoryHud.classList.add('sim-hidden');
      qualityHistoryToggleBtn.setAttribute('aria-expanded', 'false');
      qualityHistoryToggleBtn.textContent = 'Quality History ▼';
      qualityHistoryList.classList.add('sim-hidden');
      qualityHistoryList.innerHTML = '';
      qualityHistoryExpanded = false;
      return;
    }
    const arrow = qualityHistoryExpanded ? '▲' : '▼';
    qualityHistoryHud.classList.remove('sim-hidden');
    qualityHistoryToggleBtn.textContent = `Quality History (${qualityHistoryRecords.length}) ${arrow}`;
    qualityHistoryToggleBtn.setAttribute('aria-expanded', qualityHistoryExpanded ? 'true' : 'false');
    if (qualityHistoryExpanded) {
      const items = qualityHistoryRecords.map((record) => {
        const combinedLabel = `${record.machineLabel}, ${record.jobLabel}`;
        const safeLabel = escapeHtml(combinedLabel);
        const scoreText = Number.isFinite(record.score) ? Math.round(record.score).toString() : '--';
        return `<div class="quality-history__item"><span class="quality-history__label">${safeLabel}</span><span class="quality-history__score">${scoreText}</span></div>`;
      });
      qualityHistoryList.innerHTML = items.join('');
      qualityHistoryList.classList.remove('sim-hidden');
    } else {
      qualityHistoryList.classList.add('sim-hidden');
    }
  }

  function recordQualityHistoryEntry() {
    if (activeJobId == null || lastRecordedJobId === activeJobId) {
      return;
    }
    const jobLabel = getJobLabel(currentJobDescriptor);
    const timestamp = Date.now();
    let added = false;
    for (const [machineId, entry] of machineQualityMonitors.entries()) {
      const monitor = entry?.monitor;
      if (!monitor) {
        continue;
      }
      const metrics = typeof monitor.getMetrics === 'function' ? monitor.getMetrics() : monitor.metrics;
      const score = metrics?.score;
      if (!Number.isFinite(score)) {
        continue;
      }
      const machine = machines.find((machineEntry) => machineEntry.id === machineId);
      const machineLabel = getMachineDisplayName(machine);
      qualityHistoryRecords.push({
        machineId,
        jobLabel,
        machineLabel,
        score,
        timestamp,
      });
      added = true;
    }
    if (added) {
      qualityHistoryRecords.sort((a, b) => {
        if (b.score !== a.score) {
          return b.score - a.score;
        }
        return b.timestamp - a.timestamp;
      });
      if (qualityHistoryRecords.length > QUALITY_HISTORY_MAX_ENTRIES) {
        qualityHistoryRecords.length = QUALITY_HISTORY_MAX_ENTRIES;
      }
      updateQualityHistoryUI();
    }
    lastRecordedJobId = activeJobId;
    resetJobTracking();
  }

  function getMachineDisplayName(machine) {
    if (!machine) {
      return 'Machine';
    }
    if (machine.name) {
      return machine.name;
    }
    if (machine.sourceKey && usdaCatalog.has(machine.sourceKey)) {
      const entry = usdaCatalog.get(machine.sourceKey);
      if (entry?.label) {
        return entry.label;
      }
    }
    return machine.id || 'Machine';
  }

  function ensureQualityMonitorForMachine(machine) {
    if (!machine || !qualityHudEl) {
      return null;
    }
    const existing = machineQualityMonitors.get(machine.id);
    if (existing?.monitor) {
      existing.monitor.setMachineContext({
        id: machine.id,
        label: getMachineDisplayName(machine),
        tintColor: machine.tintColor || null,
      });
      existing.monitor.setEnabled(qualityEnabled);
      return existing.monitor;
    }
    const card = document.createElement('div');
    card.className = 'quality-hud__card sim-hidden';
    card.dataset.machineId = machine.id;
    qualityHudEl.appendChild(card);
    const monitor = new QualityMonitor({ hudElement: card });
    monitor.setVisibilityCallback(updateQualityHudVisibility);
    monitor.setMachineContext({
      id: machine.id,
      label: getMachineDisplayName(machine),
      tintColor: machine.tintColor || null,
    });
    monitor.setEnabled(qualityEnabled);
    machineQualityMonitors.set(machine.id, { monitor });
    if (referenceOverlayState.segments) {
      monitor.setReferenceSegments(referenceOverlayState.segments, referenceOverlayState.metadata);
    }
    updateQualityHudVisibility();
    return monitor;
  }

  function removeQualityMonitor(machineId) {
    const entry = machineQualityMonitors.get(machineId);
    if (!entry) {
      return;
    }
    const monitor = entry.monitor;
    const hudElement = monitor?.hudElement || null;
    if (monitor) {
      monitor.setVisibilityCallback(null);
      monitor.detachRemoteSystem();
    }
    if (hudElement instanceof HTMLElement && hudElement.parentElement) {
      hudElement.parentElement.removeChild(hudElement);
    }
    if (monitor) {
      monitor.dispose();
    }
    machineQualityMonitors.delete(machineId);
    updateQualityHudVisibility();
  }

  function clearQualityMonitors() {
    for (const machineId of Array.from(machineQualityMonitors.keys())) {
      removeQualityMonitor(machineId);
    }
  }

  function refreshAllQualityMonitors(force = false) {
    forEachQualityMonitor((monitor) => monitor.refreshHud(force));
  }

  function setQualityEnabledState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    if (qualityEnabled === next) {
      if (!fromToggle && qualityToggle) {
        qualityToggle.checked = next;
      }
      updateQualityHudVisibility();
      return;
    }
    qualityEnabled = next;
    forEachQualityMonitor((monitor) => monitor.setEnabled(next));
    if (!fromToggle && qualityToggle) {
      qualityToggle.checked = next;
    }
    updateQualityHudVisibility();
  }

  function setLineLayeringEnabledState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    if (lineLayeringEnabled === next) {
      if (!fromToggle && lineLayeringToggle) {
        lineLayeringToggle.checked = next;
      }
      setLineLayeringFeatureFlags(world, next);
      _updateAttachmentPoints(world);
      return;
    }
    lineLayeringEnabled = next;
    setLineLayeringFeatureFlags(world, next);
    _updateAttachmentPoints(world);
    if (!fromToggle && lineLayeringToggle) {
      lineLayeringToggle.checked = next;
    }
    if (fromToggle) {
      handleUserReset();
    }
  }

  function attachQualityMonitorsToRemoteSystem() {
    if (machineQualityMonitors.size === 0) {
      return;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    forEachQualityMonitor((monitor) => {
      monitor.attachRemoteSystem(remoteSystem);
      monitor.refreshHud();
    });
    if (qualityToggle) {
      setQualityEnabledState(qualityToggle.checked, { fromToggle: true });
    }
  }

  function resetQualityMonitors(options = {}) {
    forEachQualityMonitor((monitor) => monitor.reset(options));
  }

  function runFinalQualityChecks() {
    forEachQualityMonitor((monitor) => monitor.runFinalCheck());
    recordQualityHistoryEntry();
  }

  if (qualityToggle) {
    if (!qualityHudEl) {
      qualityToggle.checked = false;
      qualityToggle.disabled = true;
      qualityToggle.setAttribute('aria-disabled', 'true');
      qualityEnabled = false;
    } else {
      qualityToggle.addEventListener('change', () => {
        setQualityEnabledState(qualityToggle.checked, { fromToggle: true });
      });
      qualityEnabled = Boolean(qualityToggle.checked);
    }
  } else {
    qualityEnabled = false;
  }

  if (lineLayeringToggle) {
    lineLayeringToggle.addEventListener('change', () => {
      setLineLayeringEnabledState(lineLayeringToggle.checked, { fromToggle: true });
    });
    setLineLayeringEnabledState(lineLayeringToggle.checked, { fromToggle: true });
  } else {
    lineLayeringEnabled = true;
    setLineLayeringFeatureFlags(world, true);
  }

  const referenceOverlayState = {
    segments: null,
    metadata: null,
    color: '#1e90ff',
    visible: false,
    dirty: false,
    key: null,
  };
  updateQualityToggleLabel();
  updateReferenceToggleUI();
  const sceneChangeState = {
    context: null,
    pausedForSceneChange: false,
    replayInProgress: false,
    targetHistoryLength: 0,
    wasPaused: null,
    frameSnapshot: null,
    frameSnapshotNeedsApply: false,
    renderSuspended: false,
  };

  let sceneChangeQueue = Promise.resolve();

  function enqueueSceneChange(task) {
    const run = sceneChangeQueue.then(() => task());
    sceneChangeQueue = run.catch((error) => {
      console.error('hp-sim: scene change task failed', error);
    });
    return run;
  }

  const asapState = {
    active: false,
    finishingPromise: null,
    previousTimeScale: null,
    previousQualityEnabled: null,
    previousQualityToggleDisabled: null,
    previousQualityToggleChecked: null,
    previousPauseState: null,
    prevPauseDisabled: null,
    prevResetDisabled: null,
    renderSuspended: false,
    pendingFinalCheck: false,
  };

  function cloneCommandList(list) {
    if (!Array.isArray(list) || list.length === 0) {
      return [];
    }
    return list.map((cmd) => ({ ...cmd }));
  }

  function getExtruderComponent() {
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length === 0) {
      return null;
    }
    return world.getComponent(extruderEntities[0], ExtruderComponent) || null;
  }

  function replayExtrusionsIntoQualityMonitors(extrusions) {
    const snapshot = Array.isArray(extrusions) ? extrusions : [];
    resetQualityMonitors({ keepReference: true });
    for (const extrusion of snapshot) {
      forEachQualityMonitor((monitor) => monitor.recordExtrusion(extrusion));
    }
    runFinalQualityChecks();
    refreshAllQualityMonitors(true);
  }

  function captureSceneFrameSnapshot() {
    if (!canvas) {
      return;
    }
    const width = canvas.width | 0;
    const height = canvas.height | 0;
    if (width <= 0 || height <= 0) {
      return;
    }
    let snapshot = sceneChangeState.frameSnapshot;
    if (!snapshot) {
      snapshot = document.createElement('canvas');
      sceneChangeState.frameSnapshot = snapshot;
    }
    if (snapshot.width !== width) {
      snapshot.width = width;
    }
    if (snapshot.height !== height) {
      snapshot.height = height;
    }
    const snapshotCtx = snapshot.getContext('2d');
    if (!snapshotCtx) {
      sceneChangeState.frameSnapshot = null;
      sceneChangeState.frameSnapshotNeedsApply = false;
      return;
    }
    snapshotCtx.clearRect(0, 0, snapshot.width, snapshot.height);
    try {
      snapshotCtx.drawImage(canvas, 0, 0, width, height);
      sceneChangeState.frameSnapshotNeedsApply = true;
    } catch (_err) {
      sceneChangeState.frameSnapshotNeedsApply = false;
    }
  }

  function applySceneFrameSnapshot() {
    if (!sceneChangeState.frameSnapshotNeedsApply) {
      return;
    }
    if (!canvas) {
      sceneChangeState.frameSnapshotNeedsApply = false;
      return;
    }
    const snapshot = sceneChangeState.frameSnapshot;
    if (!snapshot) {
      sceneChangeState.frameSnapshotNeedsApply = false;
      return;
    }
    sceneChangeState.frameSnapshotNeedsApply = false;
  }

  function clearSceneFrameSnapshot() {
    sceneChangeState.frameSnapshotNeedsApply = false;
    sceneChangeState.frameSnapshot = null;
  }

  function setButtonDisabled(button, disabled) {
    if (!button) {
      return;
    }
    const shouldDisable = Boolean(disabled);
    button.disabled = shouldDisable;
    if (shouldDisable) {
      button.setAttribute('aria-disabled', 'true');
    } else {
      button.removeAttribute('aria-disabled');
    }
  }

  function suspendRenderSystemForSceneChange() {
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem || typeof renderSystem.setDrawingSuspended !== 'function') {
      return;
    }
    renderSystem.setDrawingSuspended(true);
    sceneChangeState.renderSuspended = true;
  }

  function showPrintStatus(message) {
    if (!printStatusEl) {
      return;
    }
    printStatusEl.textContent = message;
    printStatusEl.classList.remove('sim-hidden');
  }

  function hidePrintStatus() {
    if (!printStatusEl) {
      return;
    }
    printStatusEl.textContent = '';
    printStatusEl.classList.add('sim-hidden');
  }

  function showReplayStatus(message = 'Replaying extrusions...') {
    if (!replayStatusEl) {
      return;
    }
    replayStatusEl.textContent = message;
    replayStatusEl.classList.remove('sim-hidden');
  }

  function hideReplayStatus() {
    if (!replayStatusEl) {
      return;
    }
    replayStatusEl.textContent = '';
    replayStatusEl.classList.add('sim-hidden');
  }

  function showAsapStatus(message = 'Finishing print ASAP...') {
    if (!asapStatusEl) {
      return;
    }
    asapStatusEl.textContent = message;
    asapStatusEl.classList.remove('sim-hidden');
  }

  function hideAsapStatus() {
    if (!asapStatusEl) {
      return;
    }
    asapStatusEl.textContent = '';
    asapStatusEl.classList.add('sim-hidden');
  }

  function getReferenceToggleText({ hasData, visible }) {
    const variant = isMobileLayout() ? 'mobile' : 'desktop';
    if (!hasData) {
      return REFERENCE_LABELS.show[variant];
    }
    return visible ? REFERENCE_LABELS.hide[variant] : REFERENCE_LABELS.show[variant];
  }

  function updateReferenceToggleUI() {
    if (!referenceToggleBtn) {
      return;
    }
    const hasData = Array.isArray(referenceOverlayState.segments) && referenceOverlayState.segments.length > 0;
    referenceToggleBtn.disabled = !hasData;
    referenceToggleBtn.setAttribute('aria-pressed', referenceOverlayState.visible ? 'true' : 'false');
    if (hasData && referenceOverlayState.metadata?.label) {
      referenceToggleBtn.setAttribute('title', `Reference: ${referenceOverlayState.metadata.label}`);
      referenceToggleBtn.setAttribute('aria-label', `Toggle reference path: ${referenceOverlayState.metadata.label}`);
    } else {
      referenceToggleBtn.removeAttribute('title');
      referenceToggleBtn.setAttribute('aria-label', 'Toggle reference path visibility');
    }
    referenceToggleBtn.textContent = getReferenceToggleText({
      hasData,
      visible: referenceOverlayState.visible,
    });
  }

  function updatePositionTraceToggleUI() {
    if (!positionTraceBtn) {
      return;
    }
    const renderSystem = world.getResource('renderSystem');
    const enabled = Boolean(renderSystem?.positionTraceEnabled);
    positionTraceBtn.setAttribute('aria-pressed', enabled ? 'true' : 'false');
    positionTraceBtn.classList.toggle('is-active', enabled);
  }

  function syncReferenceOverlayToRenderSystem({ force = false } = {}) {
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem || typeof renderSystem.setReferencePaths !== 'function') {
      return;
    }
    if (!force && !referenceOverlayState.dirty) {
      return;
    }
    renderSystem.setReferencePaths(referenceOverlayState.segments || [], {
      color: referenceOverlayState.color,
      metadata: referenceOverlayState.metadata,
      visible: referenceOverlayState.visible,
    });
    referenceOverlayState.dirty = false;
  }

  function updateQualityToggleLabel() {
    if (!qualityToggleLabel) {
      return;
    }
    const variant = isMobileLayout() ? 'mobile' : 'desktop';
    qualityToggleLabel.textContent = QUALITY_LABELS[variant];
  }

  function setReferenceSegments(segments, { metadata = null, color = null } = {}) {
    referenceOverlayState.segments = Array.isArray(segments) ? segments : null;
    referenceOverlayState.metadata = metadata;
    referenceOverlayState.key = metadata?.key || null;
    if (typeof color === 'string' && color.length > 0) {
      referenceOverlayState.color = color;
    } else if (!referenceOverlayState.color) {
      referenceOverlayState.color = '#1e90ff';
    }
    referenceOverlayState.dirty = true;
    if (!referenceOverlayState.segments) {
      referenceOverlayState.visible = false;
    }
    updateReferenceToggleUI();
    forEachQualityMonitor((monitor) => {
      monitor.setReferenceSegments(referenceOverlayState.segments, referenceOverlayState.metadata);
    });
    syncReferenceOverlayToRenderSystem({ force: true });
  }

  function setReferenceVisibility(visible) {
    const target = Boolean(visible);
    if (referenceOverlayState.visible === target) {
      return;
    }
    referenceOverlayState.visible = target;
    referenceOverlayState.dirty = true;
    updateReferenceToggleUI();
    syncReferenceOverlayToRenderSystem({ force: true });
  }

  const klipperCommanderModuleUrl = new URL('../../examples/js/slideprinter/klipperCommander.js', import.meta.url);
  const rrfCommanderModuleUrl = new URL('../../examples/js/slideprinter/rrfCommander.js', import.meta.url);
  const moveCommanderModuleUrl = new URL('../../examples/js/slideprinter/moveCommander.js', import.meta.url);
  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
  }

  function getInputSystem() {
    return world.systems.find((sys) => sys instanceof InputSystem) || null;
  }

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
  }

  function escapeHtml(value) {
    if (typeof value !== 'string') {
      return '';
    }
    return value
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  function hexToRgb(hex) {
    if (typeof hex !== 'string') {
      return null;
    }
    const cleaned = hex.trim().toLowerCase();
    const match = cleaned.match(/^#?([0-9a-f]{6})$/);
    if (!match) {
      return null;
    }
    const value = parseInt(match[1], 16);
    return {
      r: (value >> 16) & 0xff,
      g: (value >> 8) & 0xff,
      b: value & 0xff,
    };
  }

  function rgbToHex({ r, g, b }) {
    const clampChannel = (channel) => Math.min(255, Math.max(0, Math.round(channel)));
    const toHex = (channel) => clampChannel(channel).toString(16).padStart(2, '0');
    return `#${toHex(r)}${toHex(g)}${toHex(b)}`;
  }

  function mixColors(baseHex, tintHex, amount = 0.5) {
    const base = hexToRgb(baseHex);
    const tint = hexToRgb(tintHex);
    if (!base || !tint) {
      return baseHex;
    }
    const mixFactor = Math.min(Math.max(Number.isFinite(amount) ? amount : 0.5, 0), 1);
    return rgbToHex({
      r: base.r * (1 - mixFactor) + tint.r * mixFactor,
      g: base.g * (1 - mixFactor) + tint.g * mixFactor,
      b: base.b * (1 - mixFactor) + tint.b * mixFactor,
    });
  }

  function normalizeWsUrl(raw) {
    if (!raw || typeof raw !== 'string') {
      return null;
    }
    const trimmed = raw.trim();
    if (!trimmed) {
      return null;
    }
    if (trimmed.startsWith('ws://') || trimmed.startsWith('wss://')) {
      return trimmed;
    }
    const cleaned = trimmed.replace(/^\/+/, '');
    return `ws://${cleaned}`;
  }

  function maybeResumeFromPause() {
    const pauseState = world.getResource('pauseState');
    if (pauseState && pauseState.paused) {
      pauseState.paused = false;
    }
  }

  function pushExternalCommands(commands) {
    if (!Array.isArray(commands) || commands.length === 0) {
      return;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      const overflow = externalCommandQueue.length + commands.length - EXTERNAL_QUEUE_LIMIT;
      if (overflow > 0) {
        externalCommandQueue.splice(0, overflow);
      }
      externalCommandQueue.push(...commands);
      return;
    }
    for (const cmd of commands) {
      remoteSystem.addCommand(cmd);
    }
    setPrintActive(true);
    maybeResumeFromPause();
  }

  function flushExternalCommandQueue() {
    if (externalCommandQueue.length === 0) {
      return;
    }
    const batch = externalCommandQueue.splice(0, externalCommandQueue.length);
    pushExternalCommands(batch);
  }

  function clearExternalReconnectTimer() {
    if (externalWsReconnectTimer) {
      clearTimeout(externalWsReconnectTimer);
      externalWsReconnectTimer = null;
    }
  }

  function resetExternalReconnectBackoff() {
    externalWsReconnectDelayMs = EXTERNAL_WS_RECONNECT_INITIAL_DELAY_MS;
  }

  function scheduleExternalCommandReconnect(reason = '') {
    if (!externalWsUrl || typeof WebSocket === 'undefined') {
      return;
    }
    if (externalCommandSocket || externalCommandSocketConnecting || externalWsReconnectTimer) {
      return;
    }
    const delay = externalWsReconnectDelayMs;
    const messageSuffix = reason ? ` (${reason})` : '';
    console.log(`hp-sim: waiting for external G-code stream${messageSuffix}, retrying in ${Math.round(delay)}ms`);
    externalWsReconnectTimer = setTimeout(() => {
      externalWsReconnectTimer = null;
      connectExternalCommandStream();
    }, delay);
    externalWsReconnectDelayMs = Math.min(
      Math.max(externalWsReconnectDelayMs * 2, EXTERNAL_WS_RECONNECT_INITIAL_DELAY_MS),
      EXTERNAL_WS_RECONNECT_MAX_DELAY_MS
    );
  }

  function resolveEncoderAngles(axes = []) {
    if (!Array.isArray(axes) || axes.length === 0) {
      return [];
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return [];
    }
    if (typeof remoteSystem._ensureAxisMapping === 'function') {
      try {
        remoteSystem._ensureAxisMapping(world);
      } catch (_err) {
        // ignore mapping errors
      }
    }
    return axes.map((axis) => {
      const mapping = remoteSystem.axisToEntity ? remoteSystem.axisToEntity[axis] : null;
      const entityIds = Array.isArray(mapping) ? mapping : (mapping != null ? [mapping] : []);
      for (const entityId of entityIds) {
        const orient = world.getComponent(entityId, OrientationComponent);
        const angleDeg = orientationToDegrees(orient);
        if (Number.isFinite(angleDeg)) {
          return angleDeg;
        }
      }
      return null;
    });
  }

  function respondToEncoderRequest(requestId, axes) {
    if (!externalCommandSocket || typeof externalCommandSocket.send !== 'function') {
      return;
    }
    if (typeof WebSocket !== 'undefined' && externalCommandSocket.readyState !== WebSocket.OPEN) {
      return;
    }
    const anglesDeg = resolveEncoderAngles(axes);
    const payload = {
      type: 'encoder_response',
      requestId,
      axes,
      anglesDeg,
    };
    try {
      externalCommandSocket.send(JSON.stringify(payload));
    } catch (err) {
      console.warn('hp-sim: failed to send encoder response.', err);
    }
  }

  function handleExternalPayload(payload) {
    if (!payload) {
      return;
    }
    if (payload.type === 'encoder_request') {
      if (payload.requestId != null && Array.isArray(payload.axes)) {
        respondToEncoderRequest(payload.requestId, payload.axes);
      }
      return;
    }
    if (payload.type === 'reset') {
      handleUserReset();
      return;
    }
    if (payload.type === 'set_speed_scale' && Number.isFinite(payload.value) && payload.value > 0) {
      const targetScale = payload.value;
      speedStatusArmed = true;
      if (gameControls && typeof gameControls.setTimeScale === 'function') {
        gameControls.setTimeScale(targetScale);
        const appliedScale = typeof gameControls.getTimeScale === 'function'
          ? gameControls.getTimeScale()
          : targetScale;
        handleTimeScaleChange(appliedScale);
        showSpeedStatus(appliedScale);
      } else {
        handleTimeScaleChange(targetScale);
        showSpeedStatus(targetScale);
      }
      return;
    }
    if (payload.type === 'position_trace_mode') {
      const renderSystem = world.getResource('renderSystem');
      if (renderSystem && typeof renderSystem.setPositionTraceEnabled === 'function') {
        renderSystem.setPositionTraceEnabled(Boolean(payload.enabled));
        renderSystem.update?.(world, 0);
      }
      updatePositionTraceToggleUI();
      return;
    }
    const commands = [];
    if (payload.type === 'command' && payload.command) {
      commands.push(payload.command);
    }
    if (Array.isArray(payload.commands)) {
      for (const cmd of payload.commands) {
        if (cmd) {
          commands.push(cmd);
        }
      }
    }
    if (commands.length > 0) {
      pushExternalCommands(commands);
    }
    if (typeof payload.reply === 'string' && payload.reply.trim().length > 0) {
      console.info('hp-sim: gcode reply', payload.reply.trim());
    }
  }

  function connectExternalCommandStream() {
    if (!externalWsUrl || externalCommandSocket || externalCommandSocketConnecting || typeof WebSocket === 'undefined') {
      return;
    }
    clearExternalReconnectTimer();
    externalCommandSocketConnecting = true;
    try {
      externalCommandSocket = new WebSocket(externalWsUrl);
    } catch (err) {
      console.warn('hp-sim: failed to open external G-code stream.', err);
      externalCommandSocket = null;
      externalCommandSocketConnecting = false;
      scheduleExternalCommandReconnect('failed to open');
      return;
    }
    externalCommandSocket.addEventListener('open', () => {
      externalCommandSocketConnecting = false;
      resetExternalReconnectBackoff();
      console.log('hp-sim: external G-code stream connected:', externalWsUrl);
      flushExternalCommandQueue();
    });
    externalCommandSocket.addEventListener('message', (event) => {
      try {
        const payload = typeof event.data === 'string' ? JSON.parse(event.data) : event.data;
        handleExternalPayload(payload);
      } catch (err) {
        console.warn('hp-sim: failed to process external G-code payload.', err);
      }
    });
    externalCommandSocket.addEventListener('close', () => {
      externalCommandSocket = null;
      externalCommandSocketConnecting = false;
      scheduleExternalCommandReconnect('connection closed');
    });
    externalCommandSocket.addEventListener('error', (err) => {
      console.warn('hp-sim: external G-code stream error.', err);
      if (externalCommandSocket) {
        try {
          externalCommandSocket.close();
        } catch (_err) {
          // Ignore socket close errors
        }
      }
    });
  }

  function colorArrayToHex(colorArr) {
    if (!Array.isArray(colorArr) || colorArr.length < 3) {
      return null;
    }
    const numeric = colorArr.map((value) => Number(value));
    const hasFinite = numeric.every((num) => Number.isFinite(num));
    if (!hasFinite) {
      return null;
    }
    const maxComponent = Math.max(...numeric.map((num) => Math.abs(num)));
    const scale = maxComponent > 1 ? 255 : 1;
    const clamp = (num) => {
      if (scale === 255) {
        return Math.min(Math.max(num, 0), 255);
      }
      if (num <= 0) return 0;
      if (num >= 1) return 1;
      return num;
    };
    const toHex = (num) => {
      const clamped = clamp(num);
      const scaled = scale === 255 ? clamped : clamped * 255;
      return Math.round(scaled).toString(16).padStart(2, '0');
    };
    return `#${toHex(colorArr[0])}${toHex(colorArr[1])}${toHex(colorArr[2])}`;
  }

  function createEmptyBounds() {
    return {
      minX: Infinity,
      minY: Infinity,
      minZ: Infinity,
      maxX: -Infinity,
      maxY: -Infinity,
      maxZ: -Infinity,
    };
  }

  function updateBoundsWithPoint(bounds, point) {
    if (!bounds || !Array.isArray(point) || point.length < 3) {
      return;
    }
    const [x, y, z] = point;
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      return;
    }
    bounds.minX = Math.min(bounds.minX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.minZ = Math.min(bounds.minZ, z);
    bounds.maxX = Math.max(bounds.maxX, x);
    bounds.maxY = Math.max(bounds.maxY, y);
    bounds.maxZ = Math.max(bounds.maxZ, z);
  }

  function finalizeBounds(bounds) {
    if (!bounds || !Number.isFinite(bounds.minX)) {
      return null;
    }
    return {
      minX: bounds.minX,
      minY: bounds.minY,
      minZ: bounds.minZ,
      maxX: bounds.maxX,
      maxY: bounds.maxY,
      maxZ: bounds.maxZ,
    };
  }

  function sanitizeGcodeLine(rawLine) {
    if (typeof rawLine !== 'string') {
      return '';
    }
    let line = rawLine.trim();
    if (!line || line.startsWith(';')) {
      return '';
    }
    line = line.replace(GCODE_INLINE_COMMENT_RE, '');
    const commentIndex = line.indexOf(';');
    if (commentIndex >= 0) {
      line = line.slice(0, commentIndex);
    }
    return line.trim();
  }

  function parseGcodeText(text) {
    if (typeof text !== 'string' || text.length === 0) {
      return { segments: [], bounds: null };
    }
    const bounds = createEmptyBounds();
    const segments = [];
    const state = {
      position: { X: 0, Y: 0, Z: 0 },
      extruder: 0,
      positionAbsolute: true,
      extrusionAbsolute: true,
      feedRate: null,
    };

    const lines = text.split(/\r?\n/);
    for (const rawLine of lines) {
      const line = sanitizeGcodeLine(rawLine);
      if (!line) {
        continue;
      }
      const tokens = line.split(/\s+/).filter(Boolean);
      if (tokens.length === 0) {
        continue;
      }
      const primaryToken = tokens[0].toUpperCase();
      const codeLetter = primaryToken.charAt(0);
      const codeNumber = Number.parseInt(primaryToken.substring(1), 10);

      if (codeLetter === 'M' && codeNumber === 82) {
        state.extrusionAbsolute = true;
        continue;
      }
      if (codeLetter === 'M' && codeNumber === 83) {
        state.extrusionAbsolute = false;
        continue;
      }
      if (codeLetter === 'G' && codeNumber === 90) {
        state.positionAbsolute = true;
        continue;
      }
      if (codeLetter === 'G' && codeNumber === 91) {
        state.positionAbsolute = false;
        continue;
      }
      if (codeLetter === 'G' && codeNumber === 92) {
        const newPos = { ...state.position };
        let newExtruder = state.extruder;
        for (let i = 1; i < tokens.length; i += 1) {
          const token = tokens[i];
          if (!token) continue;
          const letter = token.charAt(0).toUpperCase();
          const value = Number.parseFloat(token.substring(1));
          if (!Number.isFinite(value)) {
            continue;
          }
          if (letter === 'X' || letter === 'Y' || letter === 'Z') {
            newPos[letter] = value;
          } else if (letter === 'E') {
            newExtruder = value;
          }
        }
        state.position = newPos;
        state.extruder = newExtruder;
        continue;
      }

      const isMoveCommand = codeLetter === 'G' && (codeNumber === 0 || codeNumber === 1);
      if (!isMoveCommand) {
        continue;
      }

      const startPos = { ...state.position };
      const nextPos = { ...state.position };
      let rawExtrusionValue = null;
      let feedRate = state.feedRate;

      for (let i = 1; i < tokens.length; i += 1) {
        const token = tokens[i];
        if (!token || token.length < 2) {
          continue;
        }
        const letter = token.charAt(0).toUpperCase();
        const value = Number.parseFloat(token.substring(1));
        if (!Number.isFinite(value)) {
          continue;
        }
        if (letter === 'X' || letter === 'Y' || letter === 'Z') {
          if (state.positionAbsolute) {
            nextPos[letter] = value;
          } else {
            nextPos[letter] = state.position[letter] + value;
          }
        } else if (letter === 'E') {
          rawExtrusionValue = value;
        } else if (letter === 'F') {
          feedRate = value;
        }
      }

      let extrusionDelta = 0;
      let nextExtruder = state.extruder;
      if (rawExtrusionValue != null) {
        if (state.extrusionAbsolute) {
          extrusionDelta = rawExtrusionValue - state.extruder;
          nextExtruder = rawExtrusionValue;
        } else {
          extrusionDelta = rawExtrusionValue;
          nextExtruder = state.extruder + rawExtrusionValue;
        }
      }

      const deltaX = nextPos.X - startPos.X;
      const deltaY = nextPos.Y - startPos.Y;
      const deltaZ = nextPos.Z - startPos.Z;
      const moveDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

      if (extrusionDelta > GCODE_EXTRUSION_EPSILON && moveDistance > GCODE_MOVE_EPSILON) {
        const segment = {
          start: [startPos.X, startPos.Y, startPos.Z],
          end: [nextPos.X, nextPos.Y, nextPos.Z],
          extrusion: extrusionDelta,
          feedRate: Number.isFinite(feedRate) ? feedRate : null,
          length: moveDistance,
        };
        segments.push(segment);
        updateBoundsWithPoint(bounds, segment.start);
        updateBoundsWithPoint(bounds, segment.end);
      }

      state.position = nextPos;
      state.extruder = nextExtruder;
      state.feedRate = feedRate;
    }

    return {
      segments,
      bounds: finalizeBounds(bounds),
    };
  }

  function convertParsedSegmentsToSimulation(parsed, { scale = GCODE_MM_TO_SIM_SCALE } = {}) {
    if (!parsed || !Array.isArray(parsed.segments)) {
      return { segments: [], bounds: null };
    }
    const segments = parsed.segments.map((segment) => {
      const start = [
        segment.start[0] * scale,
        segment.start[1] * scale,
        segment.start[2] * scale,
      ];
      const end = [
        segment.end[0] * scale,
        segment.end[1] * scale,
        segment.end[2] * scale,
      ];
      return {
        start,
        end,
        extrusion: segment.extrusion,
        feedRate: segment.feedRate,
        length: segment.length * scale,
      };
    });

    const bounds = parsed.bounds
      ? {
          minX: parsed.bounds.minX * scale,
          minY: parsed.bounds.minY * scale,
          minZ: parsed.bounds.minZ * scale,
          maxX: parsed.bounds.maxX * scale,
          maxY: parsed.bounds.maxY * scale,
          maxZ: parsed.bounds.maxZ * scale,
        }
      : null;

    return { segments, bounds };
  }

  async function fetchGcodeText(url) {
    const response = await fetch(url, { cache: 'no-cache' });
    if (!response.ok) {
      throw new Error(`Failed to fetch G-code from ${url}: ${response.status}`);
    }
    return response.text();
  }

  async function loadReferencePath(cacheKey, loader, {
    metadataOverrides = {},
    color = null,
    setActive = false,
    makeVisible = false,
  } = {}) {
    if (cacheKey && referencePathCache.has(cacheKey)) {
      const cached = referencePathCache.get(cacheKey);
      if (setActive) {
        setReferenceSegments(cached.segments, { metadata: cached.metadata, color: cached.color || color || cached.metadata?.color });
        if (makeVisible) {
          setReferenceVisibility(true);
        }
      }
      return cached;
    }

    const text = await loader();
    const parsed = parseGcodeText(text);
    const converted = convertParsedSegmentsToSimulation(parsed);
    let totalLengthMm = 0;
    let totalExtrusionMm = 0;
    for (const segment of parsed.segments) {
      totalLengthMm += segment.length;
      totalExtrusionMm += segment.extrusion;
    }

    const metadata = {
      ...metadataOverrides,
      bounds: converted.bounds,
      originalBounds: parsed.bounds,
      segmentCount: converted.segments.length,
      units: {
        source: 'mm',
        simulation: 'm',
        scale: GCODE_MM_TO_SIM_SCALE,
      },
      totals: {
        lengthSim: totalLengthMm * GCODE_MM_TO_SIM_SCALE,
        extrusion: totalExtrusionMm,
      },
    };
    if (!metadata.key && cacheKey) {
      metadata.key = cacheKey;
    }
    if (color && typeof color === 'string') {
      metadata.color = color;
    }
    const record = {
      segments: converted.segments,
      metadata,
      color: color || metadata.color || '#1e90ff',
    };

    if (cacheKey) {
      referencePathCache.set(cacheKey, record);
    }

    if (setActive) {
      setReferenceSegments(record.segments, { metadata: record.metadata, color: record.color });
      if (makeVisible) {
        setReferenceVisibility(true);
      }
    }

    return record;
  }

  async function loadReferencePathForPreset(presetKey, { setActive = true } = {}) {
    const descriptor = PRESET_GCODE_MAP[presetKey];
    if (!descriptor || !descriptor.url) {
      return null;
    }
    const cacheKey = `preset:${presetKey}`;
    try {
      return await loadReferencePath(cacheKey, () => fetchGcodeText(descriptor.url), {
        metadataOverrides: {
          key: cacheKey,
          label: descriptor.label || presetKey,
          source: {
            type: 'preset',
            presetKey,
            href: descriptor.url,
          },
        },
        color: descriptor.color,
        setActive,
      });
    } catch (error) {
      console.error('hp-sim: failed to load preset G-code reference', presetKey, error);
      return null;
    }
  }

  async function loadReferencePathFromFile(file, { setActive = true, makeVisible = false } = {}) {
    if (!file) {
      return null;
    }
    const cacheKey = `upload:${file.name}:${file.size}:${file.lastModified}`;
    const fallbackColor = '#2dd4bf';
    try {
      return await loadReferencePath(cacheKey, () => file.text(), {
        metadataOverrides: {
          key: cacheKey,
          label: file.name,
          source: {
            type: 'upload',
            name: file.name,
            size: file.size,
          },
          uploadedAt: Date.now(),
        },
        color: fallbackColor,
        setActive,
        makeVisible,
      });
    } catch (error) {
      console.error('hp-sim: failed to parse uploaded G-code reference', error);
      return null;
    }
  }

  function getPresetKeyForUploadFile(file) {
    if (!file?.name) {
      return null;
    }
    const name = file.name.trim();
    if (!name) {
      return null;
    }
    const dotIndex = name.lastIndexOf('.');
    const extension = dotIndex >= 0 ? name.slice(dotIndex).toLowerCase() : '';
    if (!uploadPresetConfig.extensionSet.has(extension)) {
      return null;
    }
    const normalized = name.toLowerCase();
    for (const entry of uploadPresetConfig.presets) {
      if (!entry?.substring || !entry?.presetKey) {
        continue;
      }
      if (normalized.includes(entry.substring.toLowerCase())) {
        return entry.presetKey;
      }
    }
    return null;
  }

  async function handleFileUpload(file) {
    if (!file) {
      return;
    }
    const matchedPresetKey = getPresetKeyForUploadFile(file);
    if (matchedPresetKey) {
      const descriptor = PRESET_GCODE_MAP[matchedPresetKey];
      const label = descriptor?.label || matchedPresetKey;
      console.log(`hp-sim: upload "${file.name}" matched preset "${matchedPresetKey}" (${label}).`);
      void loadReferencePathForPreset(matchedPresetKey, { setActive: true });
    }
    const detectedFormat = detectFileFormat(file.name);
    if (detectedFormat === FileFormat.GCODE) {
      const wantsSimulation = window.confirm('Do you want to simulate a print of this G-code?\nPress OK to simulate, or Cancel to draw the toolpath only.');
      if (!wantsSimulation) {
        ensureReadyForNewJob();
      }
      await loadReferencePathFromFile(file, { setActive: true, makeVisible: !wantsSimulation });
      if (wantsSimulation) {
        await queueUploadedFile(file, detectedFormat);
      }
      return;
    }
    await queueUploadedFile(file, detectedFormat);
  }

  function extractMachineColors(stage, scenePrimPath) {
    const result = { tintColor: null, extrusionColor: null };
    if (!stage || !scenePrimPath) {
      return result;
    }
    const prim = stage.GetPrimAtPath(scenePrimPath);
    if (!prim) {
      return result;
    }
    const tintValue = getAttribute(prim, 'machine:tintColor') ?? getAttribute(prim, 'machine:tint');
    const extrusionValue = getAttribute(prim, 'machine:extrusionColor');
    if (tintValue) {
      result.tintColor = colorArrayToHex(tintValue);
    }
    if (extrusionValue) {
      result.extrusionColor = colorArrayToHex(extrusionValue);
    }
    return result;
  }

  function createTintPalette(tintHex) {
    if (!tintHex) {
      return null;
    }
    return {
      spool: mixColors('#a0a0a0', tintHex, 0.65),
      anchor: mixColors('#aaaaaa', tintHex, 0.55),
      pinhole: mixColors('#cccccc', tintHex, 0.5),
      cable: mixColors('#ffff00', tintHex, 0.6),
      distanceConstraint: mixColors('#00ff00', tintHex, 0.55),
      rigidGroup: mixColors('#55ff88', tintHex, 0.55),
    };
  }

  function getParentPath(path) {
    if (typeof path !== 'string') {
      return null;
    }
    const trimmed = path.trim();
    if (!trimmed || trimmed === '/') {
      return null;
    }
    const separatorIndex = trimmed.lastIndexOf('/');
    if (separatorIndex <= 0) {
      return '/';
    }
    return trimmed.slice(0, separatorIndex);
  }

  function findScenePrimPath(stage) {
    if (!stage) {
      return '/World/SlideprinterScene';
    }

    const defaultPrimName = stage?.ast?.descriptor?.defaultPrim;
    if (typeof defaultPrimName === 'string' && defaultPrimName.length > 0) {
      const defaultCandidates = [`/${defaultPrimName}`, `/World/${defaultPrimName}`];
      for (const candidate of defaultCandidates) {
        const prim = stage.GetPrimAtPath(candidate);
        if (prim) {
          return candidate;
        }
      }
    }

    try {
      for (const [path, prim] of stage.Traverse()) {
        if (!path || !prim || prim.type !== 'definition') {
          continue;
        }
        const tags = getAttribute(prim, 'ecs:tags');
        if (Array.isArray(tags) && tags.includes('Spool')) {
          const parentPath = getParentPath(path);
          if (parentPath && stage.GetPrimAtPath(parentPath)) {
            return parentPath;
          }
        }
        if (prim.defType === 'CableJoint') {
          const parentPath = getParentPath(path);
          if (parentPath && stage.GetPrimAtPath(parentPath)) {
            return parentPath;
          }
        }
      }
    } catch (error) {
      console.warn('Slideprinter demo: failed to derive scene root, falling back to default.', error);
    }

    return '/World/SlideprinterScene';
  }

  function extractTimeCodesPerSecond(stage) {
    const assignments = stage?.ast?.descriptor?.assignments;
    if (!Array.isArray(assignments)) {
      return null;
    }
    const match = assignments.find(
      (entry) => entry?.type === 'assignment' && entry?.identifier === 'timeCodesPerSecond'
    );
    return match?.value ?? null;
  }

  function registerMachine(stage, { name = null, sourceKey = null, sourceUrl = null } = {}) {
    if (!stage) {
      return null;
    }
    if (sourceKey && machines.some((machine) => machine.sourceKey === sourceKey)) {
      return machines.find((machine) => machine.sourceKey === sourceKey) || null;
    }
    const machineId = `machine-${machineIdCounter++}`;
    const scenePrimPath = findScenePrimPath(stage);
    const { tintColor, extrusionColor } = extractMachineColors(stage, scenePrimPath);
    const palette = tintColor ? createTintPalette(tintColor) : null;
    const machine = {
      id: machineId,
      stage,
      palette,
      tintColor,
      extrusionColor,
      name: name || null,
      scenePrimPath,
      sourceKey: sourceKey || null,
      sourceUrl: sourceUrl || null,
    };
    if (sourceKey && usdaCatalog.has(sourceKey)) {
      const entry = usdaCatalog.get(sourceKey);
      if (entry) {
        if (tintColor) {
          entry.tintColor = tintColor;
          entry.tintColorLoaded = true;
          entry.tintColorPromise = null;
          applyPresetMachineTint(sourceKey, tintColor);
        } else if (!entry.tintColorLoaded) {
          entry.tintColorLoaded = true;
          entry.tintColorPromise = null;
          applyPresetMachineTint(sourceKey, null);
        }
      }
    }
    machines.push(machine);
    ensureQualityMonitorForMachine(machine);
    updateMachineMenuUI();
    return machine;
  }

  function rebuildScene() {
    if (!canvas || machines.length === 0) {
      return;
    }
    let isFirst = true;
    for (const machine of machines) {
      const sceneOptions = {
        remote: false,
        append: !isFirst,
        palette: machine.palette || null,
        scenePrimPath: machine.scenePrimPath,
        namespace: machine.id,
        tintColor: machine.tintColor || null,
        extrusionColor: machine.extrusionColor || null,
      };
      setupScene(world, machine.stage, canvas, sceneOptions);
      isFirst = false;
    }
    setLineLayeringEnabledState(lineLayeringEnabled, { fromToggle: true });
    attachQualityMonitorsToRemoteSystem();
    flushExternalCommandQueue();
  }

  function createColorChip() {
    const chip = document.createElement('span');
    chip.className = 'sim-machines-option-color';
    chip.setAttribute('aria-hidden', 'true');
    return chip;
  }

  function applyColorChipTint(chip, tintHex) {
    if (!chip) {
      return;
    }
    if (typeof tintHex === 'string' && tintHex.trim().length > 0) {
      chip.style.backgroundColor = tintHex;
      chip.classList.remove('sim-machines-option-color--empty');
    } else {
      chip.style.removeProperty('background-color');
      chip.classList.add('sim-machines-option-color--empty');
    }
  }

  function applyPresetMachineTint(sourceKey, tintHex) {
    if (sourceKey && usdaCatalog.has(sourceKey)) {
      const entry = usdaCatalog.get(sourceKey);
      if (entry) {
        entry.tintColor = tintHex ?? null;
      }
    }
    if (sourceKey) {
      for (const machine of machines) {
        if (machine.sourceKey === sourceKey) {
          machine.tintColor = tintHex ?? null;
          const monitorEntry = machineQualityMonitors.get(machine.id);
          if (monitorEntry?.monitor) {
            monitorEntry.monitor.setMachineContext({
              id: machine.id,
              label: getMachineDisplayName(machine),
              tintColor: machine.tintColor,
            });
          }
        }
      }
    }
    const chip = presetOptionColorChips.get(sourceKey);
    if (chip) {
      applyColorChipTint(chip, tintHex);
    }
    const label = presetOptionLabels.get(sourceKey);
    if (label) {
      if (tintHex) {
        label.dataset.tintColor = tintHex;
      } else {
        label.removeAttribute('data-tint-color');
      }
    }
  }

  function loadPresetTintColor(sourceKey) {
    if (!sourceKey || !usdaCatalog.has(sourceKey)) {
      return Promise.resolve(null);
    }
    const entry = usdaCatalog.get(sourceKey);
    if (!entry) {
      return Promise.resolve(null);
    }
    if (entry.tintColorLoaded) {
      applyPresetMachineTint(sourceKey, entry.tintColor);
      return Promise.resolve(entry.tintColor);
    }
    if (entry.tintColorPromise) {
      return entry.tintColorPromise;
    }
    entry.tintColorPromise = (async () => {
      try {
        const stage = await UsdOpen(entry.url);
        const scenePrimPath = findScenePrimPath(stage);
        const { tintColor } = extractMachineColors(stage, scenePrimPath);
        entry.tintColor = tintColor || null;
      } catch (error) {
        console.warn(`hp-sim: unable to read tint color for USDA preset ${sourceKey}.`, error);
        entry.tintColor = null;
      } finally {
        entry.tintColorLoaded = true;
        entry.tintColorPromise = null;
        applyPresetMachineTint(sourceKey, entry.tintColor);
      }
      return entry.tintColor;
    })();
    return entry.tintColorPromise;
  }

  function buildPresetMachineOptions() {
    if (!presetMachinesList) {
      return;
    }
    presetMachinesList.innerHTML = '';
    presetOptionInputs.clear();
    presetOptionLabels.clear();
    presetOptionColorChips.clear();
    for (const [key, entry] of usdaCatalog.entries()) {
      const item = document.createElement('li');
      item.className = 'sim-machines-option';
      const label = document.createElement('label');
      label.className = 'sim-machines-option-label';
      label.dataset.sourceKey = key;

      const checkbox = document.createElement('input');
      checkbox.type = 'checkbox';
      checkbox.className = 'sim-machines-checkbox';
      checkbox.dataset.sourceKey = key;
      checkbox.value = key;

      const colorChip = createColorChip();
      applyColorChipTint(colorChip, entry.tintColor);

      const text = document.createElement('span');
      text.className = 'sim-machines-option-text';
      text.textContent = entry.label;

      label.appendChild(checkbox);
      label.appendChild(colorChip);
      label.appendChild(text);
      item.appendChild(label);
      presetMachinesList.appendChild(item);
      presetOptionInputs.set(key, checkbox);
      presetOptionLabels.set(key, label);
      presetOptionColorChips.set(key, colorChip);
      if (!entry.tintColorLoaded) {
        loadPresetTintColor(key).catch((error) => {
          console.warn(`hp-sim: tint color preload failed for ${key}.`, error);
        });
      }
    }
  }

  function syncPresetMachineSelections() {
    if (presetOptionInputs.size === 0) {
      buildPresetMachineOptions();
    }
    for (const [key, checkbox] of presetOptionInputs.entries()) {
      const isLoaded = machines.some((machine) => machine.sourceKey === key);
      checkbox.checked = isLoaded;
    }
  }

  function syncCustomMachineList() {
    if (!customMachinesSection || !customMachinesList) {
      return;
    }
    customMachinesList.innerHTML = '';
    const uploads = machines.filter((machine) => !machine.sourceKey);
    if (uploads.length === 0) {
      customMachinesSection.classList.add('sim-hidden');
      return;
    }
    customMachinesSection.classList.remove('sim-hidden');
    for (const machine of uploads) {
      const item = document.createElement('li');
      item.className = 'sim-machines-custom-item';

      const info = document.createElement('div');
      info.className = 'sim-machines-custom-info';

      const colorChip = createColorChip();
      applyColorChipTint(colorChip, machine.tintColor);

      const name = document.createElement('span');
      name.className = 'sim-machines-custom-name';
      const displayName = machine.name || 'Uploaded scene';
      name.textContent = displayName;
      name.title = machine.sourceUrl || machine.name || machine.id;
      info.appendChild(colorChip);
      info.appendChild(name);
      item.appendChild(info);

      const removeBtn = document.createElement('button');
      removeBtn.type = 'button';
      removeBtn.className = 'sim-tertiary sim-machines-remove';
      removeBtn.textContent = 'Remove';
      removeBtn.dataset.machineId = machine.id;
      item.appendChild(removeBtn);

      customMachinesList.appendChild(item);
    }
  }

  function updateMachinesToggleAccessibility() {
    if (!machinesToggle) {
      return;
    }
    const count = machines.length;
    const label = count === 0 ? 'Machines (no scenes loaded)' : `Machines (${count} scenes loaded)`;
    machinesToggle.setAttribute('aria-label', label);
  }

  function updateRemoveAllButtonState() {
    if (!machinesRemoveAllBtn) {
      return;
    }
    const disabled = machines.length === 0;
    machinesRemoveAllBtn.disabled = disabled;
    if (disabled) {
      machinesRemoveAllBtn.setAttribute('aria-disabled', 'true');
    } else {
      machinesRemoveAllBtn.removeAttribute('aria-disabled');
    }
  }

  function updateMachineMenuUI() {
    syncPresetMachineSelections();
    syncCustomMachineList();
    updateRemoveAllButtonState();
    updateMachinesToggleAccessibility();
  }

  function clearMachineMenuInlinePosition() {
    if (!machinesMenu) {
      return;
    }
    machinesMenu.style.left = '';
    machinesMenu.style.right = '';
    machinesMenu.style.transform = '';
  }

  function positionMachineMenuForMobile() {
    if (!machinesMenu || !machinesToggle) {
      return;
    }
    if (!isMobileLayout()) {
      clearMachineMenuInlinePosition();
      return;
    }
    const parentRect = machinesContainer?.getBoundingClientRect() ?? machinesMenu.parentElement?.getBoundingClientRect();
    const toggleRect = machinesToggle.getBoundingClientRect();
    const menuRect = machinesMenu.getBoundingClientRect();
    const viewportWidth = window.innerWidth || document.documentElement.clientWidth || 0;
    if (!parentRect || !menuRect || !Number.isFinite(viewportWidth) || viewportWidth <= 0) {
      return;
    }
    const menuWidth = menuRect.width;
    if (!Number.isFinite(menuWidth) || menuWidth <= 0) {
      return;
    }
    const parentLeft = parentRect.left || 0;
    const toggleCenter = toggleRect.left + toggleRect.width / 2;
    const maxTargetLeft = Math.max(
      MOBILE_MACHINES_MENU_MARGIN_PX,
      viewportWidth - menuWidth - MOBILE_MACHINES_MENU_MARGIN_PX
    );
    let targetLeft = toggleCenter - menuWidth / 2;
    if (!Number.isFinite(targetLeft)) {
      targetLeft = MOBILE_MACHINES_MENU_MARGIN_PX;
    }
    targetLeft = Math.max(MOBILE_MACHINES_MENU_MARGIN_PX, Math.min(targetLeft, maxTargetLeft));
    const relativeLeft = targetLeft - parentLeft;
    machinesMenu.style.left = `${relativeLeft}px`;
    machinesMenu.style.right = 'auto';
    machinesMenu.style.transform = 'none';
  }

  function syncMachineMenuPlacement() {
    if (!machinesMenu) {
      return;
    }
    if (!machineMenuOpen) {
      if (!isMobileLayout()) {
        clearMachineMenuInlinePosition();
      }
      return;
    }
    if (isMobileLayout()) {
      positionMachineMenuForMobile();
      return;
    }
    clearMachineMenuInlinePosition();
  }

  function scheduleMachineMenuPlacementSync() {
    const syncPlacement = () => {
      syncMachineMenuPlacement();
    };
    if (typeof window.requestAnimationFrame === 'function') {
      window.requestAnimationFrame(syncPlacement);
    } else {
      window.setTimeout(syncPlacement, 0);
    }
  }

  function isHoverCapableDesktop() {
    if (supportsMatchMedia) {
      try {
        const query = window.matchMedia('(hover: hover) and (pointer: fine)');
        return query.matches;
      } catch (error) {
        console.warn('hp-sim: unable to evaluate hover media query.', error);
      }
    }
    return !isMobileLayout();
  }

  function isPointerWithinMachineMenu() {
    const targets = [machinesMenu, machinesToggle];
    for (const element of targets) {
      if (element instanceof HTMLElement && typeof element.matches === 'function' && element.matches(':hover')) {
        return true;
      }
    }
    return false;
  }

  function clearMachineMenuHoverTimeout() {
    if (machineMenuHoverCloseTimeout !== null) {
      window.clearTimeout(machineMenuHoverCloseTimeout);
      machineMenuHoverCloseTimeout = null;
    }
  }

  function scheduleMachineMenuHoverClose() {
    if (!machineMenuHoverTrackingEnabled) {
      return;
    }
    clearMachineMenuHoverTimeout();
    machineMenuHoverCloseTimeout = window.setTimeout(() => {
      machineMenuHoverCloseTimeout = null;
      if (!machineMenuHoverTrackingEnabled || !machineMenuOpen || machineMenuPointerInside || machineMenuFocusInside) {
        return;
      }
      const activeElement = document.activeElement;
      if (machinesMenu && activeElement instanceof Node && machinesMenu.contains(activeElement)) {
        machineMenuFocusInside = true;
        return;
      }
      closeMachineMenu();
    }, MACHINE_MENU_HOVER_CLOSE_DELAY_MS);
  }

  function startMachineMenuHoverTracking() {
    if (!machinesContainer) {
      machineMenuHoverTrackingEnabled = false;
      clearMachineMenuHoverTimeout();
      return;
    }
    machineMenuHoverTrackingEnabled = isHoverCapableDesktop();
    if (!machineMenuHoverTrackingEnabled) {
      clearMachineMenuHoverTimeout();
      return;
    }
    machineMenuPointerInside = isPointerWithinMachineMenu();
    if (machineMenuPointerInside || machineMenuFocusInside) {
      clearMachineMenuHoverTimeout();
      return;
    }
    scheduleMachineMenuHoverClose();
  }

  function stopMachineMenuHoverTracking() {
    machineMenuHoverTrackingEnabled = false;
    machineMenuPointerInside = false;
    machineMenuFocusInside = false;
    clearMachineMenuHoverTimeout();
  }

  function handleMachineMenuMouseEnter() {
    if (!machineMenuHoverTrackingEnabled) {
      machineMenuPointerInside = true;
      return;
    }
    machineMenuPointerInside = true;
    clearMachineMenuHoverTimeout();
  }

  function handleMachineMenuMouseLeave(event) {
    if (!machineMenuHoverTrackingEnabled) {
      return;
    }
    const nextTarget = event?.relatedTarget;
    if (
      nextTarget instanceof Node &&
      ((machinesMenu && machinesMenu.contains(nextTarget)) || (machinesToggle && machinesToggle.contains(nextTarget)))
    ) {
      return;
    }
    machineMenuPointerInside = false;
    scheduleMachineMenuHoverClose();
  }

  function handleMachineMenuFocusIn() {
    machineMenuFocusInside = true;
    clearMachineMenuHoverTimeout();
  }

  function handleMachineMenuFocusOut(event) {
    const nextTarget = event?.relatedTarget;
    if (
      nextTarget instanceof Node &&
      ((machinesMenu && machinesMenu.contains(nextTarget)) || (machinesToggle && machinesToggle.contains(nextTarget)))
    ) {
      return;
    }
    machineMenuFocusInside = false;
    if (machineMenuHoverTrackingEnabled) {
      scheduleMachineMenuHoverClose();
    }
  }

  function addMachineMenuOutsideListeners() {
    document.addEventListener('mousedown', handleMachineMenuOutsideInteraction, true);
    document.addEventListener('touchstart', handleMachineMenuOutsideInteraction, true);
    document.addEventListener('click', handleMachineMenuOutsideInteraction, true);
    document.addEventListener('keydown', handleMachineMenuKeydown, true);
  }

  function removeMachineMenuOutsideListeners() {
    document.removeEventListener('mousedown', handleMachineMenuOutsideInteraction, true);
    document.removeEventListener('touchstart', handleMachineMenuOutsideInteraction, true);
    document.removeEventListener('click', handleMachineMenuOutsideInteraction, true);
    document.removeEventListener('keydown', handleMachineMenuKeydown, true);
  }

  function openMachineMenu() {
    if (!machinesMenu || !machinesToggle) {
      return;
    }
    if (machineMenuOpen) {
      return;
    }
    machineMenuOpen = true;
    machinesMenu.classList.remove('sim-hidden');
    machinesToggle.setAttribute('aria-expanded', 'true');
    if (machinesContainer) {
      machinesContainer.setAttribute('data-open', 'true');
    }
    scheduleMachineMenuPlacementSync();
    const firstFocusable = machinesMenu.querySelector('input, button');
    if (firstFocusable instanceof HTMLElement) {
      firstFocusable.focus({ preventScroll: true });
    } else if (machinesMenu instanceof HTMLElement) {
      machinesMenu.focus({ preventScroll: true });
    }
    addMachineMenuOutsideListeners();
    startMachineMenuHoverTracking();
  }

  function closeMachineMenu({ focusToggle = false } = {}) {
    if (!machinesMenu || !machinesToggle) {
      return;
    }
    if (!machineMenuOpen) {
      return;
    }
    machineMenuOpen = false;
    machinesMenu.classList.add('sim-hidden');
    machinesToggle.setAttribute('aria-expanded', 'false');
    if (machinesContainer) {
      machinesContainer.setAttribute('data-open', 'false');
    }
    clearMachineMenuInlinePosition();
    removeMachineMenuOutsideListeners();
    stopMachineMenuHoverTracking();
    if (focusToggle) {
      machinesToggle.focus({ preventScroll: true });
    }
  }

  function handleMachineMenuOutsideInteraction(event) {
    if (!machineMenuOpen || !machinesMenu) {
      return;
    }
    const target = event.target;
    if (!(target instanceof Node)) {
      return;
    }
    if (machinesMenu.contains(target)) {
      return;
    }
    if (machinesToggle && machinesToggle.contains(target)) {
      return;
    }
    closeMachineMenu();
  }

  function handleMachineMenuKeydown(event) {
    if (!machineMenuOpen) {
      return;
    }
    if (event.key === 'Escape' || event.key === 'Esc') {
      event.preventDefault();
      closeMachineMenu({ focusToggle: true });
    }
  }

  async function beginSceneChange({ newMachineAdded = false } = {}) {
    if (sceneChangeState.context) {
      if (newMachineAdded) {
        sceneChangeState.context.newMachineAdded = true;
      }
      return sceneChangeState.context;
    }
    captureSceneFrameSnapshot();
    suspendRenderSystemForSceneChange();
    const remoteSystem = getRemoteSystem();
    const wasPrinting = Boolean(printActive && remoteSystem);
    const pauseState = world.getResource('pauseState');
    sceneChangeState.wasPaused = pauseState ? pauseState.paused : null;
    sceneChangeState.targetHistoryLength = remoteSystem ? remoteSystem.history.length : 0;
    sceneChangeState.replayInProgress = false;

    if (wasPrinting) {
      try {
        if (moveCommanderWorker) {
          moveCommanderWorker.postMessage({ type: 'pause' });
        }
        if (klipperCommanderWorker) {
          klipperCommanderWorker.postMessage({ type: 'pause' });
        }
        if (rrfCommanderWorker) {
          rrfCommanderWorker.postMessage({ type: 'pause' });
        }
      } catch (err) {
        console.warn('hp-sim: unable to pause workers during scene change.', err);
      }
      if (pauseState) {
        pauseState.paused = true;
      }
      if (!sceneChangeState.pausedForSceneChange) {
        showPrintStatus('Print paused due to change of scene');
      }
      sceneChangeState.pausedForSceneChange = true;
      if (pauseBtn) {
        pauseBtn.textContent = 'Resume';
      }

      if (remoteSystem) {
        let settleAttempts = 0;
        const getQueueLength = typeof remoteSystem.getQueueLength === 'function'
          ? () => remoteSystem.getQueueLength()
          : () => (Array.isArray(remoteSystem.commands) ? remoteSystem.commands.length : 0);
        let previousLength = getQueueLength();
        const maxAttempts = 6;
        while (settleAttempts < maxAttempts) {
          await new Promise((resolve) => setTimeout(resolve, 0));
          const currentLength = getQueueLength();
          if (currentLength === previousLength) {
            break;
          }
          previousLength = currentLength;
          settleAttempts += 1;
        }
      }
    }

    let playbackState = null;
    if (wasPrinting && remoteSystem && typeof remoteSystem.getPlaybackState === 'function') {
      try {
        playbackState = remoteSystem.getPlaybackState();
      } catch (err) {
        console.warn('hp-sim: unable to capture playback state before scene change.', err);
      }
    }
    const extrusionSnapshot = cloneExtrusionList(getExtruderComponent()?.extrusions);

    const context = {
      wasPrinting,
      playbackState,
      extrusionSnapshot,
      worker: remoteSystem ? remoteSystem.worker : null,
      newMachineAdded: Boolean(newMachineAdded),
      targetHistoryLength: sceneChangeState.targetHistoryLength,
      wasPaused: sceneChangeState.wasPaused,
    };
    sceneChangeState.context = context;
    return context;
  }

  async function runReplayLoop(targetCount, { renderSystem = null } = {}) {
    if (!Number.isFinite(targetCount) || targetCount <= 0) {
      return;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    sceneChangeState.replayInProgress = true;
    const previousPauseDisabled = pauseBtn ? pauseBtn.disabled : null;
    if (pauseBtn) {
      setButtonDisabled(pauseBtn, true);
    }
    if (renderSystem && typeof renderSystem.setDrawingSuspended === 'function') {
      renderSystem.setDrawingSuspended(true);
    }
    const pauseState = world.getResource('pauseState');
    const dtResource = world.getResource('dt');
    const stepDt =
      typeof dtResource === 'number' && Number.isFinite(dtResource) && dtResource > 0
        ? dtResource
        : typeof simDtSec === 'number' && Number.isFinite(simDtSec) && simDtSec > 0
        ? simDtSec
        : 1 / 120;
    const maxIterations = Math.max(targetCount * 4, targetCount + 200);
    let iterations = 0;
    try {
      if (pauseState) {
        pauseState.paused = false;
      }
      while (remoteSystem.history.length < targetCount && iterations < maxIterations) {
        world.update(stepDt);
        iterations += 1;
        if (iterations % 500 === 0) {
          await new Promise((resolve) => setTimeout(resolve, 0));
        }
      }
    } finally {
      if (pauseBtn) {
        const restoreDisabled = previousPauseDisabled != null ? previousPauseDisabled : false;
        setButtonDisabled(pauseBtn, restoreDisabled);
      }
      if (pauseState) {
        pauseState.paused = true;
      }
      sceneChangeState.replayInProgress = false;
    }
    if (remoteSystem.history.length < targetCount) {
      console.warn(
        `hp-sim: replay stopped early after ${remoteSystem.history.length} commands, expected ${targetCount}.`
      );
    }
  }

  async function restorePrintAfterSceneChange(sceneChange) {
    sceneChangeState.replayInProgress = false;
    const renderSystem = world.getResource('renderSystem');
    const pauseState = world.getResource('pauseState');
    const shouldResetQuality =
      Boolean(sceneChange?.wasPrinting) && machineQualityMonitors.size > 0;
    if (shouldResetQuality) {
      resetQualityMonitors({ keepReference: true });
    }
    if (!sceneChange || !sceneChange.wasPrinting) {
      if (renderSystem && typeof renderSystem.setDrawingSuspended === 'function') {
        renderSystem.setDrawingSuspended(false);
      }
      sceneChangeState.renderSuspended = false;
      if (renderSystem && typeof renderSystem.update === 'function') {
        renderSystem.update(world, 0);
      }
      clearSceneFrameSnapshot();
      sceneChangeState.context = null;
      return;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      if (renderSystem && typeof renderSystem.setDrawingSuspended === 'function') {
        renderSystem.setDrawingSuspended(false);
      }
      sceneChangeState.renderSuspended = false;
      clearSceneFrameSnapshot();
      sceneChangeState.context = null;
      return;
    }
    const playbackState = sceneChange.playbackState || { history: [], queue: [] };
    const historyClone = cloneCommandList(playbackState.history);
    const queueClone = cloneCommandList(playbackState.queue);
    const extruderComp = getExtruderComponent();
    if (extruderComp && Array.isArray(extruderComp.extrusions)) {
      extruderComp.extrusions = [];
    }
    const extrusionSnapshot = cloneExtrusionList(sceneChange.extrusionSnapshot);
    let restoredExtrusions = [];
    const showReplay = Boolean(sceneChange.wasPrinting && historyClone.length > 0);
    if (showReplay) {
      showReplayStatus();
    }

    remoteSystem.worker = null;
    remoteSystem.wasPaused = false;
    remoteSystem.history = [];
    remoteSystem.commands = cloneCommandList(historyClone).concat(queueClone);
    if (typeof remoteSystem.resetAxisMapping === 'function') {
      remoteSystem.resetAxisMapping();
    }

    try {
      await new Promise((resolve) => setTimeout(resolve, 0));
      if (historyClone.length > 0) {
        await runReplayLoop(historyClone.length, { renderSystem });
      }
    } finally {
      if (showReplay) {
        hideReplayStatus();
      }
    }

    remoteSystem.history = historyClone;
    remoteSystem.commands = queueClone;
    remoteSystem.worker = sceneChange.worker || null;
    remoteSystem.wasPaused = false;
    if (extruderComp && extrusionSnapshot.length > 0) {
      restoredExtrusions = restoreReplayExtrusions(extruderComp, extrusionSnapshot);
    }
    if (shouldResetQuality) {
      if (restoredExtrusions.length > 0) {
        replayExtrusionsIntoQualityMonitors(restoredExtrusions);
      } else {
        runFinalQualityChecks();
        refreshAllQualityMonitors(true);
      }
    }

    if (renderSystem) {
      if (typeof renderSystem.setDrawingSuspended === 'function') {
        renderSystem.setDrawingSuspended(false);
      }
      if (typeof renderSystem.clearExtrusions === 'function') {
        renderSystem.clearExtrusions();
      }
      if ('drawnExtrusionCount' in renderSystem) {
        renderSystem.drawnExtrusionCount = 0;
      }
      if (typeof renderSystem.update === 'function') {
        renderSystem.update(world, 0);
      }
    }

    if (pauseState) {
      pauseState.paused = true;
    }
    sceneChangeState.renderSuspended = false;
    clearSceneFrameSnapshot();
    sceneChangeState.pausedForSceneChange = true;
    sceneChangeState.targetHistoryLength = 0;
    sceneChangeState.wasPaused = null;
    sceneChangeState.context = null;
  }

  async function refreshSceneAfterMachineChange({ clearExtrusions = false, resetView = false, sceneChange = null } = {}) {
    if (machines.length === 0) {
      clearQualityMonitors();
      updateQualityHudVisibility();
      world.clear();
      const renderSystem = world.getResource('renderSystem');
      if (renderSystem && typeof renderSystem.resetVisuals === 'function') {
        renderSystem.resetVisuals();
        renderSystem.update?.(world, 0);
      }
      clearSceneFrameSnapshot();
      resetViewStateDefaults();
      setPanMode(isMobileLayout());
      if (panModeBtn) {
        panModeBtn.disabled = true;
        panModeBtn.setAttribute('aria-disabled', 'true');
      }
      stageReady = false;
      setSpeedButtonsEnabled(false);
      updateZoomButtonState();
      hideReplayStatus();
      hidePrintStatus();
      sceneChangeState.context = null;
      sceneChangeState.pausedForSceneChange = false;
      sceneChangeState.targetHistoryLength = 0;
      sceneChangeState.wasPaused = null;
      stopAndClearWorkers();
      setPrintActive(false);
      return;
    }

    if (resetView) {
      resetViewStateDefaults();
    }

    rebuildScene();
    syncCanvasDimensions();
    updateZoomButtonState();
    stageReady = true;
    setSpeedButtonsEnabled(true);
    if (panModeBtn) {
      panModeBtn.disabled = false;
      panModeBtn.removeAttribute('aria-disabled');
    }
    reapplyViewState({ clearExtrusions });
    if (sceneChangeState.renderSuspended || sceneChangeState.frameSnapshotNeedsApply) {
      suspendRenderSystemForSceneChange();
    }
    applySceneFrameSnapshot();
    if (sceneChange) {
      await restorePrintAfterSceneChange(sceneChange);
    }
  }

  function removeMachine(machineId) {
    return enqueueSceneChange(async () => {
      const index = machines.findIndex((machine) => machine.id === machineId);
      if (index === -1) {
        return;
      }
      const [removedMachine] = machines.splice(index, 1);
      if (removedMachine) {
        removeQualityMonitor(removedMachine.id);
      }
      const sceneChange = await beginSceneChange({ newMachineAdded: false });
      updateMachineMenuUI();
      if (machines.length === 0) {
        await refreshSceneAfterMachineChange({ clearExtrusions: true, resetView: true, sceneChange });
        return;
      }
      await refreshSceneAfterMachineChange({ clearExtrusions: true, resetView: false, sceneChange });
    });
  }

  function removeAllMachines() {
    return enqueueSceneChange(async () => {
      if (machines.length === 0) {
        return;
      }
      machines.splice(0, machines.length);
      clearQualityMonitors();
      const sceneChange = await beginSceneChange({ newMachineAdded: false });
      updateMachineMenuUI();
      await refreshSceneAfterMachineChange({ clearExtrusions: true, resetView: true, sceneChange });
    });
  }

  async function addUsdMachineFromFile(file) {
    if (!file) {
      return;
    }
    const label = file.name || 'uploaded.usda';
    let sourceText = null;
    try {
      sourceText = await file.text();
    } catch (error) {
      console.error(`Slideprinter demo: failed to read USDA file ${label}:`, error);
      return;
    }

    let stage = null;
    try {
      stage = await UsdOpen(sourceText);
    } catch (error) {
      console.error(`Slideprinter demo: unable to parse USDA file ${label}:`, error);
      return;
    }

    return enqueueSceneChange(async () => {
      const machine = registerMachine(stage, { name: label, sourceKey: null, sourceUrl: null });
      if (!machine) {
        return;
      }

      const timeCodesPerSecond = extractTimeCodesPerSecond(stage);
      if (timeCodesPerSecond) {
        const uploadedDt = 1.0 / timeCodesPerSecond;
        if (simDtSec == null) {
          simDtSec = uploadedDt;
        } else if (Math.abs(uploadedDt - simDtSec) > 1e-6) {
          console.warn(
            `Slideprinter demo: USDA file ${label} uses timeCodesPerSecond=${timeCodesPerSecond}, which differs from the active simulation. Using existing dt=${simDtSec.toFixed(6)}s.`
          );
        }
      }

      const sceneChange = await beginSceneChange({ newMachineAdded: true });
      await refreshSceneAfterMachineChange({
        clearExtrusions: true,
        resetView: machines.length === 1,
        sceneChange,
      });
    });
  }

  async function addUsdaFromCatalog(sourceKey, { resetView = false } = {}) {
    if (!sourceKey || !usdaCatalog.has(sourceKey)) {
      return null;
    }
    const entry = usdaCatalog.get(sourceKey);
    let stage = null;
    try {
      stage = await UsdOpen(entry.url);
    } catch (error) {
      console.error(`hp-sim: unable to load USDA preset ${sourceKey}.`, error);
      return null;
    }
    if (!stage) {
      console.error(`hp-sim: USDA preset ${sourceKey} returned an invalid stage.`);
      return null;
    }

    return enqueueSceneChange(async () => {
      const existing = machines.find((machine) => machine.sourceKey === sourceKey);
      if (existing) {
        return existing;
      }

      const machine = registerMachine(stage, {
        name: entry.label,
        sourceKey,
        sourceUrl: entry.url,
      });
      if (!machine) {
        return null;
      }

      const timeCodesPerSecond = extractTimeCodesPerSecond(stage);
      if (timeCodesPerSecond) {
        const presetDt = 1.0 / timeCodesPerSecond;
        if (simDtSec == null) {
          simDtSec = presetDt;
        } else if (Math.abs(presetDt - simDtSec) > 1e-6) {
          console.warn(
            `hp-sim: USDA preset ${sourceKey} uses timeCodesPerSecond=${timeCodesPerSecond}, which differs from the active simulation. Using existing dt=${simDtSec.toFixed(6)}s.`
          );
        }
      }

      const sceneChange = await beginSceneChange({ newMachineAdded: true });
      await refreshSceneAfterMachineChange({
        clearExtrusions: true,
        resetView: resetView || machines.length === 1,
        sceneChange,
      });
      return machine;
    });
  }

  function clearSecondaryControlsInteractionDelay() {
    if (secondaryControlsInteractionEnableTimeout != null) {
      window.clearTimeout(secondaryControlsInteractionEnableTimeout);
      secondaryControlsInteractionEnableTimeout = null;
    }
  }

  function setSecondaryControlsInteractive(enabled) {
    if (!secondaryControls) {
      return;
    }
    secondaryControls.dataset.interactive = enabled ? 'true' : 'false';
  }

  function scheduleSecondaryControlsInteractionEnable() {
    if (!secondaryControls) {
      return;
    }
    clearSecondaryControlsInteractionDelay();
    if (MOBILE_SECONDARY_CONTROLS_INTERACTION_DELAY_MS <= 0) {
      setSecondaryControlsInteractive(true);
      return;
    }
    secondaryControlsInteractionEnableTimeout = window.setTimeout(() => {
      secondaryControlsInteractionEnableTimeout = null;
      setSecondaryControlsInteractive(true);
    }, MOBILE_SECONDARY_CONTROLS_INTERACTION_DELAY_MS);
  }

  if (secondaryControls) {
    setSecondaryControlsInteractive(false);
  }

  function hideSecondaryControlsForMobile({ force = false } = {}) {
    if (!secondaryControls) {
      return;
    }
    if (!force && secondaryControlsUserPreference === true) {
      return;
    }
    secondaryControls.classList.add('sim-hidden');
    clearSecondaryControlsInteractionDelay();
    setSecondaryControlsInteractive(false);
    if (secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
      secondaryControlsHideTimeout = null;
    }
  }

  function showSecondaryControlsForMobile({ persist = false } = {}) {
    if (!secondaryControls || !isMobileLayout()) {
      return;
    }
    if (!persist && secondaryControlsUserPreference === false) {
      return;
    }
    const wasHidden = secondaryControls.classList.contains('sim-hidden');
    if (wasHidden) {
      setSecondaryControlsInteractive(false);
    }
    secondaryControls.classList.remove('sim-hidden');
    secondaryControlsEverShown = true;
    if (secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
    }
    if (persist || secondaryControlsUserPreference === true) {
      secondaryControlsHideTimeout = null;
      setSecondaryControlsInteractive(true);
    } else {
      secondaryControlsHideTimeout = window.setTimeout(() => {
        hideSecondaryControlsForMobile();
      }, MOBILE_SECONDARY_CONTROLS_TIMEOUT_MS);
      if (wasHidden) {
        scheduleSecondaryControlsInteractionEnable();
      } else {
        setSecondaryControlsInteractive(true);
      }
    }
  }

  function computeSecondaryControlsDesired() {
    if (secondaryControlsUserPreference === true) {
      return true;
    }
    if (secondaryControlsUserPreference === false) {
      return false;
    }
    return secondaryControlsAutoActive;
  }

  function updateSecondaryToggleButton() {
    if (!secondaryToggleBtn) {
      return;
    }
    const shouldShow = computeSecondaryControlsDesired();
    const hasPreference = secondaryControlsUserPreference !== null;
    secondaryToggleBtn.setAttribute('aria-expanded', shouldShow ? 'true' : 'false');
    const baseTitle = shouldShow ? 'Collapse secondary controls' : 'Expand secondary controls';
    secondaryToggleBtn.title = hasPreference ? `${baseTitle} (Alt-click to reset)` : baseTitle;
  }

  function applySecondaryControlsVisibility() {
    const shouldShow = computeSecondaryControlsDesired();
    if (!secondaryControls) {
      updateSecondaryToggleButton();
      return;
    }
    if (isMobileLayout()) {
      if (shouldShow) {
        showSecondaryControlsForMobile({ persist: secondaryControlsUserPreference === true });
      } else {
        hideSecondaryControlsForMobile({ force: true });
      }
      updateSecondaryToggleButton();
      return;
    }
    if (secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
      secondaryControlsHideTimeout = null;
    }
    if (shouldShow) {
      secondaryControls.classList.remove('sim-hidden');
      secondaryControlsEverShown = true;
      clearSecondaryControlsInteractionDelay();
      setSecondaryControlsInteractive(true);
    } else if (secondaryControlsUserPreference === false || !secondaryControlsEverShown) {
      secondaryControls.classList.add('sim-hidden');
      setSecondaryControlsInteractive(false);
    } else {
      clearSecondaryControlsInteractionDelay();
      setSecondaryControlsInteractive(true);
    }
    updateSecondaryToggleButton();
  }

  function setSecondaryControlsVisible(active) {
    secondaryControlsAutoActive = Boolean(active);
    applySecondaryControlsVisibility();
  }

  function updateMainButtonsState() {
    if (simButtons) {
      simButtons.classList.toggle('is-printing', printActive);
    }
  }

  function setPrintActive(active) {
    printActive = Boolean(active);
    updateMainButtonsState();
    setSecondaryControlsVisible(printActive && machines.length > 0);
    if (!printActive) {
      sceneChangeState.context = null;
      sceneChangeState.pausedForSceneChange = false;
      hideReplayStatus();
      hidePrintStatus();
      hideAsapStatus();
    }
    updateFinishAsapButtonState();
  }

  function handleLayoutChange() {
    const matches = isMobileLayout();
    updateQualityToggleLabel();
    updateReferenceToggleUI();
    if (matches) {
      if (!lastMobileLayoutMatches && !panModeActive) {
        setPanMode(true);
      }
    } else {
      if (secondaryControlsHideTimeout) {
        clearTimeout(secondaryControlsHideTimeout);
        secondaryControlsHideTimeout = null;
      }
    }
    applySecondaryControlsVisibility();
    if (machineMenuOpen) {
      if (matches) {
        scheduleMachineMenuPlacementSync();
      } else {
        clearMachineMenuInlinePosition();
      }
    } else if (!matches) {
      clearMachineMenuInlinePosition();
    }
    lastMobileLayoutMatches = matches;
  }

  function ensureReadyForNewJob() {
    if (printActive) {
      handleUserReset();
    }
  }

  function syncRenderSystem(viewState, { clearExtrusions = false } = {}) {
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem || typeof renderSystem.setViewTransform !== 'function') {
      return;
    }
    renderSystem.setViewTransform(viewState);
    syncReferenceOverlayToRenderSystem({ force: true });
    if (clearExtrusions && typeof renderSystem.clearExtrusions === 'function') {
      renderSystem.clearExtrusions();
    }
    if (clearExtrusions && typeof renderSystem.clearPositionTrace === 'function') {
      renderSystem.clearPositionTrace({ keepMarkers: true });
    }
  }

  function updateZoomButtonState() {
    const allowZoom = stageReady && machines.length > 0;
    if (zoomInBtn) {
      const atMax = currentViewScale >= MAX_VIEW_SCALE - ZOOM_EPSILON;
      const disabled = !allowZoom || atMax;
      zoomInBtn.disabled = disabled;
      if (disabled) {
        zoomInBtn.setAttribute('aria-disabled', 'true');
      } else {
        zoomInBtn.removeAttribute('aria-disabled');
      }
    }
    if (zoomOutBtn) {
      const atMin = currentViewScale <= MIN_VIEW_SCALE + ZOOM_EPSILON;
      const disabled = !allowZoom || atMin;
      zoomOutBtn.disabled = disabled;
      if (disabled) {
        zoomOutBtn.setAttribute('aria-disabled', 'true');
      } else {
        zoomOutBtn.removeAttribute('aria-disabled');
      }
    }
  }

  function setSpeedButtonsEnabled(enabled) {
    [speedSlowerBtn, speedFasterBtn].forEach((button) => {
      if (!button) {
        return;
      }
      button.disabled = !enabled;
      if (!enabled) {
        button.setAttribute('aria-disabled', 'true');
      } else {
        button.removeAttribute('aria-disabled');
      }
    });
  }

  function updateFinishAsapButtonState() {
    if (!finishAsapBtn) {
      return;
    }
    const shouldDisable = !printActive || asapState.active;
    setButtonDisabled(finishAsapBtn, shouldDisable);
  }

  function formatTimeScale(scale) {
    if (!Number.isFinite(scale) || scale <= 0) {
      return '1';
    }
    if (scale >= 100) {
      return scale.toFixed(0);
    }
    if (scale >= 10) {
      return parseFloat(scale.toFixed(1)).toString();
    }
    const rounded = parseFloat(scale.toFixed(2));
    return Number.isInteger(rounded) ? String(rounded) : rounded.toString();
  }

  function showSpeedStatus(scale) {
    if (!speedStatusEl) {
      return;
    }
    speedStatusEl.textContent = `current speed: ${formatTimeScale(scale)}x realtime`;
    speedStatusEl.classList.remove('sim-hidden');
  }

  function applyTimeScaleToWorkers(scale) {
    const safeScale = Number.isFinite(scale) && scale > 0 ? scale : 1.0;
    if (klipperCommanderWorker) {
      klipperCommanderWorker.postMessage({ type: 'set_speed_scale', value: safeScale });
    }
    if (moveCommanderWorker) {
      moveCommanderWorker.postMessage({ type: 'set_speed_scale', value: safeScale });
    }
    if (rrfCommanderWorker) {
      rrfCommanderWorker.postMessage({ type: 'set_speed_scale', value: safeScale });
    }
  }

  function handleTimeScaleChange(scale) {
    currentTimeScale = scale;
    applyTimeScaleToWorkers(scale);
    if (speedStatusArmed) {
      showSpeedStatus(scale);
    }
  }

  function applyViewStateFromController(partial = {}, options = {}) {
    if (machines.length === 0) {
      updateZoomButtonState();
      return;
    }
    if (typeof partial.scale === 'number') {
      currentViewScale = clamp(partial.scale, MIN_VIEW_SCALE, MAX_VIEW_SCALE);
    }
    if (typeof partial.offsetX === 'number') {
      currentViewOffsetX = partial.offsetX;
    }
    if (typeof partial.offsetY === 'number') {
      currentViewOffsetY = partial.offsetY;
    }

    const inputSystem = getInputSystem();
    if (inputSystem && typeof inputSystem.setViewTransform === 'function') {
      inputSystem.setViewTransform({
        scaleMultiplier: currentViewScale,
        offsetX: currentViewOffsetX,
        offsetY: currentViewOffsetY,
      });
    }

    syncRenderSystem(
      {
        scaleMultiplier: currentViewScale,
        offsetX: currentViewOffsetX,
        offsetY: currentViewOffsetY,
      },
      options
    );

    updateZoomButtonState();
  }

  function handleInputViewChange(viewState = {}, options = {}) {
    // Derive proposed new view state, compute deltas
    const nextScale = typeof viewState.scale === 'number' ? clamp(viewState.scale, MIN_VIEW_SCALE, MAX_VIEW_SCALE) : currentViewScale;
    const nextOffsetX = typeof viewState.offsetX === 'number' ? viewState.offsetX : currentViewOffsetX;
    const nextOffsetY = typeof viewState.offsetY === 'number' ? viewState.offsetY : currentViewOffsetY;

    const forceRedraw = options?.forceRedraw || false;
    const scaleChanged = Math.abs(nextScale - currentViewScale) > ZOOM_EPSILON;
    const dOffsetX = nextOffsetX - currentViewOffsetX;
    const dOffsetY = nextOffsetY - currentViewOffsetY;

    // Update internal state first
    currentViewScale = nextScale;
    currentViewOffsetX = nextOffsetX;
    currentViewOffsetY = nextOffsetY;

    const renderSystem = world.getResource('renderSystem');

    if (!scaleChanged && !forceRedraw && renderSystem && typeof renderSystem.shiftExtrusionsForPan === 'function') {
      // Fast path for pure panning: update transform without clearing and shift cached extrusions
      syncRenderSystem(
        {
          scaleMultiplier: currentViewScale,
          offsetX: currentViewOffsetX,
          offsetY: currentViewOffsetY,
        },
        { clearExtrusions: false }
      );
      // Shift the cached extrusions layer and backfill newly exposed strips
      try {
        renderSystem.shiftExtrusionsForPan(world, dOffsetX, dOffsetY);
      } catch (_) {
        // Fallback safety: if shifting fails for any reason, clear to stay correct
        renderSystem.clearExtrusions?.();
        renderSystem.clearPositionTrace?.({ keepMarkers: true });
      }
    } else {
      // Zoom changed or renderer missing: use safe path (clear and redraw as before)
      syncRenderSystem(
        {
          scaleMultiplier: currentViewScale,
          offsetX: currentViewOffsetX,
          offsetY: currentViewOffsetY,
        },
        { clearExtrusions: true }
      );
    }

    updateZoomButtonState();
  }

  function attachInputViewListener() {
    const inputSystem = getInputSystem();
    if (!inputSystem || typeof inputSystem.setViewChangeListener !== 'function') {
      return;
    }
    if (viewListenerSystem && viewListenerSystem !== inputSystem && typeof viewListenerSystem.setViewChangeListener === 'function') {
      viewListenerSystem.setViewChangeListener(null);
    }
    if (viewListenerSystem !== inputSystem) {
      inputSystem.setViewChangeListener(handleInputViewChange);
      viewListenerSystem = inputSystem;
    }
  }

  function syncCanvasDimensions() {
    if (!canvas) {
      return;
    }
    const width = Math.max(1, Math.floor(canvas.clientWidth));
    const height = Math.max(1, Math.floor(canvas.clientHeight));
    if (width <= 0 || height <= 0) {
      return;
    }
    const resized = canvas.width !== width || canvas.height !== height;
    if (resized) {
      canvas.width = width;
      canvas.height = height;
    }
    const renderSystem = world.getResource('renderSystem');
    if (renderSystem && typeof renderSystem.setCanvasSize === 'function') {
      renderSystem.setCanvasSize(canvas.width, canvas.height);
    }
    if (resized) {
      reapplyViewState({ clearExtrusions: true });
    }
  }

  function reapplyViewState(options = {}) {
    if (machines.length === 0) {
      return;
    }
    attachInputViewListener();
    applyViewStateFromController(
      {
        scale: currentViewScale,
        offsetX: currentViewOffsetX,
        offsetY: currentViewOffsetY,
      },
      options
    );
    const inputSystem = getInputSystem();
    if (inputSystem && typeof inputSystem.setInteractionMode === 'function') {
      inputSystem.setInteractionMode(panModeActive ? 'pan' : 'select');
    }
  }

  function resetViewStateDefaults() {
    currentViewScale = DEFAULT_VIEW_SCALE;
    currentViewOffsetX = 0;
    currentViewOffsetY = 0;
  }

  function handleFullscreenChange() {
    const fullscreenElement =
      document.fullscreenElement ||
      document.webkitFullscreenElement ||
      document.msFullscreenElement ||
      null;
    const isActive = fullscreenElement === simApp;
    const wasActive = fullscreenActive;
    fullscreenActive = isActive;
    if (simApp) {
      simApp.classList.toggle('is-fullscreen', isActive);
    }
    if (fullscreenBtn) {
      fullscreenBtn.textContent = isActive ? 'Exit Fullscreen' : 'Fullscreen';
      fullscreenBtn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
      if (fullscreenBtn.disabled) {
        fullscreenBtn.disabled = false;
      }
      fullscreenBtn.removeAttribute('disabled');
      fullscreenBtn.removeAttribute('aria-disabled');
    }
    if (wasActive !== isActive) {
      const fullscreenZoomFactor = isActive ? 0.5 : 2.0;
      adjustZoom(fullscreenZoomFactor);
    }
    requestAnimationFrame(() => {
      syncCanvasDimensions();
    });
  }

  function toggleFullscreen() {
    if (!simApp) {
      return;
    }
    const fullscreenElement =
      document.fullscreenElement ||
      document.webkitFullscreenElement ||
      document.msFullscreenElement ||
      null;
    const isActive = fullscreenElement === simApp;
    const requestFullscreen =
      simApp.requestFullscreen ||
      simApp.webkitRequestFullscreen ||
      simApp.msRequestFullscreen;
    const exitFullscreen =
      document.exitFullscreen ||
      document.webkitExitFullscreen ||
      document.msExitFullscreen;

    if (!isActive) {
      if (requestFullscreen) {
        try {
          const result = requestFullscreen.call(simApp);
          if (result && typeof result.catch === 'function') {
            result.catch((error) => {
              console.warn('Slideprinter demo: unable to enter fullscreen.', error);
            });
          }
        } catch (error) {
          console.warn('Slideprinter demo: unable to enter fullscreen.', error);
        }
      }
      return;
    }

    if (exitFullscreen) {
      try {
        const result = exitFullscreen.call(document);
        if (result && typeof result.catch === 'function') {
          result.catch(() => {});
        }
      } catch (error) {
        console.warn('Slideprinter demo: unable to exit fullscreen.', error);
      }
    }
  }

  function setPanMode(active) {
    panModeActive = Boolean(active);
    if (panModeBtn) {
      panModeBtn.setAttribute('aria-pressed', panModeActive ? 'true' : 'false');
      panModeBtn.classList.toggle('is-active', panModeActive);
    }
    if (canvas) {
      canvas.style.touchAction = panModeActive ? 'none' : initialTouchAction;
    }
    // Show a hand cursor over the sim-app when panning is enabled
    if (simApp) {
      // Use 'grab' which renders as a hand on modern browsers
      simApp.style.cursor = panModeActive ? 'grab' : '';
    }
    const inputSystem = getInputSystem();
    if (inputSystem && typeof inputSystem.setInteractionMode === 'function') {
      inputSystem.setInteractionMode(panModeActive ? 'pan' : 'select');
    }
  }

  function getCanvasBaseScale() {
    if (!canvas) {
      return null;
    }
    const simHeight = world.getResource('simHeight');
    if (!Number.isFinite(simHeight) || simHeight <= 0) {
      return null;
    }
    return canvas.height / simHeight;
  }

  function applyZoomAtScale(targetScale, anchor = null) {
    if (!stageReady || machines.length === 0) {
      return;
    }
    const clampedScale = clamp(targetScale, MIN_VIEW_SCALE, MAX_VIEW_SCALE);
    if (Math.abs(clampedScale - currentViewScale) < ZOOM_EPSILON) {
      return;
    }
    let nextOffsetX = currentViewOffsetX;
    let nextOffsetY = currentViewOffsetY;
    if (anchor && canvas) {
      const baseScale = getCanvasBaseScale();
      if (baseScale && baseScale > 0) {
        const rect = canvas.getBoundingClientRect();
        const pixelX = anchor.x - rect.left;
        const pixelY = anchor.y - rect.top;
        const prevScale = baseScale * currentViewScale;
        if (prevScale > 0) {
          const worldX = (pixelX - canvas.width / 2) / prevScale + currentViewOffsetX;
          const worldY = (canvas.height / 2 - pixelY) / prevScale + currentViewOffsetY;
          const nextScale = baseScale * clampedScale;
          if (nextScale > 0) {
            nextOffsetX = worldX - (pixelX - canvas.width / 2) / nextScale;
            nextOffsetY = worldY - (canvas.height / 2 - pixelY) / nextScale;
          }
        }
      }
    }
    applyViewStateFromController(
      {
        scale: clampedScale,
        offsetX: nextOffsetX,
        offsetY: nextOffsetY,
      },
      { clearExtrusions: true }
    );
  }

  function normalizeWheelDelta(delta, deltaMode) {
    if (!Number.isFinite(delta)) {
      return 0;
    }
    if (deltaMode === 1) {
      return delta * 40;
    }
    if (deltaMode === 2) {
      return delta * 800;
    }
    return delta;
  }

  function handleCanvasWheel(event) {
    if (!stageReady || machines.length === 0) {
      return;
    }
    event.preventDefault();
    const normalized = normalizeWheelDelta(event.deltaY, event.deltaMode);
    if (normalized === 0) {
      return;
    }
    const intensity = Math.min(4, Math.max(0.05, Math.abs(normalized) / 240));
    const factor = Math.pow(ZOOM_FACTOR, intensity);
    const multiplier = normalized < 0 ? factor : 1 / factor;
    applyZoomAtScale(currentViewScale * multiplier, { x: event.clientX, y: event.clientY });
  }

  function adjustZoom(multiplier) {
    applyZoomAtScale(currentViewScale * multiplier);
  }

  function stopAndClearWorkers() {
    stopInactiveWorkers(null);
    resetRemoteQueue(null);
    if (moveCommanderWorker) {
      try {
        moveCommanderWorker.terminate();
      } catch (err) {
        console.warn('Slideprinter demo: unable to terminate move worker cleanly.', err);
      }
      moveCommanderWorker = null;
    }
    if (klipperCommanderWorker) {
      try {
        klipperCommanderWorker.terminate();
      } catch (err) {
        console.warn('Slideprinter demo: unable to terminate klipper worker cleanly.', err);
      }
      klipperCommanderWorker = null;
    }
    if (rrfCommanderWorker) {
      try {
        rrfCommanderWorker.terminate();
      } catch (err) {
        console.warn('Slideprinter demo: unable to terminate rrf worker cleanly.', err);
      }
      rrfCommanderWorker = null;
    }
  }

  function handleUserReset() {
    if (!gameControls || typeof gameControls.reset !== 'function') {
      return;
    }
    if (machines.length === 0) {
      return;
    }
    if (asapState.active) {
      console.warn('hp-sim: Reset ignored while Finish ASAP is active.');
      return;
    }
    setPrintActive(false);
    stopAndClearWorkers();
    resetJobTracking();
    gameControls.reset({ autoPause: true });
    if (typeof gameControls.setTimeScale === 'function') {
      gameControls.setTimeScale(1.0);
    } else {
      handleTimeScaleChange(1.0);
    }
    resetViewStateDefaults();
    setPanMode(isMobileLayout());
    reapplyViewState({ clearExtrusions: true });
    resetQualityMonitors({ keepReference: true });
    setReferenceVisibility(false);
    currentPresetKey = DEFAULT_PRESET_KEY;
  }

  function waitForAnimationFrames(count = 1) {
    const target = Math.max(0, Math.floor(count));
    if (target <= 0) {
      return Promise.resolve();
    }
    return new Promise((resolve) => {
      let remaining = target;
      const step = () => {
        remaining -= 1;
        if (remaining <= 0) {
          resolve();
          return;
        }
        requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    });
  }

  async function runAsapFastForward(remoteSystem) {
    if (!remoteSystem) {
      return;
    }

    const checkIntervalMs = 25;
    const maxIdleChecks = 10;
    const maxDurationMs = 10 * 60 * 1000;
    const startTime = performance.now();
    let idleChecks = 0;

    const queueLengthAccessor = typeof remoteSystem.getQueueLength === 'function'
      ? () => remoteSystem.getQueueLength()
      : () => (Array.isArray(remoteSystem.commands) ? remoteSystem.commands.length : 0);

    while (asapState.active) {
      const queueLen = queueLengthAccessor();
      const printing = printActive;
      const workerActive = Boolean(remoteSystem.worker);

      if (!printing && queueLen === 0) {
        idleChecks += 1;
        if (idleChecks >= maxIdleChecks) {
          break;
        }
      } else {
        idleChecks = 0;
      }

      if (!workerActive && queueLen === 0) {
        break;
      }

      if (performance.now() - startTime > maxDurationMs) {
        console.warn('hp-sim: Finish ASAP exceeded expected duration; continuing to finalize.');
        break;
      }

      await new Promise((resolve) => setTimeout(resolve, checkIntervalMs));
    }

    await waitForAnimationFrames(8);
  }

  async function finalizeAsapMode() {
    const remoteSystem = getRemoteSystem();
    const renderSystem = world.getResource('renderSystem');
    const previousQualityEnabled = asapState.previousQualityEnabled;
    const previousQualityToggleDisabled = asapState.previousQualityToggleDisabled;
    const previousQualityToggleChecked = asapState.previousQualityToggleChecked;
    const previousPauseState = asapState.previousPauseState;
    const previousTimeScale = asapState.previousTimeScale;
    const prevPauseDisabled = asapState.prevPauseDisabled;
    const prevResetDisabled = asapState.prevResetDisabled;

    if (typeof gameControls?.setRenderEveryNth === 'function') {
      gameControls.setRenderEveryNth(null);
    }

    if (remoteSystem && remoteSystem.worker) {
      try {
        remoteSystem.worker.postMessage({ type: 'set_asap_mode', enable: false });
      } catch (_err) {
        /* noop */
      }
    }

    if (asapState.renderSuspended && renderSystem && typeof renderSystem.setDrawingSuspended === 'function') {
      renderSystem.setDrawingSuspended(false);
      if (typeof renderSystem.update === 'function') {
        renderSystem.update(world, 0);
      }
    }

    const pauseState = world.getResource('pauseState');
    if (pauseState != null && typeof previousPauseState === 'boolean') {
      pauseState.paused = previousPauseState;
    }

    if (pauseBtn && prevPauseDisabled !== null) {
      setButtonDisabled(pauseBtn, prevPauseDisabled);
    }
    if (resetBtn && prevResetDisabled !== null) {
      setButtonDisabled(resetBtn, prevResetDisabled);
    }

    if (qualityToggle) {
      qualityToggle.disabled = Boolean(previousQualityToggleDisabled);
      if (qualityToggle.disabled) {
        qualityToggle.setAttribute('aria-disabled', 'true');
      } else {
        qualityToggle.removeAttribute('aria-disabled');
      }
      if (previousQualityToggleChecked != null) {
        qualityToggle.checked = previousQualityToggleChecked;
      }
    }

    if (previousQualityEnabled !== null) {
      setQualityEnabledState(previousQualityEnabled);
      if (previousQualityEnabled) {
        refreshAllQualityMonitors(true);
      } else {
        updateQualityHudVisibility();
      }
    } else {
      updateQualityHudVisibility();
    }

    if (previousTimeScale != null) {
      if (gameControls && typeof gameControls.setTimeScale === 'function') {
        speedStatusArmed = false;
        gameControls.setTimeScale(previousTimeScale);
      } else {
        handleTimeScaleChange(previousTimeScale);
      }
    }

    hideAsapStatus();

    asapState.active = false;
    asapState.finishingPromise = null;
    asapState.previousTimeScale = null;
    asapState.previousQualityEnabled = null;
    asapState.previousQualityToggleDisabled = null;
    asapState.previousQualityToggleChecked = null;
    asapState.previousPauseState = null;
    asapState.prevPauseDisabled = null;
    asapState.prevResetDisabled = null;
    asapState.renderSuspended = false;

    const shouldRunFinalCheck = asapState.pendingFinalCheck;
    asapState.pendingFinalCheck = false;

    if (shouldRunFinalCheck) {
      runFinalQualityChecks();
    }

    updateFinishAsapButtonState();
  }

  async function triggerFinishAsap() {
    if (asapState.active) {
      return asapState.finishingPromise;
    }
    if (!printActive) {
      showAsapStatus('Finish ASAP is available only while printing.');
      setTimeout(() => {
        if (!asapState.active) {
          hideAsapStatus();
        }
      }, 1500);
      return null;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      console.warn('hp-sim: Finish ASAP requested but remote system is unavailable.');
      return null;
    }

    asapState.active = true;
    asapState.pendingFinalCheck = false;
    asapState.previousTimeScale =
      typeof gameControls?.getTimeScale === 'function'
        ? gameControls.getTimeScale()
        : currentTimeScale;
    asapState.previousQualityEnabled = qualityEnabled;
    asapState.previousQualityToggleDisabled = qualityToggle ? qualityToggle.disabled : null;
    asapState.previousQualityToggleChecked = qualityToggle ? qualityToggle.checked : null;
    const pauseState = world.getResource('pauseState');
    asapState.previousPauseState = pauseState ? pauseState.paused : null;
    asapState.prevPauseDisabled = pauseBtn ? pauseBtn.disabled : null;
    asapState.prevResetDisabled = resetBtn ? resetBtn.disabled : null;

    if (pauseState) {
      pauseState.paused = false;
    }

    setButtonDisabled(pauseBtn, true);
    setButtonDisabled(resetBtn, true);

    hideReplayStatus();
    showAsapStatus();

    if (qualityEnabled) {
      setQualityEnabledState(false);
    } else {
      updateQualityHudVisibility();
    }
    if (qualityToggle) {
      qualityToggle.checked = false;
      qualityToggle.disabled = true;
      qualityToggle.setAttribute('aria-disabled', 'true');
    }

    const renderSystem = world.getResource('renderSystem');
    if (renderSystem && typeof renderSystem.setDrawingSuspended === 'function') {
      renderSystem.setDrawingSuspended(true);
      asapState.renderSuspended = true;
    } else {
      asapState.renderSuspended = false;
    }

    const previousScale = Math.max(1, asapState.previousTimeScale || 1);
    const asapScale = Math.min(512, Math.max(256, previousScale * 8));
    speedStatusArmed = false;
    if (gameControls && typeof gameControls.setTimeScale === 'function') {
      gameControls.setTimeScale(asapScale);
    } else {
      handleTimeScaleChange(asapScale);
    }

    if (typeof gameControls?.setRenderEveryNth === 'function') {
      gameControls.setRenderEveryNth(Number.POSITIVE_INFINITY);
    }

    if (remoteSystem.worker) {
      try {
        remoteSystem.worker.postMessage({ type: 'set_asap_mode', enable: true });
      } catch (err) {
        console.warn('hp-sim: unable to enable ASAP mode for worker.', err);
      }
    }

    updateFinishAsapButtonState();

    asapState.finishingPromise = (async () => {
      try {
        await runAsapFastForward(remoteSystem);
      } catch (error) {
        console.error('hp-sim: Finish ASAP failed.', error);
      } finally {
        await finalizeAsapMode();
      }
    })();

    return asapState.finishingPromise;
  }

  function ensureKlipperWorker() {
    if (klipperCommanderWorker) {
      return klipperCommanderWorker;
    }
    klipperCommanderWorker = new Worker(klipperCommanderModuleUrl, { type: 'module' });
    klipperCommanderWorker.onerror = (event) => {
      console.error('Slideprinter demo: Klipper worker failed.', event);
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === klipperCommanderWorker) {
        remoteSystem.worker = null;
        setPrintActive(false);
      }
    };
    klipperCommanderWorker.onmessageerror = (event) => {
      console.error('Slideprinter demo: Klipper worker message decode failed.', event);
    };
    klipperCommanderWorker.onmessage = (event) => {
      if (!event?.data) {
        return;
      }
      if (event.data.type === 'done') {
        console.log('Slideprinter demo: MCU log playback finished.');
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === klipperCommanderWorker) {
          remoteSystem.worker = null;
          setPrintActive(false);
          if (asapState.active) {
            asapState.pendingFinalCheck = true;
          } else {
            runFinalQualityChecks();
          }
        }
        return;
      }
      if (event.data.type === 'error') {
        console.error('Slideprinter demo: Worker reported an error:', event.data.message);
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === klipperCommanderWorker) {
          remoteSystem.worker = null;
          setPrintActive(false);
        }
        return;
      }
      if (event.data.action === 'gcode') {
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === klipperCommanderWorker) {
          remoteSystem.addCommand(event.data.command);
        }
      }
    };
    if (simDtSec != null) {
      klipperCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    klipperCommanderWorker.postMessage({ type: 'set_speed_scale', value: currentTimeScale });
    return klipperCommanderWorker;
  }

  function ensureRrfWorker() {
    if (rrfCommanderWorker) {
      return rrfCommanderWorker;
    }
    rrfCommanderWorker = new Worker(rrfCommanderModuleUrl, { type: 'module' });
    rrfCommanderWorker.onerror = (event) => {
      console.error('Slideprinter demo: RRF worker failed.', event);
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === rrfCommanderWorker) {
        remoteSystem.worker = null;
        setPrintActive(false);
      }
    };
    rrfCommanderWorker.onmessageerror = (event) => {
      console.error('Slideprinter demo: RRF worker message decode failed.', event);
    };
    rrfCommanderWorker.onmessage = (event) => {
      if (!event?.data) {
        return;
      }
      if (event.data.type === 'done') {
        console.log('Slideprinter demo: RRF log playback finished.');
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === rrfCommanderWorker) {
          remoteSystem.worker = null;
          setPrintActive(false);
          if (asapState.active) {
            asapState.pendingFinalCheck = true;
          } else {
            runFinalQualityChecks();
          }
        }
        return;
      }
      if (event.data.type === 'error') {
        console.error('Slideprinter demo: Worker reported an error:', event.data.message);
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === rrfCommanderWorker) {
          remoteSystem.worker = null;
          setPrintActive(false);
        }
        return;
      }
      if (event.data.action === 'gcode') {
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === rrfCommanderWorker) {
          remoteSystem.addCommand(event.data.command);
        }
      }
    };
    if (simDtSec != null) {
      rrfCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    rrfCommanderWorker.postMessage({ type: 'set_speed_scale', value: currentTimeScale });
    return rrfCommanderWorker;
  }

  function ensureMoveWorker() {
    if (moveCommanderWorker) {
      return moveCommanderWorker;
    }
    moveCommanderWorker = new Worker(moveCommanderModuleUrl, { type: 'module' });
    moveCommanderWorker.onerror = (event) => {
      console.error('Slideprinter demo: G-code worker failed.', event);
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === moveCommanderWorker) {
        remoteSystem.worker = null;
        setPrintActive(false);
      }
    };
    moveCommanderWorker.onmessageerror = (event) => {
      console.error('Slideprinter demo: G-code worker message decode failed.', event);
    };
    moveCommanderWorker.onmessage = (event) => {
      if (!event?.data) {
        return;
      }
      if (event.data.type === 'done') {
        console.log('Slideprinter demo: G-code playback finished.');
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === moveCommanderWorker) {
          remoteSystem.worker = null;
          setPrintActive(false);
          if (asapState.active) {
            asapState.pendingFinalCheck = true;
          } else {
            runFinalQualityChecks();
          }
        }
        return;
      }
      if (event.data.type === 'error') {
        console.error('Slideprinter demo: Worker reported an error:', event.data.message);
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === moveCommanderWorker) {
          remoteSystem.worker = null;
          setPrintActive(false);
        }
        return;
      }
      if (event.data.action === 'gcode') {
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === moveCommanderWorker) {
          remoteSystem.addCommand(event.data.command);
        }
      }
    };
    if (simDtSec != null) {
      moveCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    moveCommanderWorker.postMessage({ type: 'set_speed_scale', value: currentTimeScale });
    return moveCommanderWorker;
  }

  function stopInactiveWorkers(activeWorker) {
    if (moveCommanderWorker && moveCommanderWorker !== activeWorker) {
      moveCommanderWorker.postMessage({ type: 'pause' });
    }
    if (klipperCommanderWorker && klipperCommanderWorker !== activeWorker) {
      klipperCommanderWorker.postMessage({ type: 'pause' });
    }
    if (rrfCommanderWorker && rrfCommanderWorker !== activeWorker) {
      rrfCommanderWorker.postMessage({ type: 'pause' });
    }
  }

  function resetRemoteQueue(activeWorker) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    if (typeof remoteSystem.clearPlaybackState === 'function') {
      remoteSystem.clearPlaybackState();
    } else {
      if (typeof remoteSystem.clearCommandQueue === 'function') {
        remoteSystem.clearCommandQueue();
      } else if (Array.isArray(remoteSystem.commands)) {
        remoteSystem.commands.length = 0;
      }
      remoteSystem.history = [];
    }
    remoteSystem.worker = activeWorker;
    remoteSystem.wasPaused = false;
  }

  function startSimulationWithWorker(worker, jobDescriptor = null) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return false;
    }
    stopInactiveWorkers(worker);
    resetRemoteQueue(worker);
    if (worker) {
      // Workers can remain paused after being sidelined for another format (e.g. RRF -> MCU),
      // so make sure the active worker is unpaused before starting a new job.
      try {
        worker.postMessage({ type: 'resume' });
      } catch (err) {
        console.warn('hp-sim: unable to resume worker before starting job.', err);
      }
      worker.postMessage({ type: 'set_speed_scale', value: currentTimeScale });
    }
    if (gameControls && typeof gameControls.reset === 'function') {
      gameControls.reset({ autoPause: false });
      reapplyViewState({ clearExtrusions: true });
    }
    resetQualityMonitors({ keepReference: true });
    setPrintActive(true);
    hideReplayStatus();
    hidePrintStatus();
    sceneChangeState.context = null;
    sceneChangeState.pausedForSceneChange = false;
    resetJobTracking();
    beginNewJob(jobDescriptor);
    return true;
  }

  function playPreset(presetKey) {
    ensureReadyForNewJob();
    if (!stageReady) {
      return;
    }
    const preset = resolvePresetCommand(presetKey, lineLayeringEnabled);
    if (!preset || !preset.url) {
      console.warn('Slideprinter demo: unknown preset', presetKey);
      return;
    }
    const referencePresetKey = preset.referencePresetKey || presetKey;
    loadReferencePathForPreset(referencePresetKey, { setActive: true }).catch((error) => {
      console.warn('hp-sim: failed to prepare reference path for preset', referencePresetKey, error);
    });
    const format = preset.format || detectFileFormat(preset.url);
    let worker = null;
    if (format === FileFormat.GCODE) {
      worker = ensureMoveWorker();
    } else if (isMcuFormat(format)) {
      worker = ensureKlipperWorker();
    } else if (isRrfFormat(format)) {
      worker = ensureRrfWorker();
    } else {
      console.warn('Slideprinter demo: unsupported preset format', format);
      return;
    }
    currentPresetKey = referencePresetKey;
    const jobDescriptor = {
      type: 'preset',
      key: presetKey,
      label: PRESET_GCODE_MAP[referencePresetKey]?.label || presetKey,
    };
    if (!startSimulationWithWorker(worker, jobDescriptor)) {
      return;
    }
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    worker.postMessage({ type: 'filename_fetch', filename: preset.url });
  }

  async function queueUploadedFile(file, formatOverride = null) {
    if (!file) {
      return;
    }
    const format = formatOverride ?? detectFileFormat(file.name);
    if (format === FileFormat.USD_STAGE) {
      ensureReadyForNewJob();
      await addUsdMachineFromFile(file);
      return;
    }

    if (!stageReady) {
      console.warn('hp-sim: unable to queue print job because no scene is loaded.');
      return;
    }

    ensureReadyForNewJob();
    let worker = null;
    if (format === FileFormat.GCODE) {
      worker = ensureMoveWorker();
    } else if (isMcuFormat(format)) {
      worker = ensureKlipperWorker();
    } else if (isRrfFormat(format)) {
      worker = ensureRrfWorker();
    } else {
      console.warn('Slideprinter demo: unsupported upload format', file?.name || 'unknown');
      return;
    }
    const jobDescriptor = {
      type: 'upload',
      name: file?.name || 'Uploaded File',
      label: file?.name || 'Uploaded File',
      format,
    };
    if (!startSimulationWithWorker(worker, jobDescriptor)) {
      return;
    }
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    worker.postMessage({ type: 'filename_upload', filename: file });
  }

  if (printLogoBtn) {
    printLogoBtn.addEventListener('click', () => playPreset('hangprinterLogo'));
  }

  if (printSquareBtn) {
    printSquareBtn.addEventListener('click', () => playPreset('straightMoves'));
  }

  if (referenceToggleBtn) {
    referenceToggleBtn.addEventListener('click', () => {
      if (!Array.isArray(referenceOverlayState.segments) || referenceOverlayState.segments.length === 0) {
        return;
      }
      setReferenceVisibility(!referenceOverlayState.visible);
    });
    updateReferenceToggleUI();
  } else {
    updateReferenceToggleUI();
  }

  if (uploadBtn && gcodeInput) {
    uploadBtn.addEventListener('click', () => gcodeInput.click());
    gcodeInput.addEventListener('change', (event) => {
      const file = event.target.files?.[0];
      if (file) {
        handleFileUpload(file).catch((error) => {
          console.error('Slideprinter demo: upload handling failed.', error);
        });
        gcodeInput.value = '';
      }
    });
  }

  buildPresetMachineOptions();
  updateMachineMenuUI();

  if (machinesMenu) {
    machinesMenu.setAttribute('tabindex', '-1');
    machinesMenu.addEventListener('mouseenter', handleMachineMenuMouseEnter);
    machinesMenu.addEventListener('mouseleave', handleMachineMenuMouseLeave);
    machinesMenu.addEventListener('focusin', handleMachineMenuFocusIn);
    machinesMenu.addEventListener('focusout', handleMachineMenuFocusOut);
  }

  if (machinesToggle) {
    machinesToggle.addEventListener('click', (event) => {
      event.preventDefault();
      if (machineMenuOpen) {
        closeMachineMenu();
      } else {
        openMachineMenu();
      }
    });
    machinesToggle.addEventListener('mouseenter', handleMachineMenuMouseEnter);
    machinesToggle.addEventListener('mouseleave', handleMachineMenuMouseLeave);
    machinesToggle.addEventListener('focusin', handleMachineMenuFocusIn);
    machinesToggle.addEventListener('focusout', handleMachineMenuFocusOut);
  }

  if (presetMachinesList) {
    presetMachinesList.addEventListener('change', (event) => {
      const target = event.target;
      if (!(target instanceof HTMLInputElement) || target.type !== 'checkbox') {
        return;
      }
      const sourceKey = target.dataset?.sourceKey;
      if (!sourceKey) {
        return;
      }
      const shouldLoad = target.checked;
      if (shouldLoad) {
        target.disabled = true;
        addUsdaFromCatalog(sourceKey, { resetView: machines.length === 0 })
          .catch((error) => {
            console.error('hp-sim: failed to add USDA preset from catalog.', error);
            target.checked = false;
          })
          .finally(() => {
            target.disabled = false;
            updateMachineMenuUI();
          });
        return;
      }

      const machine = machines.find((entry) => entry.sourceKey === sourceKey);
      if (!machine) {
        updateMachineMenuUI();
        return;
      }
      void removeMachine(machine.id);
    });
  }

  if (customMachinesList) {
    customMachinesList.addEventListener('click', (event) => {
      const target = event.target;
      if (!(target instanceof Element)) {
        return;
      }
      const button = target.closest('button[data-machine-id]');
      if (!(button instanceof HTMLElement)) {
        return;
      }
      const machineId = button.dataset?.machineId;
      if (!machineId) {
        return;
      }
      event.preventDefault();
      void removeMachine(machineId);
    });
  }

  if (machinesRemoveAllBtn) {
    machinesRemoveAllBtn.addEventListener('click', (event) => {
      event.preventDefault();
      void removeAllMachines();
    });
  }

  if (canvas) {
    canvas.addEventListener('pointerdown', () => {
      if (isMobileLayout() && secondaryControlsUserPreference !== false) {
        showSecondaryControlsForMobile({ persist: secondaryControlsUserPreference === true });
      }
    });
    canvas.addEventListener('wheel', handleCanvasWheel, { passive: false });
    canvas.addEventListener('contextmenu', (event) => {
      const renderSystem = world.getResource('renderSystem');
      if (!renderSystem || !renderSystem.positionTraceEnabled) {
        return;
      }
      event.preventDefault();
      const nowMs = performance.now();
      if (positionTraceRightClickCount === 0 || nowMs - positionTraceFirstRightClickMs > 700) {
        positionTraceRightClickCount = 0;
        positionTraceFirstRightClickMs = nowMs;
        if (positionTraceDoubleClickTimer) {
          clearTimeout(positionTraceDoubleClickTimer);
          positionTraceDoubleClickTimer = null;
        }
      }

      positionTraceRightClickCount += 1;

      if (positionTraceRightClickCount === 2) {
        if (positionTraceDoubleClickTimer) {
          clearTimeout(positionTraceDoubleClickTimer);
        }
        positionTraceDoubleClickTimer = window.setTimeout(() => {
          if (positionTraceRightClickCount === 2) {
            renderSystem.clearPositionTraceMarkers?.();
            renderSystem.update?.(world, 0);
          }
          positionTraceRightClickCount = 0;
          positionTraceFirstRightClickMs = 0;
          positionTraceDoubleClickTimer = null;
        }, 350);
        return;
      }

      if (positionTraceRightClickCount === 3 && nowMs - positionTraceFirstRightClickMs <= 700) {
        if (positionTraceDoubleClickTimer) {
          clearTimeout(positionTraceDoubleClickTimer);
          positionTraceDoubleClickTimer = null;
        }
        positionTraceRightClickCount = 0;
        positionTraceFirstRightClickMs = 0;
        renderSystem.clearPositionTracePoints?.();
        renderSystem.clearPositionTraceMarkers?.();
        renderSystem.update?.(world, 0);
        return;
      }

      const rect = canvas.getBoundingClientRect();
      const px = event.clientX - rect.left;
      const py = event.clientY - rect.top;
      const projected = typeof renderSystem.projectCanvasToSim === 'function'
        ? renderSystem.projectCanvasToSim(px, py)
        : null;
      const simX = projected?.x ?? renderSystem.simXFromCanvas(px, py);
      const simY = projected?.y ?? renderSystem.simYFromCanvas(py, px);
      if (!Number.isFinite(simX) || !Number.isFinite(simY)) {
        return;
      }
      const mmX = simX / GCODE_MM_TO_SIM_SCALE;
      const mmY = simY / GCODE_MM_TO_SIM_SCALE;
      const label = `(${mmX.toFixed(2)}, ${mmY.toFixed(2)})`;
      renderSystem.addPositionTraceMarker?.(simX, simY, label);
      renderSystem.update?.(world, 0);
    });
  }

  if (secondaryControls) {
    secondaryControls.addEventListener('pointerdown', () => {
      if (isMobileLayout() && secondaryControlsUserPreference !== false) {
        showSecondaryControlsForMobile({ persist: secondaryControlsUserPreference === true });
      }
    });
  }

  if (secondaryToggleBtn) {
    secondaryToggleBtn.addEventListener('click', (event) => {
      event.preventDefault();
      if (event.altKey || event.metaKey) {
        secondaryControlsUserPreference = null;
      } else {
        const isCurrentlyVisible =
          secondaryControls instanceof HTMLElement ? !secondaryControls.classList.contains('sim-hidden') : computeSecondaryControlsDesired();
        secondaryControlsUserPreference = isCurrentlyVisible ? false : true;
      }
      applySecondaryControlsVisibility();
    });
  }

  if (qualityHistoryToggleBtn) {
    qualityHistoryToggleBtn.addEventListener('click', (event) => {
      event.preventDefault();
      if (qualityHistoryRecords.length === 0) {
        return;
      }
      qualityHistoryExpanded = !qualityHistoryExpanded;
      updateQualityHistoryUI();
    });
  }

  document.addEventListener('keydown', (event) => {
    if (event.defaultPrevented) {
      return;
    }
    if (event.repeat) {
      return;
    }
    const target = event.target;
    if (target) {
      const tagName = target.tagName ? target.tagName.toUpperCase() : '';
      const isEditable =
        target.isContentEditable ||
        tagName === 'INPUT' ||
        tagName === 'TEXTAREA' ||
        tagName === 'SELECT';
      if (isEditable) {
        return;
      }
    }
    let handled = false;
    const key = event.key;
    switch (key) {
      case 'r':
      case 'R':
        if (resetBtn && !resetBtn.disabled) {
          resetBtn.click();
          handled = true;
        }
        break;
      case ' ':
      case 'Spacebar':
        if (pauseBtn && !pauseBtn.disabled) {
          pauseBtn.click();
          handled = true;
        }
        break;
      case '>':
        if (speedFasterBtn && !speedFasterBtn.disabled) {
          speedFasterBtn.click();
          handled = true;
        }
        break;
      case '<':
        if (speedSlowerBtn && !speedSlowerBtn.disabled) {
          speedSlowerBtn.click();
          handled = true;
        }
        break;
      case 's':
      case 'S':
        if (referenceToggleBtn) {
          referenceToggleBtn.click();
          handled = true;
        }
        break;
      case 'q':
      case 'Q':
        if (qualityToggle && !qualityToggle.disabled) {
          qualityToggle.click();
          handled = true;
        }
        break;
      default:
        break;
    }
    if (handled) {
      event.preventDefault();
    }
  });

  setPrintActive(false);
  setSpeedButtonsEnabled(false);
  updateFinishAsapButtonState();
  updateSecondaryToggleButton();
  updateQualityHistoryUI();

  if (resetBtn) {
    resetBtn.addEventListener(
      'click',
      (event) => {
        event.preventDefault();
        event.stopImmediatePropagation();
        handleUserReset();
      },
      { capture: true }
    );
  }

  if (finishAsapBtn) {
    finishAsapBtn.addEventListener('click', (event) => {
      event.preventDefault();
      void triggerFinishAsap();
    });
  }

  if (positionTraceBtn) {
    positionTraceBtn.addEventListener('click', (event) => {
      event.preventDefault();
      const renderSystem = world.getResource('renderSystem');
      if (!renderSystem || typeof renderSystem.setPositionTraceEnabled !== 'function') {
        return;
      }
      const nextEnabled = !renderSystem.positionTraceEnabled;
      renderSystem.setPositionTraceEnabled(nextEnabled);
      renderSystem.update?.(world, 0);
      updatePositionTraceToggleUI();
    });
  }

  if (zoomInBtn) {
    zoomInBtn.addEventListener('click', (event) => {
      event.preventDefault();
      adjustZoom(ZOOM_FACTOR);
    });
  }

  if (zoomOutBtn) {
    zoomOutBtn.addEventListener('click', (event) => {
      event.preventDefault();
      adjustZoom(1 / ZOOM_FACTOR);
    });
  }

  if (panModeBtn) {
    panModeBtn.addEventListener('click', (event) => {
      event.preventDefault();
      setPanMode(!panModeActive);
    });
  }

  if (speedSlowerBtn) {
    speedSlowerBtn.addEventListener('click', (event) => {
      event.preventDefault();
      if (!stageReady || !gameControls || typeof gameControls.setTimeScale !== 'function') {
        return;
      }
      const currentScale = typeof gameControls.getTimeScale === 'function' ? gameControls.getTimeScale() : currentTimeScale;
      const nextScale = currentScale / 2;
      speedStatusArmed = true;
      gameControls.setTimeScale(nextScale);
      const appliedScale = typeof gameControls.getTimeScale === 'function' ? gameControls.getTimeScale() : nextScale;
      showSpeedStatus(appliedScale);
    });
  }

  if (speedFasterBtn) {
    speedFasterBtn.addEventListener('click', (event) => {
      event.preventDefault();
      if (!stageReady || !gameControls || typeof gameControls.setTimeScale !== 'function') {
        return;
      }
      const currentScale = typeof gameControls.getTimeScale === 'function' ? gameControls.getTimeScale() : currentTimeScale;
      const nextScale = currentScale * 2;
      speedStatusArmed = true;
      gameControls.setTimeScale(nextScale);
      const appliedScale = typeof gameControls.getTimeScale === 'function' ? gameControls.getTimeScale() : nextScale;
      showSpeedStatus(appliedScale);
    });
  }

  if (fullscreenBtn) {
    fullscreenBtn.addEventListener('click', (event) => {
      event.preventDefault();
      toggleFullscreen();
    });
  }

  document.addEventListener('fullscreenchange', handleFullscreenChange);
  document.addEventListener('webkitfullscreenchange', handleFullscreenChange);
  document.addEventListener('msfullscreenchange', handleFullscreenChange);
  window.addEventListener('resize', () => {
    syncCanvasDimensions();
    handleLayoutChange();
  });

  if (mobileLayoutQuery) {
    if (typeof mobileLayoutQuery.addEventListener === 'function') {
      mobileLayoutQuery.addEventListener('change', handleLayoutChange);
    } else if (typeof mobileLayoutQuery.addListener === 'function') {
      mobileLayoutQuery.addListener(handleLayoutChange);
    }
  }

  handleLayoutChange();

  updateZoomButtonState();
  connectExternalCommandStream();

  const bootstrapSimulation = async () => {
    if (!usdaCatalog.has(defaultUsdaKey)) {
      throw new Error(`hp-sim: default USDA '${defaultUsdaKey}' is not available.`);
    }
    await addUsdaFromCatalog(defaultUsdaKey, { resetView: true });

    const sceneInitializer = () => rebuildScene();
    gameControls = runGame(world, sceneInitializer, {
      initialTimeScale: currentTimeScale,
      onTimeScaleChange: handleTimeScaleChange,
    });
    if (pauseBtn) {
      pauseBtn.addEventListener('click', () => {
        setTimeout(() => {
          const pauseState = world.getResource('pauseState');
          if (
            sceneChangeState.pausedForSceneChange &&
            pauseState &&
            !pauseState.paused &&
            !sceneChangeState.replayInProgress
          ) {
            hidePrintStatus();
            sceneChangeState.pausedForSceneChange = false;
            try {
              if (moveCommanderWorker) {
                moveCommanderWorker.postMessage({ type: 'resume' });
              }
              if (klipperCommanderWorker) {
                klipperCommanderWorker.postMessage({ type: 'resume' });
              }
              if (rrfCommanderWorker) {
                rrfCommanderWorker.postMessage({ type: 'resume' });
              }
            } catch (err) {
              console.warn('hp-sim: unable to resume workers after user request.', err);
            }
          }
        }, 0);
      });
    }
    if (gameControls && typeof gameControls.reset === 'function') {
      gameControls.reset({ autoPause: true });
    }
    resetViewStateDefaults();
    reapplyViewState({ clearExtrusions: true });
    setPanMode(isMobileLayout());
    syncCanvasDimensions();
  };

  bootstrapSimulation().catch((error) => {
    console.error('hp-sim bootstrap failed:', error);
    setPrintActive(false);
    controlsRoot.innerHTML = '<p class="sim-error">Unable to start the simulation. Please reload the page.</p>';
  });
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initHpSim, { once: true });
} else {
  initHpSim();
}
