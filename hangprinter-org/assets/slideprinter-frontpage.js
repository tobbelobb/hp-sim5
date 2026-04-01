import { Open as UsdOpen, getAttribute } from '../../src/js/usd/stage.js';
import { World } from '../../src/js/cable_joints/ecs.js';
import { runGame } from '../../examples/js/slideprinter/runner.js';
import { setupScene } from '../../examples/js/slideprinter/setupScene.js';
import { RemoteSpoolSystem, InputSystem } from '../../examples/js/slideprinter/slideprinter_common.js';
import { detectFileFormat, FileFormat, isMcuFormat } from '../../integrations/rrf/fileFormatUtils.js';

const MCU_PRESETS = {
  hangprinterLogo: {
    url: new URL('../../public/examples/mcu_commands/Hangprinter_logo6.serial', import.meta.url).href,
    format: FileFormat.MCU_SERIAL,
  },
  straightMoves: {
    url: new URL('../../public/examples/mcu_commands/draw_squares.serial', import.meta.url).href,
    format: FileFormat.MCU_SERIAL,
  },
};

const DEFAULT_PRESET_KEY = 'hangprinterLogo';
const DEFAULT_VIEW_SCALE = 1.8;
const MIN_VIEW_SCALE = 0.01;
const MAX_VIEW_SCALE = 100;
const ZOOM_FACTOR = 1.2;
const ZOOM_EPSILON = 1e-3;

function initFrontpageSlideprinter() {
  const canvas = document.getElementById('myCanvas');
  const controlsRoot = document.getElementById('controls');
  if (!canvas || !controlsRoot) {
    return;
  }

  const printLogoBtn = document.getElementById('printLogoBtn');
  const printSquareBtn = document.getElementById('printSquareBtn');
  const uploadBtn = document.getElementById('uploadBtn');
  const gcodeInput = document.getElementById('gcodeFile');
  const resetBtn = document.getElementById('resetBtn');
  const zoomInBtn = document.getElementById('zoomInBtn');
  const zoomOutBtn = document.getElementById('zoomOutBtn');
  const panModeBtn = document.getElementById('panModeBtn');
  const speedSlowerBtn = document.getElementById('speedSlowerBtn');
  const speedFasterBtn = document.getElementById('speedFasterBtn');
  const secondaryControls = document.getElementById('simSecondaryControls');
  const fullscreenBtn = document.getElementById('fullscreenBtn');
  const speedStatusEl = document.getElementById('speedStatus');
  const simApp = canvas.closest('.sim-app');
  const initialTouchAction = canvas ? canvas.style.touchAction || '' : '';
  const simButtons = controlsRoot.querySelector('.sim-buttons');
  const startButtons = simButtons ? Array.from(simButtons.querySelectorAll('.sim-start')) : [];

  const world = new World();
  const machines = [];
  let klipperCommanderWorker = null;
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

  const klipperCommanderModuleUrl = new URL('../../examples/js/slideprinter/klipperCommander.js', import.meta.url);
  const moveCommanderModuleUrl = new URL('../../examples/js/slideprinter/moveCommander.js', import.meta.url);
  const usdaUrl = new URL('../../examples/usd_scenes/slideprinter.usda', import.meta.url);

  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
  }

  function getInputSystem() {
    return world.systems.find((sys) => sys instanceof InputSystem) || null;
  }

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
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

  function registerMachine(stage, { name = null } = {}) {
    if (!stage) {
      return null;
    }
    const machineId = `machine-${machines.length}`;
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
    };
    machines.push(machine);
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

    registerMachine(stage, { name: label });

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

    rebuildScene();
    syncCanvasDimensions();
    updateZoomButtonState();
    reapplyViewState({ clearExtrusions: true });
  }

  function setSecondaryControlsVisible(active) {
    if (!secondaryControls) {
      return;
    }
    if (active) {
      secondaryControls.classList.remove('sim-hidden');
      secondaryControlsEverShown = true;
      return;
    }
    if (!secondaryControlsEverShown) {
      secondaryControls.classList.add('sim-hidden');
    }
  }

  function updateMainButtonsState() {
    if (simButtons) {
      simButtons.classList.toggle('is-printing', printActive);
    }
  }

  function setPrintActive(active) {
    printActive = Boolean(active);
    updateMainButtonsState();
    setSecondaryControlsVisible(printActive);
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
    if (clearExtrusions && typeof renderSystem.clearExtrusions === 'function') {
      renderSystem.clearExtrusions();
    }
  }

  function updateZoomButtonState() {
    if (zoomInBtn) {
      const atMax = currentViewScale >= MAX_VIEW_SCALE - ZOOM_EPSILON;
      zoomInBtn.disabled = atMax;
      if (atMax) {
        zoomInBtn.setAttribute('aria-disabled', 'true');
      } else {
        zoomInBtn.removeAttribute('aria-disabled');
      }
    }
    if (zoomOutBtn) {
      const atMin = currentViewScale <= MIN_VIEW_SCALE + ZOOM_EPSILON;
      zoomOutBtn.disabled = atMin;
      if (atMin) {
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
  }

  function handleTimeScaleChange(scale) {
    currentTimeScale = scale;
    applyTimeScaleToWorkers(scale);
    if (speedStatusArmed) {
      showSpeedStatus(scale);
    }
  }

  function applyViewStateFromController(partial = {}, options = {}) {
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
    const simHeight = world.getResource('simHeight');
    if (renderSystem && simHeight) {
      renderSystem.baseCScale = canvas.height / simHeight;
      if (renderSystem.extrusionCanvas) {
        if (renderSystem.extrusionCanvas.width !== canvas.width) {
          renderSystem.extrusionCanvas.width = canvas.width;
        }
        if (renderSystem.extrusionCanvas.height !== canvas.height) {
          renderSystem.extrusionCanvas.height = canvas.height;
        }
      }
    }
    if (resized) {
      reapplyViewState({ clearExtrusions: true });
    }
  }

  function reapplyViewState(options = {}) {
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

  function adjustZoom(multiplier) {
    const targetScale = clamp(currentViewScale * multiplier, MIN_VIEW_SCALE, MAX_VIEW_SCALE);
    if (Math.abs(targetScale - currentViewScale) < ZOOM_EPSILON) {
      return;
    }
    applyViewStateFromController({ scale: targetScale }, { clearExtrusions: true });
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
  }

  function handleUserReset() {
    if (!gameControls || typeof gameControls.reset !== 'function') {
      return;
    }
    setPrintActive(false);
    stopAndClearWorkers();
    gameControls.reset({ autoPause: true });
    if (typeof gameControls.setTimeScale === 'function') {
      gameControls.setTimeScale(1.0);
    } else {
      handleTimeScaleChange(1.0);
    }
    resetViewStateDefaults();
    setPanMode(false);
    reapplyViewState({ clearExtrusions: true });
    currentPresetKey = DEFAULT_PRESET_KEY;
  }

  function ensureKlipperWorker() {
    if (klipperCommanderWorker) {
      return klipperCommanderWorker;
    }
    klipperCommanderWorker = new Worker(klipperCommanderModuleUrl, { type: 'module' });
    klipperCommanderWorker.onmessage = (event) => {
      if (!event?.data) {
        return;
      }
      if (event.data.type === 'done') {
        console.log('Slideprinter demo: MCU log playback finished.');
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === klipperCommanderWorker) {
          setPrintActive(false);
        }
        return;
      }
      if (event.data.type === 'error') {
        console.error('Slideprinter demo: Worker reported an error:', event.data.message);
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

  function ensureMoveWorker() {
    if (moveCommanderWorker) {
      return moveCommanderWorker;
    }
    moveCommanderWorker = new Worker(moveCommanderModuleUrl, { type: 'module' });
    moveCommanderWorker.onmessage = (event) => {
      if (!event?.data) {
        return;
      }
      if (event.data.type === 'done') {
        console.log('Slideprinter demo: G-code playback finished.');
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === moveCommanderWorker) {
          setPrintActive(false);
        }
        return;
      }
      if (event.data.type === 'error') {
        console.error('Slideprinter demo: Worker reported an error:', event.data.message);
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
  }

  function resetRemoteQueue(activeWorker) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    if (typeof remoteSystem.clearCommandQueue === 'function') {
      remoteSystem.clearCommandQueue();
    } else {
      remoteSystem.commands.length = 0;
    }
    remoteSystem.worker = activeWorker;
    remoteSystem.wasPaused = false;
  }

  function startSimulationWithWorker(worker) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return false;
    }
    stopInactiveWorkers(worker);
    resetRemoteQueue(worker);
    if (worker) {
      worker.postMessage({ type: 'set_speed_scale', value: currentTimeScale });
    }
    if (gameControls && typeof gameControls.reset === 'function') {
      gameControls.reset({ autoPause: false });
      reapplyViewState({ clearExtrusions: true });
    }
    setPrintActive(true);
    return true;
  }

  function playPreset(presetKey) {
    ensureReadyForNewJob();
    if (!stageReady) {
      return;
    }
    const preset = MCU_PRESETS[presetKey];
    if (!preset || !preset.url) {
      console.warn('Slideprinter demo: unknown preset', presetKey);
      return;
    }
    const format = preset.format || detectFileFormat(preset.url);
    let worker = null;
    if (format === FileFormat.GCODE) {
      worker = ensureMoveWorker();
    } else if (isMcuFormat(format)) {
      worker = ensureKlipperWorker();
    } else {
      console.warn('Slideprinter demo: unsupported preset format', format);
      return;
    }
    currentPresetKey = presetKey;
    if (!startSimulationWithWorker(worker)) {
      return;
    }
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    worker.postMessage({ type: 'filename_fetch', filename: preset.url });
  }

  async function queueUploadedFile(file) {
    if (!stageReady || !file) {
      return;
    }
    const format = detectFileFormat(file.name);
    if (format === FileFormat.USD_STAGE) {
      ensureReadyForNewJob();
      await addUsdMachineFromFile(file);
      return;
    }

    ensureReadyForNewJob();
    let worker = null;
    if (format === FileFormat.GCODE) {
      worker = ensureMoveWorker();
    } else if (isMcuFormat(format)) {
      worker = ensureKlipperWorker();
    } else {
      console.warn('Slideprinter demo: unsupported upload format', file?.name || 'unknown');
      return;
    }
    if (!startSimulationWithWorker(worker)) {
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

  if (uploadBtn && gcodeInput) {
    uploadBtn.addEventListener('click', () => gcodeInput.click());
    gcodeInput.addEventListener('change', (event) => {
      const file = event.target.files?.[0];
      if (file) {
        queueUploadedFile(file).catch((error) => {
          console.error('Slideprinter demo: upload handling failed.', error);
        });
        gcodeInput.value = '';
      }
    });
  }

  setPrintActive(false);
  setSpeedButtonsEnabled(false);

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
  window.addEventListener('resize', syncCanvasDimensions);

  updateZoomButtonState();

  UsdOpen(usdaUrl.href)
    .then((stage) => {
      if (!stage) {
        throw new Error('Slideprinter demo: failed to load USD stage.');
      }
      const timeCodesPerSecond = extractTimeCodesPerSecond(stage);
      if (timeCodesPerSecond) {
        simDtSec = 1.0 / timeCodesPerSecond;
        if (klipperCommanderWorker) {
          klipperCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
        }
        if (moveCommanderWorker) {
          moveCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
        }
      }

      machines.splice(0, machines.length);
      registerMachine(stage, { name: 'slideprinter.usda' });

      const sceneInitializer = () => rebuildScene();
      gameControls = runGame(world, sceneInitializer, {
        initialTimeScale: currentTimeScale,
        onTimeScaleChange: handleTimeScaleChange,
      });
      if (gameControls && typeof gameControls.reset === 'function') {
        gameControls.reset({ autoPause: true });
      }
      resetViewStateDefaults();
      reapplyViewState({ clearExtrusions: true });
      setPanMode(false);
      syncCanvasDimensions();
      stageReady = true;
      setSpeedButtonsEnabled(true);
    })
    .catch((error) => {
      console.error('Slideprinter demo initialisation failed:', error);
      setPrintActive(false);
      controlsRoot.innerHTML = '<p class="sim-error">Unable to start the simulation. Please reload the page.</p>';
    });
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initFrontpageSlideprinter, { once: true });
} else {
  initFrontpageSlideprinter();
}
