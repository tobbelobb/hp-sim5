import { Open as UsdOpen } from '../../src/js/usd/stage.js';
import { World } from '../../src/js/cable_joints/ecs.js';
import { runGame } from '../../examples/js/slideprinter/runner.js';
import { setupScene } from '../../examples/js/slideprinter/setupScene.js';
import { RemoteSpoolSystem, InputSystem } from '../../examples/js/slideprinter/slideprinter_common.js';

const MCU_PRESETS = {
  hangprinterLogo: new URL('../../examples/mcu_commands/Hangprinter_logo6.txt', import.meta.url).href,
  straightMoves: new URL('../../examples/mcu_commands/draw_squares.txt', import.meta.url).href,
};

const DEFAULT_PRESET_KEY = 'hangprinterLogo';
const DEFAULT_VIEW_SCALE = 1.25;
const MIN_VIEW_SCALE = 0.1;
const MAX_VIEW_SCALE = 15;
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
  const secondaryControls = document.getElementById('simSecondaryControls');
  const fullscreenBtn = document.getElementById('fullscreenBtn');
  const simApp = canvas.closest('.sim-app');
  const simButtons = controlsRoot.querySelector('.sim-buttons');
  const startButtons = simButtons ? Array.from(simButtons.querySelectorAll('.sim-start')) : [];

  const world = new World();
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

  const klipperCommanderModuleUrl = new URL('../../examples/js/slideprinter/klipperCommander.js', import.meta.url);
  const moveCommanderModuleUrl = new URL('../../examples/js/slideprinter/moveCommander.js', import.meta.url);
  const usdaUrl = new URL('../../examples/usd_scenes/slideprinter_copy_for_vite.usda.txt', import.meta.url);

  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
  }

  function getInputSystem() {
    return world.systems.find((sys) => sys instanceof InputSystem) || null;
  }

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
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

  function handleInputViewChange(viewState = {}) {
    if (typeof viewState.scale === 'number') {
      currentViewScale = clamp(viewState.scale, MIN_VIEW_SCALE, MAX_VIEW_SCALE);
    }
    if (typeof viewState.offsetX === 'number') {
      currentViewOffsetX = viewState.offsetX;
    }
    if (typeof viewState.offsetY === 'number') {
      currentViewOffsetY = viewState.offsetY;
    }

    syncRenderSystem(
      {
        scaleMultiplier: currentViewScale,
        offsetX: currentViewOffsetX,
        offsetY: currentViewOffsetY,
      },
      { clearExtrusions: true }
    );

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
    fullscreenActive = isActive;
    if (simApp) {
      simApp.classList.toggle('is-fullscreen', isActive);
    }
    if (fullscreenBtn) {
      fullscreenBtn.textContent = isActive ? 'Exit Fullscreen' : 'Fullscreen';
      fullscreenBtn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
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
    remoteSystem.commands.length = 0;
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
    const presetUrl = MCU_PRESETS[presetKey];
    if (!presetUrl) {
      console.warn('Slideprinter demo: unknown preset', presetKey);
      return;
    }
    currentPresetKey = presetKey;
    const worker = ensureKlipperWorker();
    if (!startSimulationWithWorker(worker)) {
      return;
    }
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    worker.postMessage({ type: 'filename_fetch', filename: presetUrl });
  }

  function queueGcodeFile(file) {
    ensureReadyForNewJob();
    if (!stageReady || !file) {
      return;
    }
    const worker = ensureMoveWorker();
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
        queueGcodeFile(file);
        gcodeInput.value = '';
      }
    });
  }

  setPrintActive(false);

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
      const assignment = stage.ast?.descriptor?.assignments?.find(
        (entry) => entry.type === 'assignment' && entry.identifier === 'timeCodesPerSecond'
      );
      const timeCodesPerSecond = assignment?.value;
      if (timeCodesPerSecond) {
        simDtSec = 1.0 / timeCodesPerSecond;
        if (klipperCommanderWorker) {
          klipperCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
        }
        if (moveCommanderWorker) {
          moveCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
        }
      }

      const sceneInitializer = () => setupScene(world, stage, canvas, { remote: false });
      gameControls = runGame(world, sceneInitializer);
      if (gameControls && typeof gameControls.reset === 'function') {
        gameControls.reset({ autoPause: true });
      }
      resetViewStateDefaults();
      reapplyViewState({ clearExtrusions: true });
      setPanMode(false);
      syncCanvasDimensions();
      stageReady = true;
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
