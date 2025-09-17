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
const VIEW_SCALE = 1.25;

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

  const world = new World();
  let klipperCommanderWorker = null;
  let moveCommanderWorker = null;
  let simDtSec = null;
  let stageReady = false;
  let currentPresetKey = DEFAULT_PRESET_KEY;
  let gameControls = null;

  const klipperCommanderModuleUrl = new URL('../../examples/js/slideprinter/klipperCommander.js', import.meta.url);
  const moveCommanderModuleUrl = new URL('../../examples/js/slideprinter/moveCommander.js', import.meta.url);
  const usdaUrl = new URL('../../examples/usd_scenes/slideprinter_copy_for_vite.usda.txt', import.meta.url);

  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
  }

  function applyViewZoom() {
    const inputSystem = world.systems.find((sys) => sys instanceof InputSystem);
    if (inputSystem) {
      inputSystem.scaleMultiplier = VIEW_SCALE;
    }
    const renderSystem = world.getResource('renderSystem');
    if (renderSystem) {
      renderSystem.viewScaleMultiplier = VIEW_SCALE;
      renderSystem.effectiveCScale = renderSystem.baseCScale * renderSystem.viewScaleMultiplier;
    }
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
    }
    applyViewZoom();
    return true;
  }

  function playPreset(presetKey) {
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
      applyViewZoom();
      stageReady = true;
    })
    .catch((error) => {
      console.error('Slideprinter demo initialisation failed:', error);
      controlsRoot.innerHTML = '<p class="sim-error">Unable to start the simulation. Please reload the page.</p>';
    });
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initFrontpageSlideprinter, { once: true });
} else {
  initFrontpageSlideprinter();
}
