import { Open as UsdOpen } from '../../src/js/usd/stage.js';
import { World } from '../../src/js/cable_joints/ecs.js';
import { runGame } from '../../examples/js/slideprinter/runner.js';
import { setupScene } from '../../examples/js/slideprinter/setupScene.js';
import { RemoteSpoolSystem } from '../../examples/js/slideprinter/slideprinter_common.js';

const DEFAULT_PRESET = 'Hangprinter_logo6.gcode';

function initFrontpageSlideprinter() {
  const canvas = document.getElementById('myCanvas');
  const controls = document.getElementById('controls');
  if (!canvas || !controls) {
    return;
  }

  const resetBtn = document.getElementById('resetBtn');
  const loadGcodeBtn = document.getElementById('loadGcodeBtn');
  const gcodeInput = document.getElementById('gcodeFile');
  const presetSelect = document.getElementById('presetGcode');

  const world = new World();
  let moveCommanderWorker = null;
  let simDtSec = null;
  let stageReady = false;
  let currentPreset = presetSelect?.value || DEFAULT_PRESET;
  let lastUploadedFile = null;

  const moveCommanderModuleUrl = new URL('../../examples/js/slideprinter/moveCommander.js', import.meta.url);
  const gcodeBaseUrl = new URL('../../examples/gcode/', import.meta.url);
  const usdaUrl = new URL('../../examples/usd_scenes/slideprinter_copy_for_vite.usda.txt', import.meta.url);

  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
  }

  function ensureWorker() {
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
        if (remoteSystem) {
          if (remoteSystem.worker !== moveCommanderWorker) {
            remoteSystem.worker = moveCommanderWorker;
          }
          remoteSystem.addCommand(event.data.command);
        }
      }
    };
    if (simDtSec != null) {
      moveCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    return moveCommanderWorker;
  }

  function resetRemoteQueue() {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    remoteSystem.commands.length = 0;
    remoteSystem.worker = moveCommanderWorker;
    remoteSystem.wasPaused = false;
  }

  function queuePreset(presetName) {
    if (!presetName) {
      return;
    }
    currentPreset = presetName;
    const remoteSystem = getRemoteSystem();
    if (!stageReady || !remoteSystem) {
      return;
    }
    const worker = ensureWorker();
    resetRemoteQueue();
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    const presetUrl = new URL(presetName, gcodeBaseUrl);
    worker.postMessage({ type: 'filename_fetch', filename: presetUrl.href });
  }

  function queueFile(file) {
    if (!file) {
      return;
    }
    lastUploadedFile = file;
    const remoteSystem = getRemoteSystem();
    if (!stageReady || !remoteSystem) {
      return;
    }
    const worker = ensureWorker();
    resetRemoteQueue();
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    worker.postMessage({ type: 'filename_upload', filename: file });
  }

  function replayCurrentSelection() {
    if (lastUploadedFile) {
      queueFile(lastUploadedFile);
    } else {
      queuePreset(currentPreset || DEFAULT_PRESET);
    }
  }

  if (loadGcodeBtn && gcodeInput) {
    loadGcodeBtn.addEventListener('click', () => gcodeInput.click());
    gcodeInput.addEventListener('change', (event) => {
      const file = event.target.files?.[0];
      if (file) {
        presetSelect?.blur();
        queueFile(file);
      }
    });
  }

  if (presetSelect) {
    presetSelect.addEventListener('change', () => {
      const value = presetSelect.value;
      if (!value) {
        return;
      }
      lastUploadedFile = null;
      queuePreset(value);
    });
  }

  if (resetBtn) {
    resetBtn.addEventListener('click', () => {
      // allow runGame to recreate the scene, then queue the current job again
      window.setTimeout(replayCurrentSelection, 0);
    });
  }

  const defaultPreset = presetSelect?.value || DEFAULT_PRESET;

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
        if (moveCommanderWorker) {
          moveCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
        }
      }

      runGame(world, () => setupScene(world, stage, canvas, { remote: false }));
      stageReady = true;
      requestAnimationFrame(() => {
        queuePreset(defaultPreset);
      });
    })
    .catch((error) => {
      console.error('Slideprinter demo initialisation failed:', error);
      controls.innerHTML = '<p class="sim-error">Unable to start the simulation. Please reload the page.</p>';
    });
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initFrontpageSlideprinter, { once: true });
} else {
  initFrontpageSlideprinter();
}
