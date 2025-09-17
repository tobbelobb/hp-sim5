import { Open as UsdOpen } from '../../src/js/usd/stage.js';
import { World } from '../../src/js/cable_joints/ecs.js';
import { runGame } from '../../examples/js/slideprinter/runner.js';
import { setupScene } from '../../examples/js/slideprinter/setupScene.js';
import { RemoteSpoolSystem } from '../../examples/js/slideprinter/slideprinter_common.js';

const PRESETS = {
  hangprinterLogo: {
    label: 'Hangprinter Logo',
    file: new URL('../../examples/mcu_commands/Hangprinter_logo6.txt', import.meta.url).href,
  },
  straightMoves: {
    label: 'Straight movements',
    file: new URL('../../examples/mcu_commands/draw_squares.txt', import.meta.url).href,
  },
};

const DEFAULT_PRESET_KEY = 'hangprinterLogo';

function initFrontpageSlideprinter() {
  const canvas = document.getElementById('myCanvas');
  const controlsRoot = document.getElementById('controls');
  if (!canvas || !controlsRoot) {
    return;
  }

  const playBtn = document.getElementById('playBtn');
  const presetSelect = document.getElementById('presetMcu');

  const world = new World();
  let klipperCommanderWorker = null;
  let simDtSec = null;
  let stageReady = false;
  let currentPresetKey = presetSelect?.value || DEFAULT_PRESET_KEY;
  let gameControls = null;

  if (presetSelect && !PRESETS[presetSelect.value]) {
    presetSelect.value = DEFAULT_PRESET_KEY;
    currentPresetKey = DEFAULT_PRESET_KEY;
  }

  const klipperCommanderModuleUrl = new URL('../../examples/js/slideprinter/klipperCommander.js', import.meta.url);
  const usdaUrl = new URL('../../examples/usd_scenes/slideprinter_copy_for_vite.usda.txt', import.meta.url);

  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
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
        if (remoteSystem) {
          if (remoteSystem.worker !== klipperCommanderWorker) {
            remoteSystem.worker = klipperCommanderWorker;
          }
          remoteSystem.addCommand(event.data.command);
        }
      }
    };
    if (simDtSec != null) {
      klipperCommanderWorker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    return klipperCommanderWorker;
  }

  function resetRemoteQueue() {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    remoteSystem.commands.length = 0;
    remoteSystem.worker = klipperCommanderWorker;
    remoteSystem.wasPaused = false;
  }

  function playPreset(presetKey) {
    const preset = PRESETS[presetKey];
    if (!preset) {
      console.warn('Slideprinter demo: unknown preset key', presetKey);
      return;
    }
    currentPresetKey = presetKey;
    if (!stageReady) {
      return;
    }

    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }

    if (gameControls && typeof gameControls.reset === 'function') {
      gameControls.reset({ autoPause: false });
    }

    const worker = ensureKlipperWorker();
    resetRemoteQueue();
    if (simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: simDtSec });
    }
    worker.postMessage({ type: 'filename_fetch', filename: preset.file });
  }

  if (presetSelect) {
    presetSelect.addEventListener('change', () => {
      const value = presetSelect.value;
      if (PRESETS[value]) {
        currentPresetKey = value;
      }
    });
  }

  if (playBtn) {
    playBtn.addEventListener('click', () => {
      playPreset(currentPresetKey || DEFAULT_PRESET_KEY);
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
      }

      const sceneInitializer = () => setupScene(world, stage, canvas, { remote: false });
      gameControls = runGame(world, sceneInitializer);
      stageReady = true;

      // Kick off the default demo once rendering is ready.
      requestAnimationFrame(() => {
        playPreset(currentPresetKey || DEFAULT_PRESET_KEY);
      });
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
