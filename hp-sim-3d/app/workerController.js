import { createKlipperRawBridge } from '../../integrations/klipper/klipperSimulatorBridge.js';

// Keep these as Vite worker URL imports. Reverting to new URL(workerSource, import.meta.url)
// broke preview/prod builds before by emitting worker files with unresolved relative imports.
import klipperMcuCommandPlayerWorkerUrl from '../../integrations/klipper/klipperMcuCommandPlayer.js?worker&url';
import rrfCanPlayerWorkerUrl from '../../integrations/rrf/rrfCanPlayer.js?worker&url';
import moveCommanderWorkerUrl from '../../example_apps/js/slideprinter/moveCommander.js?worker&url';

export const KLIPPER_UPLOAD_PIPELINE = 'player';

export function shouldUseRawKlipperUploadPipeline(pipeline = KLIPPER_UPLOAD_PIPELINE) {
  return pipeline === 'raw';
}

export function createWorkerController({
  state,
  getRemoteSystem,
  onWorkerDone = () => {},
  onWorkerError = () => {},
  klipperPacerDiagnosticsEnabled = false,
  WorkerCtor = globalThis.Worker,
} = {}) {
  let klipperMcuCommandPlayerWorker = null;
  let klipperRawUploadBridge = null;
  let rrfCanPlayerWorker = null;
  let moveCommanderWorker = null;

  function getSimDtSec() {
    return state.simDtSec;
  }

  function postTiming(worker) {
    if (!worker) {
      return;
    }
    if (getSimDtSec() != null) {
      worker.postMessage({ type: 'set_dt', dt: getSimDtSec() });
    }
    worker.postMessage({ type: 'set_speed_scale', value: state.currentTimeScale });
  }

  function handleWorkerMessage(worker, label, event) {
    if (!event?.data) {
      return;
    }
    if (event.data.type === 'done') {
      console.log(`hp-sim-3d: ${label} playback finished.`);
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === worker) {
        remoteSystem.worker = null;
        onWorkerDone(worker);
      }
      return;
    }
    if (event.data.type === 'error') {
      console.error('hp-sim-3d: Worker reported an error:', event.data.message);
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === worker) {
        remoteSystem.worker = null;
        onWorkerError(worker, event.data.message);
      }
      return;
    }
    if (event.data.action === 'gcode') {
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === worker) {
        remoteSystem.addCommand(event.data.command);
      }
    }
  }

  function createModuleWorker(url, label) {
    const worker = new WorkerCtor(url, { type: 'module' });
    worker.onerror = (event) => {
      console.error(`hp-sim-3d: ${label} worker failed.`, event);
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === worker) {
        remoteSystem.worker = null;
        onWorkerError(worker, event);
      }
    };
    worker.onmessageerror = (event) => {
      console.error(`hp-sim-3d: ${label} worker message decode failed.`, event);
    };
    worker.onmessage = (event) => handleWorkerMessage(worker, label, event);
    postTiming(worker);
    return worker;
  }

  function ensureKlipperMcuCommandPlayerWorker() {
    if (!klipperMcuCommandPlayerWorker) {
      klipperMcuCommandPlayerWorker = createModuleWorker(klipperMcuCommandPlayerWorkerUrl, 'Klipper');
    }
    return klipperMcuCommandPlayerWorker;
  }

  function terminateKlipperRawUploadBridge() {
    if (!klipperRawUploadBridge) {
      return;
    }
    try {
      klipperRawUploadBridge.terminate();
    } catch (err) {
      console.warn('hp-sim-3d: unable to terminate raw Klipper upload bridge cleanly.', err);
    }
    klipperRawUploadBridge = null;
  }

  function createKlipperRawUploadBridge() {
    terminateKlipperRawUploadBridge();
    const bridge = createKlipperRawBridge((command) => {
      const remoteSystem = getRemoteSystem();
      if (remoteSystem && remoteSystem.worker === bridge) {
        remoteSystem.addCommand(command);
      }
    }, {
      debugDiagnostics: klipperPacerDiagnosticsEnabled,
      onDone: () => {
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === bridge) {
          remoteSystem.worker = null;
          onWorkerDone(bridge);
        }
        if (klipperRawUploadBridge === bridge) {
          terminateKlipperRawUploadBridge();
        }
      },
      onWorkerError: (message) => {
        console.error('hp-sim-3d: raw Klipper upload bridge failed:', message);
        const remoteSystem = getRemoteSystem();
        if (remoteSystem && remoteSystem.worker === bridge) {
          remoteSystem.worker = null;
          onWorkerError(bridge, message);
        }
        if (klipperRawUploadBridge === bridge) {
          terminateKlipperRawUploadBridge();
        }
      },
    });
    klipperRawUploadBridge = bridge;
    postTiming(bridge);
    return bridge;
  }

  function ensureRrfWorker() {
    if (!rrfCanPlayerWorker) {
      rrfCanPlayerWorker = createModuleWorker(rrfCanPlayerWorkerUrl, 'RRF');
    }
    return rrfCanPlayerWorker;
  }

  function ensureMoveWorker() {
    if (!moveCommanderWorker) {
      moveCommanderWorker = createModuleWorker(moveCommanderWorkerUrl, 'G-code');
    }
    return moveCommanderWorker;
  }

  function stopInactiveWorkers(activeWorker) {
    if (moveCommanderWorker && moveCommanderWorker !== activeWorker) {
      moveCommanderWorker.postMessage({ type: 'pause' });
    }
    if (klipperMcuCommandPlayerWorker && klipperMcuCommandPlayerWorker !== activeWorker) {
      klipperMcuCommandPlayerWorker.postMessage({ type: 'pause' });
    }
    if (klipperRawUploadBridge && klipperRawUploadBridge !== activeWorker) {
      terminateKlipperRawUploadBridge();
    }
    if (rrfCanPlayerWorker && rrfCanPlayerWorker !== activeWorker) {
      rrfCanPlayerWorker.postMessage({ type: 'pause' });
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

  function stopAndClearWorkers() {
    stopInactiveWorkers(null);
    resetRemoteQueue(null);
    for (const [label, worker, clear] of [
      ['G-code', moveCommanderWorker, () => { moveCommanderWorker = null; }],
      ['Klipper', klipperMcuCommandPlayerWorker, () => { klipperMcuCommandPlayerWorker = null; }],
      ['RRF', rrfCanPlayerWorker, () => { rrfCanPlayerWorker = null; }],
    ]) {
      if (!worker) {
        continue;
      }
      try {
        worker.terminate();
      } catch (err) {
        console.warn(`hp-sim-3d: unable to terminate ${label} worker cleanly.`, err);
      }
      clear();
    }
    terminateKlipperRawUploadBridge();
  }

  function applyTimeScale(scale) {
    const safeScale = Number.isFinite(scale) && scale > 0 ? scale : 1.0;
    for (const worker of [klipperMcuCommandPlayerWorker, klipperRawUploadBridge, moveCommanderWorker, rrfCanPlayerWorker]) {
      worker?.postMessage?.({ type: 'set_speed_scale', value: safeScale });
    }
  }

  return {
    ensureKlipperMcuCommandPlayerWorker,
    createKlipperRawUploadBridge,
    ensureRrfWorker,
    ensureMoveWorker,
    stopInactiveWorkers,
    resetRemoteQueue,
    stopAndClearWorkers,
    applyTimeScale,
    terminateKlipperRawUploadBridge,
    getKlipperRawUploadBridge: () => klipperRawUploadBridge,
  };
}
