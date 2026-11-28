import { createKlipperRawBridge } from '../../integrations/klipper/klipperSimulatorBridge.js';

// Keep these as Vite worker URL imports. Reverting to new URL(workerSource, import.meta.url)
// broke preview/prod builds before by emitting worker files with unresolved relative imports.
import klipperMcuCommandPlayerWorkerUrl from '../../integrations/klipper/klipperMcuCommandPlayer.js?worker&url';
import rrfCanPlayerWorkerUrl from '../../integrations/rrf/rrfCanPlayer.js?worker&url';
import moveCommanderWorkerUrl from './workers/moveCommander.js?worker&url';
import simulationClockWorkerUrl from './workers/simulationClock.js?worker&url';

// Pace physics independently of rendering and the window's background timers.
// Request the next tick only after consuming this one, so slow steps cannot
// leave a backlog of clock messages when playback is paused or reset.
export function startSimulationClock(onTick, onError) {
  const worker = new Worker(simulationClockWorkerUrl, { type: 'module' });
  let stopped = false;
  const stop = () => {
    stopped = true;
    worker.terminate();
  };
  worker.onerror = onError;
  worker.onmessage = () => {
    if (stopped) return;
    const now = performance.now();
    try {
      onTick(now);
    } catch (error) {
      onError(error);
      return;
    }
    if (!stopped) worker.postMessage(Math.max(0, 16 - (performance.now() - now)));
  };
  worker.postMessage(0);
  return stop;
}

export const KLIPPER_UPLOAD_PIPELINE = 'player';

export function shouldUseRawKlipperUploadPipeline(pipeline = KLIPPER_UPLOAD_PIPELINE) {
  return pipeline === 'raw';
}

export function createWorkerController({
  state,
  getRemoteSystem,
  onWorkerDrained = () => {},
  onWorkerError = () => {},
  klipperPacerDiagnosticsEnabled = false,
  WorkerCtor = globalThis.Worker,
} = {}) {
  let klipperMcuCommandPlayerWorker = null;
  let klipperRawUploadBridge = null;
  // One RRF worker per command stream, so machines can play different files.
  let rrfCanPlayerWorkers = [];
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

  function streamFor(worker) {
    return getRemoteSystem()?.streams.find((stream) => stream.worker === worker) ?? null;
  }

  function handleWorkerMessage(worker, label, event) {
    if (!event?.data) {
      return;
    }
    const stream = streamFor(worker);
    if (event.data.type === 'done') {
      stream?.finishWorker(() => {
        console.log(`hp-sim-3d: ${label} playback finished.`);
        if (getRemoteSystem()?.isDrained()) {
          onWorkerDrained(worker);
        }
      });
      return;
    }
    if (event.data.type === 'error') {
      console.error('hp-sim-3d: Worker reported an error:', event.data.message);
      if (stream) {
        stream.worker = null;
        onWorkerError(worker, event.data.message);
      }
      return;
    }
    if (event.data.action === 'gcode') {
      stream?.addCommand(event.data.command);
    }
  }

  function createModuleWorker(url, label) {
    const worker = new WorkerCtor(url, { type: 'module' });
    worker.onerror = (event) => {
      console.error(`hp-sim-3d: ${label} worker failed.`, event);
      const stream = streamFor(worker);
      if (stream) {
        stream.worker = null;
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
          remoteSystem.finishWorker(() => onWorkerDrained(bridge));
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

  function ensureRrfWorkers(count) {
    while (rrfCanPlayerWorkers.length < count) {
      rrfCanPlayerWorkers.push(createModuleWorker(rrfCanPlayerWorkerUrl, 'RRF'));
    }
    return rrfCanPlayerWorkers.slice(0, count);
  }

  function ensureMoveWorker() {
    if (!moveCommanderWorker) {
      moveCommanderWorker = createModuleWorker(moveCommanderWorkerUrl, 'G-code');
    }
    return moveCommanderWorker;
  }

  function stopInactiveWorkers(activeWorkers = []) {
    for (const worker of [moveCommanderWorker, klipperMcuCommandPlayerWorker, ...rrfCanPlayerWorkers]) {
      if (worker && !activeWorkers.includes(worker)) {
        worker.postMessage({ type: 'pause' });
      }
    }
    if (klipperRawUploadBridge && !activeWorkers.includes(klipperRawUploadBridge)) {
      terminateKlipperRawUploadBridge();
    }
  }

  // Each { worker, machineIds } entry becomes one command stream; machineIds null drives every machine.
  function resetRemoteQueue(streams = []) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return;
    }
    remoteSystem.setStreams(streams.length > 0 ? streams.map((stream) => stream.machineIds) : [null]);
    remoteSystem.streams.forEach((stream, index) => {
      stream.clearPlaybackState();
      stream.worker = streams[index]?.worker ?? null;
      stream.wasPaused = false;
    });
  }

  function stopAndClearWorkers() {
    stopInactiveWorkers();
    resetRemoteQueue();
    for (const [label, worker] of [
      ['G-code', moveCommanderWorker],
      ['Klipper', klipperMcuCommandPlayerWorker],
      ...rrfCanPlayerWorkers.map((rrfWorker) => ['RRF', rrfWorker]),
    ]) {
      try {
        worker?.terminate();
      } catch (err) {
        console.warn(`hp-sim-3d: unable to terminate ${label} worker cleanly.`, err);
      }
    }
    moveCommanderWorker = null;
    klipperMcuCommandPlayerWorker = null;
    rrfCanPlayerWorkers = [];
    terminateKlipperRawUploadBridge();
  }

  function applyTimeScale(scale) {
    const safeScale = Number.isFinite(scale) && scale > 0 ? scale : 1.0;
    for (const worker of [klipperMcuCommandPlayerWorker, klipperRawUploadBridge, moveCommanderWorker, ...rrfCanPlayerWorkers]) {
      worker?.postMessage?.({ type: 'set_speed_scale', value: safeScale });
    }
  }

  return {
    ensureKlipperMcuCommandPlayerWorker,
    createKlipperRawUploadBridge,
    ensureRrfWorkers,
    ensureMoveWorker,
    stopInactiveWorkers,
    resetRemoteQueue,
    stopAndClearWorkers,
    applyTimeScale,
    terminateKlipperRawUploadBridge,
    getKlipperRawUploadBridge: () => klipperRawUploadBridge,
  };
}
