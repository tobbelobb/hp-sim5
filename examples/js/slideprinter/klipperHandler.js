// KlipperHandler: Connects to a raw-bytes WebSocket via a worker.
const DEBUG = false; // Note: most logging is now in the worker.

// options: { dt?: number } where dt is in seconds. If provided and > 0,
// outgoing commands are batched over windows of length dt and coalesced.
export function connectKlipperRaw(url, onCommand /* function(command) */, options = {}) {
  // Use a URL object to construct a path relative to this module's location.
  // This is more robust than hardcoding paths, especially with bundlers/vite.
  const workerPath = new URL('./klipperPacer.js', import.meta.url).href;
  const worker = new Worker(workerPath, { type: 'module' });

  worker.postMessage({ type: 'connect', url });

  // --- High-precision timing scheduler (Atomics.wait-based) ---
  const timingWorkerPath = new URL('./timingScheduler.js', import.meta.url).href;
  const timingWorker = new Worker(timingWorkerPath, { type: 'module' });

  // Shared futex used to preempt sleeps
  const sab = new SharedArrayBuffer(4);
  const sabI32 = new Int32Array(sab);
  timingWorker.postMessage({ type: 'init', sab });

  // Pending commands, sorted by 'at' (ms, performance.now() timebase)
  const pending = [];
  let nextDeadlineMs = null;

  const insertSorted = (cmd) => {
    let lo = 0, hi = pending.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (pending[mid].at <= cmd.at) lo = mid + 1; else hi = mid;
    }
    pending.splice(lo, 0, cmd);
  };

  const scheduleNextSleep = () => {
    if (pending.length === 0) {
      nextDeadlineMs = null;
      return;
    }
    const now = performance.now();
    nextDeadlineMs = pending[0].at;
    const ms = Math.max(0, nextDeadlineMs - now);
    timingWorker.postMessage({ type: 'sleep', ms });
  };

  const schedule = (cmd) => {
    if (!cmd || typeof cmd !== 'object') return;
    if (!Number.isFinite(cmd.at)) cmd.at = performance.now();
    insertSorted(cmd);
    if (nextDeadlineMs === null || cmd.at < nextDeadlineMs) {
      // Preempt current wait to an earlier deadline
      try {
        Atomics.store(sabI32, 0, 1);
        Atomics.notify(sabI32, 0, 1);
      } catch (_) {}
      scheduleNextSleep();
    }
  };

  timingWorker.onmessage = (e) => {
    const { type } = e.data || {};
    if (type === 'wakeup') {
      const now = performance.now();
      // Flush all due commands
      while (pending.length && pending[0].at <= now) {
        const cmd = pending.shift();
        try { onCommand(cmd); } catch (_) {}
      }
      scheduleNextSleep();
    }
  };

  const handleCommand = (cmd) => {
    schedule(cmd);
  };

  worker.onmessage = (e) => {
    const { type, command, args } = e.data;
    if (type === 'move') {
      if (typeof onCommand === 'function') handleCommand(command);
    } else if (type === 'closed') {
      console.log('KlipperHandler: worker indicated connection closed');
    }
  };

  worker.onerror = (err) => {
    console.error('KlipperHandler worker error:', err.message, err);
  };

  // Return an object that allows the caller to terminate the connection/worker.
  return {
    worker,
    setDt: (_newDt) => {
      // No-op: timing is driven by per-command 'at' timestamps now
    },
    close: () => {
      try { worker.postMessage({ type: 'close' }); } catch (_) {}
      try { timingWorker.postMessage({ type: 'shutdown' }); } catch (_) {}
      try {
        Atomics.store(sabI32, 0, 1);
        Atomics.notify(sabI32, 0, 1);
      } catch (_) {}
    },
  };
}
