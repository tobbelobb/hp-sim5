// KlipperHandler: Connects to a raw-bytes WebSocket via a worker.
import timingSchedulerWorkerUrl from './timingScheduler.js?worker&url';

const DEBUG = false; // Note: most logging is now in the worker.

// options: { dt?: number } where dt is in seconds. If provided and > 0,
// outgoing commands are batched over windows of length dt and coalesced.
export function connectKlipperRaw(url, onCommand /* function(command) */, options = {}) {
  // Use a URL object to construct a path relative to this module's location.
  // This is more robust than hardcoding paths, especially with bundlers/vite.
  const workerUrl = new URL('./klipperPacer.js', import.meta.url);
  const worker = new Worker(workerUrl, { type: 'module' });

  const logMove = Boolean(options.logMove);
  const logMoveFilename = typeof options.logMoveFilename === 'string' && options.logMoveFilename.trim()
    ? options.logMoveFilename.trim()
    : 'klipper_move_log.jsonl';
  const moveLogLines = logMove ? [] : null;

  worker.postMessage({ type: 'connect', url, logMove });

  // --- High-precision timing scheduler (Atomics.wait-based) ---
  const timingWorker = new Worker(timingSchedulerWorkerUrl, { type: 'module' });

  // Shared futex used to preempt sleeps
  const sab = new SharedArrayBuffer(4);
  const sabI32 = new Int32Array(sab);
  timingWorker.postMessage({ type: 'init', sab });

  // Pending commands, sorted by 'at' (ms, performance.now() timebase)
  const pending = [];
  let nextDeadlineMs = null; // latest computed earliest deadline among pending (for info)
  let currentSleepDeadlineMs = null; // deadline currently armed in timing worker
  let sleepArmed = false; // whether worker is currently sleeping for a deadline
  const EARLY_EPS_MS = 0.05; // only preempt if earlier by >= 0.05ms to reduce churn

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
      currentSleepDeadlineMs = null;
      sleepArmed = false;
      return;
    }
    const earliest = pending[0].at;
    nextDeadlineMs = earliest;
    if (!sleepArmed) {
      const now = performance.now();
      const ms = Math.max(0, earliest - now);
      timingWorker.postMessage({ type: 'sleep', ms });
      currentSleepDeadlineMs = earliest;
      sleepArmed = true;
    }
    // If already armed and a new earlier deadline arrives, we will only
    // preempt via SAB in schedule(); a new 'sleep' will be sent after wakeup.
  };

  const schedule = (cmd) => {
    if (!cmd || typeof cmd !== 'object') return;
    if (!Number.isFinite(cmd.at)) cmd.at = performance.now();
    insertSorted(cmd);
    if (!sleepArmed) {
      // Nothing armed yet: issue initial sleep
      scheduleNextSleep();
      return;
    }
    // If an earlier deadline just arrived, preempt the current wait.
    const earliest = pending[0].at;
    if (currentSleepDeadlineMs === null || earliest + EARLY_EPS_MS < currentSleepDeadlineMs) {
      try {
        Atomics.store(sabI32, 0, 1);
        Atomics.notify(sabI32, 0, 1);
      } catch (_) {}
      // Do not send a new 'sleep' message here; we'll arm after the worker wakes.
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
      // Previous sleep has completed or was preempted: allow re-arming.
      sleepArmed = false;
      currentSleepDeadlineMs = null;
      scheduleNextSleep();
    }
  };

  const handleCommand = (cmd) => {
    schedule(cmd);
  };

  const announceLogHelp = () => {
    if (!logMove || !moveLogLines) return;
    if (announceLogHelp._done) return;
    announceLogHelp._done = true;
    if (typeof console !== 'undefined') {
      console.log('[KlipperRaw] move logging enabled; call handle.downloadMoveLog() to save JSONL file.');
    }
  };

  const downloadMoveLog = () => {
    if (!logMove || !moveLogLines || moveLogLines.length === 0) return null;
    if (typeof Blob === 'undefined' || typeof URL === 'undefined') {
      return moveLogLines.join('\n');
    }
    const blob = new Blob(moveLogLines.map((line) => `${line}\n`), {
      type: 'application/json',
    });
    const url = URL.createObjectURL(blob);
    if (typeof document !== 'undefined') {
      const link = document.createElement('a');
      link.href = url;
      link.download = logMoveFilename;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      setTimeout(() => URL.revokeObjectURL(url), 30_000);
    }
    return url;
  };

  worker.onmessage = (e) => {
    const { type, command, args, logEntry } = e.data;
    if (type === 'move') {
      if (logMove && moveLogLines && logEntry) {
        try {
          const serialized = JSON.stringify(logEntry);
          moveLogLines.push(serialized);
          if (typeof console !== 'undefined') {
            console.debug('[KlipperRaw][move]', serialized);
          }
          announceLogHelp();
        } catch (err) {
          if (typeof console !== 'undefined') {
            console.warn('Failed to serialize move log entry', err);
          }
        }
      }
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
    getMoveLogLines: () => (logMove && moveLogLines ? [...moveLogLines] : []),
    downloadMoveLog,
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
