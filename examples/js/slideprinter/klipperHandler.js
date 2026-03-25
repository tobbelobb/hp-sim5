// KlipperHandler: Connects to a raw-bytes WebSocket via a worker.
import { createTimelineNormalizer, normalizeSpeedScale } from '../../../autocal/control/primitives/klipper_live_timing.mjs';
import { summarizeKlipperCommand } from '../../../autocal/control/primitives/klipper_raw_observability.mjs';

function resolveWorkerUrl(relativePath, baseUrl) {
  const fallbackBase = (() => {
    if (typeof baseUrl === 'string' && baseUrl.trim()) {
      return baseUrl.trim();
    }
    if (baseUrl instanceof URL) {
      return baseUrl;
    }
    if (typeof document !== 'undefined' && document.baseURI) {
      return document.baseURI;
    }
    if (typeof window !== 'undefined' && window.location?.href) {
      return window.location.href;
    }
    return 'file:///';
  })();
  return new URL(relativePath, fallbackBase);
}

// options: { dt?: number, initialSpeedScale?: number, traceRaw?: boolean, onTrace?: function }
// where dt is in seconds.
export function connectKlipperRaw(url, onCommand /* function(command) */, options = {}) {
  const workerUrl = resolveWorkerUrl('./klipperPacer.js', options.baseUrl);
  const worker = new Worker(workerUrl, { type: 'module' });

  const logMove = Boolean(options.logMove);
  const logMoveFilename = typeof options.logMoveFilename === 'string' && options.logMoveFilename.trim()
    ? options.logMoveFilename.trim()
    : 'klipper_move_log.jsonl';
  const moveLogLines = logMove ? [] : null;
  const onWorkerError = typeof options.onWorkerError === 'function' ? options.onWorkerError : null;
  const onClose = typeof options.onClose === 'function' ? options.onClose : null;
  const onTrace = typeof options.onTrace === 'function' ? options.onTrace : null;
  const traceRaw = Boolean(options.traceRaw || options.trace);
  const initialSpeedScale = normalizeSpeedScale(options.initialSpeedScale, 1.0);
  const initialDt = Number(options.dt);
  const hasInitialDt = Number.isFinite(initialDt) && initialDt > 0;

  const emitTrace = (trace) => {
    if (!traceRaw || !trace || typeof trace !== 'object') {
      return;
    }
    try {
      if (onTrace) {
        onTrace(trace);
        return;
      }
      if (typeof console !== 'undefined' && typeof console.debug === 'function') {
        console.debug('[KlipperRaw][trace]', trace);
      }
    } catch (err) {
      if (typeof console !== 'undefined') {
        console.warn('KlipperHandler trace callback threw an error:', err);
      }
    }
  };

  const postWorkerMessage = (message) => {
    try {
      worker.postMessage(message);
    } catch (err) {
      if (typeof console !== 'undefined') {
        console.error('KlipperHandler failed to post worker message:', err);
      }
    }
  };

  postWorkerMessage({ type: 'connect', url, logMove, speedScale: initialSpeedScale, traceRaw });
  if (hasInitialDt) {
    postWorkerMessage({ type: 'set_dt', dt: initialDt });
  }

  // --- High-precision timing scheduler (Atomics.wait-based) ---
  const timingWorker = new Worker(resolveWorkerUrl('./timingScheduler.js', options.baseUrl), { type: 'module' });
  const timelineNormalizer = createTimelineNormalizer();

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
      emitTrace({
        scope: 'handler',
        kind: 'sleep_armed',
        deadlineMs: earliest,
        waitMs: ms,
        pendingCount: pending.length,
      });
    }
    // If already armed and a new earlier deadline arrives, we will only
    // preempt via SAB in schedule(); a new 'sleep' will be sent after wakeup.
  };

  const schedule = (cmd) => {
    if (!cmd || typeof cmd !== 'object') return;
    const rawAtMs = Number(cmd.at);
    const receiveAtMs = performance.now();
    cmd.at = timelineNormalizer.normalizeAt(rawAtMs, receiveAtMs);
    emitTrace({
      scope: 'handler',
      kind: 'queue_inserted',
      rawAtMs: Number.isFinite(rawAtMs) ? rawAtMs : null,
      normalizedAtMs: cmd.at,
      transportDelayMs: Number.isFinite(rawAtMs) ? receiveAtMs - rawAtMs : null,
      pendingCount: pending.length + 1,
      command: summarizeKlipperCommand({ ...cmd, at: rawAtMs }),
    });
    insertSorted(cmd);
    if (!sleepArmed) {
      // Nothing armed yet: issue initial sleep
      scheduleNextSleep();
      return;
    }
    // If an earlier deadline just arrived, preempt the current wait.
    const earliest = pending[0].at;
    if (currentSleepDeadlineMs === null || earliest + EARLY_EPS_MS < currentSleepDeadlineMs) {
      emitTrace({
        scope: 'handler',
        kind: 'sleep_preempted',
        oldDeadlineMs: currentSleepDeadlineMs,
        newDeadlineMs: earliest,
        pendingCount: pending.length,
      });
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
      emitTrace({
        scope: 'handler',
        kind: 'wakeup',
        reason: e.data?.reason ?? null,
        nowMs: now,
        pendingCount: pending.length,
      });
      // Flush all due commands
      while (pending.length && pending[0].at <= now) {
        const cmd = pending.shift();
        emitTrace({
          scope: 'handler',
          kind: 'dispatch',
          dispatchAtMs: now,
          latenessMs: now - cmd.at,
          pendingCount: pending.length,
          command: summarizeKlipperCommand(cmd),
        });
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
    const { type, command, logEntry, message } = e.data || {};
    if (type === 'move') {
      emitTrace({
        scope: 'worker',
        kind: 'move_emitted',
        command: summarizeKlipperCommand(command),
        hasLogEntry: Boolean(logEntry),
      });
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
    } else if (type === 'trace') {
      emitTrace(e.data.trace);
    } else if (type === 'closed') {
      emitTrace({ scope: 'worker', kind: 'closed' });
      console.log('KlipperHandler: worker indicated connection closed');
      if (onClose) {
        try {
          onClose();
        } catch (err) {
          console.error('KlipperHandler onClose callback threw an error:', err);
        }
      }
    } else if (type === 'error') {
      const msg = typeof message === 'string' && message ? message : 'Klipper worker reported an error.';
      emitTrace({ scope: 'worker', kind: 'error', message: msg });
      if (typeof onWorkerError === 'function') {
        try {
          onWorkerError(msg);
        } catch (err) {
          console.error('KlipperHandler onWorkerError callback threw an error:', err);
        }
      } else if (typeof console !== 'undefined') {
        console.error('KlipperHandler worker error:', msg);
      }
    }
  };

  worker.onerror = (err) => {
    console.error('KlipperHandler worker error:', err.message, err);
  };

  // Return an object that allows the caller to terminate the connection/worker.
  return {
    worker,
    setDt: (newDt) => {
      const nextDt = Number(newDt);
      if (!Number.isFinite(nextDt) || nextDt <= 0) {
        return;
      }
      postWorkerMessage({ type: 'set_dt', dt: nextDt });
      emitTrace({
        scope: 'handler',
        kind: 'set_dt',
        dt: nextDt,
      });
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
