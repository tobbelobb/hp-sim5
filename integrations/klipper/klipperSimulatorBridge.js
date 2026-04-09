// KlipperSimulatorBridge: Connects the raw-bytes pacer worker to a timing scheduler.
import timingSchedulerWorkerUrl from './timingScheduler.js?worker&url';

const DEBUG = false; // Note: most logging is now in the worker.
const EARLY_EPS_MS = 0.05; // only preempt if earlier by >= 0.05ms to reduce churn
// Keep a substantially thicker backlog in RemoteSpoolSystem than the old
// near-just-in-time defaults. This reduces starvation under the current
// runner queue gate and makes live API playback behave more like upload paths.
const BASE_LOOKAHEAD_MIN_MS = 24;
const BASE_LOOKAHEAD_MAX_MS = 192;
const FAST_LOOKAHEAD_MIN_MS = 96;
const FAST_LOOKAHEAD_MAX_MS = 640;
const FAST_LOOKAHEAD_SCALE = 16;

function createKlipperRawHandle(onCommand, options = {}) {
  const workerUrl = new URL('./klipperPacerWorker.js', import.meta.url);
  const worker = new Worker(workerUrl, { type: 'module' });

  const logMove = Boolean(options.logMove);
  const logMoveFilename = typeof options.logMoveFilename === 'string' && options.logMoveFilename.trim()
    ? options.logMoveFilename.trim()
    : 'klipper_move_log.jsonl';
  const moveLogLines = logMove ? [] : null;
  const onWorkerError = typeof options.onWorkerError === 'function' ? options.onWorkerError : null;
  const onDone = typeof options.onDone === 'function' ? options.onDone : null;

  const timingWorker = new Worker(timingSchedulerWorkerUrl, { type: 'module' });
  const sab = new SharedArrayBuffer(4);
  const sabI32 = new Int32Array(sab);
  timingWorker.postMessage({ type: 'init', sab });

  const pending = [];
  let nextDeadlineMs = null;
  let currentSleepDeadlineMs = null;
  let sleepArmed = false;
  let paused = false;
  let sourceDone = false;
  let doneNotified = false;
  let asapMode = false;
  let fastMode = false;
  let currentSpeedScale = 1;
  let closed = false;

  const getEnqueueLookaheadMs = () => {
    if (asapMode) {
      return Number.POSITIVE_INFINITY;
    }
    const scale = Number.isFinite(currentSpeedScale) && currentSpeedScale > 0 ? currentSpeedScale : 1;
    if (fastMode) {
      return Math.min(FAST_LOOKAHEAD_MAX_MS, Math.max(FAST_LOOKAHEAD_MIN_MS, scale * FAST_LOOKAHEAD_SCALE));
    }
    return Math.min(BASE_LOOKAHEAD_MAX_MS, Math.max(BASE_LOOKAHEAD_MIN_MS, scale));
  };

  const notifyScheduler = () => {
    try {
      Atomics.store(sabI32, 0, 1);
      Atomics.notify(sabI32, 0, 1);
    } catch (_) {}
  };

  const sortPending = () => {
    pending.sort((left, right) => left.at - right.at);
  };

  const maybeNotifyDone = () => {
    if (!sourceDone || doneNotified || pending.length > 0 || closed) {
      return;
    }
    doneNotified = true;
    if (typeof onDone === 'function') {
      onDone();
    }
  };

  const clearTimingState = () => {
    nextDeadlineMs = null;
    currentSleepDeadlineMs = null;
    sleepArmed = false;
  };

  const flushReadyCommands = (referenceNow = performance.now()) => {
    const cutoff = asapMode ? referenceNow : referenceNow + getEnqueueLookaheadMs();
    while (pending.length && pending[0].at <= cutoff) {
      const cmd = pending.shift();
      try {
        onCommand(cmd);
      } catch (_) {}
    }
  };

  const scheduleNextSleep = () => {
    if (closed || paused || pending.length === 0) {
      clearTimingState();
      maybeNotifyDone();
      return;
    }
    const now = performance.now();
    flushReadyCommands(now);
    if (closed || paused || pending.length === 0) {
      clearTimingState();
      maybeNotifyDone();
      return;
    }
    const earliest = pending[0].at;
    nextDeadlineMs = earliest;
    if (!sleepArmed) {
      const lookaheadMs = getEnqueueLookaheadMs();
      const wakeAt = asapMode ? now : Math.max(now, earliest - lookaheadMs);
      const ms = Math.max(0, wakeAt - now);
      timingWorker.postMessage({ type: 'sleep', ms });
      currentSleepDeadlineMs = wakeAt;
      sleepArmed = true;
    }
  };

  const insertSorted = (cmd) => {
    let lo = 0;
    let hi = pending.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (pending[mid].at <= cmd.at) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    pending.splice(lo, 0, cmd);
  };

  const schedule = (cmd) => {
    if (!cmd || typeof cmd !== 'object' || closed) {
      return;
    }
    const normalized = { ...cmd };
    if (!Number.isFinite(normalized.at)) {
      normalized.at = performance.now();
    }
    if (asapMode) {
      normalized.at = performance.now();
    }
    if (paused) {
      normalized.remainingMs = Math.max(0, normalized.at - performance.now());
    }
    insertSorted(normalized);
    if (paused) {
      return;
    }
    flushReadyCommands(performance.now());
    if (pending.length === 0) {
      clearTimingState();
      maybeNotifyDone();
      return;
    }
    if (!sleepArmed) {
      scheduleNextSleep();
      return;
    }
    const earliest = pending[0].at;
    const wakeTarget = Math.max(performance.now(), earliest - getEnqueueLookaheadMs());
    if (currentSleepDeadlineMs === null || wakeTarget + EARLY_EPS_MS < currentSleepDeadlineMs) {
      notifyScheduler();
    }
  };

  const setPaused = (nextPaused) => {
    if (paused === Boolean(nextPaused) || closed) {
      return;
    }
    paused = Boolean(nextPaused);
    const now = performance.now();
    if (paused) {
      for (const cmd of pending) {
        cmd.remainingMs = Math.max(0, cmd.at - now);
      }
      clearTimingState();
      notifyScheduler();
      return;
    }
    for (const cmd of pending) {
      const remainingMs = Math.max(0, Number(cmd.remainingMs) || 0);
      cmd.at = asapMode ? now : now + remainingMs;
      delete cmd.remainingMs;
    }
    sortPending();
    scheduleNextSleep();
  };

  const setSpeedScale = (value) => {
    const safeScale = Number.isFinite(value) && value > 0 ? value : 1;
    const previousScale = currentSpeedScale;
    currentSpeedScale = safeScale;
    if (closed || previousScale === safeScale || pending.length === 0) {
      worker.postMessage({ type: 'set_speed_scale', value: safeScale });
      return;
    }
    const now = performance.now();
    for (const cmd of pending) {
      if (paused) {
        const logicalRemaining = Math.max(0, Number(cmd.remainingMs) || 0) * previousScale;
        cmd.remainingMs = logicalRemaining / safeScale;
      } else {
        const logicalRemaining = Math.max(0, cmd.at - now) * previousScale;
        cmd.at = now + (logicalRemaining / safeScale);
      }
    }
    sortPending();
    clearTimingState();
    notifyScheduler();
    worker.postMessage({ type: 'set_speed_scale', value: safeScale });
    scheduleNextSleep();
  };

  const setFastMode = (enabled) => {
    const nextFastMode = Boolean(enabled);
    if (fastMode === nextFastMode || closed) {
      return;
    }
    fastMode = nextFastMode;
    if (!paused) {
      flushReadyCommands(performance.now());
      clearTimingState();
      notifyScheduler();
      scheduleNextSleep();
    }
  };

  const setAsapMode = (enabled) => {
    asapMode = Boolean(enabled);
    if (!closed && asapMode) {
      const now = performance.now();
      for (const cmd of pending) {
        if (paused) {
          cmd.remainingMs = 0;
        } else {
          cmd.at = now;
        }
      }
      sortPending();
      clearTimingState();
      notifyScheduler();
      if (!paused) {
        flushReadyCommands(now);
      }
    }
    worker.postMessage({ type: 'set_asap_mode', enable: asapMode });
    scheduleNextSleep();
  };

  const announceLogHelp = () => {
    if (!logMove || !moveLogLines || announceLogHelp._done) {
      return;
    }
    announceLogHelp._done = true;
    if (typeof console !== 'undefined') {
      console.log('[KlipperRaw] move logging enabled; call handle.downloadMoveLog() to save JSONL file.');
    }
  };

  const downloadMoveLog = () => {
    if (!logMove || !moveLogLines || moveLogLines.length === 0) {
      return null;
    }
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

  const forwardWorkerMessage = (message) => {
    if (!message || typeof message !== 'object' || closed) {
      return;
    }
    const payload = { ...message };
    if (
      payload.type === 'connect'
      || payload.type === 'filename_upload'
      || payload.type === 'filename_fetch'
    ) {
      payload.logMove = logMove;
      sourceDone = false;
      doneNotified = false;
    }
    worker.postMessage(payload);
  };

  const close = () => {
    if (closed) {
      return;
    }
    closed = true;
    pending.length = 0;
    clearTimingState();
    try {
      worker.postMessage({ type: 'close' });
    } catch (_) {}
    try {
      worker.terminate();
    } catch (_) {}
    try {
      timingWorker.postMessage({ type: 'shutdown' });
    } catch (_) {}
    notifyScheduler();
  };

  worker.onmessage = (e) => {
    const { type, command, logEntry, message } = e.data || {};
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
      if (typeof onCommand === 'function') {
        schedule(command);
      }
    } else if (type === 'done') {
      sourceDone = true;
      maybeNotifyDone();
    } else if (type === 'closed') {
      console.log('KlipperSimulatorBridge: worker indicated connection closed');
    } else if (type === 'error') {
      const msg = typeof message === 'string' && message ? message : 'Klipper worker reported an error.';
      if (typeof onWorkerError === 'function') {
        try {
          onWorkerError(msg);
        } catch (err) {
          console.error('KlipperSimulatorBridge onWorkerError callback threw an error:', err);
        }
      } else if (typeof console !== 'undefined') {
        console.error('KlipperSimulatorBridge worker error:', msg);
      }
    } else if (DEBUG) {
      console.debug('KlipperSimulatorBridge: unhandled worker message', e.data);
    }
  };

  worker.onerror = (err) => {
    console.error('KlipperSimulatorBridge worker error:', err.message, err);
  };

  timingWorker.onmessage = (e) => {
    const { type } = e.data || {};
    if (type !== 'wakeup') {
      return;
    }
    clearTimingState();
    if (paused || closed) {
      return;
    }
    flushReadyCommands(performance.now());
    scheduleNextSleep();
  };

  return {
    worker,
    postMessage(message) {
      const type = message?.type;
      if (!type || closed) {
        return;
      }
      if (type === 'pause') {
        setPaused(true);
        worker.postMessage({ type: 'pause' });
        return;
      }
      if (type === 'resume') {
        setPaused(false);
        worker.postMessage({ type: 'resume' });
        return;
      }
      if (type === 'set_speed_scale') {
        setSpeedScale(message.value);
        return;
      }
      if (type === 'set_fast_mode') {
        setFastMode(message.enable);
        return;
      }
      if (type === 'set_asap_mode') {
        setAsapMode(message.enable);
        return;
      }
      if (type === 'set_dt') {
        return;
      }
      if (type === 'close') {
        close();
        return;
      }
      forwardWorkerMessage(message);
    },
    setDt: (_newDt) => {
      // No-op: timing is driven by per-command 'at' timestamps now.
    },
    getMoveLogLines: () => (logMove && moveLogLines ? [...moveLogLines] : []),
    downloadMoveLog,
    terminate: close,
    close,
  };
}

export function createKlipperRawBridge(onCommand /* function(command) */, options = {}) {
  return createKlipperRawHandle(onCommand, options);
}

// options: { dt?: number } where dt is in seconds. If provided and > 0,
// outgoing commands are batched over windows of length dt and coalesced.
export function connectKlipperRaw(url, onCommand /* function(command) */, options = {}) {
  const handle = createKlipperRawHandle(onCommand, options);
  handle.postMessage({ type: 'connect', url });
  return handle;
}

export function playKlipperRawFile(file, onCommand /* function(command) */, options = {}) {
  const handle = createKlipperRawHandle(onCommand, options);
  handle.postMessage({ type: 'filename_upload', filename: file });
  return handle;
}
