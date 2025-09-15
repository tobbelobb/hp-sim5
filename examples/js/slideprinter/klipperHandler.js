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

  // --- Batching support ---
  let dtSec = (options && typeof options.dt === 'number') ? options.dt : 0;
  let flushTimer = null;
  let pendingMove = null;          // { type: 'Move', A: angle, ... } (latest within window)
  let pendingAddRef = null;        // { type: 'Add to reference', A: dAngle, ... } (sum within window)
  let somethingArrived = false;    // Track if anything came in this window

  const scheduleFlush = () => {
    if (!(dtSec > 0)) return; // no batching
    if (flushTimer !== null) return;
    // Use setTimeout so idle periods don't fire unnecessarily.
    flushTimer = setTimeout(() => {
      flushTimer = null;
      if (!somethingArrived) return;
      // Emit accumulated Add-to-reference first, then final Move
      if (pendingAddRef && Object.keys(pendingAddRef).some(k => k !== 'type')) {
        try { onCommand(pendingAddRef); } catch (_) {}
      }
      if (pendingMove && Object.keys(pendingMove).some(k => k !== 'type')) {
        try { onCommand(pendingMove); } catch (_) {}
      }
      // Reset window state
      pendingMove = null;
      pendingAddRef = null;
      somethingArrived = false;
    }, Math.max(1, Math.floor(dtSec * 1000)));
  };

  const handleCommand = (cmd) => {
    if (!(dtSec > 0)) {
      // No batching – pass through immediately
      onCommand(cmd);
      return;
    }

    somethingArrived = true;
    if (cmd && cmd.type === 'Move') {
      // Keep the latest absolute angles (A-D) but SUM extrusions (E) within the window
      if (!pendingMove) pendingMove = { type: 'Move' };
      for (const k of Object.keys(cmd)) {
        if (k === 'type') continue;
        if (k === 'E') {
          const prev = pendingMove.E || 0;
          pendingMove.E = prev + (cmd.E || 0);
        } else {
          pendingMove[k] = cmd[k];
        }
      }
    } else if (cmd && cmd.type === 'Add to reference') {
      // Sum deltas per axis for this window
      if (!pendingAddRef) pendingAddRef = { type: 'Add to reference' };
      for (const k of Object.keys(cmd)) {
        if (k === 'type') continue;
        const prev = pendingAddRef[k] || 0;
        pendingAddRef[k] = prev + cmd[k];
      }
    } else {
      // Unknown command kinds: pass through
      onCommand(cmd);
    }

    scheduleFlush();
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
    setDt: (newDt) => {
      const v = Number(newDt);
      if (Number.isFinite(v) && v >= 0) {
        dtSec = v;
        // Reset any in-flight timer so the new dt takes effect immediately
        if (flushTimer !== null) {
          clearTimeout(flushTimer);
          flushTimer = null;
        }
        // If we already have pending data, schedule a new flush with updated dt
        if (somethingArrived) scheduleFlush();
      }
    },
    close: () => {
      if (flushTimer !== null) {
        clearTimeout(flushTimer);
        flushTimer = null;
      }
      worker.postMessage({ type: 'close' });
    },
  };
}
