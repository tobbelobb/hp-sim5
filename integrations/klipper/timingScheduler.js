/**
 * timingScheduler.js
 * A minimal worker that sleeps using Atomics.wait on a SharedArrayBuffer.
 * Messages:
 *  - { type: 'init', sab }       : Initialize with a SharedArrayBuffer (Int32 length >= 1)
 *  - { type: 'sleep', ms }       : Sleep for up to 'ms' milliseconds or until notified via SAB
 *  - { type: 'shutdown' }        : Wake and terminate the worker
 *
 * The main thread should preempt sleeps by doing:
 *   Atomics.store(i32, 0, 1); Atomics.notify(i32, 0, 1);
 */
let i32 = null;

self.onmessage = (e) => {
  const msg = e.data || {};
  if (msg.type === 'init') {
    i32 = new Int32Array(msg.sab);
    return;
  }
  if (msg.type === 'sleep') {
    if (!i32) return;
    try {
      Atomics.store(i32, 0, 0);
      const dur = Math.max(0, Math.floor(Number(msg.ms) || 0));
      const res = Atomics.wait(i32, 0, 0, dur);
      self.postMessage({ type: 'wakeup', reason: res });
    } catch (_) {
      // If SAB/Atomics are unavailable, don't block.
      self.postMessage({ type: 'wakeup', reason: 'error' });
    }
    return;
  }
  if (msg.type === 'shutdown') {
    try {
      if (i32) {
        Atomics.store(i32, 0, 1);
        Atomics.notify(i32, 0, 1);
      }
    } catch (_) {}
    self.close();
    return;
  }
};
