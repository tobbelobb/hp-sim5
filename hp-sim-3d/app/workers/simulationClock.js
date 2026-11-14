// A dedicated worker clock continues to tick when animation frames are stopped
// in a hidden tab. Browser freezing/discarding can still suspend the whole app.
const sleeper = typeof SharedArrayBuffer === 'function'
  ? new Int32Array(new SharedArrayBuffer(4))
  : null;

self.onmessage = ({ data: delay }) => {
  if (sleeper) {
    // Sleep without browser timer throttling on cross-origin-isolated hosts.
    Atomics.wait(sleeper, 0, 0, delay);
    self.postMessage(null);
  } else {
    setTimeout(() => self.postMessage(null), delay);
  }
};
