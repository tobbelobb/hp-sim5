// KlipperHandler: Connects to a raw-bytes WebSocket via a worker.
const DEBUG = true; // Note: most logging is now in the worker.

export function connectKlipperRaw(url, onCommand /* function(command) */) {
  // Use a URL object to construct a path relative to this module's location.
  // This is more robust than hardcoding paths, especially with bundlers/vite.
  const workerPath = new URL('./klipperPacer.js', import.meta.url).href;
  const worker = new Worker(workerPath, { type: 'module' });

  worker.postMessage({ type: 'connect', url, debug: DEBUG });

  worker.onmessage = (e) => {
    const { type, command, args } = e.data;
    if (type === 'move') {
      if (typeof onCommand === 'function') {
        onCommand(command);
      }
    } else if (type === 'log' && DEBUG) {
      console.log('KlipperPacer:', ...args);
    } else if (type === 'error') {
      console.error('KlipperPacer:', ...args);
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
    close: () => worker.postMessage({ type: 'close' }),
  };
}
