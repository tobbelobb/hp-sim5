// Observe the browser clock directly. Requested playback speed is never a clock.
export function createBrowserClock(request, {
  wallTimeoutMs = 120000, wallNow = () => Date.now(), poll = ms => new Promise(resolve => setTimeout(resolve, ms)),
  onProgress = message => console.log(message),
} = {}) {
  let sample;
  const clock = {
    backend: 'browser-js', source: 'browser.researchClock', settlingTimeoutMs: 30000,
    now: () => sample?.time_ms ?? 0,
    get observedWallMs() { return sample?.wall_time_ms; },
    observe(value) {
      if (!value || !Number.isFinite(value.time_ms) || !Number.isInteger(value.generation)) {
        throw new Error('Browser did not report a simulation clock; reload the updated simulator');
      }
      sample = value;
    },
    async refresh() { clock.observe((await request()).simulationClock); },
    async sleep(ms, { wallDeadlineMs = Infinity } = {}) {
      await clock.refresh();
      const target = clock.now() + Math.max(0, ms);
      const generation = sample.generation;
      const start = clock.now(), wallStart = wallNow();
      const deadline = Math.min(wallStart + wallTimeoutMs, wallDeadlineMs);
      let lastProgress = wallStart;
      while (clock.now() < target) {
        if (wallNow() >= deadline) throw new Error('Browser simulation clock did not advance before the wall-clock deadline');
        if (wallNow() - lastProgress >= 5000) {
          onProgress(`; waiting for browser clock: ${((wallNow() - wallStart) / 1000).toFixed(1)}s wall, ${((clock.now() - start) / 1000).toFixed(1)}s simulation of ${(ms / 1000).toFixed(1)}s requested duration`);
          lastProgress = wallNow();
        }
        await poll(20);
        const previous = clock.now();
        await clock.refresh();
        if (sample.generation !== generation || clock.now() < previous) {
          throw new Error('Browser scene/clock reset during a collector wait');
        }
      }
    },
  };
  return clock;
}
