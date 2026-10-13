export class PerformanceMonitor {
  constructor({
    enabled = false,
    logEverySteps = 300,
    slowStepMs = 8.0,
  } = {}) {
    this.enabled = enabled;
    this.logEverySteps = logEverySteps;
    this.slowStepMs = slowStepMs;

    this.stepCount = 0;
    this.windowStepCount = 0;
    this.stepStartMs = 0;
    this.stepTimes = [];
    this.systemTotals = new Map();
    this.systemMax = new Map();
    this.systemCounts = new Map();
  }

  beginStep() {
    if (!this.enabled) return;
    this.stepStartMs = performance.now();
  }

  endStep() {
    if (!this.enabled) return;

    const stepMs = performance.now() - this.stepStartMs;
    this.stepTimes.push(stepMs);
    if (this.logEverySteps > 0 && this.stepTimes.length > this.logEverySteps) {
      this.stepTimes.shift();
    }

    this.stepCount += 1;
    this.windowStepCount += 1;

    if (stepMs >= this.slowStepMs) {
      console.warn(`[perf] slow step: ${stepMs.toFixed(3)} ms`);
    }

    if (this.logEverySteps > 0 && this.stepCount % this.logEverySteps === 0) {
      this.report();
      this.resetWindow();
    }
  }

  measureSystem(name, fn) {
    return this.measureBlock(name, fn);
  }

  // logEverySteps=0 retains the whole measurement until resetWindow(), without
  // periodic console output. Snapshots also work for a partial browser window.
  snapshot() {
    const sortedSteps = [...this.stepTimes].sort((a, b) => a - b);
    const avg =
      this.stepTimes.reduce((sum, value) => sum + value, 0) /
      Math.max(1, this.stepTimes.length);

    const p50 = percentile(sortedSteps, 0.50);
    const p95 = percentile(sortedSteps, 0.95);
    const max = sortedSteps[sortedSteps.length - 1] || 0;

    const rows = [...this.systemTotals.entries()]
      .map(([name, totalMs]) => {
        const count = this.systemCounts.get(name) || 0;
        return {
          system: name,
          calls: count,
          totalMs,
          avgPerCallMs: totalMs / Math.max(1, count),
          avgPerStepMs: totalMs / Math.max(1, this.windowStepCount),
          maxMs: this.systemMax.get(name) || 0,
        };
      })
      .sort((a, b) => b.totalMs - a.totalMs);

    return { steps: this.windowStepCount, avgMs: avg, p50Ms: p50,
      p95Ms: p95, maxMs: max, systems: rows };
  }

  report() {
    const stats = this.snapshot();
    console.log(
      `[perf] steps=${this.stepCount} avg=${stats.avgMs.toFixed(3)}ms ` +
      `p50=${stats.p50Ms.toFixed(3)}ms p95=${stats.p95Ms.toFixed(3)}ms max=${stats.maxMs.toFixed(3)}ms`
    );
    console.table(stats.systems.map(row => Object.fromEntries(Object.entries(row)
      .map(([key, value]) => [key, typeof value === 'number' && key !== 'calls' ? Number(value.toFixed(4)) : value]))));
  }

  resetWindow() {
    this.windowStepCount = 0;
    this.stepTimes.length = 0;
    this.systemTotals.clear();
    this.systemMax.clear();
    this.systemCounts.clear();
  }

  measureBlock(name, fn) {
    if (!this.enabled) {
      return fn();
    }

    const t0 = performance.now();
    try {
      return fn();
    } finally {
      this.measureSample(name, performance.now() - t0);
    }
  }

  measureSample(name, dt) {
    if (!this.enabled) return;
    this.systemTotals.set(name, (this.systemTotals.get(name) || 0) + dt);
    this.systemMax.set(name, Math.max(this.systemMax.get(name) || 0, dt));
    this.systemCounts.set(name, (this.systemCounts.get(name) || 0) + 1);
  }
}

function percentile(sorted, p) {
  if (sorted.length === 0) return 0;
  const i = Math.min(sorted.length - 1, Math.floor(sorted.length * p));
  return sorted[i];
}
