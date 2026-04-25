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
    if (this.stepTimes.length > this.logEverySteps) {
      this.stepTimes.shift();
    }

    this.stepCount += 1;

    if (stepMs >= this.slowStepMs) {
      console.warn(`[perf] slow step: ${stepMs.toFixed(3)} ms`);
    }

    if (this.stepCount % this.logEverySteps === 0) {
      this.report();
      this.resetWindow();
    }
  }

  measureSystem(name, fn) {
    return this.measureBlock(name, fn);
  }

  report() {
    const sortedSteps = [...this.stepTimes].sort((a, b) => a - b);
    const avg =
      this.stepTimes.reduce((sum, value) => sum + value, 0) /
      Math.max(1, this.stepTimes.length);

    const p50 = percentile(sortedSteps, 0.50);
    const p95 = percentile(sortedSteps, 0.95);
    const max = sortedSteps[sortedSteps.length - 1] || 0;

    console.log(
      `[perf] steps=${this.stepCount} avg=${avg.toFixed(3)}ms ` +
      `p50=${p50.toFixed(3)}ms p95=${p95.toFixed(3)}ms max=${max.toFixed(3)}ms`
    );

    const rows = [...this.systemTotals.entries()]
      .map(([name, totalMs]) => {
        const count = this.systemCounts.get(name) || 0;
        return {
          system: name,
          calls: count,
          totalMs: Number(totalMs.toFixed(3)),
          avgPerCallMs: Number((totalMs / Math.max(1, count)).toFixed(4)),
          avgPerStepMs: Number((totalMs / Math.max(1, this.logEverySteps)).toFixed(4)),
          maxMs: Number((this.systemMax.get(name) || 0).toFixed(3)),
        };
      })
      .sort((a, b) => b.totalMs - a.totalMs);

    console.table(rows);
  }

  resetWindow() {
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
