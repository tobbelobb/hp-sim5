const DEG_TO_RAD = Math.PI / 180;

function clamp(value, min, max) {
  if (!Number.isFinite(value)) {
    return min;
  }
  return Math.min(Math.max(value, min), max);
}

function normalizeAngle(angle) {
  let a = angle;
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function distancePointToSegment(px, py, ax, ay, bx, by) {
  const abx = bx - ax;
  const aby = by - ay;
  const apx = px - ax;
  const apy = py - ay;
  const abLenSq = abx * abx + aby * aby;
  if (abLenSq <= 1e-16) {
    const dx = px - ax;
    const dy = py - ay;
    return Math.sqrt(dx * dx + dy * dy);
  }
  let t = (apx * abx + apy * aby) / abLenSq;
  t = Math.max(0, Math.min(1, t));
  const cx = ax + t * abx;
  const cy = ay + t * aby;
  const dx = px - cx;
  const dy = py - cy;
  return Math.sqrt(dx * dx + dy * dy);
}

function formatLengthMm(lengthMeters) {
  if (!Number.isFinite(lengthMeters)) {
    return '--';
  }
  const mm = lengthMeters * 1000;
  if (Math.abs(mm) >= 10) {
    return `${mm.toFixed(1)} mm`;
  }
  return `${mm.toFixed(2)} mm`;
}

function formatPercent(value) {
  if (!Number.isFinite(value)) {
    return '--';
  }
  return `${(value * 100).toFixed(1)}%`;
}

class StreamingQuantile {
  constructor(probability) {
    const clamped = clamp(Number(probability), 0, 1);
    this._prob = clamped > 0 && clamped < 1 ? clamped : 0.95;
    this.reset();
  }

  reset() {
    this._count = 0;
    this._initialized = false;
    this._initial = [];
    this._q = [0, 0, 0, 0, 0];
    this._n = [0, 0, 0, 0, 0];
    this._np = [0, 0, 0, 0, 0];
    this._dn = [0, 0, 0, 0, 0];
  }

  observe(value) {
    if (!Number.isFinite(value)) {
      return;
    }
    this._count += 1;
    if (!this._initialized) {
      this._initial.push(value);
      this._initial.sort((a, b) => a - b);
      if (this._initial.length === 5) {
        this._initializeMarkers();
      }
      return;
    }

    let k;
    if (value < this._q[0]) {
      this._q[0] = value;
      k = 0;
    } else if (value < this._q[1]) {
      k = 0;
    } else if (value < this._q[2]) {
      k = 1;
    } else if (value < this._q[3]) {
      k = 2;
    } else if (value <= this._q[4]) {
      k = 3;
    } else {
      this._q[4] = value;
      k = 3;
    }

    for (let i = k + 1; i < 5; i += 1) {
      this._n[i] += 1;
    }
    for (let i = 0; i < 5; i += 1) {
      this._np[i] += this._dn[i];
    }

    for (let i = 1; i <= 3; i += 1) {
      const delta = this._np[i] - this._n[i];
      const shouldIncrement = delta >= 1 && this._n[i + 1] - this._n[i] > 1;
      const shouldDecrement = delta <= -1 && this._n[i - 1] - this._n[i] < -1;
      if (!shouldIncrement && !shouldDecrement) {
        continue;
      }
      const direction = delta >= 0 ? 1 : -1;
      const candidate = this._parabolic(i, direction);
      if (candidate > this._q[i - 1] && candidate < this._q[i + 1]) {
        this._q[i] = candidate;
      } else {
        this._q[i] = this._linear(i, direction);
      }
      this._n[i] += direction;
    }
  }

  getEstimate() {
    if (this._count === 0) {
      return NaN;
    }
    if (!this._initialized) {
      const arr = this._initial;
      if (arr.length === 0) {
        return NaN;
      }
      if (arr.length === 1) {
        return arr[0];
      }
      const rank = this._prob * (arr.length - 1);
      const lower = Math.floor(rank);
      const upper = Math.min(arr.length - 1, lower + 1);
      const weight = rank - lower;
      if (upper === lower) {
        return arr[lower];
      }
      return arr[lower] + (arr[upper] - arr[lower]) * weight;
    }
    return this._q[2];
  }

  _initializeMarkers() {
    if (this._initial.length < 5) {
      return;
    }
    const sorted = this._initial;
    this._q = sorted.slice(0, 5);
    this._n = [1, 2, 3, 4, 5];
    const p = this._prob;
    this._np = [1, 1 + 2 * p, 1 + 4 * p, 3 + 2 * p, 5];
    this._dn = [0, p / 2, p, (1 + p) / 2, 1];
    this._initialized = true;
  }

  _parabolic(i, direction) {
    const q = this._q;
    const n = this._n;
    const denom = n[i + 1] - n[i - 1];
    if (denom === 0) {
      return q[i];
    }
    const forwardDen = n[i + 1] - n[i];
    const backwardDen = n[i] - n[i - 1];
    const forward = forwardDen === 0 ? 0 : direction * (n[i] - n[i - 1] + direction) * (q[i + 1] - q[i]) / forwardDen;
    const backward = backwardDen === 0 ? 0 : direction * (n[i + 1] - n[i] - direction) * (q[i] - q[i - 1]) / backwardDen;
    return q[i] + (forward + backward) / denom;
  }

  _linear(i, direction) {
    const q = this._q;
    const n = this._n;
    const denom = n[i + direction] - n[i];
    if (denom === 0) {
      return q[i];
    }
    return q[i] + (direction * (q[i + direction] - q[i])) / denom;
  }
}

export class QualityMonitor {
  constructor({ hudElement = null } = {}) {
    this.hudElement = hudElement || null;

    this.referenceSegments = [];
    this.segmentData = [];
    this.cornerData = [];
    this.referenceBounds = null;

    this.lastSegmentIndex = 0;
    this.pendingExtrusionCount = 0;
    this.pendingExtrusionCapacity = 0;
    this.pendingExtrusionsX = new Float64Array(0);
    this.pendingExtrusionsY = new Float64Array(0);
    this.pendingExtrusionsLength = new Float64Array(0);
    this._projectionScratch = {
      segmentIndex: 0,
      normalError: 0,
      tangentialError: 0,
      point: new Float64Array(2),
      t: 0,
      arcLength: 0,
      isStraight: false,
    };
    this._drainPromise = null;
    this.enabled = true;

    this.straightStats = {
      count: 0,
      sumSquares: 0,
      percentile95: new StreamingQuantile(0.95),
    };

    this.coverageGrid = null;
    this.coverageStats = null;

    this.remoteSystem = null;
    this.boundExtrusionListener = (event) => this.recordExtrusion(event);

    this.metrics = null;
    this.metricsDirty = true;
    this.extrusionsSinceHud = 0;

    // Tunable heuristics
    this.straightAngleThreshold = 2.5 * DEG_TO_RAD; // degrees
    this.cornerAngleThreshold = 12 * DEG_TO_RAD;
    this.cornerRadiusWindow = 0.006; // meters (6 mm)
    this.cornerRingingStart = 0.0008; // meters (0.8 mm)
    this.cornerRingingWindow = 0.004; // meters (4 mm)
    this.cornerRingingTail = 0.016; // meters (16 mm)
    this.cornerMinSamples = 3;
    this.cornerRingingMinAmplitude = 0.00005; // 0.05 mm

    this.coverageStep = 0.001; // 1 mm grid
    this.coverageMargin = 0.015; // 15 mm margin
    this.assumedBeadWidth = 0.0008; // 0.8 mm bead -> radius 0.4 mm

    this.hudUpdateInterval = 12;
  }

  attachRemoteSystem(system) {
    if (!system || typeof system.setExtrusionListener !== 'function') {
      return;
    }
    if (this.remoteSystem === system) {
      return;
    }
    if (this.remoteSystem) {
      try {
        this.remoteSystem.setExtrusionListener(null);
      } catch (_err) {
        // best effort
      }
    }
    this.remoteSystem = system;
    this.remoteSystem.setExtrusionListener(this.boundExtrusionListener);
  }

  reset({ keepReference = false } = {}) {
    this.lastSegmentIndex = 0;
    this.pendingExtrusionCount = 0;
    this.metricsDirty = true;
    this.metrics = null;
    this.extrusionsSinceHud = 0;
    this._drainPromise = null;

    this.straightStats.count = 0;
    this.straightStats.sumSquares = 0;
    if (this.straightStats.percentile95) {
      this.straightStats.percentile95.reset();
    }

    for (const corner of this.cornerData) {
      corner.samples = [];
      corner.ringingSamples = [];
    }

    if (this.coverageGrid && this.coverageStats) {
      this.coverageGrid.extrMask.fill(0);
      this.coverageStats.extruderCount = 0;
      this.coverageStats.intersectionCount = 0;
    }

    if (!keepReference) {
      this.referenceSegments = [];
      this.segmentData = [];
      this.cornerData = [];
      this.referenceBounds = null;
      this.coverageGrid = null;
      this.coverageStats = null;
    }

    this.refreshHud();
  }

  clearReference() {
    this.reset({ keepReference: false });
  }

  setReferenceSegments(segments, metadata = null) {
    if (!Array.isArray(segments) || segments.length === 0) {
      this.clearReference();
      return;
    }
    this.referenceSegments = segments.slice();
    this._prepareSegmentData();
    this._prepareCornerData();

    const bounds = metadata?.bounds || this._computeBoundsFromSegments();
    this.referenceBounds = bounds;
    this._buildCoverageGrid(bounds);

    this.reset({ keepReference: true });
    this.refreshHud();
  }

  recordExtrusion(extrusionEvent) {
    if (!extrusionEvent) {
      return;
    }
    if (!this.enabled) {
      const normalized = this._normalizeExtrusionEvent(extrusionEvent);
      if (normalized) {
        this._queueNormalizedExtrusion(normalized.x, normalized.y, normalized.length);
      }
      return;
    }
    this._recordExtrusionActive(extrusionEvent);
  }

  setEnabled(flag) {
    const next = Boolean(flag);
    if (this.enabled === next) {
      return;
    }
    this.enabled = next;
    if (this.enabled) {
      const drainPromise = this._drainPendingExtrusions();
      if (drainPromise && typeof drainPromise.then === 'function') {
        drainPromise
          .then(() => {
            this.refreshHud(true);
          })
          .catch((err) => {
            console.error('QualityMonitor: failed draining queued extrusions on enable.', err);
          });
      } else {
        this.refreshHud(true);
      }
    } else if (this.hudElement) {
      this.hudElement.classList.add('sim-hidden');
    }
  }

  runFinalCheck() {
    if (this.segmentData.length === 0) {
      return Promise.resolve();
    }
    const drainPromise = this._drainPendingExtrusions();
    if (!drainPromise || typeof drainPromise.then !== 'function') {
      this.metricsDirty = true;
      this.refreshHud(true);
      this.extrusionsSinceHud = 0;
      return Promise.resolve();
    }
    return drainPromise.then(() => {
      this.metricsDirty = true;
      this.refreshHud(true);
      this.extrusionsSinceHud = 0;
    });
  }

  _normalizeExtrusionEvent(extrusionEvent) {
    if (!extrusionEvent) {
      return null;
    }
    if (typeof extrusionEvent.x === 'number' && typeof extrusionEvent.y === 'number') {
      const x = Number(extrusionEvent.x);
      const y = Number(extrusionEvent.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        return null;
      }
      const length = Number.isFinite(extrusionEvent.length) ? Number(extrusionEvent.length) : 0;
      return { x, y, length };
    }

    const posSource = Array.isArray(extrusionEvent.pos)
      ? extrusionEvent.pos
      : extrusionEvent.pos && typeof extrusionEvent.pos === 'object'
        ? [extrusionEvent.pos.x, extrusionEvent.pos.y]
        : null;
    if (!posSource || posSource.length < 2) {
      return null;
    }
    const x = Number(posSource[0]);
    const y = Number(posSource[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return null;
    }
    const length = Number.isFinite(extrusionEvent.length)
      ? Number(extrusionEvent.length)
      : 0;
    return { x, y, length };
  }

  _ensurePendingExtrusionCapacity(minCount) {
    if (this.pendingExtrusionCapacity >= minCount) {
      return;
    }
    let nextCapacity = this.pendingExtrusionCapacity > 0 ? this.pendingExtrusionCapacity : 256;
    while (nextCapacity < minCount) {
      nextCapacity *= 2;
    }
    const nextX = new Float64Array(nextCapacity);
    const nextY = new Float64Array(nextCapacity);
    const nextLength = new Float64Array(nextCapacity);
    if (this.pendingExtrusionCount > 0) {
      nextX.set(this.pendingExtrusionsX.subarray(0, this.pendingExtrusionCount));
      nextY.set(this.pendingExtrusionsY.subarray(0, this.pendingExtrusionCount));
      nextLength.set(this.pendingExtrusionsLength.subarray(0, this.pendingExtrusionCount));
    }
    this.pendingExtrusionsX = nextX;
    this.pendingExtrusionsY = nextY;
    this.pendingExtrusionsLength = nextLength;
    this.pendingExtrusionCapacity = nextCapacity;
  }

  _queueNormalizedExtrusion(x, y, length) {
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return;
    }
    const safeLength = Number.isFinite(length) ? length : 0;
    const index = this.pendingExtrusionCount;
    this._ensurePendingExtrusionCapacity(index + 1);
    this.pendingExtrusionsX[index] = x;
    this.pendingExtrusionsY[index] = y;
    this.pendingExtrusionsLength[index] = safeLength;
    this.pendingExtrusionCount = index + 1;
  }

  _onExtrusionsProcessed(processedCount) {
    if (!Number.isFinite(processedCount) || processedCount <= 0) {
      return;
    }
    this.metricsDirty = true;
    this.extrusionsSinceHud += processedCount;
    if (!this.enabled || this.hudUpdateInterval <= 0) {
      return;
    }
    if (this.extrusionsSinceHud >= this.hudUpdateInterval) {
      this.refreshHud();
      this.extrusionsSinceHud %= this.hudUpdateInterval;
    }
  }

  _recordNormalizedExtrusion(x, y, length) {
    const scratch = this._projectionScratch;
    if (!scratch || !this._projectToPathIntoScratch(x, y, scratch)) {
      return false;
    }

    this._accumulateStraightError(scratch);
    this._accumulateCornerSamples(scratch);
    this._updateCoverageForExtrusion(x, y, length);
    return true;
  }

  _recordExtrusionActive(extrusionEvent) {
    if (this.segmentData.length === 0) {
      return;
    }
    const normalized = this._normalizeExtrusionEvent(extrusionEvent);
    if (!normalized) {
      return;
    }
    if (this._recordNormalizedExtrusion(normalized.x, normalized.y, normalized.length)) {
      this._onExtrusionsProcessed(1);
    }
  }

  _drainPendingExtrusions(options = {}) {
    if (this._drainPromise) {
      return this._drainPromise;
    }

    const { chunkSize = 4096, budgetMs = 24 } = options || {};
    const runDrain = async () => {
      const total = this.pendingExtrusionCount | 0;
      if (total === 0) {
        return 0;
      }
      const xs = this.pendingExtrusionsX;
      const ys = this.pendingExtrusionsY;
      const lengths = this.pendingExtrusionsLength;
      const record = this._recordNormalizedExtrusion;
      let processed = 0;
      let index = 0;
      const getNow = typeof performance !== 'undefined' && typeof performance.now === 'function'
        ? () => performance.now()
        : () => Date.now();
      const useChunking = this._canYieldToMainLoop() && total > chunkSize;

      while (index < total) {
        const sliceStart = getNow();
        const sliceEnd = useChunking ? Math.min(index + chunkSize, total) : total;
        for (let i = index; i < sliceEnd; i += 1) {
          const x = xs[i];
          const y = ys[i];
          const length = lengths[i];
          if (record.call(this, x, y, length)) {
            processed += 1;
          }
        }
        index = sliceEnd;
        if (!useChunking) {
          break;
        }
        if (index < total && (getNow() - sliceStart) >= budgetMs && this._shouldYieldToMainLoop()) {
          await this._yieldToMainLoop();
        }
      }

      this.pendingExtrusionCount = 0;
      this._onExtrusionsProcessed(processed);
      return processed;
    };

    const pending = runDrain();
    this._drainPromise = pending
      .then((result) => {
        this._drainPromise = null;
        return result;
      })
      .catch((error) => {
        this._drainPromise = null;
        throw error;
      });
    return this._drainPromise;
  }

  refreshHud(force = false) {
    if (this.segmentData.length === 0) {
      this.metrics = null;
      this.metricsDirty = false;
      if (this.hudElement) {
        this.hudElement.textContent = 'Quality metrics available once a reference path is loaded.';
        this.hudElement.classList.add('sim-hidden');
      }
      return;
    }
    if (this.metricsDirty) {
      this.metrics = this._computeMetrics();
      this.metricsDirty = false;
    }
    if (!this.hudElement) {
      return;
    }
    if (!force && !this.enabled) {
      this.hudElement.classList.add('sim-hidden');
      return;
    }
    const metrics = this.metrics;
    if (!metrics) {
      this.hudElement.textContent = 'Collecting metrics...';
      this.hudElement.classList.remove('sim-hidden');
      return;
    }
    const {
      rmseStraight,
      p95Straight,
      cornerRadiusAvg,
      cornerRadiusMax,
      ringingAmplitude,
      ringingDecay,
      coverage,
      iou,
      score,
    } = metrics;

    const lines = [];
    lines.push(`<div class="quality-hud__score">Quality <strong>${score.toFixed(0)}</strong></div>`);
    lines.push(`<div class="quality-hud__metric"><span>RMSE_n (straight)</span><span>${formatLengthMm(rmseStraight)}</span></div>`);
    lines.push(`<div class="quality-hud__metric"><span>95th |e_n|</span><span>${formatLengthMm(p95Straight)}</span></div>`);
    lines.push(`<div class="quality-hud__metric"><span>Corner radius avg / max</span><span>${formatLengthMm(cornerRadiusAvg)} / ${formatLengthMm(cornerRadiusMax)}</span></div>`);
    lines.push(`<div class="quality-hud__metric"><span>Ringing amplitude</span><span>${formatLengthMm(ringingAmplitude)}</span></div>`);
    lines.push(`<div class="quality-hud__metric"><span>Damping estimate</span><span>${Number.isFinite(ringingDecay) ? ringingDecay.toFixed(2) : '--'}</span></div>`);
    lines.push(`<div class="quality-hud__metric"><span>Coverage / IoU</span><span>${formatPercent(coverage)} / ${formatPercent(iou)}</span></div>`);

    this.hudElement.innerHTML = lines.join('');
    this.hudElement.classList.remove('sim-hidden');
  }

  _prepareSegmentData() {
    this.segmentData = [];
    let cumulative = 0;
    let prevAngle = null;
    for (const segment of this.referenceSegments) {
      if (!segment || !Array.isArray(segment.start) || !Array.isArray(segment.end)) {
        continue;
      }
      const start = segment.start;
      const end = segment.end;
      const sx = Number(start[0]);
      const sy = Number(start[1]);
      const ex = Number(end[0]);
      const ey = Number(end[1]);
      if (!Number.isFinite(sx) || !Number.isFinite(sy) || !Number.isFinite(ex) || !Number.isFinite(ey)) {
        continue;
      }
      const dx = ex - sx;
      const dy = ey - sy;
      const length = Math.hypot(dx, dy);
      if (!(length > 1e-9)) {
        continue;
      }
      const tangentX = dx / length;
      const tangentY = dy / length;
      const normalX = -tangentY;
      const normalY = tangentX;
      const angle = Math.atan2(dy, dx);
      const deltaAngle = prevAngle == null ? 0 : normalizeAngle(angle - prevAngle);
      const isStraight = prevAngle == null || Math.abs(deltaAngle) < this.straightAngleThreshold;

      this.segmentData.push({
        start: [sx, sy],
        end: [ex, ey],
        length,
        tangent: [tangentX, tangentY],
        normal: [normalX, normalY],
        angle,
        deltaAngle,
        isStraight,
        cumulativeStart: cumulative,
        cumulativeEnd: cumulative + length,
      });
      cumulative += length;
      prevAngle = angle;
    }
  }

  _prepareCornerData() {
    this.cornerData = [];
    for (let i = 1; i < this.segmentData.length; i += 1) {
      const seg = this.segmentData[i];
      if (Math.abs(seg.deltaAngle) < this.cornerAngleThreshold) {
        continue;
      }
      this.cornerData.push({
        index: i,
        arcLength: seg.cumulativeStart,
        angle: seg.deltaAngle,
        pivot: [...seg.start],
        samples: [],
        ringingSamples: [],
      });
    }
  }

  _computeBoundsFromSegments() {
    if (this.segmentData.length === 0) {
      return null;
    }
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const seg of this.segmentData) {
      minX = Math.min(minX, seg.start[0], seg.end[0]);
      minY = Math.min(minY, seg.start[1], seg.end[1]);
      maxX = Math.max(maxX, seg.start[0], seg.end[0]);
      maxY = Math.max(maxY, seg.start[1], seg.end[1]);
    }
    return {
      minX,
      minY,
      maxX,
      maxY,
    };
  }

  _buildCoverageGrid(bounds) {
    if (!bounds) {
      this.coverageGrid = null;
      this.coverageStats = null;
      return;
    }
    const margin = this.coverageMargin;
    const minX = bounds.minX - margin;
    const maxX = bounds.maxX + margin;
    const minY = bounds.minY - margin;
    const maxY = bounds.maxY + margin;
    const step = this.coverageStep;
    const width = Math.max(1, Math.ceil((maxX - minX) / step) + 1);
    const height = Math.max(1, Math.ceil((maxY - minY) / step) + 1);
    const refMask = new Uint8Array(width * height);
    const extrMask = new Uint8Array(width * height);
    this.coverageGrid = {
      step,
      originX: minX,
      originY: minY,
      width,
      height,
      refMask,
      extrMask,
    };
    this.coverageStats = {
      referenceCount: 0,
      extruderCount: 0,
      intersectionCount: 0,
    };
    this._rasterizeReferenceMask();
  }

  _rasterizeReferenceMask() {
    if (!this.coverageGrid) {
      return;
    }
    const grid = this.coverageGrid;
    const stats = this.coverageStats;
    const beadRadius = this.assumedBeadWidth * 0.5;
    for (const seg of this.segmentData) {
      const minX = Math.min(seg.start[0], seg.end[0]) - beadRadius;
      const maxX = Math.max(seg.start[0], seg.end[0]) + beadRadius;
      const minY = Math.min(seg.start[1], seg.end[1]) - beadRadius;
      const maxY = Math.max(seg.start[1], seg.end[1]) + beadRadius;
      const ix0 = clamp(Math.floor((minX - grid.originX) / grid.step), 0, grid.width - 1);
      const ix1 = clamp(Math.ceil((maxX - grid.originX) / grid.step), 0, grid.width - 1);
      const iy0 = clamp(Math.floor((minY - grid.originY) / grid.step), 0, grid.height - 1);
      const iy1 = clamp(Math.ceil((maxY - grid.originY) / grid.step), 0, grid.height - 1);
      for (let ix = ix0; ix <= ix1; ix += 1) {
        const cx = grid.originX + (ix + 0.5) * grid.step;
        for (let iy = iy0; iy <= iy1; iy += 1) {
          const cy = grid.originY + (iy + 0.5) * grid.step;
          const dist = distancePointToSegment(
            cx,
            cy,
            seg.start[0],
            seg.start[1],
            seg.end[0],
            seg.end[1],
          );
          if (dist <= beadRadius) {
            const index = iy * grid.width + ix;
            if (grid.refMask[index] === 0) {
              grid.refMask[index] = 1;
              stats.referenceCount += 1;
            }
          }
        }
      }
    }
  }

  _estimateExtrusionRadius(length) {
    if (!Number.isFinite(length) || length <= 0) {
      return this.assumedBeadWidth * 0.5;
    }
    return Math.max(
      this.assumedBeadWidth * 0.4,
      Math.sqrt(length / Math.PI) * 0.01 * 0.5,
    );
  }

  _updateCoverageForExtrusion(x, y, length) {
    if (!this.coverageGrid || !this.coverageStats) {
      return;
    }
    const radius = this._estimateExtrusionRadius(length);
    const grid = this.coverageGrid;
    const stats = this.coverageStats;
    const minX = x - radius;
    const maxX = x + radius;
    const minY = y - radius;
    const maxY = y + radius;
    const ix0 = clamp(Math.floor((minX - grid.originX) / grid.step), 0, grid.width - 1);
    const ix1 = clamp(Math.ceil((maxX - grid.originX) / grid.step), 0, grid.width - 1);
    const iy0 = clamp(Math.floor((minY - grid.originY) / grid.step), 0, grid.height - 1);
    const iy1 = clamp(Math.ceil((maxY - grid.originY) / grid.step), 0, grid.height - 1);
    for (let ix = ix0; ix <= ix1; ix += 1) {
      const cx = grid.originX + (ix + 0.5) * grid.step;
      for (let iy = iy0; iy <= iy1; iy += 1) {
        const cy = grid.originY + (iy + 0.5) * grid.step;
        const dx = cx - x;
        const dy = cy - y;
        if (dx * dx + dy * dy <= radius * radius) {
          const index = iy * grid.width + ix;
          if (grid.extrMask[index] === 0) {
            grid.extrMask[index] = 1;
            stats.extruderCount += 1;
            if (grid.refMask[index] === 1) {
              stats.intersectionCount += 1;
            }
          }
        }
      }
    }
  }

  _projectToPathIntoScratch(x, y, scratch) {
    if (!scratch || this.segmentData.length === 0) {
      return false;
    }
    let bestDistSq = Infinity;
    let bestIndex = -1;
    const segments = this.segmentData;
    const count = segments.length;
    const searchRadius = 6;
    const hint = clamp(this.lastSegmentIndex, 0, count - 1);

    const evaluateSegment = (index) => {
      const seg = segments[index];
      const sx = seg.start[0];
      const sy = seg.start[1];
      const ex = seg.end[0];
      const ey = seg.end[1];
      const vx = ex - sx;
      const vy = ey - sy;
      const lengthSq = seg.length * seg.length;
      if (!(lengthSq > 1e-16)) {
        return;
      }
      let t = ((x - sx) * vx + (y - sy) * vy) / lengthSq;
      t = Math.max(0, Math.min(1, t));
      const nx = sx + t * vx;
      const ny = sy + t * vy;
      const dx = x - nx;
      const dy = y - ny;
      const distSq = dx * dx + dy * dy;
      if (distSq >= bestDistSq) {
        return;
      }
      bestDistSq = distSq;
      bestIndex = index;
      scratch.segmentIndex = index;
      scratch.normalError = dx * seg.normal[0] + dy * seg.normal[1];
      scratch.tangentialError = dx * seg.tangent[0] + dy * seg.tangent[1];
      scratch.point[0] = nx;
      scratch.point[1] = ny;
      scratch.t = t;
      scratch.arcLength = seg.cumulativeStart + t * seg.length;
      scratch.isStraight = Boolean(seg.isStraight);
    };

    const startIdx = Math.max(0, hint - searchRadius);
    const endIdx = Math.min(count - 1, hint + searchRadius);
    for (let i = startIdx; i <= endIdx; i += 1) {
      evaluateSegment(i);
    }
    if (bestIndex === -1) {
      for (let i = 0; i < count; i += 1) {
        evaluateSegment(i);
      }
    }
    if (bestIndex !== -1) {
      this.lastSegmentIndex = bestIndex;
      return true;
    }
    return false;
  }

  _canYieldToMainLoop() {
    return typeof requestAnimationFrame === 'function' || typeof setTimeout === 'function';
  }

  _shouldYieldToMainLoop() {
    if (typeof navigator !== 'undefined' && navigator?.scheduling?.isInputPending) {
      try {
        return navigator.scheduling.isInputPending();
      } catch (_err) {
        return true;
      }
    }
    return true;
  }

  _yieldToMainLoop() {
    if (typeof requestAnimationFrame === 'function') {
      return new Promise((resolve) => requestAnimationFrame(resolve));
    }
    if (typeof setTimeout === 'function') {
      return new Promise((resolve) => setTimeout(resolve, 0));
    }
    return Promise.resolve();
  }

  _accumulateStraightError(projection) {
    if (!projection.isStraight) {
      return;
    }
    const err = projection.normalError;
    this.straightStats.count += 1;
    this.straightStats.sumSquares += err * err;
    if (this.straightStats.percentile95) {
      this.straightStats.percentile95.observe(Math.abs(err));
    }
  }

  _accumulateCornerSamples(projection) {
    if (this.cornerData.length === 0) {
      return;
    }
    for (const corner of this.cornerData) {
      const delta = projection.arcLength - corner.arcLength;
      if (Math.abs(delta) <= this.cornerRadiusWindow) {
        corner.samples.push({
          delta,
          normalError: projection.normalError,
        });
      }
      if (delta >= this.cornerRingingStart && delta <= this.cornerRingingTail) {
        corner.ringingSamples.push({
          s: delta,
          normalError: projection.normalError,
        });
      }
    }
  }

  _computeMetrics() {
    const stats = this.straightStats;
    const rmse = stats.count > 0 ? Math.sqrt(stats.sumSquares / stats.count) : NaN;
    let p95 = NaN;
    if (stats.percentile95) {
      const estimate = stats.percentile95.getEstimate();
      if (Number.isFinite(estimate)) {
        p95 = estimate;
      }
    }

    const radiusValues = [];
    const radiusMaxValues = [];
    const amplitudeValues = [];
    const dampingValues = [];

    for (const corner of this.cornerData) {
      const windowSamples = corner.samples.filter((sample) => Math.abs(sample.delta) <= this.cornerRadiusWindow);
      if (windowSamples.length >= this.cornerMinSamples) {
        const avg = windowSamples.reduce((sum, sample) => sum + Math.abs(sample.normalError), 0) / windowSamples.length;
        const max = windowSamples.reduce((acc, sample) => Math.max(acc, Math.abs(sample.normalError)), 0);
        radiusValues.push(avg);
        radiusMaxValues.push(max);
      }

      const ringingSamples = corner.ringingSamples;
      if (ringingSamples.length > 0) {
        let firstWindowMax = 0;
        let secondWindowMax = 0;
        for (const sample of ringingSamples) {
          if (sample.s <= this.cornerRingingStart + this.cornerRingingWindow) {
            firstWindowMax = Math.max(firstWindowMax, Math.abs(sample.normalError));
          } else if (sample.s <= this.cornerRingingStart + 2 * this.cornerRingingWindow) {
            secondWindowMax = Math.max(secondWindowMax, Math.abs(sample.normalError));
          }
        }
        if (firstWindowMax > this.cornerRingingMinAmplitude) {
          amplitudeValues.push(firstWindowMax);
        }
        if (firstWindowMax > this.cornerRingingMinAmplitude && secondWindowMax > this.cornerRingingMinAmplitude) {
          const delta = Math.log(firstWindowMax / secondWindowMax);
          if (Number.isFinite(delta) && delta > 0) {
            const damping = delta / Math.sqrt(4 * Math.PI * Math.PI + delta * delta);
            dampingValues.push(damping);
          }
        }
      }
    }

    const radiusAvg = radiusValues.length > 0
      ? radiusValues.reduce((sum, value) => sum + value, 0) / radiusValues.length
      : NaN;
    const radiusMax = radiusMaxValues.length > 0
      ? radiusMaxValues.reduce((max, value) => Math.max(max, value), 0)
      : NaN;
    const amplitude = amplitudeValues.length > 0
      ? amplitudeValues.reduce((sum, value) => sum + value, 0) / amplitudeValues.length
      : NaN;
    const damping = dampingValues.length > 0
      ? dampingValues.reduce((sum, value) => sum + value, 0) / dampingValues.length
      : NaN;

    let coverage = NaN;
    let iou = NaN;
    if (this.coverageStats) {
      const ref = this.coverageStats.referenceCount;
      const extr = this.coverageStats.extruderCount;
      const inter = this.coverageStats.intersectionCount;
      const union = ref + extr - inter;
      coverage = ref > 0 ? inter / ref : NaN;
      iou = union > 0 ? inter / union : NaN;
    }

    const score = this._composeScore({
      rmse,
      p95,
      radiusAvg,
      amplitude,
      coverage,
      iou,
    });

    return {
      rmseStraight: rmse,
      p95Straight: p95,
      cornerRadiusAvg: radiusAvg,
      cornerRadiusMax: radiusMax,
      ringingAmplitude: amplitude,
      ringingDecay: damping,
      coverage,
      iou,
      score,
    };
  }

  _composeScore({ rmse, p95, radiusAvg, amplitude, coverage, iou }) {
    let score = 100;
    if (Number.isFinite(rmse)) {
      const rmseMm = rmse * 1000;
      score -= clamp(rmseMm * 10, 0, 35);
    }
    if (Number.isFinite(p95)) {
      const p95Mm = p95 * 1000;
      score -= clamp(Math.max(0, p95Mm - 0.2) * 8, 0, 25);
    }
    if (Number.isFinite(radiusAvg)) {
      const radiusMm = radiusAvg * 1000;
      score -= clamp(radiusMm * 6, 0, 20);
    }
    if (Number.isFinite(amplitude)) {
      const ampMm = amplitude * 1000;
      score -= clamp(ampMm * 8, 0, 15);
    }
    if (Number.isFinite(coverage)) {
      score -= clamp((1 - coverage) * 60, 0, 15);
    }
    if (Number.isFinite(iou)) {
      score -= clamp((1 - iou) * 50, 0, 15);
    }
    return clamp(score, 0, 100);
  }
}
