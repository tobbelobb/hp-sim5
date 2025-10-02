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

export class QualityMonitor {
  constructor({ hudElement = null } = {}) {
    this.hudElement = hudElement || null;

    this.referenceSegments = [];
    this.segmentData = [];
    this.cornerData = [];
    this.referenceBounds = null;

    this.extrusions = [];
    this.lastSegmentIndex = 0;

    this.straightStats = {
      count: 0,
      sumSquares: 0,
      absErrors: [],
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
    this.extrusions = [];
    this.lastSegmentIndex = 0;
    this.metricsDirty = true;
    this.metrics = null;
    this.extrusionsSinceHud = 0;

    this.straightStats.count = 0;
    this.straightStats.sumSquares = 0;
    this.straightStats.absErrors = [];

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
    if (!extrusionEvent || !extrusionEvent.pos || this.segmentData.length === 0) {
      return;
    }
    const pos = Array.isArray(extrusionEvent.pos)
      ? extrusionEvent.pos
      : [extrusionEvent.pos?.x, extrusionEvent.pos?.y];
    if (pos.length < 2) {
      return;
    }
    const x = Number(pos[0]);
    const y = Number(pos[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return;
    }
    const projection = this._projectToPath(x, y);
    if (!projection) {
      return;
    }

    this.extrusions.push({ x, y, length: extrusionEvent.length || 0 });

    this._accumulateStraightError(projection);
    this._accumulateCornerSamples(projection);
    this._updateCoverageForExtrusion(x, y, extrusionEvent.length || 0);

    this.metricsDirty = true;
    this.extrusionsSinceHud += 1;
    if (this.extrusionsSinceHud >= this.hudUpdateInterval) {
      this.refreshHud();
      this.extrusionsSinceHud = 0;
    }
  }

  refreshHud() {
    if (!this.hudElement) {
      this.metricsDirty = false;
      return;
    }
    if (this.segmentData.length === 0) {
      this.hudElement.textContent = 'Quality metrics available once a reference path is loaded.';
      this.hudElement.classList.add('sim-hidden');
      return;
    }
    if (this.metricsDirty) {
      this.metrics = this._computeMetrics();
      this.metricsDirty = false;
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

  _projectToPath(x, y) {
    if (this.segmentData.length === 0) {
      return null;
    }
    let best = null;
    let bestDistSq = Infinity;
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
      if (distSq < bestDistSq) {
        const normalError = dx * seg.normal[0] + dy * seg.normal[1];
        const tangentialError = dx * seg.tangent[0] + dy * seg.tangent[1];
        bestDistSq = distSq;
        best = {
          segmentIndex: index,
          normalError,
          tangentialError,
          point: [nx, ny],
          t,
          arcLength: seg.cumulativeStart + t * seg.length,
          isStraight: seg.isStraight,
        };
      }
    };

    const startIdx = Math.max(0, hint - searchRadius);
    const endIdx = Math.min(count - 1, hint + searchRadius);
    for (let i = startIdx; i <= endIdx; i += 1) {
      evaluateSegment(i);
    }
    if (!best) {
      for (let i = 0; i < count; i += 1) {
        evaluateSegment(i);
      }
    }
    if (best) {
      this.lastSegmentIndex = best.segmentIndex;
    }
    return best;
  }

  _accumulateStraightError(projection) {
    if (!projection.isStraight) {
      return;
    }
    const err = projection.normalError;
    this.straightStats.count += 1;
    this.straightStats.sumSquares += err * err;
    this.straightStats.absErrors.push(Math.abs(err));
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
    if (stats.absErrors.length > 0) {
      const sorted = stats.absErrors.slice().sort((a, b) => a - b);
      const idx = Math.min(sorted.length - 1, Math.floor(0.95 * (sorted.length - 1)));
      p95 = sorted[idx];
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

