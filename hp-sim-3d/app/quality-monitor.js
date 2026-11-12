const DEG_TO_RAD = Math.PI / 180;

const DEFAULT_HUD_ACCENT = '#7ae2ff';
const REMOTE_SYSTEM_BINDINGS = new WeakMap();

function clamp(value, min, max) {
  if (!Number.isFinite(value)) {
    return min;
  }
  return Math.min(Math.max(value, min), max);
}

function formatLengthMm(lengthMeters) {
  if (!Number.isFinite(lengthMeters)) {
    return '--';
  }
  const mm = lengthMeters * 1000;
  if (Math.abs(mm) >= 10) {
    return `${mm.toFixed(1)}`;
  }
  return `${mm.toFixed(2)}`;
}

function formatPercent(value) {
  if (!Number.isFinite(value)) {
    return '--';
  }
  return `${(value * 100).toFixed(1)}%`;
}

function escapeHtml(value) {
  if (typeof value !== 'string') {
    return '';
  }
  return value
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function sanitizeHexColor(value) {
  if (typeof value !== 'string') {
    return null;
  }
  const trimmed = value.trim();
  if (/^#[0-9a-fA-F]{6}$/.test(trimmed)) {
    return trimmed;
  }
  return null;
}

function now() {
  return globalThis.performance?.now?.() ?? Date.now();
}

// Distances binned on a log scale (2 % wide bins from 0.1 µm to 10 m), so any
// percentile is read off to within one bin, in O(1) memory and time per sample.
class DistanceHistogram {
  constructor() {
    this.min = 1e-7;
    this.logRatio = Math.log(1.02);
    this.counts = new Float64Array(Math.ceil(Math.log(10 / this.min) / this.logRatio) + 1);
    this.reset();
  }

  reset() {
    this.counts.fill(0);
    this.count = 0;
    this.sumSquares = 0;
    this.max = 0;
  }

  observe(distance) {
    const bin = distance <= this.min
      ? 0
      : Math.min(this.counts.length - 1, Math.ceil(Math.log(distance / this.min) / this.logRatio));
    this.counts[bin] += 1;
    this.count += 1;
    this.sumSquares += distance * distance;
    if (distance > this.max) {
      this.max = distance;
    }
  }

  rms() {
    return this.count > 0 ? Math.sqrt(this.sumSquares / this.count) : NaN;
  }

  // Upper edge of the bin holding the p-quantile, capped by the exact maximum.
  quantile(p) {
    if (this.count === 0) {
      return NaN;
    }
    const rank = p * this.count;
    let cumulative = 0;
    for (let bin = 0; bin < this.counts.length; bin += 1) {
      cumulative += this.counts[bin];
      if (cumulative >= rank) {
        return Math.min(this.max, bin === 0 ? this.min : this.min * Math.exp(bin * this.logRatio));
      }
    }
    return this.max;
  }
}

function distancePointToSegment(px, py, pz, ax, ay, az, bx, by, bz) {
  const abx = bx - ax;
  const aby = by - ay;
  const abz = bz - az;
  const lengthSq = abx * abx + aby * aby + abz * abz;
  let t = 0;
  if (lengthSq > 0) {
    t = clamp(((px - ax) * abx + (py - ay) * aby + (pz - az) * abz) / lengthSq, 0, 1);
  }
  return Math.hypot(px - (ax + t * abx), py - (ay + t * aby), pz - (az + t * abz));
}

// Compares where each extrusion sample was deposited with the G-code it came
// from. The cumulative extrusion (E) of the command stream locates every sample
// on the reference path, which gives:
// - position error: distance to the intended point at that instant (tracking
//   error) and its along-path part (lag);
// - the systematic offset: least-squares shift of the error across the path;
// - path deviation: distance to the intended path near that point (contour
//   error, as ISO 9283 path accuracy), split into horizontal and vertical;
// - per sharp corner (after removing the mean offset): deviation of the
//   attained path from the corner point,
//   the largest deviation after it, and the distance travelled until the path
//   stays within tolerance (ISO 9283 cornering deviations / stabilization).
// The 0-100 quality score weighs what matters to someone looking at the part:
// its shape (path deviation once the print's mean offset is removed) counts
// most, so a rigidly shifted but otherwise perfect print still scores high,
// while distortion, layer shifts and blow-ups score low. See SCORE_TERMS.

// Each term deducts weight * (1 - exp(-value / scale)) points; weights sum to 100.
const SCORE_TERMS = Object.freeze([
  { key: 'shapeP95', label: 'Shape 95%', weight: 45, scale: 0.001 },
  { key: 'shapeRms', label: 'Shape RMS', weight: 20, scale: 0.0006 },
  { key: 'cornerError', label: 'Corners', weight: 10, scale: 0.001 },
  { key: 'settleDistance', label: 'Ringing', weight: 5, scale: 0.005 },
  { key: 'offset', label: 'Offset', weight: 10, scale: 0.005 },
  { key: 'missedSteps', label: 'Missed steps', weight: 10, scale: 20 },
]);
export class QualityMonitor {
  constructor({ hudElement = null } = {}) {
    this.hudElement = hudElement || null;

    this.machineId = null;
    this.machineLabel = null;
    this.machineTintColor = null;
    this.hudAccentColor = DEFAULT_HUD_ACCENT;

    this.visibilityCallback = null;
    this._lastHudHidden = null;

    this.entries = [];
    this.corners = [];
    this.totalExtrusion = 0;

    this.enabled = true;

    this.contourHistogram = new DistanceHistogram();
    this.trackingHistogram = new DistanceHistogram();
    this.shapeHistogram = new DistanceHistogram();

    this.remoteSystem = null;
    this.boundExtrusionListener = (event) => this.recordExtrusion(event);
    this.motorDiagnosticsProvider = null;
    this.qualityDetailsExpanded = false;
    this.missedStepsExpanded = false;
    this.detailsExpandedCallback = null;
    this.boundHudPointerDown = (event) => this._handleHudPointerDown(event);
    this.boundHudClick = (event) => this._handleHudClick(event);
    if (this.hudElement && typeof this.hudElement.addEventListener === 'function') {
      this.hudElement.addEventListener('pointerdown', this.boundHudPointerDown);
      this.hudElement.addEventListener('click', this.boundHudClick);
    }

    this.metrics = null;
    this.metricsDirty = true;
    this.lastHudRefreshMs = -Infinity;

    // Tunable heuristics
    this.tolerance = 0.0005; // meters: within-tolerance share, settling, extrusion tint starts
    this.fineTolerance = 0.00025; // meters: second, stricter within-tolerance share
    this.penaltyHardThreshold = 0.0015; // meters (1.5 mm) saturates the extrusion tint
    this.contourSearchWindow = 0.005; // meters of path either side of the intended point
    this.travelSyncExtrusion = 0.1; // filament length a line before a travel may be short by
    this.cornerAngleThreshold = 45 * DEG_TO_RAD;
    this.cornerWindow = 0.006; // meters of path before/after a corner for corner deviation
    this.cornerSettleWindow = 0.016; // meters of path after a corner; its end is the settled reference
    this.hudRefreshIntervalMs = 250;

    this._resetAccumulators();
  }

  setVisibilityCallback(callback) {
    this.visibilityCallback = typeof callback === 'function' ? callback : null;
    this._notifyHudVisibility();
  }

  setDetailsExpandedCallback(callback) {
    this.detailsExpandedCallback = typeof callback === 'function' ? callback : null;
  }

  _notifyHudVisibility() {
    if (!this.hudElement || !this.visibilityCallback) {
      return;
    }
    const hidden = this.hudElement.classList.contains('sim-hidden');
    if (this._lastHudHidden === hidden) {
      return;
    }
    this._lastHudHidden = hidden;
    try {
      this.visibilityCallback(!hidden);
    } catch (_err) {
      // Silently ignore callback errors
    }
  }

  setMachineContext({ id = null, label = null, tintColor = null } = {}) {
    this.machineId = typeof id === 'string' && id.length > 0 ? id : null;
    this.machineLabel = typeof label === 'string' && label.length > 0 ? label : null;
    const sanitizedTint = sanitizeHexColor(tintColor);
    this.machineTintColor = sanitizedTint;
    this.hudAccentColor = sanitizedTint || DEFAULT_HUD_ACCENT;
    if (this.hudElement) {
      this.hudElement.style.setProperty('--quality-accent', this.hudAccentColor);
    }
    this.refreshHud(true);
  }

  attachRemoteSystem(system) {
    if (!system || typeof system.setExtrusionListener !== 'function') {
      return;
    }
    if (this.remoteSystem === system) {
      return;
    }
    this.detachRemoteSystem();
    let binding = REMOTE_SYSTEM_BINDINGS.get(system);
    if (!binding) {
      binding = {
        monitors: new Set(),
        listener: (event) => {
          for (const monitor of binding.monitors) {
            try {
              monitor.boundExtrusionListener(event);
            } catch (err) {
              console.warn('QualityMonitor: extrusion listener error', err);
            }
          }
        },
      };
      REMOTE_SYSTEM_BINDINGS.set(system, binding);
      system.setExtrusionListener(binding.listener);
    }
    binding.monitors.add(this);
    this.remoteSystem = system;
  }

  setMotorDiagnosticsProvider(provider) {
    this.motorDiagnosticsProvider = typeof provider === 'function' ? provider : null;
    this.refreshHud(true);
  }

  detachRemoteSystem() {
    if (!this.remoteSystem) {
      return;
    }
    const binding = REMOTE_SYSTEM_BINDINGS.get(this.remoteSystem);
    if (binding) {
      binding.monitors.delete(this);
      if (binding.monitors.size === 0) {
        try {
          this.remoteSystem.setExtrusionListener(null);
        } catch (_err) {
          // best effort
        }
        REMOTE_SYSTEM_BINDINGS.delete(this.remoteSystem);
      }
    }
    this.remoteSystem = null;
  }

  _resetAccumulators() {
    this.extrusionCursor = 0;
    this.entryIndex = 0;
    this.nextCornerIndex = 0;
    this.activeCorners = [];
    this.sampleCount = 0;
    this.withinToleranceCount = 0;
    this.withinFineToleranceCount = 0;
    this.contourHistogram.reset();
    this.trackingHistogram.reset();
    this.shapeHistogram.reset();
    this.contourHorizontalSumSquares = 0;
    this.contourVerticalSumSquares = 0;
    // Least-squares offset from the error across the path (along-path lag is
    // not an offset): normal matrix Σ(I - t tᵀ) as xx, xy, xz, yy, yz, zz and Σ(I - t tᵀ)e.
    this.offsetNormal = [0, 0, 0, 0, 0, 0];
    this.offsetRight = [0, 0, 0];
    this.offsetEstimate = new Float64Array(3);
    this.alongPathSum = 0;
    this.cornerStats = {
      count: 0,
      deviationSum: 0,
      deviationMax: 0,
      overshootSum: 0,
      overshootMax: 0,
      settleSum: 0,
      settleMax: 0,
    };
  }

  reset({ keepReference = false } = {}) {
    this.metricsDirty = true;
    this.metrics = null;
    this._resetAccumulators();

    if (!keepReference) {
      this.entries = [];
      this.corners = [];
      this.totalExtrusion = 0;
    }

    this.refreshHud();
  }

  dispose() {
    this.detachRemoteSystem();
    this.setVisibilityCallback(null);
    if (this.hudElement && typeof this.hudElement.removeEventListener === 'function') {
      this.hudElement.removeEventListener('pointerdown', this.boundHudPointerDown);
      this.hudElement.removeEventListener('click', this.boundHudClick);
    }
    this.hudElement = null;
  }

  clearReference() {
    this.reset({ keepReference: false });
  }

  // Segments in G-code order, each with the filament `extrusion` it consumes.
  // Zero-length segments are extrusion without motion (primes after retracts).
  setReferenceSegments(segments) {
    if (!Array.isArray(segments) || segments.length === 0) {
      this.clearReference();
      return;
    }
    this._prepareEntries(segments);
    this._prepareCorners();
    this.reset({ keepReference: true });
  }

  recordExtrusion(extrusionEvent) {
    if (!extrusionEvent) {
      return;
    }
    if (this.machineId) {
      const eventMachineId = typeof extrusionEvent.machineId === 'string' ? extrusionEvent.machineId : null;
      if (!eventMachineId || eventMachineId !== this.machineId) {
        return;
      }
    }
    if (this.entries.length === 0) {
      return;
    }
    const pos = extrusionEvent.pos;
    const length = Number(extrusionEvent.length);
    if (!Array.isArray(pos) || pos.length < 3 || !(length > 0)) {
      return;
    }
    const x = Number(pos[0]);
    const y = Number(pos[1]);
    const z = Number(pos[2]);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      return;
    }
    // Samples are always scored as they arrive (well under a microsecond
    // each), so finishing a print never has a backlog to work through. The
    // toggle only controls the HUD and the extrusion tint.
    this._recordNormalizedExtrusion(x, y, z, length, this.enabled ? extrusionEvent : null);
    if (this.enabled && now() - this.lastHudRefreshMs >= this.hudRefreshIntervalMs) {
      this.refreshHud();
    }
  }

  setEnabled(flag) {
    const next = Boolean(flag);
    if (this.enabled === next) {
      return;
    }
    this.enabled = next;
    if (this.enabled) {
      this.refreshHud(true);
    } else if (this.hudElement) {
      this.hudElement.classList.add('sim-hidden');
      this._notifyHudVisibility();
    }
  }

  runFinalCheck() {
    if (this.entries.length === 0) {
      return;
    }
    for (const corner of this.activeCorners) {
      this._finalizeCorner(corner);
    }
    this.activeCorners = [];
    this.metricsDirty = true;
    this.refreshHud(true);
  }

  getMetrics() {
    if (!this.metrics) {
      return null;
    }
    return { ...this.metrics };
  }

  _prepareEntries(segments) {
    this.entries = [];
    let arcLength = 0;
    let extrusion = 0;
    for (const segment of segments) {
      const start = segment?.start;
      const end = segment?.end;
      const segmentExtrusion = Number(segment?.extrusion);
      if (!Array.isArray(start) || !Array.isArray(end) || !(segmentExtrusion > 0)) {
        continue;
      }
      const s = [Number(start[0]), Number(start[1]), Number(start[2])];
      const e = [Number(end[0]), Number(end[1]), Number(end[2])];
      if (!s.every(Number.isFinite) || !e.every(Number.isFinite)) {
        continue;
      }
      const delta = [e[0] - s[0], e[1] - s[1], e[2] - s[2]];
      const length = Math.hypot(delta[0], delta[1], delta[2]);
      const tangent = length > 0 ? delta.map((value) => value / length) : [0, 0, 0];
      const previous = this.entries[this.entries.length - 1];
      const contiguous = Boolean(previous) && previous.length > 0 && length > 0
        && Math.hypot(s[0] - previous.end[0], s[1] - previous.end[1], s[2] - previous.end[2]) < 1e-9;
      this.entries.push({
        start: s,
        end: e,
        delta,
        length,
        tangent,
        contiguous,
        arcStart: arcLength,
        extrusionStart: extrusion,
        extrusion: segmentExtrusion,
      });
      arcLength += length;
      extrusion += segmentExtrusion;
    }
    this.totalExtrusion = extrusion;
    // Flat copies for the per-sample nearest-point scan.
    const count = this.entries.length;
    const ref = {
      count,
      startX: new Float64Array(count),
      startY: new Float64Array(count),
      startZ: new Float64Array(count),
      deltaX: new Float64Array(count),
      deltaY: new Float64Array(count),
      deltaZ: new Float64Array(count),
      inverseLengthSq: new Float64Array(count),
      arcStart: new Float64Array(count),
      arcEnd: new Float64Array(count),
    };
    this.entries.forEach((entry, i) => {
      ref.startX[i] = entry.start[0];
      ref.startY[i] = entry.start[1];
      ref.startZ[i] = entry.start[2];
      ref.deltaX[i] = entry.delta[0];
      ref.deltaY[i] = entry.delta[1];
      ref.deltaZ[i] = entry.delta[2];
      ref.inverseLengthSq[i] = entry.length > 0 ? 1 / (entry.length * entry.length) : 0;
      ref.arcStart[i] = entry.arcStart;
      ref.arcEnd[i] = entry.arcStart + entry.length;
    });
    this.referenceArrays = ref;
  }

  _prepareCorners() {
    this.corners = [];
    const entries = this.entries;
    // Bounds of each contiguous run, so corner windows never cross a travel.
    const runStart = new Float64Array(entries.length);
    const runEnd = new Float64Array(entries.length);
    for (let i = 0; i < entries.length; i += 1) {
      runStart[i] = entries[i].contiguous ? runStart[i - 1] : entries[i].arcStart;
    }
    for (let i = entries.length - 1; i >= 0; i -= 1) {
      runEnd[i] = i + 1 < entries.length && entries[i + 1].contiguous
        ? runEnd[i + 1]
        : entries[i].arcStart + entries[i].length;
    }
    for (let i = 1; i < entries.length; i += 1) {
      const entry = entries[i];
      if (!entry.contiguous) {
        continue;
      }
      const a = entries[i - 1].tangent;
      const b = entry.tangent;
      const angle = Math.acos(clamp(a[0] * b[0] + a[1] * b[1] + a[2] * b[2], -1, 1));
      if (angle < this.cornerAngleThreshold) {
        continue;
      }
      this.corners.push({
        arcLength: entry.arcStart,
        vertex: entry.start,
        angle,
        windowStart: Math.max(runStart[i], entry.arcStart - this.cornerWindow),
        windowEnd: Math.min(runEnd[i], entry.arcStart + Math.max(this.cornerWindow, this.cornerSettleWindow)),
        settleEnd: Math.min(runEnd[i], entry.arcStart + this.cornerSettleWindow),
      });
    }
    // A later corner cuts short the settling window of the one before it.
    for (let i = 0; i + 1 < this.corners.length; i += 1) {
      const next = this.corners[i + 1].arcLength;
      this.corners[i].settleEnd = Math.min(this.corners[i].settleEnd, next);
    }
  }

  // Intended point for the extrusion consumed so far, stored in this._intended*.
  // The sample's own extrusion picks the entry, so a sample right after a
  // travel belongs to the line it starts, not the end of the previous one.
  _locateIntended(x, y, z, length) {
    const entries = this.entries;
    const upcoming = this.extrusionCursor + 0.5 * length;
    let index = this.entryIndex;
    while (index + 1 < entries.length && entries[index + 1].extrusionStart <= upcoming) {
      index += 1;
    }
    // Firmware rounds E to whole steps, so the reference can still hold a few
    // µm of filament of the line before a travel when the sample has already
    // travelled. Such a sample starts the next line: resynchronise E there.
    const next = entries[index + 1];
    if (next && !next.contiguous && next.extrusionStart - upcoming <= this.travelSyncExtrusion) {
      const end = entries[index].end;
      const toNext = (x - next.start[0]) ** 2 + (y - next.start[1]) ** 2 + (z - next.start[2]) ** 2;
      if (toNext < (x - end[0]) ** 2 + (y - end[1]) ** 2 + (z - end[2]) ** 2) {
        index += 1;
        this.extrusionCursor = next.extrusionStart;
      }
    }
    this.entryIndex = index;
    const entry = entries[index];
    const fraction = clamp((this.extrusionCursor - entry.extrusionStart) / entry.extrusion, 0, 1);
    this._intendedArc = entry.arcStart + fraction * entry.length;
    this._intendedX = entry.start[0] + fraction * entry.delta[0];
    this._intendedY = entry.start[1] + fraction * entry.delta[1];
    this._intendedZ = entry.start[2] + fraction * entry.delta[2];
  }

  // Nearest points of the reference within the search window around the
  // intended arc length, for the raw sample (path deviation, this._contour*)
  // and the offset-corrected sample (shape, this._shape*), in one pass.
  _scanReference(x, y, z, sx, sy, sz) {
    const ref = this.referenceArrays;
    const index = this.entryIndex;
    const low = this._intendedArc - this.contourSearchWindow;
    const high = this._intendedArc + this.contourSearchWindow;
    let first = index;
    while (first > 0 && ref.arcEnd[first - 1] >= low) {
      first -= 1;
    }
    let last = index;
    while (last + 1 < ref.count && ref.arcStart[last + 1] <= high) {
      last += 1;
    }
    let best = Infinity;
    let bestDx = 0;
    let bestDy = 0;
    let bestDz = 0;
    let shapeBest = Infinity;
    let shapeDx = 0;
    let shapeDy = 0;
    let shapeDz = 0;
    for (let i = first; i <= last; i += 1) {
      const ax = ref.startX[i];
      const ay = ref.startY[i];
      const az = ref.startZ[i];
      const bx = ref.deltaX[i];
      const by = ref.deltaY[i];
      const bz = ref.deltaZ[i];
      const inverse = ref.inverseLengthSq[i];
      let t = ((x - ax) * bx + (y - ay) * by + (z - az) * bz) * inverse;
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      let dx = x - (ax + t * bx);
      let dy = y - (ay + t * by);
      let dz = z - (az + t * bz);
      let distSq = dx * dx + dy * dy + dz * dz;
      if (distSq < best) {
        best = distSq;
        bestDx = dx;
        bestDy = dy;
        bestDz = dz;
      }
      t = ((sx - ax) * bx + (sy - ay) * by + (sz - az) * bz) * inverse;
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      dx = sx - (ax + t * bx);
      dy = sy - (ay + t * by);
      dz = sz - (az + t * bz);
      distSq = dx * dx + dy * dy + dz * dz;
      if (distSq < shapeBest) {
        shapeBest = distSq;
        shapeDx = dx;
        shapeDy = dy;
        shapeDz = dz;
      }
    }
    this._contourDistance = Math.sqrt(best);
    this._contourDx = bestDx;
    this._contourDy = bestDy;
    this._contourDz = bestDz;
    this._shapeDistance = Math.sqrt(shapeBest);
    this._shapeDx = shapeDx;
    this._shapeDy = shapeDy;
    this._shapeDz = shapeDz;
  }

  _recordNormalizedExtrusion(x, y, z, length, sourceEvent = null) {
    this._locateIntended(x, y, z, length);
    this.extrusionCursor += length;

    const tx = x - this._intendedX;
    const ty = y - this._intendedY;
    const tz = z - this._intendedZ;
    const tangent = this.entries[this.entryIndex].tangent;
    // Shape: path deviation with the print's mean offset (so far) removed.
    const offset = this._updateOffsetEstimate();
    const sx = x - offset[0];
    const sy = y - offset[1];
    const sz = z - offset[2];
    this._scanReference(x, y, z, sx, sy, sz);
    const contourDistance = this._contourDistance;
    this.shapeHistogram.observe(this._shapeDistance);
    this.trackingHistogram.observe(Math.sqrt(tx * tx + ty * ty + tz * tz));
    const t0 = tangent[0];
    const t1 = tangent[1];
    const t2 = tangent[2];
    const along = tx * t0 + ty * t1 + tz * t2;
    const normal = this.offsetNormal;
    normal[0] += 1 - t0 * t0;
    normal[1] -= t0 * t1;
    normal[2] -= t0 * t2;
    normal[3] += 1 - t1 * t1;
    normal[4] -= t1 * t2;
    normal[5] += 1 - t2 * t2;
    this.offsetRight[0] += tx - t0 * along;
    this.offsetRight[1] += ty - t1 * along;
    this.offsetRight[2] += tz - t2 * along;
    this.alongPathSum += along;

    this.contourHistogram.observe(contourDistance);
    this.contourHorizontalSumSquares += this._contourDx * this._contourDx + this._contourDy * this._contourDy;
    this.contourVerticalSumSquares += this._contourDz * this._contourDz;
    this.sampleCount += 1;
    if (contourDistance <= this.tolerance) {
      this.withinToleranceCount += 1;
    }
    if (contourDistance <= this.fineTolerance) {
      this.withinFineToleranceCount += 1;
    }

    // Corners are judged on shape too: rounding and overshoot, not the offset.
    this._accumulateCorners(sx, sy, sz, this._intendedArc);

    if (sourceEvent && typeof sourceEvent === 'object') {
      const penalty = contourDistance > this.tolerance
        ? clamp((contourDistance - this.tolerance) / (this.penaltyHardThreshold - this.tolerance), 0, 1)
        : 0;
      sourceEvent.penalty = penalty;
      sourceEvent.qualityColor = this._colorFromPenalty(penalty);
    }
    this.metricsDirty = true;
  }

  // Solves the offset normal equations into this.offsetEstimate; a small ridge
  // keeps directions the path has not yet constrained (e.g. along a single
  // first line) at zero.
  _updateOffsetEstimate() {
    const out = this.offsetEstimate;
    const n = this.sampleCount;
    if (n === 0) {
      out.fill(0);
      return out;
    }
    const ridge = 1e-6 * n;
    const normal = this.offsetNormal;
    const b = normal[1];
    const c = normal[2];
    const e = normal[4];
    const xx = normal[0] + ridge;
    const yy = normal[3] + ridge;
    const zz = normal[5] + ridge;
    const c00 = yy * zz - e * e;
    const c01 = c * e - b * zz;
    const c02 = b * e - c * yy;
    const c11 = xx * zz - c * c;
    const c12 = b * c - xx * e;
    const c22 = xx * yy - b * b;
    const det = xx * c00 + b * c01 + c * c02;
    const r0 = this.offsetRight[0];
    const r1 = this.offsetRight[1];
    const r2 = this.offsetRight[2];
    out[0] = (c00 * r0 + c01 * r1 + c02 * r2) / det;
    out[1] = (c01 * r0 + c11 * r1 + c12 * r2) / det;
    out[2] = (c02 * r0 + c12 * r1 + c22 * r2) / det;
    return out;
  }

  // Uses the shape deviation of the sample from this._shape*.
  _accumulateCorners(x, y, z, arcLength) {
    while (this.nextCornerIndex < this.corners.length && this.corners[this.nextCornerIndex].windowStart <= arcLength) {
      const corner = this.corners[this.nextCornerIndex];
      this.nextCornerIndex += 1;
      this.activeCorners.push({
        corner,
        deviation: Infinity,
        overshoot: 0,
        settleSamples: [],
        before: false,
        after: false,
        hasLastSample: false,
        lastX: 0,
        lastY: 0,
        lastZ: 0,
      });
    }
    const active = this.activeCorners;
    let kept = 0;
    for (let i = 0; i < active.length; i += 1) {
      const state = active[i];
      const corner = state.corner;
      if (arcLength > corner.windowEnd) {
        this._finalizeCorner(state);
        continue;
      }
      if (arcLength - corner.arcLength <= this.cornerWindow) {
        const vertex = corner.vertex;
        const deviation = state.hasLastSample
          ? distancePointToSegment(vertex[0], vertex[1], vertex[2], state.lastX, state.lastY, state.lastZ, x, y, z)
          : Math.hypot(x - vertex[0], y - vertex[1], z - vertex[2]);
        if (deviation < state.deviation) {
          state.deviation = deviation;
        }
      }
      state.hasLastSample = true;
      state.lastX = x;
      state.lastY = y;
      state.lastZ = z;
      if (arcLength < corner.arcLength) {
        state.before = true;
      } else if (arcLength > corner.arcLength) {
        state.after = true;
        if (arcLength - corner.arcLength <= this.cornerWindow && this._shapeDistance > state.overshoot) {
          state.overshoot = this._shapeDistance;
        }
        if (arcLength <= corner.settleEnd) {
          state.settleSamples.push(arcLength - corner.arcLength, this._shapeDx, this._shapeDy, this._shapeDz);
        }
      }
      active[kept] = state;
      kept += 1;
    }
    active.length = kept;
  }

  _finalizeCorner(active) {
    if (!active.before || !active.after || !Number.isFinite(active.deviation)) {
      return;
    }
    // Settled once the deviation stays within tolerance of where it ends up on
    // the outgoing line (the mean of the window's last quarter), so a steady
    // offset is not mistaken for ringing.
    const samples = active.settleSamples;
    const count = samples.length / 4;
    const tail = Math.max(1, Math.floor(count / 4));
    const final = [0, 0, 0];
    for (let k = count - tail; k < count; k += 1) {
      final[0] += samples[4 * k + 1] / tail;
      final[1] += samples[4 * k + 2] / tail;
      final[2] += samples[4 * k + 3] / tail;
    }
    let settle = 0;
    for (let k = count - 1; k >= 0; k -= 1) {
      const drift = Math.hypot(samples[4 * k + 1] - final[0], samples[4 * k + 2] - final[1], samples[4 * k + 3] - final[2]);
      if (drift > this.tolerance) {
        settle = k + 1 < count ? samples[4 * (k + 1)] : samples[4 * k];
        break;
      }
    }
    const stats = this.cornerStats;
    stats.count += 1;
    stats.deviationSum += active.deviation;
    stats.deviationMax = Math.max(stats.deviationMax, active.deviation);
    stats.overshootSum += active.overshoot;
    stats.overshootMax = Math.max(stats.overshootMax, active.overshoot);
    stats.settleSum += settle;
    stats.settleMax = Math.max(stats.settleMax, settle);
  }

  _computeMetrics() {
    const n = this.sampleCount;
    const corners = this.cornerStats;
    const withinTolerance = n > 0 ? this.withinToleranceCount / n : NaN;
    return {
      samples: n,
      progress: this.totalExtrusion > 0 ? this.extrusionCursor / this.totalExtrusion : NaN,
      tolerance: this.tolerance,
      withinTolerance,
      fineTolerance: this.fineTolerance,
      withinFineTolerance: n > 0 ? this.withinFineToleranceCount / n : NaN,
      shapeRms: this.shapeHistogram.rms(),
      shapeP95: this.shapeHistogram.quantile(0.95),
      contourRms: this.contourHistogram.rms(),
      contourP95: this.contourHistogram.quantile(0.95),
      contourMax: n > 0 ? this.contourHistogram.max : NaN,
      contourHorizontalRms: n > 0 ? Math.sqrt(this.contourHorizontalSumSquares / n) : NaN,
      contourVerticalRms: n > 0 ? Math.sqrt(this.contourVerticalSumSquares / n) : NaN,
      trackingRms: this.trackingHistogram.rms(),
      trackingP95: this.trackingHistogram.quantile(0.95),
      trackingMax: n > 0 ? this.trackingHistogram.max : NaN,
      meanOffset: n > 0 ? Array.from(this._updateOffsetEstimate()) : [NaN, NaN, NaN],
      meanLag: n > 0 ? -this.alongPathSum / n : NaN,
      cornerCount: corners.count,
      cornerDeviationAvg: corners.count > 0 ? corners.deviationSum / corners.count : NaN,
      cornerDeviationMax: corners.count > 0 ? corners.deviationMax : NaN,
      cornerOvershootAvg: corners.count > 0 ? corners.overshootSum / corners.count : NaN,
      cornerOvershootMax: corners.count > 0 ? corners.overshootMax : NaN,
      settleDistanceAvg: corners.count > 0 ? corners.settleSum / corners.count : NaN,
      settleDistanceMax: corners.count > 0 ? corners.settleMax : NaN,
    };
  }

  _composeScore(m, totalMissedSteps) {
    if (!(m.samples > 0)) {
      return { score: NaN, scoreDeductions: [] };
    }
    const values = {
      shapeP95: m.shapeP95,
      shapeRms: m.shapeRms,
      cornerError: m.cornerCount > 0 ? Math.max(m.cornerDeviationAvg, m.cornerOvershootAvg) : 0,
      settleDistance: m.cornerCount > 0 ? m.settleDistanceAvg : 0,
      offset: Math.hypot(m.meanOffset[0], m.meanOffset[1], m.meanOffset[2]),
      missedSteps: totalMissedSteps,
    };
    const scoreDeductions = SCORE_TERMS.map((term) => ({
      label: term.label,
      points: term.weight * (1 - Math.exp(-values[term.key] / term.scale)),
    }));
    const score = 100 - scoreDeductions.reduce((sum, deduction) => sum + deduction.points, 0);
    return { score, scoreDeductions };
  }

  refreshHud(force = false) {
    this.lastHudRefreshMs = now();
    if (this.entries.length === 0) {
      this.metrics = null;
      this.metricsDirty = false;
      if (this.hudElement) {
        this.hudElement.textContent = 'Quality metrics available once a reference path is loaded.';
        this.hudElement.classList.add('sim-hidden');
        this._notifyHudVisibility();
      }
      return;
    }
    if (this.metricsDirty) {
      this.metrics = this._computeMetrics();
      this.metricsDirty = false;
    }
    // Missed steps change without extrusion, so the score is always recomposed.
    const motorDiagnostics = this._getMotorDiagnostics();
    Object.assign(this.metrics, this._composeScore(this.metrics, motorDiagnostics.totalMissedSteps));
    if (!this.hudElement) {
      return;
    }
    if (!force && !this.enabled) {
      this.hudElement.classList.add('sim-hidden');
      this._notifyHudVisibility();
      return;
    }
    const m = this.metrics;
    const qualityArrow = this.qualityDetailsExpanded ? '▲' : '▼';
    const missedStepsArrow = this.missedStepsExpanded ? '▲' : '▼';

    const headerLabel = escapeHtml(this.machineLabel || this.machineId || 'Machine');
    const accent = sanitizeHexColor(this.machineTintColor) || this.hudAccentColor || DEFAULT_HUD_ACCENT;
    this.hudElement.style.setProperty('--quality-accent', accent);

    const metric = (label, value) => `<div class="quality-hud__metric"><span>${label}</span><span>${value}</span></div>`;
    const triple = (a, b, c) => `${formatLengthMm(a)} / ${formatLengthMm(b)} / ${formatLengthMm(c)} mm`;
    const pair = (a, b) => `${formatLengthMm(a)} / ${formatLengthMm(b)} mm`;
    const score = Number.isFinite(m.score) ? m.score.toFixed(0) : '--';

    const lines = [];
    lines.push(
      `<div class="quality-hud__header">` +
        `<span class="quality-hud__swatch${this.machineTintColor ? '' : ' quality-hud__swatch--empty'}"${
          this.machineTintColor ? ` style="background-color: ${accent};"` : ''
        }></span>` +
        `<span class="quality-hud__title">${headerLabel}</span>` +
      `</div>`
    );
    lines.push(
      `<div class="quality-hud__summary quality-hud__summary--primary">` +
        `<span class="quality-hud__summary-copy">Quality <strong>${score}</strong></span>` +
        `<button type="button" class="quality-hud__toggle" data-section="quality-details" aria-expanded="${this.qualityDetailsExpanded ? 'true' : 'false'}" aria-label="${this.qualityDetailsExpanded ? 'Hide quality details' : 'Show quality details'}">${qualityArrow}</button>` +
      `</div>`
    );
    lines.push(
      `<div class="quality-hud__details${this.qualityDetailsExpanded ? '' : ' quality-hud__details--hidden'}">` +
        metric('Score deductions', escapeHtml(m.scoreDeductions.filter((d) => d.points >= 0.5)
          .map((d) => `${d.label} −${d.points.toFixed(0)}`).join(', ') || 'none')) +
        metric('Shape error RMS / 95%', pair(m.shapeRms, m.shapeP95)) +
        metric('Path deviation RMS / 95% / max', triple(m.contourRms, m.contourP95, m.contourMax)) +
        metric(`Path within ${formatLengthMm(m.fineTolerance)} mm`, formatPercent(m.withinFineTolerance)) +
        metric(`Path within ${formatLengthMm(m.tolerance)} mm`, formatPercent(m.withinTolerance)) +
        metric('Path deviation RMS horiz. / vert.', pair(m.contourHorizontalRms, m.contourVerticalRms)) +
        metric('Position error RMS / 95% / max', triple(m.trackingRms, m.trackingP95, m.trackingMax)) +
        metric('Mean offset X / Y / Z', triple(m.meanOffset[0], m.meanOffset[1], m.meanOffset[2])) +
        metric('Mean lag along path', `${formatLengthMm(m.meanLag)} mm`) +
        metric(`Corner deviation avg / max (${m.cornerCount})`, pair(m.cornerDeviationAvg, m.cornerDeviationMax)) +
        metric('Corner overshoot avg / max', pair(m.cornerOvershootAvg, m.cornerOvershootMax)) +
        metric('Corner settling avg / max', pair(m.settleDistanceAvg, m.settleDistanceMax)) +
        metric('Progress', formatPercent(m.progress)) +
      `</div>`
    );
    lines.push(
      `<div class="quality-hud__summary quality-hud__summary--secondary">` +
        `<span class="quality-hud__summary-copy">Missed Steps <strong>${motorDiagnostics.totalMissedSteps}</strong></span>` +
        `<button type="button" class="quality-hud__toggle" data-section="missed-steps" aria-expanded="${this.missedStepsExpanded ? 'true' : 'false'}" aria-label="${this.missedStepsExpanded ? 'Hide missed step details' : 'Show missed step details'}">${missedStepsArrow}</button>` +
      `</div>`
    );
    lines.push(
      `<div class="quality-hud__details${this.missedStepsExpanded ? '' : ' quality-hud__details--hidden'}">` +
        this._renderMotorDiagnostics(motorDiagnostics.motors) +
      `</div>`
    );

    this.hudElement.innerHTML = lines.join('');
    this.hudElement.classList.remove('sim-hidden');
    this._notifyHudVisibility();
  }

  _getMotorDiagnostics() {
    if (typeof this.motorDiagnosticsProvider !== 'function') {
      return { totalMissedSteps: 0, motors: [] };
    }
    try {
      const diagnostics = this.motorDiagnosticsProvider();
      return {
        totalMissedSteps: diagnostics.totalMissedSteps,
        motors: diagnostics.motors,
      };
    } catch (_err) {
      return { totalMissedSteps: 0, motors: [] };
    }
  }

  _renderMotorDiagnostics(motors) {
    if (!Array.isArray(motors) || motors.length === 0) {
      return `<div class="quality-hud__metric"><span>No motors</span><span>--</span></div>`;
    }
    return motors
      .map((motor) => {
        const value = motor.closedLoop
          ? 'closed loop'
          : `${motor.missedSteps} (${motor.slips} slips, net ${motor.netSlips >= 0 ? '+' : ''}${motor.netSlips})`;
        return `<div class="quality-hud__metric"><span>${escapeHtml(motor.axis)}</span><span>${value}</span></div>`;
      })
      .join('');
  }

  _toggleHudSection(section) {
    if (section === 'quality-details') {
      this.qualityDetailsExpanded = !this.qualityDetailsExpanded;
      this.detailsExpandedCallback?.(this.qualityDetailsExpanded);
    } else if (section === 'missed-steps') {
      this.missedStepsExpanded = !this.missedStepsExpanded;
    } else {
      return false;
    }
    this.refreshHud(true);
    return true;
  }

  _handleHudPointerDown(event) {
    const button = event?.target?.closest?.('.quality-hud__toggle');
    if (!button) {
      return;
    }
    if (Number.isFinite(event.button) && event.button !== 0) {
      return;
    }
    event.preventDefault?.();
    this._toggleHudSection(button.dataset?.section);
  }

  _handleHudClick(event) {
    const button = event?.target?.closest?.('.quality-hud__toggle');
    if (!button) {
      return;
    }
    if ((event?.detail ?? 0) > 0) {
      return;
    }
    event.preventDefault?.();
    this._toggleHudSection(button.dataset?.section);
  }

  _colorFromPenalty(penalty) {
    if (penalty <= 0) {
      return '#ffffff';
    }
    const channel = Math.round(255 * (1 - 0.75 * penalty));
    return `rgb(255, ${channel}, ${channel})`;
  }
}
