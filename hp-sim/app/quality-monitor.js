const DEG_TO_RAD = Math.PI / 180;

const DEFAULT_HUD_ACCENT = '#7ae2ff';
const REMOTE_SYSTEM_BINDINGS = new WeakMap();

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

    this.machineId = null;
    this.machineLabel = null;
    this.machineTintColor = null;
    this.hudAccentColor = DEFAULT_HUD_ACCENT;

    this.visibilityCallback = null;
    this._lastHudHidden = null;

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
    this.enabled = true;
    this.projectionHintValid = false;

    this.straightStats = {
      count: 0,
      sumSquares: 0,
      percentile95: new StreamingQuantile(0.95),
    };

    this.coverageGrid = null;
    this.coverageStats = null;

    this.remoteSystem = null;
    this.boundExtrusionListener = (event) => this.recordExtrusion(event);
    this.motorDiagnosticsProvider = null;
    this.qualityDetailsExpanded = false;
    this.missedStepsExpanded = false;
    this.boundHudPointerDown = (event) => this._handleHudPointerDown(event);
    this.boundHudClick = (event) => this._handleHudClick(event);
    if (this.hudElement && typeof this.hudElement.addEventListener === 'function') {
      this.hudElement.addEventListener('pointerdown', this.boundHudPointerDown);
    }
    if (this.hudElement && typeof this.hudElement.addEventListener === 'function') {
      this.hudElement.addEventListener('click', this.boundHudClick);
    }

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

    this.segmentSearchRadius = 20; // segments to scan around hint
    this.projectionFallbackDistance = 0.02; // meters before forcing full search
    this.penaltySoftThreshold = 0.00025; // meters (0.25 mm) before coloring
    this.penaltyHardThreshold = 0.0015; // meters (1.5 mm) saturates penalty

    this.coverageStep = 0.001; // 1 mm grid
    this.coverageMargin = 0.015; // 15 mm margin
    this.assumedBeadWidth = 0.0008; // 0.8 mm bead -> radius 0.4 mm

    this.hudUpdateInterval = 12;
  }

  setVisibilityCallback(callback) {
    this.visibilityCallback = typeof callback === 'function' ? callback : null;
    this._notifyHudVisibility();
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

  reset({ keepReference = false } = {}) {
    this.lastSegmentIndex = 0;
    this.projectionHintValid = false;
    this.pendingExtrusionCount = 0;
    this.metricsDirty = true;
    this.metrics = null;
    this.extrusionsSinceHud = 0;

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
    if (this.machineId) {
      const eventMachineId = typeof extrusionEvent.machineId === 'string' ? extrusionEvent.machineId : null;
      if (!eventMachineId || eventMachineId !== this.machineId) {
        return;
      }
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
      this.projectionHintValid = false;
      this._drainPendingExtrusions();
      this.refreshHud(true);
    } else if (this.hudElement) {
      this.hudElement.classList.add('sim-hidden');
      this._notifyHudVisibility();
    }
  }

  runFinalCheck() {
    if (this.segmentData.length === 0) {
      return;
    }
    this._drainPendingExtrusions();
    this.metricsDirty = true;
    this.refreshHud(true);
    this.extrusionsSinceHud = 0;
  }

  getMetrics() {
    if (!this.metrics) {
      return null;
    }
    return { ...this.metrics };
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

  _recordNormalizedExtrusion(x, y, length, sourceEvent = null) {
    const projection = this._projectToPath(x, y);
    if (!projection) {
      return;
    }

    const nearestCornerDelta = this._accumulateCornerSamples(projection);
    const penalty = this._computeExtrusionPenalty(projection, nearestCornerDelta);
    this._applyPenaltyToExtrusion(sourceEvent, penalty);

    this._accumulateStraightError(projection);
    this._updateCoverageForExtrusion(x, y, length);

    this.metricsDirty = true;
    this.extrusionsSinceHud += 1;
    if (this.enabled && this.extrusionsSinceHud >= this.hudUpdateInterval) {
      this.refreshHud();
      this.extrusionsSinceHud = 0;
    }
  }

  _recordExtrusionActive(extrusionEvent) {
    if (this.segmentData.length === 0) {
      return;
    }
    const normalized = this._normalizeExtrusionEvent(extrusionEvent);
    if (!normalized) {
      return;
    }
    this._recordNormalizedExtrusion(normalized.x, normalized.y, normalized.length, extrusionEvent);
  }

  _drainPendingExtrusions() {
    if (this.pendingExtrusionCount === 0) {
      return;
    }
    const count = this.pendingExtrusionCount;
    const xs = this.pendingExtrusionsX;
    const ys = this.pendingExtrusionsY;
    const lengths = this.pendingExtrusionsLength;
    for (let i = 0; i < count; i += 1) {
      this._recordNormalizedExtrusion(xs[i], ys[i], lengths[i]);
    }
    this.pendingExtrusionCount = 0;
  }

  refreshHud(force = false) {
    if (this.segmentData.length === 0) {
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
    if (!this.hudElement) {
      return;
    }
    if (!force && !this.enabled) {
      this.hudElement.classList.add('sim-hidden');
      this._notifyHudVisibility();
      return;
    }
    const metrics = this.metrics;
    if (!metrics) {
      this.hudElement.textContent = 'Collecting metrics...';
      this.hudElement.classList.remove('sim-hidden');
      this._notifyHudVisibility();
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
    const motorDiagnostics = this._getMotorDiagnostics();
    const qualityArrow = this.qualityDetailsExpanded ? '▲' : '▼';
    const missedStepsArrow = this.missedStepsExpanded ? '▲' : '▼';
    const missedSteps = Number.isFinite(motorDiagnostics.totalMissedSteps)
      ? Math.max(0, Math.round(motorDiagnostics.totalMissedSteps))
      : 0;

    const headerLabel = escapeHtml(this.machineLabel || this.machineId || 'Machine');
    const accent = sanitizeHexColor(this.machineTintColor) || this.hudAccentColor || DEFAULT_HUD_ACCENT;
    if (this.hudElement) {
      this.hudElement.style.setProperty('--quality-accent', accent);
    }

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
        `<span class="quality-hud__summary-copy">Quality <strong>${score.toFixed(0)}</strong></span>` +
        `<button type="button" class="quality-hud__toggle" data-section="quality-details" aria-expanded="${this.qualityDetailsExpanded ? 'true' : 'false'}" aria-label="${this.qualityDetailsExpanded ? 'Hide quality details' : 'Show quality details'}">${qualityArrow}</button>` +
      `</div>`
    );
    lines.push(
      `<div class="quality-hud__details${this.qualityDetailsExpanded ? '' : ' quality-hud__details--hidden'}">` +
        `<div class="quality-hud__metric"><span>RMSE_n (straight)</span><span>${formatLengthMm(rmseStraight)}</span></div>` +
        `<div class="quality-hud__metric"><span>95th |e_n|</span><span>${formatLengthMm(p95Straight)}</span></div>` +
        `<div class="quality-hud__metric"><span>Corner radius avg / max</span><span>${formatLengthMm(cornerRadiusAvg)} / ${formatLengthMm(cornerRadiusMax)}</span></div>` +
        `<div class="quality-hud__metric"><span>Ringing amplitude</span><span>${formatLengthMm(ringingAmplitude)}</span></div>` +
        `<div class="quality-hud__metric"><span>Damping estimate</span><span>${Number.isFinite(ringingDecay) ? ringingDecay.toFixed(2) : '--'}</span></div>` +
        `<div class="quality-hud__metric"><span>Coverage / IoU</span><span>${formatPercent(coverage)} / ${formatPercent(iou)}</span></div>` +
      `</div>`
    );
    lines.push(
      `<div class="quality-hud__summary quality-hud__summary--secondary">` +
        `<span class="quality-hud__summary-copy">Missed Steps <strong>${missedSteps}</strong></span>` +
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
      const motors = Array.isArray(diagnostics?.motors)
        ? diagnostics.motors
            .map((motor) => {
              const axis = typeof motor?.axis === 'string' && motor.axis.trim().length > 0
                ? motor.axis.trim()
                : null;
              if (!axis) {
                return null;
              }
              const missedSteps = Number.isFinite(motor?.missedSteps)
                ? Math.max(0, Math.round(motor.missedSteps))
                : 0;
              return {
                axis,
                missedSteps,
              };
            })
            .filter(Boolean)
            .sort((a, b) => a.axis.localeCompare(b.axis, undefined, { numeric: true, sensitivity: 'base' }))
        : [];
      const totalMissedSteps = motors.reduce((sum, motor) => sum + motor.missedSteps, 0);
      return { totalMissedSteps, motors };
    } catch (_err) {
      return { totalMissedSteps: 0, motors: [] };
    }
  }

  _renderMotorDiagnostics(motors) {
    if (!Array.isArray(motors) || motors.length === 0) {
      return `<div class="quality-hud__metric"><span>No motors</span><span>--</span></div>`;
    }
    return motors
      .map((motor) => (
        `<div class="quality-hud__metric"><span>${escapeHtml(motor.axis)}</span><span>${motor.missedSteps}</span></div>`
      ))
      .join('');
  }

  _toggleHudSection(section) {
    if (section === 'quality-details') {
      this.qualityDetailsExpanded = !this.qualityDetailsExpanded;
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
    const searchRadius = Math.max(0, Math.floor(this.segmentSearchRadius));
    const hasHint = this.projectionHintValid && Number.isInteger(this.lastSegmentIndex);
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

    const startIdx = hasHint ? Math.max(0, hint - searchRadius) : 0;
    const endIdx = hasHint ? Math.min(count - 1, hint + searchRadius) : count - 1;
    for (let i = startIdx; i <= endIdx; i += 1) {
      evaluateSegment(i);
    }

    const fallbackDistanceSq = Number.isFinite(this.projectionFallbackDistance) && this.projectionFallbackDistance > 0
      ? this.projectionFallbackDistance * this.projectionFallbackDistance
      : Infinity;
    const searchedFullRange = !hasHint;
    const needFallbackSearch = !searchedFullRange && bestDistSq > fallbackDistanceSq;
    if (needFallbackSearch) {
      for (let i = 0; i < count; i += 1) {
        if (i >= startIdx && i <= endIdx) {
          continue;
        }
        evaluateSegment(i);
      }
    }
    if (best) {
      this.lastSegmentIndex = best.segmentIndex;
      this.projectionHintValid = true;
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
    if (this.straightStats.percentile95) {
      this.straightStats.percentile95.observe(Math.abs(err));
    }
  }

  _accumulateCornerSamples(projection) {
    if (this.cornerData.length === 0) {
      return Infinity;
    }
    let closestAbsDelta = Infinity;
    for (const corner of this.cornerData) {
      const delta = projection.arcLength - corner.arcLength;
      const absDelta = Math.abs(delta);
      if (absDelta < closestAbsDelta) {
        closestAbsDelta = absDelta;
      }
      if (absDelta <= this.cornerRadiusWindow) {
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
    return closestAbsDelta;
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

  _computeExtrusionPenalty(projection, nearestCornerDelta = Infinity) {
    const errorAbs = Math.abs(projection.normalError);
    const soft = Number.isFinite(this.penaltySoftThreshold) && this.penaltySoftThreshold > 0
      ? this.penaltySoftThreshold
      : 0;
    const hard = Number.isFinite(this.penaltyHardThreshold) && this.penaltyHardThreshold > soft
      ? this.penaltyHardThreshold
      : soft + 1e-6;
    let penalty = 0;
    if (errorAbs > soft) {
      penalty = clamp((errorAbs - soft) / (hard - soft), 0, 1);
    }
    if (Number.isFinite(nearestCornerDelta) && nearestCornerDelta <= this.cornerRadiusWindow) {
      const proximity = clamp(1 - nearestCornerDelta / this.cornerRadiusWindow, 0, 1);
      penalty = clamp(penalty + 0.35 * proximity, 0, 1);
    }
    return penalty;
  }

  _colorFromPenalty(penalty) {
    const p = clamp(Number.isFinite(penalty) ? penalty : 0, 0, 1);
    if (p <= 0) {
      return '#ffffff';
    }
    const channel = Math.round(255 * (1 - 0.75 * p));
    return `rgb(255, ${channel}, ${channel})`;
  }

  _applyPenaltyToExtrusion(extrusionEvent, penalty) {
    if (!extrusionEvent || typeof extrusionEvent !== 'object') {
      return;
    }
    const safePenalty = clamp(Number.isFinite(penalty) ? penalty : 0, 0, 1);
    extrusionEvent.penalty = safePenalty;
    extrusionEvent.qualityColor = this._colorFromPenalty(safePenalty);
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
