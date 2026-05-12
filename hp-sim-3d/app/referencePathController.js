import { PRESET_GCODE_MAP } from './commandPresetResolver.js';

const GCODE_MM_TO_SIM_SCALE = 0.001;
const GCODE_EXTRUSION_EPSILON = 1e-6;
const GCODE_MOVE_EPSILON = 1e-9;
const GCODE_INLINE_COMMENT_RE = /\(.*?\)/g;

function createEmptyBounds() {
  return {
    minX: Infinity,
    minY: Infinity,
    minZ: Infinity,
    maxX: -Infinity,
    maxY: -Infinity,
    maxZ: -Infinity,
  };
}

function updateBoundsWithPoint(bounds, point) {
  if (!bounds || !Array.isArray(point) || point.length < 3) {
    return;
  }
  const [x, y, z] = point;
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return;
  }
  bounds.minX = Math.min(bounds.minX, x);
  bounds.minY = Math.min(bounds.minY, y);
  bounds.minZ = Math.min(bounds.minZ, z);
  bounds.maxX = Math.max(bounds.maxX, x);
  bounds.maxY = Math.max(bounds.maxY, y);
  bounds.maxZ = Math.max(bounds.maxZ, z);
}

function finalizeBounds(bounds) {
  if (!bounds || !Number.isFinite(bounds.minX)) {
    return null;
  }
  return {
    minX: bounds.minX,
    minY: bounds.minY,
    minZ: bounds.minZ,
    maxX: bounds.maxX,
    maxY: bounds.maxY,
    maxZ: bounds.maxZ,
  };
}

function sanitizeGcodeLine(rawLine) {
  if (typeof rawLine !== 'string') {
    return '';
  }
  let line = rawLine.trim();
  if (!line || line.startsWith(';')) {
    return '';
  }
  line = line.replace(GCODE_INLINE_COMMENT_RE, '');
  const commentIndex = line.indexOf(';');
  if (commentIndex >= 0) {
    line = line.slice(0, commentIndex);
  }
  return line.trim();
}

export function parseGcodeText(text) {
  if (typeof text !== 'string' || text.length === 0) {
    return { segments: [], bounds: null };
  }
  const bounds = createEmptyBounds();
  const segments = [];
  const state = {
    position: { X: 0, Y: 0, Z: 0 },
    extruder: 0,
    positionAbsolute: true,
    extrusionAbsolute: true,
    feedRate: null,
  };

  for (const rawLine of text.split(/\r?\n/)) {
    const line = sanitizeGcodeLine(rawLine);
    if (!line) {
      continue;
    }
    const tokens = line.split(/\s+/).filter(Boolean);
    const primaryToken = tokens[0]?.toUpperCase() || '';
    const codeLetter = primaryToken.charAt(0);
    const codeNumber = Number.parseInt(primaryToken.substring(1), 10);

    if (codeLetter === 'M' && codeNumber === 82) {
      state.extrusionAbsolute = true;
      continue;
    }
    if (codeLetter === 'M' && codeNumber === 83) {
      state.extrusionAbsolute = false;
      continue;
    }
    if (codeLetter === 'G' && codeNumber === 90) {
      state.positionAbsolute = true;
      continue;
    }
    if (codeLetter === 'G' && codeNumber === 91) {
      state.positionAbsolute = false;
      continue;
    }
    if (codeLetter === 'G' && codeNumber === 92) {
      const nextPosition = { ...state.position };
      let nextExtruder = state.extruder;
      for (const token of tokens.slice(1)) {
        const letter = token.charAt(0).toUpperCase();
        const value = Number.parseFloat(token.substring(1));
        if (!Number.isFinite(value)) {
          continue;
        }
        if (letter === 'X' || letter === 'Y' || letter === 'Z') {
          nextPosition[letter] = value;
        } else if (letter === 'E') {
          nextExtruder = value;
        }
      }
      state.position = nextPosition;
      state.extruder = nextExtruder;
      continue;
    }

    if (!(codeLetter === 'G' && (codeNumber === 0 || codeNumber === 1))) {
      continue;
    }

    const startPos = { ...state.position };
    const nextPos = { ...state.position };
    let rawExtrusionValue = null;
    let feedRate = state.feedRate;

    for (const token of tokens.slice(1)) {
      if (!token || token.length < 2) {
        continue;
      }
      const letter = token.charAt(0).toUpperCase();
      const value = Number.parseFloat(token.substring(1));
      if (!Number.isFinite(value)) {
        continue;
      }
      if (letter === 'X' || letter === 'Y' || letter === 'Z') {
        nextPos[letter] = state.positionAbsolute ? value : state.position[letter] + value;
      } else if (letter === 'E') {
        rawExtrusionValue = value;
      } else if (letter === 'F') {
        feedRate = value;
      }
    }

    let extrusionDelta = 0;
    let nextExtruder = state.extruder;
    if (rawExtrusionValue != null) {
      if (state.extrusionAbsolute) {
        extrusionDelta = rawExtrusionValue - state.extruder;
        nextExtruder = rawExtrusionValue;
      } else {
        extrusionDelta = rawExtrusionValue;
        nextExtruder = state.extruder + rawExtrusionValue;
      }
    }

    const deltaX = nextPos.X - startPos.X;
    const deltaY = nextPos.Y - startPos.Y;
    const deltaZ = nextPos.Z - startPos.Z;
    const moveDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    if (extrusionDelta > GCODE_EXTRUSION_EPSILON && moveDistance > GCODE_MOVE_EPSILON) {
      const segment = {
        start: [startPos.X, startPos.Y, startPos.Z],
        end: [nextPos.X, nextPos.Y, nextPos.Z],
        extrusion: extrusionDelta,
        feedRate: Number.isFinite(feedRate) ? feedRate : null,
        length: moveDistance,
      };
      segments.push(segment);
      updateBoundsWithPoint(bounds, segment.start);
      updateBoundsWithPoint(bounds, segment.end);
    }

    state.position = nextPos;
    state.extruder = nextExtruder;
    state.feedRate = feedRate;
  }

  return { segments, bounds: finalizeBounds(bounds) };
}

function convertParsedSegmentsToSimulation(parsed, { scale = GCODE_MM_TO_SIM_SCALE } = {}) {
  if (!parsed || !Array.isArray(parsed.segments)) {
    return { segments: [], bounds: null };
  }
  const segments = parsed.segments.map((segment) => ({
    start: segment.start.map((value) => value * scale),
    end: segment.end.map((value) => value * scale),
    extrusion: segment.extrusion,
    feedRate: segment.feedRate,
    length: segment.length * scale,
  }));
  const bounds = parsed.bounds
    ? {
        minX: parsed.bounds.minX * scale,
        minY: parsed.bounds.minY * scale,
        minZ: parsed.bounds.minZ * scale,
        maxX: parsed.bounds.maxX * scale,
        maxY: parsed.bounds.maxY * scale,
        maxZ: parsed.bounds.maxZ * scale,
      }
    : null;
  return { segments, bounds };
}

async function fetchGcodeText(url) {
  const response = await fetch(url, { cache: 'no-cache' });
  if (!response.ok) {
    throw new Error(`Failed to fetch G-code from ${url}: ${response.status}`);
  }
  return response.text();
}

export function createReferencePathController({
  world,
  state,
  dom,
  quality,
  isMobileLayout = () => false,
} = {}) {
  const referencePathCache = new Map();
  let measurePointerCandidate = null;

  function getOverlayState() {
    return state.referenceOverlay;
  }

  function getReferenceToggleText({ hasData, visible }) {
    const mobile = isMobileLayout();
    if (!hasData) {
      return mobile ? 'Show Reference' : 'Show Reference Path';
    }
    return visible
      ? (mobile ? 'Hide Reference' : 'Hide Reference Path')
      : (mobile ? 'Show Reference' : 'Show Reference Path');
  }

  function updateReferenceToggleUI() {
    const button = dom.referenceToggleBtn;
    if (!button) {
      return;
    }
    const overlay = getOverlayState();
    const hasData = Array.isArray(overlay.segments) && overlay.segments.length > 0;
    button.disabled = !hasData;
    button.setAttribute('aria-pressed', overlay.visible ? 'true' : 'false');
    if (hasData && overlay.metadata?.label) {
      button.setAttribute('title', `Reference: ${overlay.metadata.label}`);
      button.setAttribute('aria-label', `Toggle reference path: ${overlay.metadata.label}`);
    } else {
      button.removeAttribute('title');
      button.setAttribute('aria-label', 'Toggle reference path visibility');
    }
    button.textContent = getReferenceToggleText({ hasData, visible: overlay.visible });
  }

  function syncToRenderSystem({ force = false } = {}) {
    const overlay = getOverlayState();
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem || typeof renderSystem.setReferencePaths !== 'function') {
      return;
    }
    if (!force && !overlay.dirty) {
      return;
    }
    renderSystem.setReferencePaths(overlay.segments || [], {
      color: overlay.color,
      metadata: overlay.metadata,
      visible: overlay.visible,
    });
    overlay.dirty = false;
  }

  function setSegments(segments, { metadata = null, color = null } = {}) {
    const overlay = getOverlayState();
    overlay.segments = Array.isArray(segments) ? segments : null;
    overlay.metadata = metadata;
    overlay.key = metadata?.key || null;
    if (typeof color === 'string' && color.length > 0) {
      overlay.color = color;
    } else if (!overlay.color) {
      overlay.color = '#1e90ff';
    }
    overlay.dirty = true;
    if (!overlay.segments) {
      overlay.visible = false;
    }
    updateReferenceToggleUI();
    quality?.forEachQualityMonitor?.((monitor) => {
      monitor.setReferenceSegments(overlay.segments, overlay.metadata);
    });
    syncToRenderSystem({ force: true });
  }

  function setVisibility(visible) {
    const overlay = getOverlayState();
    const target = Boolean(visible);
    if (overlay.visible === target) {
      return;
    }
    overlay.visible = target;
    overlay.dirty = true;
    updateReferenceToggleUI();
    syncToRenderSystem({ force: true });
  }

  async function loadReferencePath(cacheKey, loader, {
    metadataOverrides = {},
    color = null,
    setActive = false,
    makeVisible = false,
  } = {}) {
    if (cacheKey && referencePathCache.has(cacheKey)) {
      const cached = referencePathCache.get(cacheKey);
      if (setActive) {
        setSegments(cached.segments, {
          metadata: cached.metadata,
          color: cached.color || color || cached.metadata?.color,
        });
        if (makeVisible) {
          setVisibility(true);
        }
      }
      return cached;
    }

    const text = await loader();
    const parsed = parseGcodeText(text);
    const converted = convertParsedSegmentsToSimulation(parsed);
    let totalLengthMm = 0;
    let totalExtrusionMm = 0;
    for (const segment of parsed.segments) {
      totalLengthMm += segment.length;
      totalExtrusionMm += segment.extrusion;
    }

    const metadata = {
      ...metadataOverrides,
      bounds: converted.bounds,
      originalBounds: parsed.bounds,
      segmentCount: converted.segments.length,
      units: {
        source: 'mm',
        simulation: 'm',
        scale: GCODE_MM_TO_SIM_SCALE,
      },
      totals: {
        lengthSim: totalLengthMm * GCODE_MM_TO_SIM_SCALE,
        extrusion: totalExtrusionMm,
      },
    };
    if (!metadata.key && cacheKey) {
      metadata.key = cacheKey;
    }
    if (color && typeof color === 'string') {
      metadata.color = color;
    }
    const record = {
      segments: converted.segments,
      metadata,
      color: color || metadata.color || '#1e90ff',
    };
    if (cacheKey) {
      referencePathCache.set(cacheKey, record);
    }
    if (setActive) {
      setSegments(record.segments, { metadata: record.metadata, color: record.color });
      if (makeVisible) {
        setVisibility(true);
      }
    }
    return record;
  }

  async function loadForPreset(presetKey, { setActive = true } = {}) {
    const descriptor = PRESET_GCODE_MAP[presetKey];
    if (!descriptor?.url) {
      return null;
    }
    const cacheKey = `preset:${presetKey}`;
    try {
      return await loadReferencePath(cacheKey, () => fetchGcodeText(descriptor.url), {
        metadataOverrides: {
          key: cacheKey,
          label: descriptor.label || presetKey,
          source: { type: 'preset', presetKey, href: descriptor.url },
        },
        color: descriptor.color,
        setActive,
      });
    } catch (error) {
      console.error('hp-sim-3d: failed to load preset G-code reference', presetKey, error);
      return null;
    }
  }

  async function loadFromFile(file, { setActive = true, makeVisible = false } = {}) {
    if (!file) {
      return null;
    }
    const cacheKey = `upload:${file.name}:${file.size}:${file.lastModified}`;
    try {
      return await loadReferencePath(cacheKey, () => file.text(), {
        metadataOverrides: {
          key: cacheKey,
          label: file.name,
          source: { type: 'upload', name: file.name, size: file.size },
          uploadedAt: Date.now(),
        },
        color: '#2dd4bf',
        setActive,
        makeVisible,
      });
    } catch (error) {
      console.error('hp-sim-3d: failed to parse uploaded G-code reference', error);
      return null;
    }
  }

  function updatePositionTraceToggleUI() {
    const renderSystem = world.getResource('renderSystem');
    const enabled = Boolean(renderSystem?.positionTraceEnabled);
    dom.positionTraceBtn?.setAttribute('aria-pressed', enabled ? 'true' : 'false');
    dom.positionTraceBtn?.classList.toggle('is-active', enabled);
    dom.positionTraceClearBtn?.classList.toggle('sim-hidden', !enabled);
    dom.positionTraceClearBtn?.closest('.sim-tool-button')?.classList.toggle('is-open', enabled);
  }

  function updateMeasureToggleUI() {
    const renderSystem = world.getResource('renderSystem');
    const enabled = Boolean(renderSystem?.measureEnabled);
    dom.measureBtn?.setAttribute('aria-pressed', enabled ? 'true' : 'false');
    dom.measureBtn?.classList.toggle('is-active', enabled);
    dom.measureClearBtn?.classList.toggle('sim-hidden', !enabled);
    dom.measureClearBtn?.closest('.sim-tool-button')?.classList.toggle('is-open', enabled);
    if (!enabled) {
      renderSystem?.clearPositionTracePreview?.();
    }
  }

  function formatPositionTraceLabel(point) {
    const mmX = point.x / GCODE_MM_TO_SIM_SCALE;
    const mmY = point.y / GCODE_MM_TO_SIM_SCALE;
    const mmZ = (Number.isFinite(point.z) ? point.z : 0.0) / GCODE_MM_TO_SIM_SCALE;
    return `(${mmX.toFixed(2)}, ${mmY.toFixed(2)}, ${mmZ.toFixed(2)})`;
  }

  function resolvePositionTracePoint(renderSystem, event) {
    const resolved = typeof renderSystem.resolvePositionTracePoint === 'function'
      ? renderSystem.resolvePositionTracePoint(world, event.clientX, event.clientY)
      : null;
    if (resolved?.point) {
      return resolved;
    }
    const projected = typeof renderSystem.projectClientToSim === 'function'
      ? renderSystem.projectClientToSim(event.clientX, event.clientY)
      : null;
    if (!projected || !Number.isFinite(projected.x) || !Number.isFinite(projected.y)) {
      return null;
    }
    return {
      point: {
        x: projected.x,
        y: projected.y,
        z: Number.isFinite(projected.z) ? projected.z : 0.0,
      },
      snapped: false,
      entityId: null,
    };
  }

  function updatePositionTracePreviewFromPointer(event) {
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem?.measureEnabled || event.target !== dom.canvas) {
      renderSystem?.clearPositionTracePreview?.();
      return;
    }
    const resolved = resolvePositionTracePoint(renderSystem, event);
    if (!resolved?.point) {
      renderSystem.clearPositionTracePreview?.();
      return;
    }
    renderSystem.setPositionTracePreview?.(resolved.point, formatPositionTraceLabel(resolved.point));
  }

  function placeMeasurePointFromPointer(event) {
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem?.measureEnabled || event.target !== dom.canvas) {
      return;
    }
    const resolved = resolvePositionTracePoint(renderSystem, event);
    const point = resolved?.point;
    if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) {
      return;
    }
    const label = formatPositionTraceLabel(point);
    renderSystem.addMeasureMarker?.(point.x, point.y, label, point.z);
    renderSystem.setPositionTracePreview?.(point, label);
    renderSystem.update?.(world, 0);
  }

  function handleMeasurePointerDown(event) {
    const renderSystem = world.getResource('renderSystem');
    const isMousePrimary = event.pointerType === 'mouse' && event.button === 0;
    const isTouchLike = event.pointerType === 'touch' || event.pointerType === 'pen';
    if (!renderSystem?.measureEnabled || event.target !== dom.canvas || (!isMousePrimary && !isTouchLike)) {
      measurePointerCandidate = null;
      return;
    }
    measurePointerCandidate = {
      pointerId: event.pointerId,
      clientX: event.clientX,
      clientY: event.clientY,
      startedAtMs: performance.now(),
      moved: false,
    };
  }

  function handleMeasurePointerMove(event) {
    if (!measurePointerCandidate || event.pointerId !== measurePointerCandidate.pointerId) {
      return;
    }
    const dx = event.clientX - measurePointerCandidate.clientX;
    const dy = event.clientY - measurePointerCandidate.clientY;
    if ((dx * dx) + (dy * dy) > 36) {
      measurePointerCandidate.moved = true;
    }
  }

  function handleMeasurePointerUp(event) {
    if (!measurePointerCandidate || event.pointerId !== measurePointerCandidate.pointerId) {
      return;
    }
    const candidate = measurePointerCandidate;
    measurePointerCandidate = null;
    const durationMs = performance.now() - candidate.startedAtMs;
    if (!candidate.moved && durationMs <= 350 && event.target === dom.canvas) {
      placeMeasurePointFromPointer(event);
    }
  }

  function clearMeasurePointer() {
    measurePointerCandidate = null;
    world.getResource('renderSystem')?.clearPositionTracePreview?.();
  }

  function bindUi() {
    if (dom.referenceToggleBtn) {
      dom.referenceToggleBtn.addEventListener('click', () => {
        const overlay = getOverlayState();
        if (!Array.isArray(overlay.segments) || overlay.segments.length === 0) {
          return;
        }
        setVisibility(!overlay.visible);
      });
    }
    if (dom.positionTraceBtn) {
      dom.positionTraceBtn.addEventListener('click', (event) => {
        event.preventDefault();
        const renderSystem = world.getResource('renderSystem');
        if (!renderSystem || typeof renderSystem.setPositionTraceEnabled !== 'function') {
          return;
        }
        renderSystem.setPositionTraceEnabled(!renderSystem.positionTraceEnabled);
        renderSystem.update?.(world, 0);
        updatePositionTraceToggleUI();
      });
    }
    if (dom.positionTraceClearBtn) {
      dom.positionTraceClearBtn.addEventListener('click', (event) => {
        event.preventDefault();
        event.stopPropagation();
        const renderSystem = world.getResource('renderSystem');
        renderSystem?.clearPositionTracePoints?.();
        renderSystem?.update?.(world, 0);
      });
    }
    if (dom.measureBtn) {
      dom.measureBtn.addEventListener('click', (event) => {
        event.preventDefault();
        const renderSystem = world.getResource('renderSystem');
        if (!renderSystem || typeof renderSystem.setMeasureEnabled !== 'function') {
          return;
        }
        renderSystem.setMeasureEnabled(!renderSystem.measureEnabled);
        renderSystem.update?.(world, 0);
        updateMeasureToggleUI();
      });
    }
    if (dom.measureClearBtn) {
      dom.measureClearBtn.addEventListener('click', (event) => {
        event.preventDefault();
        event.stopPropagation();
        const renderSystem = world.getResource('renderSystem');
        renderSystem?.clearMeasureMarkers?.();
        renderSystem?.update?.(world, 0);
      });
    }
    updateReferenceToggleUI();
  }

  return {
    bindUi,
    getState: getOverlayState,
    setSegments,
    setVisibility,
    syncToRenderSystem,
    loadForPreset,
    loadFromFile,
    updateReferenceToggleUI,
    updatePositionTraceToggleUI,
    updateMeasureToggleUI,
    handleMeasurePointerDown,
    handleMeasurePointerMove,
    handleMeasurePointerUp,
    clearMeasurePointer,
    updatePositionTracePreviewFromPointer,
  };
}
