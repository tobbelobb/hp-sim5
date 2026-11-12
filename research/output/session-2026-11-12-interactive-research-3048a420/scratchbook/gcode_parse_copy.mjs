
const GCODE_MM_TO_SIM_SCALE = 0.001;
const GCODE_EXTRUSION_EPSILON = 1e-6;
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
    // Extrusion without motion (a prime after a retract) stays as a
    // zero-length segment so the reference accounts for all filament.
    if (extrusionDelta > GCODE_EXTRUSION_EPSILON) {
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
