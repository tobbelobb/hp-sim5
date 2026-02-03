const DEBUG_STATE = Symbol('autocal.debugSweep');

export function attachDebugState(sendFn, updates = {}) {
  if (!sendFn) {
    return null;
  }
  const existing = sendFn[DEBUG_STATE] || {};
  const next = { ...existing, ...updates };
  if (next.enabled == null) {
    next.enabled = false;
  }
  sendFn[DEBUG_STATE] = next;
  return next;
}

export function getDebugState(sendFn) {
  return sendFn ? sendFn[DEBUG_STATE] || null : null;
}

export function parseCallSite(stack, { skip = 0 } = {}) {
  if (!stack) {
    return null;
  }
  const matches = [...stack.matchAll(/((?:autocal\/control|scripts)\/[^:\n]+\.mjs):(\d+):\d+/g)];
  if (matches.length === 0) {
    return null;
  }
  const idx = Math.min(Math.max(0, skip), matches.length - 1);
  const file = matches[idx][1];
  const line = parseInt(matches[idx][2], 10);
  if (!Number.isFinite(line)) {
    return null;
  }
  return { file, line };
}

export function formatCallSite(callSite) {
  if (!callSite) {
    return 'unknown';
  }
  return `${callSite.file}:${callSite.line}`;
}

export function updateDebugAngles(sendFn, anglesDeg) {
  const state = getDebugState(sendFn);
  if (!state?.enabled) {
    return;
  }
  state.lastAngles = Array.isArray(anglesDeg) ? anglesDeg.slice() : null;
  const mmPerDeg = state.mmPerDeg;
  if (!Array.isArray(mmPerDeg) || !Array.isArray(anglesDeg)) {
    return;
  }
  const lengths = anglesDeg.map((angle, idx) => {
    const mmPer = mmPerDeg[idx];
    if (!Number.isFinite(angle) || !Number.isFinite(mmPer)) {
      return null;
    }
    return angle * mmPer;
  });
  state.lastLengths = lengths;
}
