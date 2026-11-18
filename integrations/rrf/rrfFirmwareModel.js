export const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);
export const STEP_CLOCK_HZ_RRF_HOST = 48_000_000 / 64; // 750 kHz step clock
export const EXTRUDER_MM_PER_STEP_RRF = 1 / 95.922; // Matches M92 E95.922 from the RRF config

export const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E', 'I', 'J', 'L', 'O'];
export const RRF_VISIBLE_AXIS_TO_SPOOL_AXIS = new Map([
    ['X', 'A'],
    ['Y', 'B'],
    ['Z', 'C'],
    ['U', 'D'],
    ['V', 'I'],
    ['W', 'J'],
    ['A', 'L'],
    ['B', 'O'],
    ['E', 'E'],
]);
export const MOTOR_AXIS_MAP = new Map([
    [40, 'A'],
    [41, 'B'],
    [42, 'C'],
    [43, 'D'], // ABCDEIJLO is the new default order of can addresses
    [44, 'I'],
    [45, 'J'],
    [46, 'L'],
    [47, 'O'],
    [48, 'E'],
]);

export function computeTicksPerBucket(dt) {
  return Math.max(1, Math.round(STEP_CLOCK_HZ_RRF_HOST * dt));
}

export function rrfMotorId(canAddress, driver = 0) {
  // Keep legacy address-only keys for driver zero. Other slots use a string
  // so the driver index is never interpreted as a decimal fraction.
  return driver === 0 ? canAddress : `${canAddress}.${driver}`;
}

export function parseRrfDriveSpecifier(value) {
  if (value == null) {
    return null;
  }
  const text = String(value).trim();
  if (!text) {
    return null;
  }
  const match = /^(\d+)(?:\.(\d+))?$/.exec(text);
  if (!match) {
    return null;
  }
  return {
    canAddress: Number(match[1]),
    driver: Number(match[2] ?? 0),
  };
}

export function parseRrfMotorAxisMapFromConfigText(configText, {
  visibleAxisToSpoolAxis = RRF_VISIBLE_AXIS_TO_SPOOL_AXIS,
} = {}) {
  const result = new Map();
  if (typeof configText !== 'string' || configText.length === 0) {
    return result;
  }
  const tokenPattern = /\b([A-Za-z])\s*([-+]?\d+(?:\.\d+)?(?::[-+]?\d+(?:\.\d+)?)*)/g;
  for (const rawLine of configText.split(/\r?\n/)) {
    const line = rawLine.split(';')[0]?.trim() || '';
    if (!/^M584\b/i.test(line)) {
      continue;
    }
    let match = tokenPattern.exec(line);
    while (match) {
      const visibleAxis = match[1].toUpperCase();
      const spoolAxis = visibleAxisToSpoolAxis.get(visibleAxis);
      if (spoolAxis) {
        for (const driveToken of match[2].split(':')) {
          const descriptor = parseRrfDriveSpecifier(driveToken);
          if (descriptor && Number.isFinite(descriptor.canAddress)) {
            result.set(rrfMotorId(descriptor.canAddress, descriptor.driver), spoolAxis);
          }
        }
      }
      match = tokenPattern.exec(line);
    }
    tokenPattern.lastIndex = 0;
  }
  return result;
}
