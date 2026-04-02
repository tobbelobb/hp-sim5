export const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);
export const STEP_CLOCK_HZ_RRF_HOST = 48_000_000 / 64; // 750 kHz step clock
export const EXTRUDER_MM_PER_STEP_RRF = 1 / 95.922; // Matches M92 E95.922 from the RRF config

export const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E', 'I', 'J', 'K', 'L', 'O'];
export const MOTOR_AXIS_MAP = new Map([
    [40, 'A'],
    [41, 'B'],
    [42, 'C'],
    [43, 'D'], // ABCDEIJKLO is the new default order of can addresses
    [44, 'E'],
    [45, 'I'],
    [46, 'J'],
    [47, 'K'],
    [48, 'L'],
    [49, 'O'],
]);

export function computeTicksPerBucket(dt) {
  return Math.max(1, Math.round(STEP_CLOCK_HZ_RRF_HOST * dt));
}
