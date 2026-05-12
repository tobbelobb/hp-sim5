export const MIN_VIEW_SCALE = 0.01;
export const MAX_VIEW_SCALE = 200;
export const ZOOM_FACTOR = 1.2;
export const ZOOM_EPSILON = 1e-3;

export function clampViewScale(value, min = MIN_VIEW_SCALE, max = MAX_VIEW_SCALE) {
  return Math.min(Math.max(value, min), max);
}

