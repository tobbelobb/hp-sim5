function matchesTapSequence(previousTap, clientX, clientY, nowMs, maxDelayMs, maxDistancePx) {
  if (!previousTap) {
    return false;
  }
  if (!(nowMs - previousTap.timeMs <= maxDelayMs)) {
    return false;
  }
  return Math.hypot(clientX - previousTap.clientX, clientY - previousTap.clientY) <= maxDistancePx;
}

export function createTapRecord(clientX, clientY, timeMs) {
  return { clientX, clientY, timeMs };
}

export function resolveNavigationCursorZoomAnchor(anchor, navigationCursorActive) {
  return navigationCursorActive ? null : anchor;
}

export function shouldHideNavigationCursorOnClick({ navigationCursorActive, clickDetail }) {
  return Boolean(navigationCursorActive) && Number.isFinite(clickDetail) && clickDetail >= 3;
}

export function resolveTouchNavigationCursorTapAction({
  navigationCursorActive,
  lastCanvasTap,
  lastNavigationCursorTap,
  clientX,
  clientY,
  nowMs,
  movementPx,
  durationMs,
  maxDelayMs,
  maxDistancePx,
  maxMovementPx,
  maxDurationMs,
}) {
  if (
    !Number.isFinite(clientX)
    || !Number.isFinite(clientY)
    || !Number.isFinite(nowMs)
    || !Number.isFinite(movementPx)
    || !Number.isFinite(durationMs)
  ) {
    return 'ignore';
  }
  if (movementPx > maxMovementPx || durationMs > maxDurationMs) {
    return 'ignore';
  }
  if (
    navigationCursorActive
    && matchesTapSequence(lastNavigationCursorTap, clientX, clientY, nowMs, maxDelayMs, maxDistancePx)
  ) {
    return 'hide';
  }
  if (matchesTapSequence(lastCanvasTap, clientX, clientY, nowMs, maxDelayMs, maxDistancePx)) {
    return 'focus';
  }
  return 'remember';
}
