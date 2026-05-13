import { DEFAULT_VIEW_SCALE } from './appState.js';
import { InputSystem } from './hangprinter_input.js';
import {
  createTapRecord,
  resolveNavigationCursorZoomAnchor,
  resolveTouchNavigationCursorTapAction,
  shouldHideNavigationCursorOnClick,
} from './navigation_cursor_interactions.js';

export const MIN_VIEW_SCALE = 0.01;
export const MAX_VIEW_SCALE = 200;
export const ZOOM_FACTOR = 1.2;
export const ZOOM_EPSILON = 1e-3;

export function clampViewScale(value, min = MIN_VIEW_SCALE, max = MAX_VIEW_SCALE) {
  return Math.min(Math.max(value, min), max);
}

const MOBILE_SECONDARY_CONTROLS_TIMEOUT_MS = 3000;
const MOBILE_SECONDARY_CONTROLS_INTERACTION_DELAY_MS = 150;
const DOUBLE_TAP_MAX_DELAY_MS = 320;
const DOUBLE_TAP_MAX_DURATION_MS = 280;
const DOUBLE_TAP_MAX_DISTANCE_PX = 24;
const DOUBLE_TAP_MAX_MOVEMENT_PX = 18;

export function createViewController({
  document,
  window,
  world,
  state,
  dom,
  runtime,
  machines,
  machineMenu,
  inspectionTools,
  getCommands = () => null,
} = {}) {
  const supportsMatchMedia = typeof window.matchMedia === 'function';
  const mobileLayoutQuery = supportsMatchMedia ? window.matchMedia('(max-width: 600px)') : null;
  const initialTouchAction = dom.canvas ? dom.canvas.style.touchAction || '' : '';
  const isMobileLayout = () => (mobileLayoutQuery ? mobileLayoutQuery.matches : window.innerWidth <= 600);
  let secondaryControlsHideTimeout = null;
  let secondaryControlsInteractionEnableTimeout = null;
  let secondaryControlsEverShown = false;
  let secondaryControlsUserPreference = null;
  let secondaryControlsAutoActive = false;
  let viewListenerSystem = null;
  const touchTapStarts = new Map();
  const activeCanvasTouchPointers = new Set();
  let touchGestureHadMultiplePointers = false;
  let lastCanvasTap = null;
  let lastNavigationCursorTouchTap = null;
  let lastTouchNavigationCursorFocusMs = Number.NEGATIVE_INFINITY;

  function getInputSystem() {
    return world.systems.find((sys) => sys instanceof InputSystem) || null;
  }

  function setButtonDisabled(button, disabled) {
    if (!button) {
      return;
    }
    button.disabled = Boolean(disabled);
    if (button.disabled) {
      button.setAttribute('aria-disabled', 'true');
    } else {
      button.removeAttribute('aria-disabled');
    }
  }

  function clearSecondaryControlsInteractionDelay() {
    if (secondaryControlsInteractionEnableTimeout != null) {
      window.clearTimeout(secondaryControlsInteractionEnableTimeout);
      secondaryControlsInteractionEnableTimeout = null;
    }
  }

  function setSecondaryControlsInteractive(enabled) {
    if (dom.secondaryControls) {
      dom.secondaryControls.dataset.interactive = enabled ? 'true' : 'false';
    }
  }

  function scheduleSecondaryControlsInteractionEnable() {
    if (!dom.secondaryControls) {
      return;
    }
    clearSecondaryControlsInteractionDelay();
    secondaryControlsInteractionEnableTimeout = window.setTimeout(() => {
      secondaryControlsInteractionEnableTimeout = null;
      setSecondaryControlsInteractive(true);
    }, MOBILE_SECONDARY_CONTROLS_INTERACTION_DELAY_MS);
  }

  function hideSecondaryControlsForMobile({ force = false } = {}) {
    if (!dom.secondaryControls) {
      return;
    }
    if (!force && secondaryControlsUserPreference === true) {
      return;
    }
    dom.secondaryControls.classList.add('sim-hidden');
    clearSecondaryControlsInteractionDelay();
    setSecondaryControlsInteractive(false);
    if (secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
      secondaryControlsHideTimeout = null;
    }
  }

  function showSecondaryControlsForMobile({ persist = false } = {}) {
    if (!dom.secondaryControls || !isMobileLayout()) {
      return;
    }
    if (!persist && secondaryControlsUserPreference === false) {
      return;
    }
    const wasHidden = dom.secondaryControls.classList.contains('sim-hidden');
    if (wasHidden) {
      setSecondaryControlsInteractive(false);
    }
    dom.secondaryControls.classList.remove('sim-hidden');
    secondaryControlsEverShown = true;
    if (secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
    }
    if (persist || secondaryControlsUserPreference === true) {
      secondaryControlsHideTimeout = null;
      setSecondaryControlsInteractive(true);
      return;
    }
    secondaryControlsHideTimeout = window.setTimeout(() => {
      hideSecondaryControlsForMobile();
    }, MOBILE_SECONDARY_CONTROLS_TIMEOUT_MS);
    if (wasHidden) {
      scheduleSecondaryControlsInteractionEnable();
    } else {
      setSecondaryControlsInteractive(true);
    }
  }

  function computeSecondaryControlsDesired() {
    if (secondaryControlsUserPreference === true) {
      return true;
    }
    if (secondaryControlsUserPreference === false) {
      return false;
    }
    return secondaryControlsAutoActive;
  }

  function updateSecondaryToggleButton() {
    if (!dom.secondaryToggleBtn) {
      return;
    }
    const shouldShow = computeSecondaryControlsDesired();
    const hasPreference = secondaryControlsUserPreference !== null;
    dom.secondaryToggleBtn.setAttribute('aria-expanded', shouldShow ? 'true' : 'false');
    const baseTitle = shouldShow ? 'Collapse secondary controls' : 'Expand secondary controls';
    dom.secondaryToggleBtn.title = hasPreference ? `${baseTitle} (Alt-click to reset)` : baseTitle;
  }

  function applySecondaryControlsVisibility() {
    const shouldShow = computeSecondaryControlsDesired();
    if (!dom.secondaryControls) {
      updateSecondaryToggleButton();
      return;
    }
    if (isMobileLayout()) {
      if (shouldShow) {
        showSecondaryControlsForMobile({ persist: secondaryControlsUserPreference === true });
      } else {
        hideSecondaryControlsForMobile({ force: true });
      }
      updateSecondaryToggleButton();
      return;
    }
    if (secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
      secondaryControlsHideTimeout = null;
    }
    if (shouldShow) {
      dom.secondaryControls.classList.remove('sim-hidden');
      secondaryControlsEverShown = true;
      clearSecondaryControlsInteractionDelay();
      setSecondaryControlsInteractive(true);
    } else if (secondaryControlsUserPreference === false || !secondaryControlsEverShown) {
      dom.secondaryControls.classList.add('sim-hidden');
      setSecondaryControlsInteractive(false);
    } else {
      clearSecondaryControlsInteractionDelay();
      setSecondaryControlsInteractive(true);
    }
    updateSecondaryToggleButton();
  }

  function setSecondaryControlsVisible(active) {
    secondaryControlsAutoActive = Boolean(active);
    applySecondaryControlsVisibility();
  }

  function updatePrintingState() {
    dom.simButtons?.classList.toggle('is-printing', Boolean(state.printActive));
    setSecondaryControlsVisible(state.printActive && machines.getMachines().length > 0);
  }

  function updateQualityToggleLabel() {
    const label = dom.qualityToggleLabel;
    if (!label) {
      return;
    }
    label.textContent = isMobileLayout() ? 'Live QC' : 'Live Quality Checks';
  }

  function setSpeedButtonsEnabled(enabled) {
    [dom.speedSlowerBtn, dom.speedFasterBtn].forEach((button) => {
      setButtonDisabled(button, !enabled);
    });
  }

  function updateZoomButtonState() {
    const allowZoom = runtime.isStageReady() && machines.getMachines().length > 0;
    if (dom.zoomInBtn) {
      setButtonDisabled(dom.zoomInBtn, !allowZoom || state.view.scale >= MAX_VIEW_SCALE - ZOOM_EPSILON);
    }
    if (dom.zoomOutBtn) {
      setButtonDisabled(dom.zoomOutBtn, !allowZoom || state.view.scale <= MIN_VIEW_SCALE + ZOOM_EPSILON);
    }
  }

  function syncRenderSystem(viewState, { clearExtrusions = false } = {}) {
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem || typeof renderSystem.setViewTransform !== 'function') {
      return;
    }
    renderSystem.setViewTransform(viewState);
    renderSystem.setNavigationCursorVisible?.(state.navigationCursorActive);
    inspectionTools?.syncToRenderSystem?.({ force: true });
    if (clearExtrusions) {
      renderSystem.clearExtrusions?.();
      renderSystem.clearPositionTrace?.({ keepMarkers: true });
    }
  }

  function applyViewStateFromController(partial = {}, options = {}) {
    if (machines.getMachines().length === 0) {
      updateZoomButtonState();
      return;
    }
    if (typeof partial.scale === 'number') {
      state.view.scale = clampViewScale(partial.scale);
    }
    if (typeof partial.offsetX === 'number') {
      state.view.offsetX = partial.offsetX;
    }
    if (typeof partial.offsetY === 'number') {
      state.view.offsetY = partial.offsetY;
    }
    if (typeof partial.offsetZ === 'number') {
      state.view.offsetZ = partial.offsetZ;
    }
    getInputSystem()?.setViewTransform?.({
      scaleMultiplier: state.view.scale,
      offsetX: state.view.offsetX,
      offsetY: state.view.offsetY,
      offsetZ: state.view.offsetZ,
    });
    syncRenderSystem({
      scaleMultiplier: state.view.scale,
      offsetX: state.view.offsetX,
      offsetY: state.view.offsetY,
      offsetZ: state.view.offsetZ,
    }, options);
    updateZoomButtonState();
  }

  function handleInputViewChange(viewState = {}, options = {}) {
    const nextScale = typeof viewState.scale === 'number' ? clampViewScale(viewState.scale) : state.view.scale;
    const nextOffsetX = typeof viewState.offsetX === 'number' ? viewState.offsetX : state.view.offsetX;
    const nextOffsetY = typeof viewState.offsetY === 'number' ? viewState.offsetY : state.view.offsetY;
    const nextOffsetZ = typeof viewState.offsetZ === 'number' ? viewState.offsetZ : state.view.offsetZ;
    const renderSystem = world.getResource('renderSystem');
    const scaleChanged = Math.abs(nextScale - state.view.scale) > ZOOM_EPSILON;
    const dOffsetX = nextOffsetX - state.view.offsetX;
    const dOffsetY = nextOffsetY - state.view.offsetY;
    const offsetDepthChanged = Math.abs(nextOffsetZ - state.view.offsetZ) > ZOOM_EPSILON;
    state.view.scale = nextScale;
    state.view.offsetX = nextOffsetX;
    state.view.offsetY = nextOffsetY;
    state.view.offsetZ = nextOffsetZ;
    if (
      !scaleChanged
      && !options?.forceRedraw
      && !offsetDepthChanged
      && renderSystem
      && typeof renderSystem.shiftExtrusionsForPan === 'function'
    ) {
      syncRenderSystem({
        scaleMultiplier: state.view.scale,
        offsetX: state.view.offsetX,
        offsetY: state.view.offsetY,
        offsetZ: state.view.offsetZ,
      });
      try {
        renderSystem.shiftExtrusionsForPan(world, dOffsetX, dOffsetY);
      } catch (_err) {
        renderSystem.clearExtrusions?.();
        renderSystem.clearPositionTrace?.({ keepMarkers: true });
      }
    } else {
      syncRenderSystem({
        scaleMultiplier: state.view.scale,
        offsetX: state.view.offsetX,
        offsetY: state.view.offsetY,
        offsetZ: state.view.offsetZ,
      }, { clearExtrusions: true });
    }
    updateZoomButtonState();
  }

  function attachInputViewListener() {
    const inputSystem = getInputSystem();
    if (!inputSystem || typeof inputSystem.setViewChangeListener !== 'function') {
      return;
    }
    if (viewListenerSystem && viewListenerSystem !== inputSystem) {
      viewListenerSystem.setViewChangeListener?.(null);
    }
    if (viewListenerSystem !== inputSystem) {
      inputSystem.setViewChangeListener(handleInputViewChange);
      viewListenerSystem = inputSystem;
    }
  }

  function syncCanvasDimensions() {
    const canvas = dom.canvas;
    if (!canvas) {
      return;
    }
    const width = Math.max(1, Math.floor(canvas.clientWidth));
    const height = Math.max(1, Math.floor(canvas.clientHeight));
    const resized = canvas.width !== width || canvas.height !== height;
    if (resized) {
      canvas.width = width;
      canvas.height = height;
    }
    world.getResource('renderSystem')?.setCanvasSize?.(canvas.width, canvas.height);
    if (resized) {
      reapplyViewState({ clearExtrusions: true });
    }
  }

  function reapplyViewState(options = {}) {
    if (machines.getMachines().length === 0) {
      return;
    }
    attachInputViewListener();
    applyViewStateFromController({
      scale: state.view.scale,
      offsetX: state.view.offsetX,
      offsetY: state.view.offsetY,
      offsetZ: state.view.offsetZ,
    }, options);
    getInputSystem()?.setInteractionMode?.(state.panModeActive ? 'pan' : 'select');
  }

  function resetViewStateDefaults() {
    state.view.scale = DEFAULT_VIEW_SCALE;
    state.view.offsetX = 0;
    state.view.offsetY = 0;
    state.view.offsetZ = 0;
  }

  function setNavigationCursorActive(active) {
    state.navigationCursorActive = Boolean(active);
    world.getResource('renderSystem')?.setNavigationCursorVisible?.(state.navigationCursorActive);
    if (!state.navigationCursorActive) {
      lastNavigationCursorTouchTap = null;
    }
  }

  function setPanMode(active) {
    state.panModeActive = Boolean(active);
    dom.panModeBtn?.setAttribute('aria-pressed', state.panModeActive ? 'true' : 'false');
    dom.panModeBtn?.classList.toggle('is-active', state.panModeActive);
    if (dom.canvas) {
      dom.canvas.style.touchAction = state.panModeActive || isMobileLayout() ? 'none' : initialTouchAction;
    }
    if (dom.simApp) {
      dom.simApp.style.cursor = state.panModeActive ? 'grab' : '';
    }
    getInputSystem()?.setInteractionMode?.(state.panModeActive ? 'pan' : 'select');
  }

  function handleLayoutChange() {
    const mobile = isMobileLayout();
    updateQualityToggleLabel();
    inspectionTools?.updateReferenceToggleUI?.();
    setPanMode(state.panModeActive);
    if (!mobile && secondaryControlsHideTimeout) {
      clearTimeout(secondaryControlsHideTimeout);
      secondaryControlsHideTimeout = null;
    }
    applySecondaryControlsVisibility();
    machineMenu?.syncPlacement?.();
  }

  function getCanvasBaseScale() {
    const simHeight = world.getResource('simHeight');
    return Number.isFinite(simHeight) && simHeight > 0 ? dom.canvas.height / simHeight : null;
  }

  function getViewPlaneMetrics(scaleMultiplier = state.view.scale) {
    const renderSystem = world.getResource('renderSystem');
    const metrics = renderSystem?.getViewPlaneMetrics?.(scaleMultiplier);
    if (metrics && Number.isFinite(metrics.worldUnitsPerPixel) && metrics.worldUnitsPerPixel > 0) {
      return metrics;
    }
    const baseScale = getCanvasBaseScale();
    if (!baseScale || !(scaleMultiplier > 0)) {
      return null;
    }
    return {
      right: { x: 1, y: 0, z: 0 },
      up: { x: 0, y: 1, z: 0 },
      worldUnitsPerPixel: 1 / (baseScale * scaleMultiplier),
    };
  }

  function getViewPlaneOffsetVector(clientX, clientY, metrics) {
    if (!dom.canvas || !metrics || !(metrics.worldUnitsPerPixel > 0)) {
      return null;
    }
    const rect = dom.canvas.getBoundingClientRect();
    if (!rect || rect.width <= 0 || rect.height <= 0) {
      return null;
    }
    const canvasPixelX = (clientX - rect.left) * (dom.canvas.width / rect.width);
    const canvasPixelY = (clientY - rect.top) * (dom.canvas.height / rect.height);
    const pixelX = canvasPixelX - dom.canvas.width / 2;
    const pixelY = dom.canvas.height / 2 - canvasPixelY;
    return {
      x: pixelX * metrics.worldUnitsPerPixel * (metrics.right?.x ?? 0) + pixelY * metrics.worldUnitsPerPixel * (metrics.up?.x ?? 0),
      y: pixelX * metrics.worldUnitsPerPixel * (metrics.right?.y ?? 0) + pixelY * metrics.worldUnitsPerPixel * (metrics.up?.y ?? 0),
      z: pixelX * metrics.worldUnitsPerPixel * (metrics.right?.z ?? 0) + pixelY * metrics.worldUnitsPerPixel * (metrics.up?.z ?? 0),
    };
  }

  function computeViewPlaneShift(previousAnchor, nextAnchor, nextScaleMultiplier = state.view.scale) {
    const previousMetrics = getViewPlaneMetrics(state.view.scale);
    const nextMetrics = getViewPlaneMetrics(nextScaleMultiplier);
    const previousOffset = getViewPlaneOffsetVector(previousAnchor.x, previousAnchor.y, previousMetrics);
    const nextOffset = getViewPlaneOffsetVector(nextAnchor.x, nextAnchor.y, nextMetrics);
    if (!previousOffset || !nextOffset) {
      return null;
    }
    return {
      x: previousOffset.x - nextOffset.x,
      y: previousOffset.y - nextOffset.y,
      z: previousOffset.z - nextOffset.z,
    };
  }

  function getCurrentViewTarget() {
    const target = getViewPlaneMetrics(state.view.scale)?.target;
    if (target && Number.isFinite(target.x) && Number.isFinite(target.y) && Number.isFinite(target.z)) {
      return target;
    }
    return { x: state.view.offsetX, y: state.view.offsetY, z: state.view.offsetZ };
  }

  function isPointOnPrintSurface(renderSystem, point) {
    if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) {
      return false;
    }
    const board = renderSystem?.board;
    if (!board?.scale) {
      return true;
    }
    const halfWidth = Math.abs(board.scale.x || 0) * 0.5 + 1e-6;
    const halfHeight = Math.abs(board.scale.y || 0) * 0.5 + 1e-6;
    const centerX = Number.isFinite(board.position?.x) ? board.position.x : 0;
    const centerY = Number.isFinite(board.position?.y) ? board.position.y : 0;
    return Math.abs(point.x - centerX) <= halfWidth && Math.abs(point.y - centerY) <= halfHeight;
  }

  function focusNavigationCursorAtPoint(point) {
    if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y) || !Number.isFinite(point.z)) {
      return false;
    }
    const currentTarget = getCurrentViewTarget();
    applyViewStateFromController({
      offsetX: state.view.offsetX + point.x - currentTarget.x,
      offsetY: state.view.offsetY + point.y - currentTarget.y,
      offsetZ: state.view.offsetZ + point.z - currentTarget.z,
    }, { clearExtrusions: true });
    setNavigationCursorActive(true);
    return true;
  }

  function focusNavigationCursorAtClientPoint(clientX, clientY) {
    if (!runtime.isStageReady() || machines.getMachines().length === 0) {
      return false;
    }
    const renderSystem = world.getResource('renderSystem');
    if (!renderSystem || typeof renderSystem.projectClientToSim !== 'function') {
      return false;
    }
    const point = renderSystem.projectClientToSim(clientX, clientY);
    if (!isPointOnPrintSurface(renderSystem, point)) {
      return false;
    }
    return focusNavigationCursorAtPoint(point);
  }

  function resetCanvasTapTracking() {
    touchTapStarts.clear();
    activeCanvasTouchPointers.clear();
    touchGestureHadMultiplePointers = false;
    lastCanvasTap = null;
    lastNavigationCursorTouchTap = null;
  }

  function handleCanvasTouchPointerDown(event) {
    if (event.pointerType !== 'touch') {
      return;
    }
    activeCanvasTouchPointers.add(event.pointerId);
    touchTapStarts.set(event.pointerId, {
      clientX: event.clientX,
      clientY: event.clientY,
      timeMs: performance.now(),
    });
    if (activeCanvasTouchPointers.size > 1) {
      touchGestureHadMultiplePointers = true;
      lastCanvasTap = null;
    }
  }

  function finalizeCanvasTouchPointer(pointerId) {
    activeCanvasTouchPointers.delete(pointerId);
    touchTapStarts.delete(pointerId);
    if (activeCanvasTouchPointers.size === 0) {
      touchGestureHadMultiplePointers = false;
    }
  }

  function handleCanvasTouchPointerUp(event) {
    if (event.pointerType !== 'touch') {
      return;
    }
    const tapStart = touchTapStarts.get(event.pointerId);
    const hadMultiplePointers = touchGestureHadMultiplePointers;
    finalizeCanvasTouchPointer(event.pointerId);
    if (!tapStart) {
      return;
    }
    const nowMs = performance.now();
    const movementPx = Math.hypot(event.clientX - tapStart.clientX, event.clientY - tapStart.clientY);
    const durationMs = nowMs - tapStart.timeMs;
    if (hadMultiplePointers) {
      if (activeCanvasTouchPointers.size === 0) {
        lastCanvasTap = null;
      }
      return;
    }
    const action = resolveTouchNavigationCursorTapAction({
      navigationCursorActive: state.navigationCursorActive,
      lastCanvasTap,
      lastNavigationCursorTap: lastNavigationCursorTouchTap,
      clientX: event.clientX,
      clientY: event.clientY,
      nowMs,
      movementPx,
      durationMs,
      maxDelayMs: DOUBLE_TAP_MAX_DELAY_MS,
      maxDistancePx: DOUBLE_TAP_MAX_DISTANCE_PX,
      maxMovementPx: DOUBLE_TAP_MAX_MOVEMENT_PX,
      maxDurationMs: DOUBLE_TAP_MAX_DURATION_MS,
    });
    if (action === 'hide') {
      setNavigationCursorActive(false);
      event.preventDefault();
      lastTouchNavigationCursorFocusMs = nowMs;
      lastCanvasTap = null;
      return;
    }
    if (action === 'focus') {
      if (focusNavigationCursorAtClientPoint(event.clientX, event.clientY)) {
        event.preventDefault();
        lastTouchNavigationCursorFocusMs = nowMs;
        lastNavigationCursorTouchTap = createTapRecord(event.clientX, event.clientY, nowMs);
      } else {
        lastNavigationCursorTouchTap = null;
      }
      lastCanvasTap = null;
      return;
    }
    if (action === 'remember') {
      lastCanvasTap = createTapRecord(event.clientX, event.clientY, nowMs);
      return;
    }
    if (activeCanvasTouchPointers.size === 0) {
      lastCanvasTap = null;
    }
  }

  function handleCanvasDoubleClick(event) {
    if (performance.now() - lastTouchNavigationCursorFocusMs <= DOUBLE_TAP_MAX_DELAY_MS) {
      return;
    }
    if (focusNavigationCursorAtClientPoint(event.clientX, event.clientY)) {
      event.preventDefault();
    }
  }

  function handleCanvasClick(event) {
    if (performance.now() - lastTouchNavigationCursorFocusMs <= DOUBLE_TAP_MAX_DELAY_MS) {
      return;
    }
    if (!shouldHideNavigationCursorOnClick({ navigationCursorActive: state.navigationCursorActive, clickDetail: event.detail })) {
      return;
    }
    setNavigationCursorActive(false);
    event.preventDefault();
  }

  function applyZoomAtScale(targetScale, anchor = null) {
    if (!runtime.isStageReady() || machines.getMachines().length === 0) {
      return;
    }
    const clampedScale = clampViewScale(targetScale);
    if (Math.abs(clampedScale - state.view.scale) < ZOOM_EPSILON) {
      return;
    }
    let nextOffsetX = state.view.offsetX;
    let nextOffsetY = state.view.offsetY;
    let nextOffsetZ = state.view.offsetZ;
    if (anchor && dom.canvas) {
      const shift = computeViewPlaneShift(anchor, anchor, clampedScale);
      if (shift) {
        nextOffsetX += shift.x;
        nextOffsetY += shift.y;
        nextOffsetZ += shift.z;
      }
    }
    applyViewStateFromController({
      scale: clampedScale,
      offsetX: nextOffsetX,
      offsetY: nextOffsetY,
      offsetZ: nextOffsetZ,
    }, { clearExtrusions: true });
  }

  function normalizeWheelDelta(delta, deltaMode) {
    if (!Number.isFinite(delta)) {
      return 0;
    }
    if (deltaMode === 1) {
      return delta * 40;
    }
    if (deltaMode === 2) {
      return delta * 800;
    }
    return delta;
  }

  function handleCanvasWheel(event) {
    if (!runtime.isStageReady() || machines.getMachines().length === 0) {
      return;
    }
    event.preventDefault();
    const normalized = normalizeWheelDelta(event.deltaY, event.deltaMode);
    if (normalized === 0) {
      return;
    }
    const intensity = Math.min(4, Math.max(0.05, Math.abs(normalized) / 240));
    const factor = Math.pow(ZOOM_FACTOR, intensity);
    applyZoomAtScale(
      state.view.scale * (normalized < 0 ? factor : 1 / factor),
      resolveNavigationCursorZoomAnchor({ x: event.clientX, y: event.clientY }, state.navigationCursorActive)
    );
  }

  function adjustZoom(multiplier) {
    applyZoomAtScale(state.view.scale * multiplier);
  }

  function handleFullscreenChange() {
    const fullscreenElement =
      document.fullscreenElement ||
      document.webkitFullscreenElement ||
      document.msFullscreenElement ||
      null;
    const isActive = fullscreenElement === dom.simApp;
    const wasActive = state.fullscreenActive;
    state.fullscreenActive = isActive;
    dom.simApp?.classList.toggle('is-fullscreen', isActive);
    if (dom.fullscreenBtn) {
      dom.fullscreenBtn.textContent = isActive ? 'Exit Fullscreen' : 'Fullscreen';
      dom.fullscreenBtn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
      dom.fullscreenBtn.disabled = false;
      dom.fullscreenBtn.removeAttribute('disabled');
      dom.fullscreenBtn.removeAttribute('aria-disabled');
    }
    if (wasActive !== isActive) {
      adjustZoom(isActive ? 0.5 : 2.0);
    }
    requestAnimationFrame(() => syncCanvasDimensions());
  }

  function toggleFullscreen() {
    if (!dom.simApp) {
      return;
    }
    const fullscreenElement =
      document.fullscreenElement ||
      document.webkitFullscreenElement ||
      document.msFullscreenElement ||
      null;
    const isActive = fullscreenElement === dom.simApp;
    const requestFullscreen = dom.simApp.requestFullscreen || dom.simApp.webkitRequestFullscreen || dom.simApp.msRequestFullscreen;
    const exitFullscreen = document.exitFullscreen || document.webkitExitFullscreen || document.msExitFullscreen;
    try {
      const result = !isActive && requestFullscreen
        ? requestFullscreen.call(dom.simApp)
        : (isActive && exitFullscreen ? exitFullscreen.call(document) : null);
      result?.catch?.((error) => console.warn('Slideprinter demo: fullscreen request failed.', error));
    } catch (error) {
      console.warn('Slideprinter demo: fullscreen request failed.', error);
    }
  }

  function formatTimeScale(scale) {
    if (!Number.isFinite(scale) || scale <= 0) {
      return '1';
    }
    if (scale >= 100) {
      return scale.toFixed(0);
    }
    if (scale >= 10) {
      return parseFloat(scale.toFixed(1)).toString();
    }
    const rounded = parseFloat(scale.toFixed(2));
    return Number.isInteger(rounded) ? String(rounded) : rounded.toString();
  }

  function showSpeedStatus(scale) {
    if (!dom.speedStatusEl) {
      return;
    }
    dom.speedStatusEl.textContent = `current speed: ${formatTimeScale(scale)}x realtime`;
    dom.speedStatusEl.classList.remove('sim-hidden');
  }

  function resetForEmptyScene() {
    world.clear();
    const renderSystem = world.getResource('renderSystem');
    renderSystem?.resetVisuals?.();
    renderSystem?.update?.(world, 0);
    resetViewStateDefaults();
    setNavigationCursorActive(false);
    resetCanvasTapTracking();
    setPanMode(false);
    setButtonDisabled(dom.panModeBtn, true);
    updateZoomButtonState();
  }

  function setSceneControlsEnabled(enabled) {
    setSpeedButtonsEnabled(enabled);
    setButtonDisabled(dom.panModeBtn, !enabled);
    updateZoomButtonState();
  }

  function bindUi() {
    if (dom.secondaryControls) {
      setSecondaryControlsInteractive(false);
    }
    dom.zoomInBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      adjustZoom(ZOOM_FACTOR);
    });
    dom.zoomOutBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      adjustZoom(1 / ZOOM_FACTOR);
    });
    dom.panModeBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      setPanMode(!state.panModeActive);
    });
    dom.fullscreenBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      toggleFullscreen();
    });
    dom.secondaryToggleBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      if (event.altKey || event.metaKey) {
        secondaryControlsUserPreference = null;
      } else {
        const isVisible = dom.secondaryControls ? !dom.secondaryControls.classList.contains('sim-hidden') : computeSecondaryControlsDesired();
        secondaryControlsUserPreference = isVisible ? false : true;
      }
      applySecondaryControlsVisibility();
    });
    dom.canvas?.addEventListener('pointerdown', (event) => {
      if (isMobileLayout() && secondaryControlsUserPreference !== false) {
        showSecondaryControlsForMobile({ persist: secondaryControlsUserPreference === true });
      }
      inspectionTools?.handleMeasurePointerDown?.(event);
      handleCanvasTouchPointerDown(event);
    });
    dom.canvas?.addEventListener('pointerup', (event) => {
      inspectionTools?.handleMeasurePointerUp?.(event);
      handleCanvasTouchPointerUp(event);
    });
    dom.canvas?.addEventListener('pointercancel', (event) => {
      inspectionTools?.clearMeasurePointer?.();
      if (event.pointerType === 'touch') {
        finalizeCanvasTouchPointer(event.pointerId);
      }
    });
    dom.canvas?.addEventListener('click', handleCanvasClick);
    dom.canvas?.addEventListener('dblclick', handleCanvasDoubleClick);
    dom.canvas?.addEventListener('wheel', handleCanvasWheel, { passive: false });
    dom.canvas?.addEventListener('pointermove', (event) => {
      inspectionTools?.handleMeasurePointerMove?.(event);
      inspectionTools?.updatePositionTracePreviewFromPointer?.(event);
    });
    dom.canvas?.addEventListener('pointerleave', () => {
      inspectionTools?.clearMeasurePointer?.();
    });
    dom.secondaryControls?.addEventListener('pointerdown', () => {
      if (isMobileLayout() && secondaryControlsUserPreference !== false) {
        showSecondaryControlsForMobile({ persist: secondaryControlsUserPreference === true });
      }
    });
    document.addEventListener('fullscreenchange', handleFullscreenChange);
    document.addEventListener('webkitfullscreenchange', handleFullscreenChange);
    document.addEventListener('msfullscreenchange', handleFullscreenChange);
    window.addEventListener('resize', () => {
      syncCanvasDimensions();
      handleLayoutChange();
    });
    if (mobileLayoutQuery) {
      if (typeof mobileLayoutQuery.addEventListener === 'function') {
        mobileLayoutQuery.addEventListener('change', handleLayoutChange);
      } else if (typeof mobileLayoutQuery.addListener === 'function') {
        mobileLayoutQuery.addListener(handleLayoutChange);
      }
    }
    document.addEventListener('keydown', (event) => {
      if (event.defaultPrevented || event.repeat) {
        return;
      }
      const target = event.target;
      const tagName = target?.tagName ? target.tagName.toUpperCase() : '';
      if (target?.isContentEditable || tagName === 'INPUT' || tagName === 'TEXTAREA' || tagName === 'SELECT') {
        return;
      }
      const keyToButton = {
        r: dom.resetBtn,
        R: dom.resetBtn,
        ' ': dom.pauseBtn,
        Spacebar: dom.pauseBtn,
        '>': dom.speedFasterBtn,
        '<': dom.speedSlowerBtn,
        s: dom.referenceToggleBtn,
        S: dom.referenceToggleBtn,
        q: dom.qualityToggle,
        Q: dom.qualityToggle,
      };
      const button = keyToButton[event.key];
      if (button && !button.disabled) {
        button.click();
        event.preventDefault();
      }
    });
    updateQualityToggleLabel();
    setSpeedButtonsEnabled(false);
    updateSecondaryToggleButton();
    updateZoomButtonState();
  }

  return {
    bindUi,
    isMobileLayout,
    setButtonDisabled,
    updatePrintingState,
    setSecondaryControlsVisible,
    setSpeedButtonsEnabled,
    setSceneControlsEnabled,
    updateZoomButtonState,
    showSpeedStatus,
    handleLayoutChange,
    syncCanvasDimensions,
    reapplyViewState,
    resetViewStateDefaults,
    setNavigationCursorActive,
    resetCanvasTapTracking,
    setPanMode,
    resetForEmptyScene,
  };
}
