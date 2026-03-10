import {
  createTapRecord,
  resolveNavigationCursorZoomAnchor,
  resolveTouchNavigationCursorTapAction,
  shouldHideNavigationCursorOnClick,
} from '../../../hp-sim-3d/assets/navigation_cursor_interactions.js';

describe('hp-sim navigation cursor interactions', () => {
  test('keeps wheel zoom centered on the nav cursor when it is active', () => {
    const anchor = { x: 180, y: 240 };

    expect(resolveNavigationCursorZoomAnchor(anchor, false)).toBe(anchor);
    expect(resolveNavigationCursorZoomAnchor(anchor, true)).toBe(null);
  });

  test('turns a double tap into focus and the next tap into hide', () => {
    const firstTap = createTapRecord(180, 240, 1000);
    const focusAction = resolveTouchNavigationCursorTapAction({
      navigationCursorActive: false,
      lastCanvasTap: firstTap,
      lastNavigationCursorTap: null,
      clientX: 184,
      clientY: 236,
      nowMs: 1200,
      movementPx: 3,
      durationMs: 80,
      maxDelayMs: 320,
      maxDistancePx: 24,
      maxMovementPx: 18,
      maxDurationMs: 280,
    });

    expect(focusAction).toBe('focus');

    const hideAction = resolveTouchNavigationCursorTapAction({
      navigationCursorActive: true,
      lastCanvasTap: null,
      lastNavigationCursorTap: createTapRecord(184, 236, 1200),
      clientX: 186,
      clientY: 238,
      nowMs: 1360,
      movementPx: 2,
      durationMs: 70,
      maxDelayMs: 320,
      maxDistancePx: 24,
      maxMovementPx: 18,
      maxDurationMs: 280,
    });

    expect(hideAction).toBe('hide');
  });

  test('ignores long or dragged taps for nav cursor gestures', () => {
    expect(resolveTouchNavigationCursorTapAction({
      navigationCursorActive: false,
      lastCanvasTap: null,
      lastNavigationCursorTap: null,
      clientX: 180,
      clientY: 240,
      nowMs: 1000,
      movementPx: 20,
      durationMs: 80,
      maxDelayMs: 320,
      maxDistancePx: 24,
      maxMovementPx: 18,
      maxDurationMs: 280,
    })).toBe('ignore');

    expect(resolveTouchNavigationCursorTapAction({
      navigationCursorActive: false,
      lastCanvasTap: null,
      lastNavigationCursorTap: null,
      clientX: 180,
      clientY: 240,
      nowMs: 1000,
      movementPx: 3,
      durationMs: 400,
      maxDelayMs: 320,
      maxDistancePx: 24,
      maxMovementPx: 18,
      maxDurationMs: 280,
    })).toBe('ignore');
  });

  test('uses triple click detail to hide the active nav cursor', () => {
    expect(shouldHideNavigationCursorOnClick({ navigationCursorActive: true, clickDetail: 3 })).toBe(true);
    expect(shouldHideNavigationCursorOnClick({ navigationCursorActive: true, clickDetail: 2 })).toBe(false);
    expect(shouldHideNavigationCursorOnClick({ navigationCursorActive: false, clickDetail: 3 })).toBe(false);
  });
});
