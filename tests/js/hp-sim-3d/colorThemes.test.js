import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

import {
  COLOR_THEMES,
  DEFAULT_COLOR_THEME,
  resolveColorTheme,
} from '../../../src/js/cable_joints_3d/color_themes.js';

const SCENE_KEYS = [
  'label', 'backdrop', 'floor', 'line', 'slack', 'frame', 'rotor', 'rotorBack',
  'eyelet', 'axes', 'axesOpacity', 'print', 'printShading',
];

describe('colour themes', () => {
  test('every theme defines every scene colour the renderer reads', () => {
    for (const [id, theme] of Object.entries(COLOR_THEMES)) {
      for (const key of SCENE_KEYS) {
        expect([id, key, key in theme]).toEqual([id, key, true]);
      }
      expect(theme.axes).toHaveLength(3);
      if (theme.frame !== null) {
        expect(Number.isFinite(theme.frameTint)).toBe(true);
      }
    }
  });

  test('unknown ids fall back to the default theme', () => {
    expect(resolveColorTheme('no-such-theme')).toBe(COLOR_THEMES[DEFAULT_COLOR_THEME]);
    expect(resolveColorTheme('graphite')).toBe(COLOR_THEMES.graphite);
  });

  test('every non-default theme has UI colour tokens', () => {
    const css = readFileSync(resolve(__dirname, '../../../hp-sim-3d/style.css'), 'utf8');
    for (const id of Object.keys(COLOR_THEMES)) {
      if (id !== DEFAULT_COLOR_THEME) {
        expect(css).toContain(`:root[data-theme='${id}']`);
      }
    }
  });
});
