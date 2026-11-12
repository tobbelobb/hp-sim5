import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';
import { machineForScene } from '../../machines/catalog.js';

// Each preset plays the active machine's RRF compilation of one G-code file.
const COMMAND_PRESETS = Object.freeze({
  hangprinterLogo: Object.freeze({ gcode: 'public/gcode/Hangprinter_logo6.gcode', referencePresetKey: 'hangprinterLogo' }),
  straightMoves: Object.freeze({ gcode: 'public/gcode/draw_squares_bigger.gcode', referencePresetKey: 'straightMovesBigger' }),
});

// Uploaded scenes have no catalogue entry and play the Slideprinter compilations.
const UPLOADED_SCENE_SOURCE_KEY = 'slideprinter.usda';

export function machineForSourceKey(sourceKey, lineLayering) {
  return machineForScene(`public/usd_scenes/${sourceKey}`, lineLayering);
}

export const PRESET_GCODE_MAP = Object.freeze({
  hangprinterLogo: {
    url: new URL('../../public/gcode/Hangprinter_logo6.gcode', import.meta.url).href,
    label: 'Hangprinter Logo (G-code)',
    color: '#ff7a18',
  },
  straightMoves: {
    url: new URL('../../public/gcode/draw_squares.gcode', import.meta.url).href,
    label: 'Draw Squares (G-code)',
    color: '#00b2ff',
  },
  straightMovesBigger: {
    url: new URL('../../public/gcode/draw_squares_bigger.gcode', import.meta.url).href,
    label: 'Draw Bigger Squares (G-code)',
    color: '#00b2ff',
  },
});

export function resolvePresetCommand(presetKey, lineLayeringEnabled, activeSourceKeys = []) {
  const preset = COMMAND_PRESETS[presetKey];
  if (!preset) {
    return null;
  }
  const machine = [...activeSourceKeys, UPLOADED_SCENE_SOURCE_KEY]
    .map((sourceKey) => machineForSourceKey(sourceKey, lineLayeringEnabled === true))
    .find(Boolean);
  return {
    path: machine.compiled_commands[preset.gcode].rrf,
    format: FileFormat.RRF_CAN_BINARY,
    referencePresetKey: preset.referencePresetKey,
  };
}

export function getPresetActionLabel(presetKey) {
  if (presetKey === 'hangprinterLogo') {
    return 'Print Logo';
  }
  if (presetKey === 'straightMoves') {
    return 'Print Squares';
  }
  return presetKey;
}

export function describeSelectedPresetFile(url, baseHref = globalThis.window?.location?.href) {
  if (typeof url !== 'string' || url.length === 0) {
    return null;
  }
  try {
    return new URL(url, baseHref).pathname;
  } catch (_error) {
    return url;
  }
}

