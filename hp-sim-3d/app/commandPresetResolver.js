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
  return machineForScene(`public/usd_scenes/${sourceKey}`, lineLayering)
    ?? machineForScene(`public/usd_scenes/${sourceKey}`, false);
}

export const PRESET_GCODE_MAP = Object.freeze({
  hangprinterLogo: {
    url: new URL('../../public/gcode/Hangprinter_logo6.gcode', import.meta.url).href,
    label: 'Hangprinter Logo (G-code)',
    color: '#e3a572',
  },
  straightMoves: {
    url: new URL('../../public/gcode/draw_squares.gcode', import.meta.url).href,
    label: 'Draw Squares (G-code)',
    color: '#79b1e0',
  },
  copacabanaPlaneATracker: {
    url: new URL('../../public/gcode/copacabana_planeA_tracker.gcode', import.meta.url).href,
    label: 'PlaneA · FARO tracker (13.9 kg)',
    includeTravel: true,
    color: '#79b1e0',
  },
  straightMovesBigger: {
    url: new URL('../../public/gcode/draw_squares_bigger.gcode', import.meta.url).href,
    label: 'Draw Bigger Squares (G-code)',
    color: '#79b1e0',
  },
});

export function resolvePresetCommand(presetKey, lineLayeringEnabled, activeSourceKeys = []) {
  if (!COMMAND_PRESETS[presetKey]) {
    if (activeSourceKeys.length !== 1) return null;
    const replay = machineForSourceKey(activeSourceKeys[0], lineLayeringEnabled)?.reference_replay;
    const path = replay?.streams[presetKey];
    return path ? { path, referencePresetKey: replay.referencePresetKey, traceEntity: replay.traceEntity,
      preserveTiming: true, kinematicMotors: Boolean(replay.kinematicStreams?.includes(presetKey)),
      format: FileFormat.RRF_CAN_BINARY } : null;
  }
  const preset = COMMAND_PRESETS[presetKey];
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

