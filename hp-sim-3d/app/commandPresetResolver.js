import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';
import {
  CUBECORNERS_USDA_KEY,
  FOUR_HIGH_ANCHORS_USDA_KEY,
  HP3_USDA_KEY,
  HP4_USDA_KEY,
} from './machineCatalog.js';

export const COMMAND_PRESET_VARIANTS = Object.freeze({
  hangprinterLogo: Object.freeze({
    default: Object.freeze({
      url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_slideprinter_no_buildup.can', import.meta.url).href,
      format: FileFormat.RRF_CAN_BINARY,
      referencePresetKey: 'hangprinterLogo',
    }),
    lineLayered: Object.freeze({
      url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_slideprinter_w_line_layers.can', import.meta.url).href,
      format: FileFormat.RRF_CAN_BINARY,
      referencePresetKey: 'hangprinterLogo',
    }),
    machineOverrides: Object.freeze({
      [HP4_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_hp4_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
      }),
      [HP3_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_hp3_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_hp3_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
      }),
      [FOUR_HIGH_ANCHORS_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_skycam_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_skycam_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
      }),
      [CUBECORNERS_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_cubecorners_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_cubecorners_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'hangprinterLogo',
        }),
      }),
    }),
  }),
  straightMoves: Object.freeze({
    default: Object.freeze({
      url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_slideprinter_no_buildup.can', import.meta.url).href,
      format: FileFormat.RRF_CAN_BINARY,
      referencePresetKey: 'straightMovesBigger',
    }),
    lineLayered: Object.freeze({
      url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_slideprinter_w_line_layers.can', import.meta.url).href,
      format: FileFormat.RRF_CAN_BINARY,
      referencePresetKey: 'straightMovesBigger',
    }),
    machineOverrides: Object.freeze({
      [HP4_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_hp4_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_hp4_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
      }),
      [HP3_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_hp3_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_hp3_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
      }),
      [FOUR_HIGH_ANCHORS_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_skycam_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_skycam_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
      }),
      [CUBECORNERS_USDA_KEY]: Object.freeze({
        default: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_cubecorners_no_buildup.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
        lineLayered: Object.freeze({
          url: new URL('../../public/RRF_CAN_commands/draw_squares_bigger_cubecorners_w_line_layers.can', import.meta.url).href,
          format: FileFormat.RRF_CAN_BINARY,
          referencePresetKey: 'straightMovesBigger',
        }),
      }),
    }),
  }),
});

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
  const variants = COMMAND_PRESET_VARIANTS[presetKey];
  if (variants === null || variants === undefined) {
    return null;
  }
  const sourceKeys = Array.isArray(activeSourceKeys) ? activeSourceKeys : [];
  const selectedVariants =
    sourceKeys.map((sourceKey) => variants.machineOverrides?.[sourceKey]).find((override) => override) || variants;
  if (lineLayeringEnabled === true && selectedVariants.lineLayered?.url) {
    return selectedVariants.lineLayered;
  }
  if (selectedVariants.default?.url) {
    return selectedVariants.default;
  }
  if (selectedVariants.lineLayered?.url) {
    return selectedVariants.lineLayered;
  }
  return null;
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

