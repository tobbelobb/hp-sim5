import { FileFormat } from '../../../integrations/shared/fileFormatUtils.js';
import { resolvePresetCommand } from '../../../hp-sim-3d/app/commandPresetResolver.js';

const HP3_USDA_KEY = 'hp3.usda';
const FOUR_HIGH_ANCHORS_USDA_KEY = 'four_high_anchors.usda';
const CUBECORNERS_USDA_KEY = 'cubecorners.usda';

describe('hp-sim-3d preset selection', () => {
  test('uses hp3 CAN logo defaults when the hp3 scene is loaded', () => {
    const layeredPreset = resolvePresetCommand('hangprinterLogo', true, [HP3_USDA_KEY]);
    const plainPreset = resolvePresetCommand('hangprinterLogo', false, [HP3_USDA_KEY]);

    expect(layeredPreset.path).toContain('Hangprinter_logo6_hp3_w_line_layers.can');
    expect(layeredPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainPreset.path).toContain('Hangprinter_logo6_hp3_no_buildup.can');
    expect(plainPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
  });

  test('uses hp3 CAN square defaults when the hp3 scene is loaded', () => {
    const layeredPreset = resolvePresetCommand('straightMoves', true, [HP3_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [HP3_USDA_KEY]);

    expect(layeredPreset.path).toContain('draw_squares_bigger_hp3_w_line_layers.can');
    expect(layeredPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.path).toContain('draw_squares_bigger_hp3_no_buildup.can');
    expect(plainPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('uses skycam CAN logo defaults when the Four High Anchors scene is loaded', () => {
    const layeredPreset = resolvePresetCommand('hangprinterLogo', true, [FOUR_HIGH_ANCHORS_USDA_KEY]);
    const plainPreset = resolvePresetCommand('hangprinterLogo', false, [FOUR_HIGH_ANCHORS_USDA_KEY]);

    expect(layeredPreset.path).toContain('Hangprinter_logo6_skycam_w_line_layers.can');
    expect(layeredPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainPreset.path).toContain('Hangprinter_logo6_skycam_no_buildup.can');
    expect(plainPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
  });

  test('uses skycam CAN square defaults when the Four High Anchors scene is loaded', () => {
    const layeredPreset = resolvePresetCommand('straightMoves', true, [FOUR_HIGH_ANCHORS_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [FOUR_HIGH_ANCHORS_USDA_KEY]);

    expect(layeredPreset.path).toContain('draw_squares_bigger_skycam_w_line_layers.can');
    expect(layeredPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.path).toContain('draw_squares_bigger_skycam_no_buildup.can');
    expect(plainPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('uses cubecorners CAN square defaults when the cubecorners scene is loaded', () => {
    const layeredPreset = resolvePresetCommand('straightMoves', true, [CUBECORNERS_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [CUBECORNERS_USDA_KEY]);

    expect(layeredPreset.path).toContain('draw_squares_bigger_cubecorners_w_line_layers.can');
    expect(layeredPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.path).toContain('draw_squares_bigger_cubecorners_no_buildup.can');
    expect(plainPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('uses slideprinter presets for uploaded scenes', () => {
    const layeredLogoPreset = resolvePresetCommand('hangprinterLogo', true, []);
    const plainLogoPreset = resolvePresetCommand('hangprinterLogo', false, []);
    const layeredSquarePreset = resolvePresetCommand('straightMoves', true, []);
    const plainSquarePreset = resolvePresetCommand('straightMoves', false, []);

    expect(layeredLogoPreset.path).toContain('Hangprinter_logo6_slideprinter_w_line_layers.can');
    expect(layeredLogoPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainLogoPreset.path).toContain('Hangprinter_logo6_slideprinter_no_buildup.can');
    expect(plainLogoPreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(layeredSquarePreset.path).toContain('draw_squares_bigger_slideprinter_w_line_layers.can');
    expect(layeredSquarePreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(layeredSquarePreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainSquarePreset.path).toContain('draw_squares_bigger_slideprinter_no_buildup.can');
    expect(plainSquarePreset.format).toBe(FileFormat.RRF_CAN_BINARY);
    expect(plainSquarePreset.referencePresetKey).toBe('straightMovesBigger');
  });
});
