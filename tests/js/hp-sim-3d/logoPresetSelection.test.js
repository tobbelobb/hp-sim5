import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

function loadPresetResolver() {
  const source = readWorkspaceFile('hp-sim-3d/app/commandPresetResolver.js')
    .replace(/import[\s\S]*?from '\.\/machineCatalog\.js';\n/, '')
    .replace("import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';\n", '')
    .replaceAll('export ', '')
    .replaceAll('import.meta', 'importMeta');

  return new Function(
    'FileFormat',
    `
      const importMeta = {
        url: 'file:///home/torbjorn/repos/hp-sim5/hp-sim-3d/app/commandPresetResolver.js',
      };
      const HP3_USDA_KEY = 'hp3_rigid_body.usda';
      const HP4_USDA_KEY = 'hp4_rigid_body.usda';
      const FOUR_HIGH_ANCHORS_USDA_KEY = 'four_high_anchors_rigid_body.usda';
      const CUBECORNERS_USDA_KEY = 'cubecorners_rigid_body.usda';
      ${source}
      return { HP3_USDA_KEY, FOUR_HIGH_ANCHORS_USDA_KEY, CUBECORNERS_USDA_KEY, resolvePresetCommand };
    `
  )({
    MCU_SERIAL: 'MCU_SERIAL',
    RRF_CAN_BINARY: 'RRF_CAN_BINARY',
  });
}

describe('hp-sim-3d preset selection', () => {
  test('uses hp3 CAN logo defaults when the hp3 scene is loaded', () => {
    const { HP3_USDA_KEY, resolvePresetCommand } = loadPresetResolver();
    const layeredPreset = resolvePresetCommand('hangprinterLogo', true, [HP3_USDA_KEY]);
    const plainPreset = resolvePresetCommand('hangprinterLogo', false, [HP3_USDA_KEY]);

    expect(layeredPreset.url).toContain('Hangprinter_logo6_hp3_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.url).toContain('Hangprinter_logo6_hp3_no_buildup.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
  });

  test('uses hp3 CAN square defaults when the hp3 scene is loaded', () => {
    const { HP3_USDA_KEY, resolvePresetCommand } = loadPresetResolver();
    const layeredPreset = resolvePresetCommand('straightMoves', true, [HP3_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [HP3_USDA_KEY]);

    expect(layeredPreset.url).toContain('draw_squares_bigger_hp3_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.url).toContain('draw_squares_bigger_hp3_no_buildup.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('uses skycam CAN logo defaults when the Four High Anchors scene is loaded', () => {
    const { FOUR_HIGH_ANCHORS_USDA_KEY, resolvePresetCommand } = loadPresetResolver();
    const layeredPreset = resolvePresetCommand('hangprinterLogo', true, [FOUR_HIGH_ANCHORS_USDA_KEY]);
    const plainPreset = resolvePresetCommand('hangprinterLogo', false, [FOUR_HIGH_ANCHORS_USDA_KEY]);

    expect(layeredPreset.url).toContain('Hangprinter_logo6_skycam_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.url).toContain('Hangprinter_logo6_skycam_no_buildup.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
  });

  test('uses skycam CAN square defaults when the Four High Anchors scene is loaded', () => {
    const { FOUR_HIGH_ANCHORS_USDA_KEY, resolvePresetCommand } = loadPresetResolver();
    const layeredPreset = resolvePresetCommand('straightMoves', true, [FOUR_HIGH_ANCHORS_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [FOUR_HIGH_ANCHORS_USDA_KEY]);

    expect(layeredPreset.url).toContain('draw_squares_bigger_skycam_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.url).toContain('draw_squares_bigger_skycam_no_buildup.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('uses cubecorners CAN square defaults when the cubecorners scene is loaded', () => {
    const { CUBECORNERS_USDA_KEY, resolvePresetCommand } = loadPresetResolver();
    const layeredPreset = resolvePresetCommand('straightMoves', true, [CUBECORNERS_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [CUBECORNERS_USDA_KEY]);

    expect(layeredPreset.url).toContain('draw_squares_bigger_cubecorners_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.url).toContain('draw_squares_bigger_cubecorners_no_buildup.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('keeps slideprinter preset defaults unchanged', () => {
    const { resolvePresetCommand } = loadPresetResolver();
    const layeredLogoPreset = resolvePresetCommand('hangprinterLogo', true, []);
    const plainLogoPreset = resolvePresetCommand('hangprinterLogo', false, []);
    const layeredSquarePreset = resolvePresetCommand('straightMoves', true, []);
    const plainSquarePreset = resolvePresetCommand('straightMoves', false, []);

    expect(layeredLogoPreset.url).toContain('Hangprinter_logo6_slideprinter_w_line_layers.can');
    expect(layeredLogoPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainLogoPreset.url).toContain('Hangprinter_logo6_slideprinter_no_buildup.can');
    expect(plainLogoPreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredSquarePreset.url).toContain('draw_squares_bigger_slideprinter_w_line_layers.can');
    expect(layeredSquarePreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredSquarePreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainSquarePreset.url).toContain('draw_squares_bigger_slideprinter_no_buildup.can');
    expect(plainSquarePreset.format).toBe('RRF_CAN_BINARY');
    expect(plainSquarePreset.referencePresetKey).toBe('straightMovesBigger');
  });
});
