import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

function extractPresetResolverSource(source) {
  const start = source.indexOf("const HP3_USDA_KEY = 'hp3.usda';");
  const end = source.indexOf('const PRESET_GCODE_MAP');
  if (start === -1 || end === -1 || end <= start) {
    return null;
  }
  return source.slice(start, end);
}

function loadPresetResolver(source) {
  const resolverSource = extractPresetResolverSource(source);
  if (!resolverSource) {
    throw new Error('Unable to extract hp-sim-3d preset resolver source.');
  }
  return new Function(
    'FileFormat',
    `
      const importMeta = { url: 'file:///home/torbjorn/repos/hp-sim5/hp-sim-3d/app/hp-sim.js' };
      ${resolverSource.replaceAll('import.meta', 'importMeta')}
      return { HP3_USDA_KEY, resolvePresetCommand };
    `
  )({
    MCU_SERIAL: 'MCU_SERIAL',
    RRF_CAN_BINARY: 'RRF_CAN_BINARY',
  });
}

describe('hp-sim-3d preset selection', () => {
  test('uses hp3 CAN logo defaults when the hp3 scene is loaded', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim.js');
    const { HP3_USDA_KEY, resolvePresetCommand } = loadPresetResolver(source);

    const layeredPreset = resolvePresetCommand('hangprinterLogo', true, [HP3_USDA_KEY]);
    const plainPreset = resolvePresetCommand('hangprinterLogo', false, [HP3_USDA_KEY]);

    expect(layeredPreset.url).toContain('Hangprinter_logo6_hp3_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.url).toContain('Hangprinter_logo6_hp3.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
  });

  test('uses hp3 CAN square defaults when the hp3 scene is loaded', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim.js');
    const { HP3_USDA_KEY, resolvePresetCommand } = loadPresetResolver(source);

    const layeredPreset = resolvePresetCommand('straightMoves', true, [HP3_USDA_KEY]);
    const plainPreset = resolvePresetCommand('straightMoves', false, [HP3_USDA_KEY]);

    expect(layeredPreset.url).toContain('draw_squares_bigger_hp3_w_line_layers.can');
    expect(layeredPreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredPreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainPreset.url).toContain('draw_squares_bigger_hp3.can');
    expect(plainPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainPreset.referencePresetKey).toBe('straightMovesBigger');
  });

  test('keeps non-hp3 preset defaults unchanged', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim.js');
    const { resolvePresetCommand } = loadPresetResolver(source);

    const layeredLogoPreset = resolvePresetCommand('hangprinterLogo', true, []);
    const plainLogoPreset = resolvePresetCommand('hangprinterLogo', false, []);
    const layeredSquarePreset = resolvePresetCommand('straightMoves', true, []);
    const plainSquarePreset = resolvePresetCommand('straightMoves', false, []);

    expect(layeredLogoPreset.url).toContain('Hangprinter_logo6_w_line_layers.can');
    expect(layeredLogoPreset.format).toBe('RRF_CAN_BINARY');
    expect(plainLogoPreset.url).toContain('Hangprinter_logo6.serial');
    expect(plainLogoPreset.format).toBe('MCU_SERIAL');
    expect(layeredSquarePreset.url).toContain('draw_squares_bigger_w_line_layers.can');
    expect(layeredSquarePreset.format).toBe('RRF_CAN_BINARY');
    expect(layeredSquarePreset.referencePresetKey).toBe('straightMovesBigger');
    expect(plainSquarePreset.url).toContain('draw_squares.serial');
    expect(plainSquarePreset.format).toBe('MCU_SERIAL');
    expect(plainSquarePreset.referencePresetKey).toBe('straightMoves');
  });

  test('passes loaded machine presets into preset resolution before printing', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim.js');

    expect(source).toContain('const activeSourceKeys = machines.map((machine) => machine.sourceKey).filter(Boolean);');
    expect(source).toContain('const preset = resolvePresetCommand(presetKey, lineLayeringEnabled, activeSourceKeys);');
  });
});
