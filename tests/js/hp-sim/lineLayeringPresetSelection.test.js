import fs from 'fs';
import path from 'path';

function readHpSimSource() {
  const scriptPath = path.resolve(process.cwd(), 'hp-sim/assets/hp-sim.js');
  return fs.readFileSync(scriptPath, 'utf8');
}

function extractResolvePresetCommand(source) {
  const start = source.indexOf('function resolvePresetCommand');
  const end = source.indexOf('function parseUploadPresetMappings');
  if (start === -1 || end === -1 || end <= start) {
    return null;
  }
  return source.slice(start, end);
}

describe('hp-sim line layering preset selection', () => {
  test('registers line-layered CAN presets for logo and squares', () => {
    const source = readHpSimSource();

    expect(source).toContain('Hangprinter_logo6_w_line_layers.can');
    expect(source).toContain('draw_squares_bigger_w_line_layers.can');
    expect(source).toContain('format: FileFormat.RRF_CAN_BINARY');
  });

  test('maps layered square command preset to bigger square reference path', () => {
    const source = readHpSimSource();

    expect(source).toContain("referencePresetKey: 'straightMovesBigger'");
    expect(source).toContain('const referencePresetKey = preset.referencePresetKey || presetKey;');
    expect(source).toContain('loadReferencePathForPreset(referencePresetKey, { setActive: true })');
  });

  test('chooses line-layered preset variants when line layering is enabled', () => {
    const source = readHpSimSource();
    const resolveSource = extractResolvePresetCommand(source);

    expect(resolveSource).not.toBeNull();
    expect(resolveSource).toContain('lineLayeringEnabled === true');
    expect(resolveSource).toContain('return variants.lineLayered;');
    expect(source).toContain('const preset = resolvePresetCommand(presetKey, lineLayeringEnabled);');
  });
});
