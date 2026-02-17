import fs from 'fs';
import path from 'path';

function readHpSimSource() {
  const scriptPath = path.resolve(process.cwd(), 'hp-sim/assets/hp-sim.js');
  return fs.readFileSync(scriptPath, 'utf8');
}

function extractSetLineLayeringEnabledState(source) {
  const start = source.indexOf('function setLineLayeringEnabledState');
  const end = source.indexOf('function attachQualityMonitorsToRemoteSystem');
  if (start === -1 || end === -1 || end <= start) {
    return null;
  }
  return source.slice(start, end);
}

function extractRebuildScene(source) {
  const start = source.indexOf('function rebuildScene()');
  const end = source.indexOf('function createColorChip()');
  if (start === -1 || end === -1 || end <= start) {
    return null;
  }
  return source.slice(start, end);
}

describe('hp-sim line layering toggle attachment updates', () => {
  test('imports attachment updater from cable joints core', () => {
    const source = readHpSimSource();
    expect(source).toMatch(
      /import\s*\{\s*_updateAttachmentPoints\s*\}\s*from\s*['"]\.\.\/\.\.\/src\/js\/cable_joints\/cable_joints_core\.js['"]/
    );
  });

  test('updates attachment points for both layering states', () => {
    const source = readHpSimSource();
    const handlerSource = extractSetLineLayeringEnabledState(source);

    expect(handlerSource).not.toBeNull();

    const updateCalls = handlerSource.match(/_updateAttachmentPoints\(world\);/g) || [];
    expect(updateCalls.length).toBe(2);
    expect(handlerSource).not.toMatch(/if\s*\(next\)\s*\{\s*_updateAttachmentPoints\(world\);\s*\}/);
  });

  test('reapplies layering attachment update after rebuilding the scene', () => {
    const source = readHpSimSource();
    const rebuildSource = extractRebuildScene(source);

    expect(rebuildSource).not.toBeNull();
    expect(rebuildSource).toContain('setLineLayeringEnabledState(lineLayeringEnabled, { fromToggle: true });');
  });
});
