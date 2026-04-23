import fs from 'fs';
import path from 'path';

function readSource(relativePath) {
  return fs.readFileSync(path.resolve(process.cwd(), relativePath), 'utf8');
}

describe('USDA load-time baking', () => {
  test.each([
    ['hp-sim', 'hp-sim/app/hp-sim.js'],
    ['hp-sim-3d', 'hp-sim-3d/app/hp-sim-3d.js'],
  ])('%s opens catalog and upload USDA files through the baker', (_name, relativePath) => {
    const source = readSource(relativePath);

    expect(source).toContain('OpenText as UsdOpenText');
    expect(source).toContain('bakeCableSceneUsdaSource');
    expect(source).toContain('function openBakedUsdaStage');
    expect(source).toContain('function refreshMachineBakedStage');
    expect(source).toContain('function loadCatalogUsdaSource');
    expect(source).toContain('sourceText');
    expect(source).toContain('stage = openBakedUsdaStage(sourceText, lineLayeringEnabled)');
    expect(source).not.toContain('Open as UsdOpen');
    expect(source).not.toContain('await UsdOpen(entry.url)');
    expect(source).not.toContain('await UsdOpen(sourceText)');
  });
});
