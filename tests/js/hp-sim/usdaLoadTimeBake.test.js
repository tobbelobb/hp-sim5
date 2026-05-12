import fs from 'fs';
import path from 'path';

function readSource(relativePath) {
  return fs.readFileSync(path.resolve(process.cwd(), relativePath), 'utf8');
}

describe('USDA load-time baking', () => {
  test.each([
    ['hp-sim', 'hp-sim/app/appBootstrap.js', 'function refreshMachineBakedStage', 'stage = openBakedUsdaStage(sourceText, lineLayeringEnabled)'],
    ['hp-sim-3d', 'hp-sim-3d/app/machineSceneController.js', 'function refreshBakedStage', 'stage = openBakedUsdaStage(sourceText, featureFlags.lineLayeringEnabled)'],
  ])('%s opens catalog and upload USDA files through the baker', (_name, relativePath, refreshFunctionName, stageOpenSnippet) => {
    const source = readSource(relativePath);

    expect(source).toContain('OpenText as UsdOpenText');
    expect(source).toContain('bakeCableSceneUsdaSource');
    expect(source).toContain('function openBakedUsdaStage');
    expect(source).toContain(refreshFunctionName);
    expect(source).toContain('function loadCatalogUsdaSource');
    expect(source).toContain('sourceText');
    expect(source).toContain(stageOpenSnippet);
    expect(source).not.toContain('Open as UsdOpen');
    expect(source).not.toContain('await UsdOpen(entry.url)');
    expect(source).not.toContain('await UsdOpen(sourceText)');
  });
});
