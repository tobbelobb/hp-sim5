let readFileSync;

async function importFs() {
  if (readFileSync) {
    return;
  }
  ({ readFileSync } = await import('node:fs'));
}

function readFixture(relativePath) {
  return readFileSync(relativePath, 'utf8');
}

describe('Slideprinter line-layering metadata', () => {
  beforeAll(async () => {
    await importFs();
  });

  test('all slideprinter USDA cable paths declare the standard half-width', () => {
    const files = [
      'public/usd_scenes/slideprinter_multi_unit.usda',
      'public/usd_scenes/slideprinter.usda',
      'public/usd_scenes/slideprinter_small.usda',
      'public/usd_scenes/slideprinter_single_pinholes.usda',
      'public/usd_scenes/slideprinter_hexagon.usda',
    ];

    for (const relativePath of files) {
      const text = readFixture(relativePath);
      const cablePathCount = (text.match(/def Xform "CablePath/g) || []).length;
      const halfWidthCount = (text.match(/custom double cablePath:halfWidth = 0\.001/g) || []).length;

      expect(cablePathCount).toBeGreaterThan(0);
      expect(halfWidthCount).toBe(cablePathCount);
    }
  });
});
