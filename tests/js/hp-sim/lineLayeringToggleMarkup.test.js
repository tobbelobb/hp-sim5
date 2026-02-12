import fs from 'fs';
import path from 'path';

describe('hp-sim line layering toggle markup', () => {
  test('adds a line layering toggle beside quality checks', () => {
    const htmlPath = path.resolve(process.cwd(), 'hp-sim/index.html');
    const html = fs.readFileSync(htmlPath, 'utf8');
    const qualityToggleIndex = html.indexOf('id="qualityToggleWrapper"');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');

    expect(qualityToggleIndex).toBeGreaterThan(-1);
    expect(lineLayeringToggleIndex).toBeGreaterThan(-1);
    expect(lineLayeringToggleIndex).toBeGreaterThan(qualityToggleIndex);
    expect(html).toContain('id="lineLayeringToggle"');
    expect(html).toContain('<span>Line Layering</span>');
  });
});
