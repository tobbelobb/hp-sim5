import fs from 'fs';
import path from 'path';

describe('flipper 3d layout markup', () => {
  test('lets the game container use the full viewport with only a small inset', () => {
    const htmlPath = path.resolve(process.cwd(), 'examples/js/flipper_3d/index.html');
    const html = fs.readFileSync(htmlPath, 'utf8');

    expect(html).not.toContain('place-items: center;');
    expect(html).toContain('width: 100vw;');
    expect(html).toContain('height: 100dvh;');
    expect(html).toContain('padding: clamp(8px, 1.2vw, 14px);');
    expect(html).toContain('grid-template-rows: min-content minmax(0, 1fr);');
  });
});
