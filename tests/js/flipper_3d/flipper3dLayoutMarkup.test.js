import fs from 'fs';
import path from 'path';

describe('flipper 3d layout markup', () => {
  test('keeps centered controls above a full-height playfield without overlaying the canvas', () => {
    const htmlPath = path.resolve(process.cwd(), 'examples/js/flipper_3d/index.html');
    const html = fs.readFileSync(htmlPath, 'utf8');
    const controlsIndex = html.indexOf('<div id="controls">');
    const playfieldIndex = html.indexOf('<div id="playfield">');
    const canvasIndex = html.indexOf('<canvas id="myCanvas"></canvas>');

    expect(html).not.toContain('place-items: center;');
    expect(html).toContain('width: 100vw;');
    expect(html).toContain('height: 100dvh;');
    expect(html).toContain('padding: clamp(8px, 1.2vw, 14px);');
    expect(html).toContain('#playfield {');
    expect(html).toContain('#fxLayer {');
    expect(html).toContain('pointer-events: none;');
    expect(html).toContain('grid-template-rows: min-content minmax(0, 1fr);');
    expect(html).toContain('justify-self: center;');
    expect(controlsIndex).toBeGreaterThan(-1);
    expect(playfieldIndex).toBeGreaterThan(-1);
    expect(canvasIndex).toBeGreaterThan(-1);
    expect(playfieldIndex).toBeGreaterThan(controlsIndex);
    expect(canvasIndex).toBeGreaterThan(playfieldIndex);
  });

  test('lets Vite resolve Three.js instead of pinning node_modules paths in the HTML', () => {
    const htmlPath = path.resolve(process.cwd(), 'examples/js/flipper_3d/index.html');
    const html = fs.readFileSync(htmlPath, 'utf8');

    expect(html).not.toContain('node_modules/three/build/three.module.js');
    expect(html).not.toContain('node_modules/three/examples/jsm/');
    expect(html).not.toContain('<script type="importmap">');
  });
});
